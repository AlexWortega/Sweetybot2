"""Persisted, bounded SweetieMini PPO curriculum orchestration.

Runs only its own sequential children, never manages other host jobs. Every
second since the persisted start (training, evaluation, setup and downtime) is
charged to the budget. Development chooses champions; final held-out seeds are
used exactly once at the end. A champion can still be below the quality gate.
"""
from __future__ import annotations
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any

TASKS=("locomotion","recovery","running")
POLICY_FILES={"locomotion":"walk.pt","recovery":"recover.pt","running":"run.pt"}


def migrate_schedule(state):
    """Extend an existing run without resetting time, checkpoints or history."""
    if state.get('schedule_version',1)<2:
        counts={task:sum(row.get('kind')=='training' and row.get('task')==task
                        and row.get('status')=='completed' for row in state.get('children',[])) for task in TASKS}
        state['completed_chunks_by_task']=counts
        state['schedule_index']=2  # The requested new specialist gets the next slot.
        state['schedule_version']=2
        state.setdefault('migration_log',[]).append({'epoch':time.time(),'reason':'user_requested_running_mode',
            'preserved_deadline_epoch':state['deadline_epoch'],'source_snapshot':str(Path(__file__).resolve().parents[1])})
    for task in TASKS:state.setdefault('consecutive_failures',{}).setdefault(task,0)
    return state


def atomic_json(path: Path, value: Any):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    os.replace(temporary, path)


def sha256(path: Path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run_child(command: list[str], log: Path, deadline: float,
              per_child_seconds: float, environment: dict[str, str]):
    """Kill a timed-out process group created by this invocation only."""
    remaining = min(deadline - time.time(), per_child_seconds)
    if remaining <= 0:
        return {"returncode": None, "status": "budget_exhausted", "wall_seconds": 0.0}
    log.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    with log.open("a", buffering=1) as stream:
        stream.write(json.dumps({"command": command, "started": datetime.now(timezone.utc).isoformat()}) + "\n")
        child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                 env=environment, start_new_session=True)
        try:
            returncode = child.wait(timeout=remaining)
            status = "completed" if returncode == 0 else "failed"
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as exception:
            # child.pid identifies a fresh session/group created immediately above.
            try:
                os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                child.wait(timeout=min(5.0, max(.01, deadline-time.time())))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait(timeout=5)
            if isinstance(exception, KeyboardInterrupt):
                raise
            returncode, status = child.returncode, "timed_out"
    return {"returncode": returncode, "status": status,
            "wall_seconds": time.monotonic()-started, "log": str(log)}


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def score_development(task: str, results: list[dict[str, Any]]):
    """Fixed ranking; never treat shaped training return as task success."""
    if not results:
        raise ValueError("No development evaluations")
    if task not in TASKS:raise ValueError(f'Unknown task {task}')
    if task == "running":
        survival,linear,yaw,rates,minima,speeds=[],[],[],[],[],[]
        coverage=True
        for result in results:
            for key in ('survival_fraction','linear_command_rmse','yaw_command_rmse','running_success_rate'):
                if not _finite(result.get(key)):raise ValueError(f'Missing finite running metric {key}')
            survival.append(result['survival_fraction']);linear.append(result['linear_command_rmse'])
            yaw.append(result['yaw_command_rmse']);rates.append(result['running_success_rate'])
            conditions=result.get('per_command',{})
            expected=('forward','fast_forward','run_turn')
            coverage &= all(name in conditions for name in expected)
            for name in expected:
                row=conditions.get(name,{})
                values=(row.get('running_success_rate'),row.get('survival_fraction'),row.get('mean_local_vx'))
                if not all(_finite(v) for v in values):coverage=False;continue
                minima.append(values[0]);speeds.append(values[2])
                coverage &= values[1]>=.8
        aggregate={'running_success_rate':sum(rates)/len(rates),'worst_command_running_success_rate':min(minima) if minima else None,
            'survival_fraction':sum(survival)/len(survival),'linear_command_rmse':sum(linear)/len(linear),
            'yaw_command_rmse':sum(yaw)/len(yaw),'worst_command_mean_forward_velocity_mps':min(speeds) if speeds else None,
            'command_coverage_complete':bool(coverage),'running_evidence_requires_repeated_flight':True}
        score=2*aggregate['survival_fraction']+2*aggregate['running_success_rate']+(min(minima) if minima else 0)-aggregate['linear_command_rmse']-.1*aggregate['yaw_command_rmse']
        qualified=bool(coverage and minima and speeds and min(survival)>=.8 and min(minima)>=.5
                       and min(speeds)>=.35 and max(linear)<=.20 and max(yaw)<=.5)
    elif task == "locomotion":
        survival, linear, yaw = [], [], []
        nontrivial_motion = False
        command_survivals = []
        command_coverage_complete = True
        expected_commands = {"stand", "forward", "fast_forward", "backward", "sideways", "turn"}
        for result in results:
            for key in ("survival_fraction", "linear_command_rmse", "yaw_command_rmse"):
                if not _finite(result.get(key)):
                    raise ValueError(f"Missing finite development metric {key}")
            survival.append(result["survival_fraction"])
            linear.append(result["linear_command_rmse"])
            yaw.append(result["yaw_command_rmse"])
            conditions = result.get("per_command", {})
            if not isinstance(conditions, dict) or not expected_commands.issubset(conditions):
                command_coverage_complete = False
                conditions = conditions if isinstance(conditions, dict) else {}
            for condition in conditions.values():
                command_survival = condition.get("survival_fraction")
                if _finite(command_survival):
                    command_survivals.append(command_survival)
                else:
                    command_coverage_complete = False
                # Evaluator emits actual body-frame velocity independently of PPO.
                velocity = condition.get("mean_linear_velocity")
                measured_speed = condition.get("measured_speed_mps", condition.get("mean_speed_mps"))
                if isinstance(velocity, (list, tuple)) and len(velocity) >= 2:
                    measured_speed = math.hypot(float(velocity[0]), float(velocity[1]))
                if (_finite(measured_speed) and measured_speed >= .06
                        and _finite(condition.get("command_speed_mps"))
                        and condition["command_speed_mps"] >= .1):
                    nontrivial_motion = True
        aggregate = {"survival_fraction": sum(survival)/len(survival),
                     "linear_command_rmse": sum(linear)/len(linear),
                     "yaw_command_rmse": sum(yaw)/len(yaw),
                     "nontrivial_motion": nontrivial_motion,
                     "command_coverage_complete": command_coverage_complete,
                     "worst_command_survival_fraction": min(command_survivals) if command_survivals else None}
        score = 2*aggregate["survival_fraction"]-aggregate["linear_command_rmse"]-.1*aggregate["yaw_command_rmse"]
        qualified = (min(survival) >= .6 and max(linear) <= .25 and max(yaw) <= .5
                     and nontrivial_motion and command_coverage_complete
                     and bool(command_survivals) and min(command_survivals) >= .6)
    else:
        rates, minima = [], []
        for result in results:
            if not _finite(result.get("recovery_success_rate")):
                raise ValueError("Missing recovery_success_rate")
            rates.append(result["recovery_success_rate"])
            orientations = result.get("per_orientation", {})
            expected = [orientations.get(name, {}).get("success_rate") for name in ("prone", "left", "right", "supine")]
            if not all(_finite(x) for x in expected):
                raise ValueError("Recovery evaluation must report all four orientations")
            minima.append(min(expected))
        aggregate = {"recovery_success_rate": sum(rates)/len(rates),
                     "worst_orientation_success_rate": min(minima)}
        score = aggregate["recovery_success_rate"]+aggregate["worst_orientation_success_rate"]
        qualified = min(minima) >= .5
    # Manufacturer rated torque is a continuous-load engineering screen,
    # not a calibrated thermal model or evidence of physical validation.
    torque_values = []
    torque_complete = True
    for result in results:
        reported = result.get("rms_joint_torque_nm")
        values = reported if isinstance(reported, (list, tuple)) else [reported]
        if not values or not all(_finite(value) and value >= 0 for value in values):
            torque_complete = False
        else:
            torque_values.extend(values)
    observed_max = max(torque_values) if torque_values else None
    torque_gate = torque_complete and observed_max is not None and observed_max <= 1.5691
    aggregate.update(max_observed_rms_joint_torque_nm=observed_max,
                     rated_torque_screen_limit_nm=1.5691,
                     torque_gate_passed=bool(torque_gate),
                     torque_screen_physical_thermal_validation=False)
    qualified = qualified and torque_gate
    return float(score), bool(qualified), aggregate


def regresses(score: float, qualified: bool, champion: dict[str, Any] | None):
    return champion is not None and (
        score < champion["score"]-.10
        or (champion["quality_gate_passed"] and not qualified))


def promote(checkpoint: Path, link: Path):
    """Point the playback entry to the scored champion, never the latest run."""
    temporary = link.with_name(link.name + ".tmp")
    if temporary.exists() or temporary.is_symlink():
        temporary.unlink()
    temporary.symlink_to(checkpoint.resolve())
    os.replace(temporary, link)


def select_gpu(gpus: list[dict[str, Any]], preferred: str, minimum_free_mib: float = 3072.):
    """Pure capacity selection: preserve other jobs and fall back to CPU."""
    available = [gpu for gpu in gpus
                 if _finite(gpu.get("memory_free_mib"))
                 and gpu["memory_free_mib"] >= minimum_free_mib
                 and _finite(gpu.get("utilization_percent"))]
    preferred = preferred.split(",")[0].strip()
    selected = next((gpu for gpu in available if str(gpu["index"]) == preferred), None)
    reason = "preferred_gpu_has_capacity"
    if selected is None and available:
        selected = min(available, key=lambda gpu: (gpu["utilization_percent"],
                       -gpu["memory_free_mib"], str(gpu["index"])))
        reason = "alternate_gpu_has_capacity"
    if selected is None:
        return {"device": "cpu", "physical_gpu": None,
                "reason": "no_gpu_with_3072_mib_free", "capacity_snapshot": gpus}
    return {"device": "cuda:0", "physical_gpu": str(selected["index"]),
            "reason": reason, "capacity_snapshot": gpus}


def capacity_guard(preferred: str, requested_device: str, environment: dict[str, str]):
    if requested_device == "cpu":
        return {"device": "cpu", "physical_gpu": None,
                "reason": "explicit_cpu_requested", "capacity_snapshot": []}
    try:
        response = subprocess.run(["nvidia-smi", "--query-gpu=index,memory.free,utilization.gpu",
                                   "--format=csv,noheader,nounits"],
                                  capture_output=True, text=True, timeout=10, env=environment)
        if response.returncode:
            raise RuntimeError(f"nvidia-smi return code {response.returncode}: {response.stderr.strip()}")
        gpus = []
        for row in csv.reader(response.stdout.splitlines()):
            if not row:
                continue
            if len(row) != 3:
                raise ValueError("Malformed nvidia-smi capacity response")
            gpus.append({"index": row[0].strip(), "memory_free_mib": float(row[1]),
                         "utilization_percent": float(row[2])})
        return select_gpu(gpus, preferred)
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        return {"device": "cpu", "physical_gpu": None,
                "reason": "gpu_capacity_query_failed", "error": str(error), "capacity_snapshot": []}


def preflight(environment: dict[str, str]):
    result = {"timestamp": datetime.now(timezone.utc).isoformat(),
              "hostname": os.uname().nodename, "pid": os.getpid(),
              "cuda_visible_devices": environment["CUDA_VISIBLE_DEVICES"],
              "task_local_caches": {name: environment.get(name) for name in
                                    ("TMPDIR", "XDG_CACHE_HOME", "CUDA_CACHE_PATH", "TORCH_HOME")},
              "python": sys.executable, "other_processes_preserved": True}
    for name, command in (
        ("gpus", ["nvidia-smi", "--query-gpu=index,uuid,name,driver_version,memory.total,memory.used,utilization.gpu", "--format=csv"]),
        ("compute_processes", ["nvidia-smi", "--query-compute-apps=gpu_uuid,pid,process_name,used_memory", "--format=csv"]),
    ):
        try:
            check = subprocess.run(command, capture_output=True, text=True, timeout=10, env=environment)
            result[name] = {"returncode": check.returncode, "stdout": check.stdout, "stderr": check.stderr}
        except (OSError, subprocess.TimeoutExpired) as error:
            result[name] = {"error": str(error)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True)
    parser.add_argument("--hours", type=float, default=24)
    parser.add_argument("--prior-seconds", type=float, default=0,
                        help="Charge preceding pilot/setup/evaluation wall time to the same total budget")
    parser.add_argument("--initial-walk", help="Own pilot checkpoint for locomotion warm start; not automatically a champion")
    parser.add_argument("--initial-recover", help="Own pilot checkpoint for recovery warm start; not automatically a champion")
    parser.add_argument("--chunk-seconds", type=float, default=1200)
    parser.add_argument("--eval-seconds", type=float, default=12)
    parser.add_argument("--eval-timeout", type=float, default=300)
    parser.add_argument("--final-reserve-seconds", type=float, default=1200)
    parser.add_argument("--nenvs", type=int, default=128)
    parser.add_argument("--threads", type=int, default=16)
    parser.add_argument("--cuda-visible-devices", default="1")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--seed", type=int, default=37)
    args = parser.parse_args()
    if not 0 < args.hours <= 24 or args.chunk_seconds <= 0:
        raise ValueError("Budget must be >0 and <=24h; chunk must be positive")
    if not math.isfinite(args.prior_seconds) or args.prior_seconds < 0:
        raise ValueError("Prior seconds must be finite and nonnegative")
    if not 1 <= args.threads <= 16:
        raise ValueError("CPU worker cap is 16")
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    # Keep one orchestrator per output tree, including across accidental launches.
    lock = (out / "driver.lock").open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError("Another driver already owns this output directory")
    def handle_termination(signum, frame):
        raise KeyboardInterrupt(f"Driver received signal {signum}")
    signal.signal(signal.SIGTERM, handle_termination)
    state_path = out / "status.json"
    if state_path.exists():
        state = json.loads(state_path.read_text())
        if state.get("complete"):
            print("This driver already completed; choose another output for another authorized run.")
            return
        if abs(state["budget_hours"]-args.hours) > 1e-9:
            raise ValueError("A persisted run cannot silently change its budget")
    else:
        started = time.time()-args.prior_seconds
        warm_starts={}
        for task,checkpoint in (("locomotion",args.initial_walk),("recovery",args.initial_recover)):
            if checkpoint:
                path=Path(checkpoint).resolve()
                if not path.is_file():raise FileNotFoundError(path)
                warm_starts[task]=str(path)
        state = {"started_epoch": started, "deadline_epoch": started+args.hours*3600,
                 "initial_prior_seconds": args.prior_seconds,
                 "budget_hours": args.hours, "complete": False,
                 "round": 0, "children": [], "latest_training": warm_starts,
                 "champions": {}, "consecutive_failures": {"locomotion": 0, "recovery": 0},
                 "final_started": False, "config": vars(args)}
        state.update(schedule_version=2,schedule_index=0,completed_chunks_by_task={task:0 for task in TASKS})
        atomic_json(state_path, state)
    migrate_schedule(state)
    state['runtime_config']=vars(args)
    state['runtime_source_snapshot']=str(Path(__file__).resolve().parents[1])
    state.pop('stop_reason',None)
    deadline = state["deadline_epoch"]
    environment = dict(os.environ)
    environment.update(CUDA_VISIBLE_DEVICES=args.cuda_visible_devices,
                       OMP_NUM_THREADS=str(args.threads), OPENBLAS_NUM_THREADS="2",
                       MKL_NUM_THREADS="2", MUJOCO_GL="egl", PYTHONUNBUFFERED="1")
    cache_root = Path(__file__).resolve().parents[1] / ".run-cache"
    for variable, relative in (("TMPDIR", "tmp"), ("XDG_CACHE_HOME", "xdg"),
                               ("CUDA_CACHE_PATH", "cuda"), ("TORCH_HOME", "torch")):
        directory = cache_root / relative
        directory.mkdir(parents=True, exist_ok=True)
        environment[variable] = str(directory)
    atomic_json(out / f"preflight_{int(time.time())}.json", preflight(environment))

    def persist(phase):
        now = time.time()
        state["phase"] = phase
        state["updated_epoch"] = now
        state["charged_wall_seconds"] = max(0, now-state["started_epoch"])
        state["remaining_seconds"] = max(0, deadline-now)
        atomic_json(state_path, state)
        atomic_json(out / "budget.json", {key: state[key] for key in (
            "budget_hours", "started_epoch", "deadline_epoch", "charged_wall_seconds", "remaining_seconds")})

    def evaluate(task, checkpoint, directory, terrain, seed, final=False):
        directory.mkdir(parents=True, exist_ok=True)
        command = [sys.executable, "-m", "learning.evaluate", "--checkpoint", str(checkpoint),
                   "--out", str(directory), "--task", task, "--terrain", terrain,
                   "--difficulty", "1.0" if final else ".2", "--episodes", "8",
                   "--seconds", str(args.eval_seconds), "--seed", str(seed),
                   "--threads", str(min(8, args.threads))]
        result = run_child(command, directory / "process.log", deadline,
                           args.eval_timeout, environment)
        state["children"].append({"kind": "final" if final else "development", "task": task, "terrain": terrain, **result})
        persist("final_evaluation" if final else "development_evaluation")
        output = directory / "result.json"
        if result["status"] != "completed" or not output.exists():
            raise RuntimeError(f"{task}/{terrain} evaluation {result['status']}")
        return json.loads(output.read_text())

    def run_final_once():
        if state.get("final_started"):
            # Never repeatedly query the independent held-out set on restart.
            return
        state["final_started"] = True
        persist("final_evaluation")
        finals = {}
        atomic_json(out / "final_results.json", finals)
        for task, champion in state["champions"].items():
            finals[task] = {}
            for index, terrain in enumerate(("flat", "rough", "slope", "steps")):
                if time.time() >= deadline:
                    finals[task][terrain] = {"status": "budget_exhausted"}
                    continue
                try:
                    finals[task][terrain] = evaluate(task, Path(champion["checkpoint"]), out / "final" / task / terrain,
                                                     terrain, 3_000_000+index, final=True)
                except Exception as error:
                    finals[task][terrain] = {"status": "failed", "error": str(error)}
                atomic_json(out / "final_results.json", finals)
        state["final_results"] = str(out / "final_results.json")

    persist("starting")
    try:
        while True:
            # Reserve independent final evaluation plus all development surfaces.
            remaining = deadline-time.time()
            spare = remaining-args.final_reserve_seconds-4*args.eval_timeout-30
            if spare < 30:
                state["stop_reason"] = "reserved_final_evaluation_budget"
                break
            round_index = state["round"]
            available=[TASKS[(state['schedule_index']+offset)%len(TASKS)] for offset in range(len(TASKS))
                       if state['consecutive_failures'][TASKS[(state['schedule_index']+offset)%len(TASKS)]]<3]
            if not available:
                state['stop_reason']='all_tasks_failed_three_consecutive_chunks';break
            task=available[0]
            # Both specialists bootstrap on flat, then visit every procedural
            # surface with flat consolidation between successive terrain cycles.
            task_round = state['completed_chunks_by_task'][task]
            terrain_cycle = ("flat", "mixed", "slope", "mixed", "steps", "mixed")
            terrain = terrain_cycle[task_round % len(terrain_cycle)]
            difficulty = min(1., .12+.04*max(0, task_round-1)) if terrain != "flat" else 0.
            seed = (args.seed+round_index*7919) % 1_000_000
            directory = out / "runs" / f"{round_index:04d}_{task}_{terrain}"
            if directory.exists():
                # Preserve artifacts left by an interrupted attempt at this round.
                directory = directory.with_name(directory.name + f"_retry_{time.time_ns()}")
            directory.mkdir(parents=True, exist_ok=True)
            seconds = min(args.chunk_seconds, spare)
            selection_device = capacity_guard(args.cuda_visible_devices, args.device, environment)
            chunk_environment = dict(environment)
            chunk_environment["CUDA_VISIBLE_DEVICES"] = selection_device["physical_gpu"] or ""
            command = [sys.executable, "-m", "learning.ppo", "--task", task,
                       "--out", str(directory), "--nenvs", str(args.nenvs),
                       "--threads", str(args.threads), "--seconds", str(seconds),
                       "--seed", str(seed), "--device", selection_device["device"],
                       "--terrain", terrain, "--difficulty", str(difficulty),
                       "--gpu-memory-gib", "2", "--save-interval", "60"]
            if task in state["latest_training"]:
                command.extend(("--resume", state["latest_training"][task]))
            elif task=='running':
                donor=state['champions'].get('locomotion',{}).get('checkpoint') or state['latest_training'].get('locomotion')
                if donor:command.extend(('--initialize-from',donor))
            state["active_run"] = {"round": round_index, "task": task, "terrain": terrain,
                                   "directory": str(directory), "seconds": seconds,
                                   "selected_device": selection_device}
            persist("training")
            result = run_child(command, directory / "process.log", deadline,
                               seconds+300, chunk_environment)
            state["children"].append({"kind": "training", "round": round_index, "task": task,
                                      "selected_device": selection_device, **result})
            checkpoint = directory / "latest.pt"
            try:
                if result["status"] != "completed" or not checkpoint.exists():
                    raise RuntimeError(f"Training {result['status']}")
                state['completed_chunks_by_task'][task]+=1
                # Latest may be resumed even if it does not beat the champion.
                state["latest_training"][task] = str(checkpoint)
                evaluations = [evaluate(task, checkpoint, directory / "development" / surface,
                                         surface, 1_000_000+offset)
                               for offset, surface in enumerate(("flat", "rough", "slope", "steps"))]
                score, qualified, metrics = score_development(task, evaluations)
                selection = {"score": score, "quality_gate_passed": qualified,
                             "metrics": metrics, "checkpoint": str(checkpoint),
                             "checkpoint_sha256": sha256(checkpoint), "round": round_index}
                atomic_json(directory / "selection.json", selection)
                old = state["champions"].get(task)
                regressed = regresses(score, qualified, old)
                selection["discarded_for_resume"] = bool(regressed)
                if regressed:
                    state["latest_training"][task] = old["checkpoint"]
                    selection["resume_rollback_checkpoint"] = old["checkpoint"]
                elif old is None or score > old["score"]:
                    state["champions"][task] = selection
                    promote(checkpoint, out / POLICY_FILES[task])
                atomic_json(directory / "selection.json", selection)
                state["consecutive_failures"][task] = 0
            except Exception as error:
                state["consecutive_failures"][task] += 1
                rollback = state["champions"].get(task, {}).get("checkpoint")
                if rollback:
                    state["latest_training"][task] = rollback
                else:
                    # An unevaluated candidate cannot be the bootstrap source.
                    state["latest_training"].pop(task, None)
                atomic_json(directory / "failure.json", {"error": str(error), "training": result,
                            "discarded_for_resume": True, "resume_rollback_checkpoint": rollback})
            state["round"] += 1
            state['schedule_index']=(TASKS.index(task)+1)%len(TASKS)
            persist("between_chunks")
        run_final_once()
        state["complete"] = True
        persist("complete")
    except KeyboardInterrupt:
        state["stop_reason"] = "user_interrupted"
        persist("interrupted")
        raise
    finally:
        persist(state.get("phase", "stopped"))
    print(json.dumps({"status": state["phase"], "rounds": state["round"],
                      "champions": state["champions"], "out": str(out)}, indent=2))


if __name__ == "__main__":
    main()

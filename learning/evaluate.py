"""Deterministic, no-reset evaluation of command tracking and self-righting."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from scripts.common import load,continuous_limits
import time

import numpy as np


COMMANDS = {
    "stand": (0., 0., 0.),
    "forward": (.20, 0., 0.),
    "fast_forward": (.55, 0., 0.),
    "backward": (-.15, 0., 0.),
    "sideways": (0., .10, 0.),
    "turn": (0., 0., .75),
}
RUNNING_COMMANDS = {
    "forward": (.40, 0., 0.),
    "fast_forward": (.55, 0., 0.),
    "run_turn": (.45, 0., .50),
}
POSES = ("stand", "folded_crouch", "left", "right", "supine")


class FlightEvidence:
    """Count complete sustained flight intervals, including one trailing interval.

    Flight must be forward, upright and clear of all nonfoot contacts. Counting
    qualifying ticks only after their interval ends excludes isolated contact
    glitches from both the interval count and the reported fraction.
    """

    def __init__(self, size, dt, minimum_duration=.04, nominal_height=.199):
        self.dt = float(dt)
        self.nominal_height=nominal_height
        self.minimum_ticks = math.ceil(minimum_duration / self.dt - 1e-9)
        self.streak = np.zeros(size, dtype=int)
        self.intervals = np.zeros(size, dtype=int)
        self.valid_ticks = np.zeros(size, dtype=int)
        self.longest_ticks = np.zeros(size, dtype=int)

    def _finish(self, ended):
        valid = ended & (self.streak >= self.minimum_ticks)
        self.intervals[valid] += 1
        self.valid_ticks[valid] += self.streak[valid]
        self.longest_ticks[valid] = np.maximum(self.longest_ticks[valid], self.streak[valid])
        self.streak[ended] = 0

    def update(self, contacts, bad_contacts, up, height, local_vx, active, self_contacts=None):
        valid = (~np.asarray(contacts, bool).any(axis=1)) & (np.asarray(bad_contacts) == 0)
        if self_contacts is not None:valid &= np.asarray(self_contacts)==0
        valid &= (np.asarray(up) > .8) & (np.asarray(height) >= .75*self.nominal_height) & (np.asarray(height) <= 1.4*self.nominal_height)
        valid &= (np.asarray(local_vx) >= .35) & np.asarray(active, bool)
        self._finish(~valid)
        self.streak[valid] += 1

    def finish(self):
        # Clearing the streak makes finalization idempotent.
        self._finish(np.ones_like(self.streak, dtype=bool))


def running_successes(alive, terminal_stable, mean_vx, command_vx, intervals, air_fraction):
    return (np.asarray(alive, bool) & np.asarray(terminal_stable, bool)
            & (np.asarray(mean_vx) >= .35) & (np.asarray(mean_vx) >= .7*np.asarray(command_vx))
            & (np.asarray(intervals) >= 2) & (np.asarray(air_fraction) >= .03))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _mean(values):
    finite = [float(value) for value in values if value is not None and np.isfinite(value)]
    return float(np.mean(finite)) if finite else None


def _binomial_interval(successes, count):
    """Wilson 95% interval; denominators remain visible beside every estimate."""
    z = 1.959963984540054
    proportion = successes / count
    scale = 1 + z*z/count
    center = (proportion + z*z/(2*count)) / scale
    radius = z * math.sqrt(proportion*(1-proportion)/count + z*z/(4*count*count)) / scale
    return [max(0., center-radius), min(1., center+radius)]


def evaluate(checkpoint=None, task="locomotion", terrain="flat", difficulty=.25,
             episodes=16, seed=1_000_000, seconds=12., threads=8, record=None):
    import mujoco
    from learning.env import SweetieVecEnv
    from scripts.common import ROOT

    started = time.monotonic()
    if task not in ("locomotion", "running", "recovery"):
        raise ValueError(task)
    if episodes < 1 or seconds <= 0 or not np.isfinite([seconds, difficulty]).all():
        raise ValueError("Expected positive episodes/seconds and finite difficulty")
    commands = RUNNING_COMMANDS if task == "running" else COMMANDS
    conditions = POSES if task == "recovery" else tuple(commands)
    n = episodes * len(conditions)
    labels = np.repeat(conditions, episodes)
    policy,metadata=None,{}
    mode='legacy'
    if checkpoint is not None:
        import torch
        from learning.ppo import load_policy,observation_mode_for_checkpoint
        torch.set_num_threads(1)
        policy,metadata=load_policy(checkpoint,device='cpu')
        mode=observation_mode_for_checkpoint(metadata)
    env = SweetieVecEnv(task=task, num_envs=n, threads=threads, seed=seed,
                       terrain=terrain, difficulty=difficulty, randomize=False,
                       auto_reset=False,observation_mode=mode)
    env.manual_command = True
    env.reset_pose("stand")
    if task == "recovery":
        for name in conditions:
            env._reset_indices(np.flatnonzero(labels == name), pose=name)
        command = np.zeros((n, 3))
    else:
        command = np.array([commands[label] for label in labels])
    env.commands[:] = command
    env.history[:, -1] = env._frame()
    initial = env.state["qpos"][:, :3].copy()
    initial_up=env.rot[:,2,2].copy()
    initial_height=env.height.copy()
    if checkpoint is not None:
        from learning.ppo import validate_environment_contract
        validate_environment_contract(metadata,env)
        if metadata.get("obs_dim", env.obs_dim) != env.obs_dim or metadata.get("action_dim", 12) != 12:
            raise ValueError("Checkpoint observation/action dimensions differ from environment")
        trained_task = metadata.get("config", {}).get("task")
        if trained_task is not None and trained_task != task:
            raise ValueError(f"Checkpoint was trained for {trained_task!r}, requested {task!r}")

    alive = np.ones(n, bool)
    first_fall = np.full(n, np.nan)
    first_success = np.full(n, np.nan)
    count = np.zeros(n, int)
    error2 = np.zeros((n, 3))
    velocity_sum = np.zeros((n, 3))
    world_velocity_sum = np.zeros((n, 3))
    torque_peak = np.zeros((n, 16))
    speed_peak = np.zeros((n, 16))
    torque_squared = np.zeros((n, 16))
    aerial = np.zeros(n, int)
    forward_aerial = np.zeros(n, int)
    aerial_streak = np.zeros(n)
    longest_aerial = np.zeros(n)
    contact_counts = np.zeros(n)
    self_contact_ticks = np.zeros(n,int)
    flight = FlightEvidence(n, env.dt, nominal_height=env.nominal_height)
    renderer = writer = None
    record_index = 0 if task == "running" else episodes  # forward run/walk or prone recovery
    recorded_frames = 0
    model_hash = sha256(env.model_path)
    terrain_files = {path.name: sha256(path) for path in Path(env.model_path).parent.glob(f"{Path(env.model_path).stem}.*")}
    peak_kind = "sampled_policy_boundary"
    try:
        if record is not None:
            import imageio.v2 as imageio
            Path(record).parent.mkdir(parents=True, exist_ok=True)
            render_model = mujoco.MjModel.from_xml_path(str(env.model_path))
            render_data = mujoco.MjData(render_model)
            renderer = mujoco.Renderer(render_model, width=960, height=720)
            writer = imageio.get_writer(str(record), fps=25, codec="libx264", quality=7)
            camera = mujoco.MjvCamera()
            camera.distance, camera.azimuth, camera.elevation = .8, 135, -20
        for tick in range(math.ceil(seconds / env.dt)):
            if policy is None:
                actions = np.zeros((n, 12), dtype=np.float32)
            else:
                import torch
                with torch.inference_mode():
                    actions = policy.act(env.observe(), deterministic=True).cpu().numpy()
            _, _, done, info = env.step(actions)
            state = env.state
            if not np.isfinite(state["qpos"]).all() or not np.isfinite(state["qvel"]).all():
                raise RuntimeError(f"Nonfinite state at t={(tick+1)*env.dt:.3f}s")
            now = (tick+1) * env.dt
            # Keep failed worlds stepping without resetting them. Their first
            # failure is permanent, and post-fall motion cannot improve tracking.
            active = np.ones(n, bool) if task == "recovery" else alive.copy()
            velocity = np.column_stack((env.local_velocity[:, :2], env.local_angular[:, 2]))
            count[active] += 1
            error2[active] += np.square(velocity[active] - command[active])
            velocity_sum[active] += velocity[active]
            world_velocity_sum[active] += state["qvel"][active, :3]
            torque_data = np.abs(state["torque"])
            if "torque_peak" in state:
                torque_data = state["torque_peak"]
                peak_kind = "all_500Hz_substeps"
            torque_peak[active] = np.maximum(torque_peak[active], torque_data[active])
            speed_peak[active] = np.maximum(speed_peak[active], state["max_speed"][active])
            torque_squared[active] += np.square(state["torque_rms"][active])
            contact_counts[active] += state["bad_contacts"][active]
            self_contact_ticks[active] += state['self_contacts'][active]>0
            flight.update(state["contacts"], state["bad_contacts"], env.rot[:, 2, 2],
                          env.height, env.local_velocity[:, 0], active,state["self_contacts"])
            air = (~state["contacts"].any(axis=1)) & (env.rot[:, 2, 2] > .75) & (env.height > .6*env.nominal_height) & active
            aerial += air
            forward_aerial += air & (env.local_velocity[:, 0] > .10)
            aerial_streak = np.where(air, aerial_streak + env.dt, 0.)
            longest_aerial = np.maximum(longest_aerial, aerial_streak)
            failed = (env.rot[:, 2, 2] < .45) | (env.height < .43*env.nominal_height)
            failed |= (np.abs(state["qpos"][:, :2]) > 3.5).any(axis=1)
            new_fall = alive & failed
            first_fall[new_fall] = now
            alive &= ~failed
            newly_recovered = np.isnan(first_success) & env.success
            first_success[newly_recovered] = now
            if writer is not None and tick % 2 == 0:
                render_data.qpos[:] = state["qpos"][record_index]
                render_data.qvel[:] = state["qvel"][record_index]
                mujoco.mj_forward(render_model, render_data)
                camera.lookat[:] = render_data.qpos[:3]
                renderer.update_scene(render_data, camera)
                writer.append_data(renderer.render())
                recorded_frames += 1
        flight.finish()
        denominator = np.maximum(count, 1)
        mean_velocity = velocity_sum / denominator[:, None]
        terminal_stable = ((env.rot[:, 2, 2] > .8) & (env.height >= .75*env.nominal_height) & (env.height <= 1.4*env.nominal_height)
                           & (env.state["bad_contacts"] == 0) & (env.state["self_contacts"] == 0))
        valid_forward_air_fraction = flight.valid_ticks / denominator
        run_success = running_successes(alive, terminal_stable, mean_velocity[:, 0], command[:, 0],
                                        flight.intervals, valid_forward_air_fraction)
        run_success &= self_contact_ticks==0
        rms_components = np.sqrt(error2 / denominator[:, None])
        rms_torque = np.sqrt(torque_squared / denominator[:, None])
        # env.success means at least 0.7 continuous seconds upright and slow.
        # Requiring that stability at the end excludes transient get-ups followed
        # by another fall from the primary success rate.
        final_success = (env.stable >= 35) & np.isfinite(first_success)
        records = []
        for index in range(n):
            records.append({
                "condition": str(labels[index]), "replicate": int(index % episodes),
                "initial_up_z":float(initial_up[index]),"initial_body_height_m":float(initial_height[index]),
                "recovery_waypoints_reached":{name:bool(env.recovery_waypoints.earned[index,i])
                    for i,name in enumerate(env.recovery_waypoints.names)} if task=='recovery' else {},
                "command": command[index].tolist(), "samples": int(count[index]),
                "survived": bool(alive[index]),
                "first_fall_s": float(first_fall[index]) if np.isfinite(first_fall[index]) else None,
                "stable_recovery": bool(final_success[index]),
                "ever_recovered": bool(np.isfinite(first_success[index])),
                "time_to_first_stable_stand_s": float(first_success[index]) if np.isfinite(first_success[index]) else None,
                "linear_command_rmse": float(np.linalg.norm(rms_components[index, :2])),
                "yaw_command_rmse": float(rms_components[index, 2]),
                "command_rmse": float(np.linalg.norm(rms_components[index])),
                "mean_local_vx": float(mean_velocity[index, 0]),
                "mean_local_vy": float(mean_velocity[index, 1]),
                "mean_yaw_rate": float(mean_velocity[index, 2]),
                "mean_world_velocity": (world_velocity_sum[index]/denominator[index]).tolist(),
                "displacement_world_m": (env.state["qpos"][index, :3]-initial[index]).tolist(),
                "aerial_upright_fraction": float(aerial[index]/denominator[index]),
                "forward_aerial_fraction": float(forward_aerial[index]/denominator[index]),
                "max_aerial_duration_s": float(longest_aerial[index]),
                "nonfoot_contacts_mean": float(contact_counts[index]/denominator[index]),
                "self_contact_fraction":float(self_contact_ticks[index]/denominator[index]),
                "terminal_stable": bool(terminal_stable[index]),
                "running_success": bool(run_success[index]),
                "valid_flight_intervals": int(flight.intervals[index]),
                "valid_forward_air_ticks": int(flight.valid_ticks[index]),
                "max_valid_flight_duration_s": float(flight.longest_ticks[index]*env.dt),
                "valid_forward_air_fraction": float(valid_forward_air_fraction[index]),
            })
        grouped = {}
        for name in conditions:
            rows = [row for row in records if row["condition"] == name]
            ix = np.flatnonzero(labels == name)
            grouped[name] = {
                "episodes": len(rows), "survivals": int(alive[ix].sum()),
                "survival_fraction": float(alive[ix].mean()),
                "survival_ci95": _binomial_interval(int(alive[ix].sum()), len(rows)),
                "successes": int(final_success[ix].sum()), "success_rate": float(final_success[ix].mean()),
                "success_ci95": _binomial_interval(int(final_success[ix].sum()), len(rows)),
                "waypoint_reach_rates":{name:float(env.recovery_waypoints.earned[ix,i].mean())
                    for i,name in enumerate(env.recovery_waypoints.names)} if task=='recovery' else {},
                "ever_recovered": int(np.isfinite(first_success[ix]).sum()),
                "time_to_stand_mean_s": _mean(first_success[ix]),
                "command": command[ix[0]].tolist(),
                "command_rmse": _mean([row["command_rmse"] for row in rows]),
                "linear_command_rmse": _mean([row["linear_command_rmse"] for row in rows]),
                "yaw_command_rmse": _mean([row["yaw_command_rmse"] for row in rows]),
                "mean_local_vx": _mean(mean_velocity[ix, 0]),
                "mean_local_vy": _mean(mean_velocity[ix, 1]),
                "measured_speed_mps": float(np.linalg.norm(mean_velocity[ix, :2], axis=1).mean()),
                "command_speed_mps": float(np.linalg.norm(command[ix[0], :2])),
                "aerial_upright_fraction": float((aerial[ix]/denominator[ix]).mean()),
                "forward_aerial_fraction": float((forward_aerial[ix]/denominator[ix]).mean()),
                "running_successes": int(run_success[ix].sum()),
                "running_success_rate": float(run_success[ix].mean()),
                "running_success_ci95": _binomial_interval(int(run_success[ix].sum()), len(rows)),
                "mean_valid_flight_intervals": float(flight.intervals[ix].mean()),
                "max_valid_flight_duration_s": float(flight.longest_ticks[ix].max()*env.dt),
                "valid_forward_air_fraction": float(valid_forward_air_fraction[ix].mean()),
                "valid_forward_air_ticks": int(flight.valid_ticks[ix].sum()),
                "tracking_samples": int(count[ix].sum()),
            }
        moving = np.linalg.norm(command, axis=1) > 0
        recovery_mask = labels != "stand"
        public_terrain = {key: value for key, value in env.terrain.items() if key != "height_samples"}
        result = {
            "schema_version": 2, "observation_contract":env.observation_contract, "reward_version":env.reward_version, "task": task, "policy": "checkpoint" if checkpoint else "zero_action_baseline",
            "checkpoint": str(Path(checkpoint).resolve()) if checkpoint else None,
            "checkpoint_sha256": sha256(checkpoint) if checkpoint else None,
            "checkpoint_metadata": metadata, "seed": int(seed),
            "split": "final" if seed >= 4_000_000 else "preview" if seed >= 2_000_000 else "development" if seed >= 1_000_000 else "training_range",
            "episodes_per_condition": episodes, "episodes": n, "seconds": float(seconds),
            "self_contact_fraction":float(np.max(self_contact_ticks/denominator)),
            "survival_fraction": float(alive.mean()) if task != "recovery" else None,
            "command_rmse": float(np.linalg.norm(rms_components[moving], axis=1).mean()) if moving.any() else None,
            "linear_command_rmse": float(np.linalg.norm(rms_components[moving, :2], axis=1).mean()) if moving.any() else None,
            "yaw_command_rmse": float(rms_components[moving, 2].mean()) if moving.any() else None,
            "recovery_success_rate": float(final_success[recovery_mask].mean()) if task == "recovery" else None,
            "recovery_successes": int(final_success[recovery_mask].sum()) if task == "recovery" else None,
            "recovery_episodes": int(recovery_mask.sum()) if task == "recovery" else None,
            "per_command": grouped if task != "recovery" else {},
            "per_orientation": grouped if task == "recovery" else {},
            "continuous_torque_limits_nm": continuous_limits(load()), "joint_names": env.names, "max_joint_speed_rad_s": speed_peak.max(axis=0).tolist(),
            "max_joint_torque_nm": torque_peak.max(axis=0).tolist(),
            "torque_peak_sampling": peak_kind,
            "rms_joint_torque_nm": np.max(rms_torque, axis=0).tolist(),
            "rms_aggregation": "maximum episode RMS per joint across conditions and replicates",
            "terrain": public_terrain, "model_path": str(env.model_path), "model_sha256": model_hash,
            "terrain_files_sha256": terrain_files, "mujoco_version": mujoco.__version__,
            "source_sha256": {str(path.relative_to(ROOT)): sha256(path) for path in
                              (ROOT/'config/robot.yaml', ROOT/'simulation/model.py', ROOT/'learning/env.py',
                               ROOT/'learning/native.cpp', ROOT/'learning/evaluate.py', ROOT/'learning/sensors.py', ROOT/'learning/rewards.py')},
            "auto_reset": False, "randomize": False, "root_support_forces": False,
            "recovery_success_definition": "upright up_z>.90, height>84% nominal stance, horizontal speed<.25m/s for .7s, still stable at episode end",
            "running_success_definition": "survive and end upright up_z>.8, height 75..140% nominal stance without nonfoot contact; mean local vx>=.35m/s and >=70% commanded vx; >=2 forward flight intervals lasting >=.04s each; valid forward flight >=3% of tracking samples. Flight requires no feet or nonfoot contacts, up_z>.8, height 75..140% nominal stance, local vx>=.35m/s",
            "running_successes": int(run_success.sum()) if task == "running" else None,
            "running_success_rate": float(run_success.mean()) if task == "running" else None,
            "running_success_ci95": _binomial_interval(int(run_success.sum()), n) if task == "running" else None,
            "running_episodes": n if task == "running" else None,
            "min_per_command_running_success_rate": min(row["running_success_rate"] for row in grouped.values()) if task == "running" else None,
            "tracking_denominator": "policy steps through the first locomotion failure, including its terminal step; failures remain failures",
            "recording": {"path": str(record), "condition": str(labels[record_index]), "replicate": 0, "frames": recorded_frames} if record else None,
            "episodes_detail": records, "wall_seconds": time.monotonic()-started,
        }
        # A requested high speed alone is not evidence of running.
        if task == "locomotion":
            forward = grouped["fast_forward"]
            result["running_evidence"] = bool(forward["running_success_rate"] >= .8)
        elif task == "running":
            result["running_evidence"] = bool(result["min_per_command_running_success_rate"] >= .8)
        return result
    finally:
        if writer is not None:
            writer.close()
        if renderer is not None:
            renderer.close()
        env.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--out", required=True, type=Path, help="JSON filename or directory receiving result.json")
    parser.add_argument("--task", choices=("locomotion", "running", "recovery"), default="locomotion")
    parser.add_argument("--terrain", choices=("flat", "rough", "slope", "steps"), default="flat")
    parser.add_argument("--difficulty", type=float, default=.25)
    parser.add_argument("--episodes", type=int, default=16, help="Replicates per command/orientation condition")
    parser.add_argument("--seed", type=int, default=1_000_000)
    parser.add_argument("--seconds", type=float, default=12.)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--record", type=Path)
    args = parser.parse_args(argv)
    result = evaluate(args.checkpoint, args.task, args.terrain, args.difficulty, args.episodes,
                      args.seed, args.seconds, args.threads, args.record)
    output = args.out if args.out.suffix == ".json" else args.out/'result.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name+'.tmp')
    temporary.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    temporary.replace(output)
    print(json.dumps({key: result[key] for key in ("task", "episodes", "survival_fraction", "command_rmse", "recovery_success_rate", "wall_seconds")}), flush=True)


if __name__ == "__main__":
    main()

> **Current v05:** use the [v05 build and protocol](../docs/v05.md). V04 was withdrawn. The 591-value sensor contract now includes a dynamics signature, CAD-derived self-collisions and solver-warning rejection. The original v02 cycle below is complete and incompatible. Start a new directory; `--require-gpu` waits for an available GPU rather than using CPU fallback. Waiting counts against the configured wall-time limit.

# SweetieMini v02 learning and joystick replay

This trains walking, running and recovery policies for the accepted v02 quadruped. Microduck supplies research and design references; its biped checkpoints are incompatible. The existing `simulation/run.py` crawl is a scripted virtual-wrench controller, not a trained policy. See [the fixed experiment protocol](PLAN.md).

Training and replay use native MuJoCo, 50 Hz policy decisions and bounded servo position targets. Twelve leg joints are learned; head and ears retain neutral targets. The actor uses IMU-like orientation/angular velocity, joint state, prior actions, commands and history. No virtual support wrench or world-position controller assists the learned policy. Actuator dynamics, inertias and contact geometry remain approximations; hardware transfer is unverified.

## Separate Python environment

Use Python 3.11 or 3.12 for learning; retain the existing Python 3.13 CAD environment.

```sh
uv venv --python 3.11 .venv-learning
uv pip install --python .venv-learning/bin/python -r learning/requirements.txt
.venv-learning/bin/python -m learning.build_native
# For a gamepad:
uv pip install --python .venv-learning/bin/python pygame==2.6.1
```

On a CUDA machine install a Torch 2.5.1 wheel compatible with its GPU and driver before the remaining requirements. The training process must use only the assigned GPU and its own output directory. Current commands and checkpoints are recorded alongside run results; a running job does not establish policy quality.

## Training and progress

Start a bounded experiment on your own compute host (adjust GPU and worker counts):

```sh
.venv-learning/bin/python -m learning.driver --out learning/runs/main --hours 24 --threads 12 --nenvs 128 --cuda-visible-devices 0
```

The driver charges all elapsed time, including setup and evaluation, to its deadline. It saves `status.json`, `budget.json`, per-run `metrics.jsonl` and `selection.json`. Development checks flat ground, roughness, slopes and steps; failed candidates remain available and training can roll back to a better checkpoint. Final held-out seeds are reserved until the budget ends. File presence alone does not establish a successful skill.

Preview weights for the README animation are available in the [preview release](https://github.com/AlexWortega/Sweetybot2/releases/tag/v0.2.0-preview). Extract `preview-policies.zip` into the project root; files appear under `learning/checkpoints/`. All three preview policies currently fail their complete quality gates.

Optional remote progress sync (supply your own SSH host/path):

```sh
.venv-learning/bin/python -m learning.watch --remote gpu-host:/data/sweetybot2/training --dest learning/results --once
```

## Native MuJoCo replay

From the project root, substitute checkpoints produced by training:

```sh
.venv-learning/bin/python -m learning.play --walk /path/to/walk.pt --recover /path/to/recover.pt
.venv-learning/bin/python -m learning.play --walk /path/to/walk.pt --run /path/to/run.pt --recover /path/to/recover.pt --mode run
.venv-learning/bin/python -m learning.play --walk /path/to/walk.pt --recover /path/to/recover.pt --joystick --terrain rough --difficulty 0.25
.venv-learning/bin/python -m learning.play --walk /path/to/walk.pt --recover /path/to/recover.pt --headless --pose left --seconds 15 --record renders/recovery_left.mp4
.venv-learning/bin/python -m learning.play --walk /path/to/walk.pt --headless --command 0.15 0 0 --seconds 20 --record renders/policy_walk.mp4
.venv-learning/bin/python -m learning.play --walk /path/to/walk.pt --cad --headless --command 0.20 0 0 --seconds 8 --record renders/cad_policy_walk.mp4
```

Linux headless rendering may require `MUJOCO_GL=egl` or an available OSMesa installation. On macOS a passive MuJoCo viewer requires the environment's `mjpython` launcher: `.venv-learning/bin/mjpython -m learning.play ...`.

`--cad` displays the saved v02 CAD meshes while the native backend continues stepping the same lightweight collision model. It adds no mass or contacts and can take more memory to load. Omit it for the primitive preview. The initial training experiment and preliminary measured results are documented in [the run record](../docs/rl-training.md).

Keyboard commands latch: W/S forward/back, A/D sideways, Q/E turn, **B toggles walk/run**, Space zero command, F explicit fall disturbance, R explicit stand reset, Escape quit. Falling does not automatically reset the robot. Recovery has priority over both motion modes and yields only after a stable upright interval; the selected motion mode is retained, with velocity ramping from zero. Reset and disturbance actions are printed so they cannot be mistaken for autonomous recovery. Without `--recover`, automatic recovery switching is disabled.

Gamepad input uses SDL's standard controller layout: left stick commands forward/sideways, right-stick X commands yaw. **Hold LB/L1** to move; **press RB/R1** to toggle walk/run. Holding RB does not repeatedly toggle. Releasing LB, disconnecting, or invalid/stale input immediately commands zero velocity. An unrecognized raw joystick uses axes0/1 for the left stick, axis3 for yaw, button4 for LB and button5 for RB; override with `--yaw-axis`, `--deadman-button`, `--run-button`. Walking defaults to0.25m/s forward and running to0.60m/s (`--max-vx`/`--run-max-vx`); normal mode changes have a0.6m/s² acceleration bound. Reverse, lateral-only motion and turning in place use the walking specialist. These are requested limits, not demonstrated speeds.

`--run` accepts only a checkpoint trained for `running`. Without it the runtime explicitly labels run mode `walk-fast-fallback`: the walking policy receives a faster target and no successful-running claim is made. The running specialist is trained separately from walking using `--task running --initialize-from <walk.pt>`; initialization copies weights and observation normalization, starts a fresh optimizer/RNG/counters and records the exact donor SHA. `--resume` remains restricted to the same task. The supervisor rotates all three tasks within the original24-hour deadline, downloads `run.pt` alongside `walk.pt`/`recover.pt`, and evaluates actual forward speed plus repeated clean flight intervals. It never raises servo limits for running.

## Reading results

Run an independent deterministic evaluation or omit the checkpoint for the zero-action baseline:

```sh
.venv-learning/bin/python -m learning.evaluate --checkpoint /path/to/walk.pt --task locomotion --terrain rough --difficulty 0.25 --seed 1000000 --episodes 16 --out reports/walk_development
.venv-learning/bin/python -m learning.evaluate --checkpoint /path/to/recover.pt --task recovery --seed 1000001 --episodes 16 --out reports/recovery_development.json
.venv-learning/bin/python -m learning.evaluate --task recovery --episodes 4 --out reports/recovery_zero_action.json
```

`--out` accepts a JSON filename or a directory receiving `result.json`. `--episodes` is the denominator **per condition**: six fixed commands for locomotion; stand control, folded crouch (legacy name prone), left, right and supine for recovery. The aggregate recovery score excludes the stand control. No automatic resets occur during evaluation. A locomotion fall remains a failure even if the robot later rises; tracking statistics cover steps through the first failure. Primary recovery success requires 0.7 continuous seconds upright and slow, with that stability retained at the episode end. Reports include per-condition denominators and Wilson intervals, actual velocity, joint speed/torque, aerial phases, model/checkpoint/source hashes, terrain parameters and the seed range. Training seeds are below 1000000; development seeds are 1000000..1000099; final held-out seeds are 3000000..3000099 and reserved for the driver's final evaluation. The earlier player preview range 2000000 is excluded from final testing.

Keep locomotion and recovery results separate, with all evaluation episode counts. Report tracking errors, actual speeds, falls and measured aerial phases before calling a gait running. Recovery tests must include prone, supine and both sides, count stable completed stands and time-to-stand, and exclude explicit pose resets from success. Terrain coverage means tested ranges of slopes, roughness, step heights and friction; this project cannot establish “any terrain.” Checkpoints and videos require independent evaluation before being described as successful policies.

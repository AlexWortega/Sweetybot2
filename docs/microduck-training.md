# MicroDuck-inspired v05 training revision

This revision adapts training ideas from [pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl/tree/b02fdea4902c2b93b7d34a83a6f05da4ececb7d3), checked on 2026-10-19. Implementation is specific to Sweetybot2's quadruped, sensor ABI and motor model. Inspiration: the [stand-up curriculum](https://github.com/pollen-robotics/microduck_rl/blob/b02fdea4902c2b93b7d34a83a6f05da4ececb7d3/src/mjlab_microduck/tasks/microduck_standup_env_cfg.py), [reward-design lessons](https://github.com/pollen-robotics/microduck_rl/blob/b02fdea4902c2b93b7d34a83a6f05da4ececb7d3/AGENTS.md), and [VelStand teacher/student recipe](https://github.com/pollen-robotics/microduck_rl/blob/b02fdea4902c2b93b7d34a83a6f05da4ececb7d3/docs/velstand_policy.md).

## Changes

- Curriculum counts accumulated policy steps **per environment**, persisted in checkpoints. A 20-minute worker restart cannot return it to stage zero. The default horizon is 50,000 steps/environment, independent of wall-time throughput.
- Recovery starts with 90% standing/crouched episodes and partial side rolls; full inverted starts enter later. Later stages retain easy episodes while increasing side/supine practice. Partial rolls provide starts partway through the maneuver; only episode initialization is changed. No base forces, mid-episode pose fixes or scripted get-up path are added.
- Recovery stays on flat terrain until it passes the existing independent development quality gate. Four-surface evaluation is retained from the beginning.
- Motion smoothing is weak during discovery and ramps later. Arrival damping acts near upright standing height only. Torque-speed saturation, overload costs, physical collision geometry and self-collision costs stay active throughout.
- A frozen walking policy is an optional **near-upright recovery anchor**. It receives zero velocity commands, uses its own observation normalizer, and is eligible only with healthy/recent IMU and at least two valid support contacts. Walking's home-relative actions are converted to recovery's incremental coordinates before the BC loss. It supplies no labels while the robot is on its side/back and is **not a learned get-up teacher**. PPO must discover that part. Teacher eligibility/loss are logged.
- Exact-zero idle, backward, sideways and pure yaw commands have explicit sampling buckets. A bounded directional progress reward supplies a signal when the commanded velocity is far from the achieved velocity; wrong-way motion and body-supported motion cannot earn it.
- Per-term reward contributions, curriculum stage and BC metrics are logged beside actual independent task scores. Held-out final seeds are unchanged.

## Transfer and budget

Old checkpoints remain strict for playback and optimizer resume. An explicit `--initialize-from ... --initializer-source ...` operation permits weights/normalizer transfer only when the donor matches its source snapshot and the geometry/native motor/sensor signatures are identical. The target checkpoint records both recipe signatures, source checkpoint SHA, and a fresh optimizer. This does not relabel old evaluation results as results of the new recipe.

The new phase uses a fresh source snapshot/output directory and preserves the original campaign start `1792350507.774651`, budget 36 hours, and deadline `1792480107.774651` (2026-10-20 09:08:27 Europe/Belgrade). Preparation and evaluation consume the same budget. Archive the old driver's status and stop only its own child before handing off. Existing host services and v02 outputs remain untouched.

The existing native CPU MuJoCo physics backend is retained for this bounded revision. MicroDuck's MuJoCo Warp backend and calibrated XL330 BAM parameters are not installed or substituted for our SM105BL/STS models. Calibrating these servos requires measurements from the actual hardware.

## Interpretation

Baseline v05 walking passed its preliminary easy-terrain development gate but undertracked speed; selected running and recovery policies had zero strict task successes. This revision is a training hypothesis. Unit tests, neutral physics acceptance, checkpoint transfer and short PPO smokes establish implementation/liveness only. Learned improvements must be measured on unchanged development tests, then the reserved final set, and watched in rollouts.

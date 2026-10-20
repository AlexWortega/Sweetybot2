# Compact proprioceptive policy, revision 3

Adapted from [MicroDuck's velocity actor/critic configuration](https://github.com/pollen-robotics/microduck_rl/blob/b02fdea4902c2b93b7d34a83a6f05da4ececb7d3/src/mjlab_microduck/tasks/microduck_velocity_env_cfg.py) and [VelStand warm-start practice](https://github.com/pollen-robotics/microduck_rl/blob/b02fdea4902c2b93b7d34a83a6f05da4ececb7d3/src/mjlab_microduck/tasks/microduck_velstand_env_cfg.py), inspected 2026-10-20. This is our quadruped-specific implementation. No MicroDuck weights or XL330 motor parameters are substituted.

## Actor and critic

The default `learning.ppo` / `learning.driver` mode is `compact`.

| Actor input | Values |
|---|---:|
| Estimated gravity + gyro | 6 |
| Positions + velocities of all 16 joints | 32 |
| Previous 12 leg actions | 12 |
| Velocity command | 3 |
| Four hoof switches + four validity flags | 8 |
| IMU validity + sample age | 2 |
| Total, one sampled frame | **63** |

`hardware/policy_observation.py` defines contract `v05-compact-proprio-1`. The 197-value raw sensor encoder, IMU/contact estimator, native physics, masses, meshes, motors, RGB and ToF devices remain unchanged. The compact actor has no ToF, camera, accelerometer, clock, three-frame stack, world position or privileged state. Sensor sampling noise is not resampled when projecting a stored frame.

The critic's 81-value training-only contract is `v05-privileged-value-1`: 63 clean proprioceptive values, local linear velocity (3), normalized height (1), normalized contact forces (4), ground-relative hoof heights (4), per-foot air times (4), body/self-contact counts (2). Clean IMU/encoders and contacts replace noisy estimates in its base 63 values. Actor and critic have separate observation normalizers and separate **512–256–128 ELU** MLPs. Actor output remains 12 squashed Gaussian leg actions, with the same home-relative walking/running and incremental recovery conventions. Only `act(actor_obs)` is needed for replay; privileged values never enter it.

Timeout values are bootstrapped from the **terminal critic observation before autoreset**. GAE stops at episode boundaries. Actor and critic normalization statistics are frozen throughout collection and the associated update. Checkpoints record architecture, both contracts, normalizers, optimizer/RNG/curriculum state and source hashes. Old state dictionaries use their original 256–128–128 architecture and remain readable; playback still requires their matching immutable environment source.

## Commands and learning

Compact walking samples fixed modest ranges: 20% idle, 15% turn-in-place, 20% forward, 10% backward, 10% sideways, 25% simultaneous translation/turn commands. Forward-only targets are 0.08–0.30 m/s; backward 0.08–0.18; sideways 0.04–0.12; turns 0.20–1.0 rad/s. Mixed targets use vx −0.15..0.30, vy ±0.10, yaw ±0.60. These bounds do not expand simply because time passed. Running samples forward 0.35–0.55 m/s, vy ±0.05, yaw ±0.35, with 10% idle. Actual achieved speeds remain evaluation outputs.

Every four seconds a new training target is sampled; translation and yaw commands approach it at bounded 0.6 units/s. Manual/evaluation commands remain explicit and constant to preserve evaluation comparability. The existing manual controller smooths joystick motion. Compact PPO adapts learning rate within 1e-5..1e-3 using desired KL 0.01; this schedule does not discard trajectories or restore weights. Existing batch defaults and the native CPU physics backend remain; this is not a MuJoCo Warp port.

`latest_training` and `champions` now serve separate purposes. Every completed valid worker continues its weights **and optimizer** into the next chunk even after poor development scores. Only a better champion is promoted to the replay link; an unqualified candidate cannot replace a qualified champion. Worker/numerical failures restore the previous training checkpoint. An evaluation failure preserves successful training and is recorded separately. All candidates and evaluations stay saved.

## Preserve an existing walking policy

The previous 591-input actor has a different architecture and cannot be resumed as a 63-input actor. `learning.distill` loads it as a frozen **label-producing teacher** only after its original source hashes, recipe and physical/sensor signature match the supplied snapshot. It collects actual MuJoCo teacher/student roll-ins. Only compact observations enter the new actor. BC fits the teacher's leg actions; a fresh critic and fresh PPO optimizer are exported with PPO steps/updates zero. BC samples and provenance are recorded separately. This is an initializer, not proof of retained walking quality, and must be evaluated independently.

```sh
python -m learning.distill --teacher /path/to/legacy/walk.pt \
  --teacher-source /path/to/immutable/source-596bf65 \
  --out learning/runs/compact-warmstart --seconds 60 --max-steps 8192

# Run only after allocating a NEW explicit campaign budget.
python -m learning.driver --out /path/to/fresh/compact-campaign \
  --hours HOURS --observation-mode compact --require-gpu \
  --initial-walk learning/runs/compact-warmstart/latest.pt
```

A frozen walking teacher can also anchor near-upright recovery with `--recovery-teacher` and an explicit `--teacher-source` for a legacy donor. Legacy history is training-only label input, not student input. IMU/contact health gates and home-to-incremental action conversion are retained. No labels from a walking policy are presented as a side/back get-up expert.

## Evaluation and budget

Prior final results used seeds 4000000..4000099 and are now known. Compact revision 3 reserves **5000000..5000099**; the driver queries those only once at the end of a new authorized cycle. Development still uses all four surfaces and fixed command/orientation conditions. The 0.55 m/s walking challenge remains unchanged even though it exceeds the compact walking training range; this is an explicit extrapolation challenge, not an achieved-speed claim.

The old v05 campaign completed on 2026-10-20 and is immutable. Its deadline is not extended. On 2026-10-20 the user separately authorized a new **200-hour** compact campaign on eva01. The driver accepts a finite cap up to 200 hours and persists its absolute start/deadline; setup, waiting and all evaluations count against the same window. Only one training GPU is used at a time. The launch receipt identifies the immutable executor/output; CPU implementation checks and short distillation/PPO runs establish wiring and persistence only. Whether compact observations, privileged values and uninterrupted exploration improve quality requires independent evaluation; no benefit is claimed yet. Physical assembly/thermal validation remains outstanding.

### 200-hour launch receipt

Executor `6491c4a6a78d65bee7906f424216e3ab3ce0aa61` is immutable at `/home/alexw/sweetybot2-v05-20261018/source-6491c4a`; output is `training-compact-200h`. The total window is 2026-10-20 11:40:15 UTC to 2026-10-28 19:40:15 UTC (20:40:15 Europe/Belgrade after the DST change). An enabled user service restarts failed drivers with the same absolute budget; a persistent independent timer stops only its cgroup before expiry. Launch verification observed physical GPU 1 and a checkpoint with 10 updates / 81,920 transitions. This proves active optimization and persistence, not learned quality. [Receipt](../reports/compact_200h_launch.json).

> **Current v05:** mechanical redesign, 12 SM105 leg motors, CAD-derived convex self-collisions and a dynamics-hashed 591-value sensor contract. Read [v05 digital acceptance and training scope](../docs/v05.md). V04 was withdrawn; v02 training below is historical and remains isolated.

> **V04 update (2026-10-18):** root model now uses heterogeneous stronger servos and 591 observations (197 × 3), including IMU, binary contacts/health and 8×8 ToF. V02 checkpoints/results and the original 24h run remain separate. Start v04 from scratch in a fresh output directory; final held-out seeds are 4000000..4000099. [Current build, rewards and limits](../docs/v04.md). The historical 165-input protocol below describes v02.

# SweetieMini v02: command locomotion and recovery

User accepts revision 02 and requests MuJoCo policies trained on a dedicated compute host, joystick commands, falls, self-righting and varied terrain. Preserve CAD dimensions. No learned controller existed before this work.

## Protocol fixed before implementation

PPO actor receives IMU-like projected gravity/angular velocity, measured joint state, previous actions, commands and short history. No world pose, root velocity, contact forces or virtual support wrench reaches the actor. Twelve leg position offsets; four expression joints use neutral bounded PD. Critic may use privileged simulator state. Train and evaluation use the same native MuJoCo servo stepping backend. No root support forces or pose corrections during episodes. Disturbances are explicitly logged impulses.

Two specialists (locomotion and recovery) and a hysteretic supervisor avoid starving locomotion with fallen resets. Recovery keeps full torso, head, ears and conservative horn contacts. Deployment is simulation only until actuator and inertial identification exists.

Training procedural terrain seeds 0..999999; development 1000000..1000099; final held-out 3000000..3000099. Test flat, slopes, uneven ground and discrete steps separately. Recovery: folded crouch (legacy name prone), supine, left and right, report each denominator, successful stable stands and time. Velocity tracking: forward/back/side/yaw commands, falls, actual speed and torque; running requires measured aerial phases, not a command label. Hardware envelope remains unmeasured.

Pre-launch protocol amendment: the initial player default used seed 2000000 for a pilot preview. That preview was inspected. Reserve fresh 3000000..3000099 for the final evaluation, never use the preview range for model selection, and default interactive previews to development seed 1000090. This amendment precedes the 24-hour curriculum and its model selections.

The speed curriculum requests up to 0.60 m/s; a separate 0.55 m/s development condition tests the fast end. These are targets to learn, not certified attainable speeds. No running claim unless forward motion and airborne phases are measured together.

## Research-grounded hypotheses

| ID | Angle | Mechanism | Expected delta (hypothesis) | Falsification | Source |
|---|---|---|---|---|---|
| E0 | F efficiency | Native multithread CPU MuJoCo + small GPU PPO avoids incompatible V100 Warp stacks and Python substep overhead | >3x transitions/s vs serial Python, equal dynamics | no speed gain or state mismatch | MuJoCo simulation/rollout docs |
| E1 | A optimization | Bounded position targets + stable per-joint servo damping remove explicit-PD chatter, allowing PPO to learn balance and command tracking | flat survival >90%, command RMSE <0.20m/s | survival <= neutral baseline or speed chatter | Playground Go1 joystick |
| E2 | D curriculum | Easy flat walking then rough/slope/step curriculum expands attainable terrain without destroying balance | rough success +15pp at matched steps | worse flat and rough success | Microduck Velocity/VelStand |
| E3 | B objective | Separate get-up policy with orientation/height potential progress and stable-stand bonus avoids rewarding sitting | recovery +25pp over neutral PD | neither side recovers more often | Microduck VelStand; Playground Go1 Getup |
| E4 | C data | Staged crouch, side and inverted resets retain attainable near-goal examples | recovery >=50% in each held-out orientation | progress only on upright resets | Microduck StandUp/VelStand |
| E5 | K scale | More rollout environments and longer verified training improve coverage of rare recovery transitions | +10pp held-out success with 2x steps | success unchanged/worse | PPO / Playground training |
| E6 | E robustness | Mass, friction, voltage, delay, encoder and gain randomization reduce shared-seed overfitting | stress success +10pp | nominal collapses or stress unchanged | Microduck sim2real configuration |
| E7 | B gait objective | A separate running specialist initialized from locomotion, trained on faster forward commands and bounded forward-flight reward, separates running from a mere higher joystick scale | speed >=0.35m/s and >=2 airborne intervals >=40ms per evaluation episode, survival >=80% | fast shuffle/jump-in-place or falls fails running gate | https://arxiv.org/abs/2103.06484 ; Playground Go1 joystick |

User requested an explicit running mode after the initial launch. Add walk/run toggle (keyboard B, gamepad RB/R1 while LB/L1 remains deadman), ramped velocity changes, an optional `run.pt` specialist and recovery priority. Retain the 165-observation/12-action contract and all servo limits. Running is a new task with home-relative position targets; initialize only policy+normalizer from walking and start a fresh optimizer. Keep the existing 24-hour deadline and all checkpoints. Migrate the remote process using a checkpointed handoff into a new source snapshot; never mutate the source files of an active training worker. Final held-out seeds remain unused. Running evaluation requires forward velocity plus repeated sustained flight without body-ground support; a UI toggle or faster command alone never qualifies as learned running.

Each run is bounded, checkpointed atomically with RNG/optimizer/normalizer and exact config. Select by development metrics; final test is independent. Retain failures. No claim of universal terrain. User authorized 24 GPU-hours; one GPU process with allocator <=2GiB (CUDA context additional), CPU workers <=16, never stop existing services. Timing includes reserved process wall time, compilation and evaluation. A resumed optimizer starts fresh episodes; native dynamic state is not serialized.

## Sources

- https://mujoco.readthedocs.io/en/3.3.0/python.html
- https://mujoco.readthedocs.io/en/3.6.0/programming/simulation.html
- https://github.com/google-deepmind/mujoco_playground
- https://github.com/pollen-robotics/microduck_rl
- https://github.com/pollen-robotics/microduck_rl/blob/main/src/mjlab_microduck/tasks/microduck_velstand_env_cfg.py
- https://arxiv.org/abs/1707.06347

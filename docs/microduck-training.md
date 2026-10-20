# MicroDuck-inspired v05 training revision

**Current 2026-10-20:** [revision 3](compact-policy.md) replaces the default actor input with 63 proprioceptive values and adds an 81-value privileged critic. It separates training continuation from champion promotion, adds fixed mixed commands, smooth resampling, KL-adaptive learning rate and explicit teacher distillation. The completed 36-hour cycle described below used the legacy 591-input policy; its budget is not extended. New compact final seeds are 5000000..5000099. Earlier transfer/parity claims below apply only within their original architecture.

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

## Expanded rewards, revision 2 (2026-10-19)

The user subsequently requested all applicable MicroDuck reward components and explicitly emphasized actual speed. Recipe `v05-microduck-rewards-2` adds:

| Component | Adaptation for this quadruped |
|---|---|
| Linear/yaw tracking | Broader Gaussian tracking gradients plus bounded commanded-direction progress; explicit under-speed, over-speed and under-turn costs. Exact small sideways commands are normalized by their own magnitude. Speed in the wrong direction earns no progress. |
| Foot air time | Per-foot touchdown events after 0.10–0.35 s swing for walking (0.08–0.30 s for running); only after prior support, useful clearance, actual commanded translation/rotation, upright posture and clean contacts. |
| Swing/clearance | Approximately 25 mm target foot lift, horizontal swing-motion gate, landing peak-height score, drag/excess-lift costs. Stationary leg waving and in-place jumping cannot earn positive foot shaping. |
| Standing posture | Stronger home-pose preference at zero command, relaxed posture during movement, plus weak position-limit proximity costs. |
| Dense get-up target | Broad/sharp body-height scores, normalized absolute height error, linear/sharp upright terms, leg pose and multiplicative height × upright × pose score. Goal bonuses require at least three feet, no body-floor bracing and no self-contact. |
| Rising | Direct positive height-rate signal, paid only for newly achieved heights after initial floor contact. An initial drop is excluded and repeated bobbing below an old peak earns no additional rise reward. |
| Smoothness | Late normalized torque-change and vertical-impact costs; rotation damping remains limited to upright arrival. Motor saturation and overload costs stay active. |

All reward measurements remain training-only. The actor still receives the same 591 values, produces the same 12 leg actions, and uses identical native motor physics, masses and collision geometry. No neck/head-pose commands, ball/trick objectives, roller objectives or XL330-specific parameters are added: these do not match this policy's action/command interface. Running still requires independently measured sustained forward flight; per-foot air-time reward does not qualify a run by itself.

Recovery curriculum counters remain persistent, while difficult starts are unlocked by development success: crouch, both sides, then supine (25% threshold per condition). The original transferred policies are evaluated on all four development surfaces **before** further training and enter the normal champion/rollback mechanism. A candidate losing a qualified baseline's quality gate cannot replace it merely by increasing aggregate score.

Revision 2 uses another immutable source/output phase within the **same original 36-hour campaign** and the same final held-out protocol. Initializers preserve actor/normalizer weights and record the exact parent/source hashes; optimizers start fresh for the changed reward objective. Actual learned gains remain to be measured.

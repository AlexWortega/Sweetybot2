# Recovery through intermediate poses

Recipe `v05-recovery-waypoints-1` extends the direct-stand discovery signal
with an episode-local geometric milestone ledger. It is an experiment, not a
demonstration of learned self-righting.

| Milestone | Measured condition | Required dwell | One-time bonus |
| --- | --- | --- | --- |
| Leave the back / reach side | torso up component >= -0.20 | 0.12 s | 0.50 |
| Belly with a foot support | up >= 0.55, >= 1 hoof contacting terrain | 0.16 s | 0.75 |
| Supported crouch | up >= 0.85, height >= 50% stance, >= 2 hooves, no body contact | 0.20 s | 1.00 |
| Stable stand | existing stable-stand gate, >= 3 hooves, no body contact | existing 0.70 s | 1.50 |

All milestones require real terrain contact, no self-contact and angular speed
below 4 rad/s. The stages describe supported body poses rather than an animated
joint sequence. The existing stable-stand and evaluation success conditions are
retained. A torso initially upright receives no side/belly/crouch milestone
credit. Starting on the back and ending on the back remains a failed recovery.

Dense progress pays only increases of the current stage's episode high-water
score. Each milestone is paid at most once per episode. Holding a partial pose,
oscillating across a threshold, or revisiting an earned stage cannot repeatedly
collect its bonus. The ledger resets independently for each completed world.
The high-water budget bounds the exploration incentive; ultimate success still
requires the robot to stand and remain standing at episode end.

Full-height and upright penalties are reduced while inverted to allow lowering
the body and folding the legs to roll. Torque, collision and joint-limit costs
remain. Initial-state sampling retains full side/back cases at every curriculum
stage. Passing the easy crouch gate permits progress to 0.74, avoiding the old
0.49 cap which prevented most full-side experience. This does not relax the
evaluation requirements or enable harder terrain before full recovery passes.

Waypoint achievements are recorded in PPO episode metrics and evaluation's
per-orientation `waypoint_reach_rates`. Evaluation records initial up component
and height so standing from an already-upright crouch cannot be confused with
getting up from the back. These privileged measurements shape rewards only;
the actor still receives the same 63 sampled IMU/encoder/contact values.

The recipe is installed as a new immutable source/output. Actor weights and
normalizer transfer only after checking the donor's original source hashes and
unchanged physical/sensor signature; critics and PPO optimizers start fresh.
Old checkpoints, source and quality results are retained. The revised driver
uses the original 200-hour start/deadline, not an additional budget. Startup
baseline uses one deterministic replicate of each condition on all four
surfaces, for latency; subsequent development/final protocols remain eight.

Validation covers milestone order/dwell, threshold chatter, repeated cycles,
airborne/self-contact/spinning counterexamples, reset isolation, exact actor
transfer and native supine rollouts. Neutral-physics acceptance and a short PPO
run verify integration, not successful recovery. New success must be established
by the unchanged side/back evaluation and uninterrupted recorded rollouts.

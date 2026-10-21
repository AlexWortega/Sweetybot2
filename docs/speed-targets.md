# Higher command targets

The optional `higher-v1` command profile raises walking forward targets to
0.40 m/s and starts running targets at 0.65 m/s (previous ceilings 0.30/0.55).
Half of non-idle forward examples retain the original slower range. Idle,
reverse, sideways and turn coverage, command smoothing, actuator physics,
recovery milestones and the 63-value actor contract remain in place.

Running ceilings are **0.65 → 0.80 → 1.00 m/s**. Elapsed training alone cannot
unlock them. Each increase requires at least 90% survival in every command
bucket of the unchanged four-surface development suite, clean self-contact and
torque gates, plus a separate 8-second flat probe near the current ceiling.
Each probe condition must have at least 90% survival, linear RMSE <=0.15 m/s
and mean forward speed >=80% of its command. Probe failures do not rewind valid
PPO weights/optimizer state. Stage and probe evidence persist in `status.json`.
These values are targets, not achieved or physically validated speeds.

PPO checkpoint config, metrics and source hashes record the command profile
and stage. The profile is a configurable sampler outside robot dynamics; the
physical/reward/observation signature is unchanged. The same live campaign
continues with its existing checkpoint, optimizer and absolute deadline.
Replay defaults for new-profile checkpoints use their recorded walk/run
ceiling; explicit CLI limits remain available.

## Evaluation boundary correction

The previous evaluator incorrectly marked crossing |x| or |y|=3.5 m as a
failure although the environment already used a 10 m terrain extent. Protocol
`terrain-boundary-v3` uses extent minus 0.5 m (9.5 m here), matching the physical
environment boundary. The normal 12-second command suite is unchanged. New
high-speed probes use 8 seconds so even the 1.0 m/s target fits in the terrain.
This correction can improve reported survival without any policy improvement;
do not compare old and corrected survival as a controlled learning gain. Old
results remain saved with their source provenance. Actual tilt/height failures,
torque, self-contact and repeated-flight running criteria are retained.

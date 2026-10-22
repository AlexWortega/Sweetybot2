# Stair and turning curriculum

The `navigation-v1` command profile adds clockwise/counterclockwise pivots and
forward arcs while retaining idle, backward, lateral and forward commands.
Yaw-rate targets reach 1.2 rad/s for pivots, 1.0 rad/s for arcs. These are
commands, not measured robot performance. The actor remains 63 inputs and
12 leg actions; the critic remains 81 inputs. No heading goal, simulator
position, terrain map, camera or ToF image is added to the actor.

`stairs` is separate from the original low periodic `steps`. Each flight has
five 20 cm treads, 1–4 cm risers and an 80 cm landing, followed by a descending
flight and another landing. Full difficulty reaches a 20 cm total rise.
This is scaled robot terrain, not a claim about ordinary human staircases.
The central spawn landing is flat. Training also starts across the course
facing both directions. Ground-height queries use the exact same segments as
the MuJoCo boxes; vertical rays validate their agreement. Only initial poses
are cleared of penetrations; no mid-episode lifting or base support is used. Leaving the terrain arena
terminates locomotion/running episodes as failures, preventing escape onto
the surrounding flat floor from bypassing stair practice.

Walking training visits stairs, flat, rough, slopes and the old small steps.
Fast-motion and recovery specialists retain their existing terrain schedules.
Stair rewards set swing clearance to 2.5 cm plus the riser height; standing
and static foot lifting cannot earn the motion-gated swing reward. Navigation
adds penalties for excessive yaw rate and translation during a pivot. Existing
motor, thermal, contact and overload costs remain active.

The separate `learning.evaluate_navigation` suite measures left/right 180 and
360 degree pivots, left/right 90 degree arcs, stair ascent and descent. It
unwraps actual heading, requires settling near the requested angle, limits
pivot drift, and keeps falls permanent. Arcs require actual path length;
stairs require forward progress and a ground-height change spanning most of
a flight, ending on at least two hooves at a valid body height below 0.35 m/s.
Airborne flybys cannot count as success. Torque, self-contact and body-contact gates apply. External test
logic converts the heading goal into joystick velocity commands, then commands
stopping; it never rotates the robot or injects forces. Goals are not actor
inputs. Initial poses are fixed with small lateral offsets, so these scores
are not estimates for arbitrary starts or all stairs.

The existing four-surface command suite remains an anchor. Navigation has its
own baseline and per-case results after every walking block. Final navigation
uses a separate reserved seed range only once. Passing old walking metrics
does not establish stair or heading-goal success.

The explicit continuation preserves the original 200-hour start/deadline and
single-GPU limit. Actor/normalizer weights are transferred from verified
snapshots with fresh critics/optimizers. The moving fast-mode actor initializes
the new walking policy: both use the same home-relative action mapping. A
separate transferred standing actor remains the near-upright recovery teacher.
Old policies and scores remain archived; all new policies are evaluated again.

Example independent evaluation (the checkpoint must match this source):

```sh
python -m learning.evaluate_navigation --checkpoint learning/runs/walk/latest.pt \
  --out learning/results/navigation.json --episodes 8 --seconds 12 --difficulty 1
```

Joystick playback supports `--terrain stairs`; use the normal walking policy
for stair traversal and pivots. Adding environments and passing software
checks does not establish learned climbing, turning or hardware readiness.

Launch verified on physical GPU 3: the first walking-on-stairs block saved
7 PPO updates / 57,344 transitions at difficulty 0.12 (1.36 cm risers).
[Timestamped receipt](../reports/navigation-launch-verified.json). This is
training liveness, not demonstrated stair or pivot competence.

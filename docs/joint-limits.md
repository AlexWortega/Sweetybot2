# Hip-roll range for recovery (2026-10-21)

The previous symmetric roll range was +/-0.15 rad (8.59 degrees). It frequently
clamped recovery actions in the inspected 12-second back/side rollouts. This
is evidence of restricted movement, not proof that the restriction alone
caused recovery failure.

The new canonical left-leg range is -6 to +45 degrees. Right-leg limits are
mirrored (-45 to +6), retaining the same axes, encoders and motor signs.
Positive roll points outward on the left and inward on the right. The shared
`joint_spec` drives MJCF, URDF, native controller target clamps and command
validation. Recovery actions can accumulate to the full outward limit.

Large inward motion is deliberately excluded: coordinated inward movement
at the old 8.59-degree limit already produced shin proxy intersections in a
standing pose. The 6-degree inward limit leaves clearance in the added cohort.

CAD collision sources were regenerated from the current configuration.
Mechanical acceptance includes all previous home, single-joint, gait and
crouch poses, plus 45 coordinated outward/inward configurations, including
standing, folded and extended legs. Acceptance covers sampled configurations,
not every combination of all joints or continuous trajectories. Random poses
remain diagnostics; collisions during exploration remain enabled and penalized.
The widened range grants no fabrication approval.

Masses, hulls, servo models, torque/speed/thermal limits, sensor transport,
actor observations (63), critic observations (81), rewards and speed targets
are unchanged. The existing analytical servo screen covers its stated home
and transition loads; it does not certify weight-bearing at 45-degree roll.
An unloaded recovery sweep and a loaded wide stance require different torque.
The native actuator envelope continues to saturate excessive demands.
`reports/hip_roll_load_screen.json` records the vertical-load example: at
45 degrees the 50%-design-weight case needs about 14 Nm including the 1.5
margin, compared with the conditional 3.8 Nm roll envelope. That pose cannot
be treated as an allowed sustained load merely because it clears the CAD.

Changing joint limits changes the dynamics signature. Normal checkpoint
resume remains strict. `learning.recipe_bootstrap --joint-limit-transfer`
permits an explicit actor/normalizer-only handoff from a verified immutable
source after current mechanical and RL acceptance. It checks that the only
physical differences are configured roll limits, their mirrored interpretation,
and matching MJCF joint ranges; all other physical source bytes and hulls
must match. The critic and optimizer start fresh. Old champion scores are
archived and must be measured again under the new limits.

The continuation retains the original 200-hour campaign start and absolute
deadline, using one training GPU. Expanded movement is not a claim of learned
back recovery; that requires separate held-out back and side evaluations.

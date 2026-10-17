# Validation ledger and physical release gates

The authoritative measured results are generated into `reports/validation.json` and `docs/results.md`. Failed pilot runs remain under `reports/pilots/`; they are not counted as successful final tests.

## Digital checks

- Saved STEP validity, positive solid volume, individual print envelopes, explicit counts and solid-PETG mass budget.
- Catalog source checksums and revision-pinned upstream snapshots.
- FK agreement with MuJoCo across independent sampled poses; IK round trips, joint ID uniqueness and limit conventions.
- URDF strict structural/inertia/mesh validation, plus saved visual review and a sampled full joint sweep. External `check_urdf` is reported separately if unavailable.
- Floating-base standing for 60s, >1m crawl with bounded lateral/yaw/path error, 90-degree turn, and crouch/rise. Check completion, falls, unexpected contacts, target limits, maximum speed, peak/RMS torque and tracking error.
- Parameter variations for mass, friction, delay and backlash. A failed variation defines an operating limitation; it must remain visible in the report.

## Physical sequence (not run)

1. **Fit coupons and shaft interface:** inspect horn spline/bolt pattern, 623 bearing fit, axle coaxiality, clamp clearance, layer orientation and fastener tool access. Measure assembled mass and COM; update both descriptions.
2. **Loaded single servo:** use a guarded lever and known weights/load cell at full and low battery voltage. Test the calculated stance load for at least 20min and transition load in representative duty cycles. Record angle error, supply current, case temperature and thermal shutdown. Require no position loss, stable temperature below the manufacturer's limit with >=10°C margin, and measured capacity >=1.5 times the application's sustained load. If no continuous rating is supplied, this test establishes the provisional envelope; stall torque does not.
3. **One complete leg:** test the printed load path, bearings, passive shaft adapter and horn for slip/creep. Measure backlash, friction and position-step response. Identify the real position-loop stiffness before attempting virtual-torque position offsets.
4. **Power harness:** exercise normal load and a controlled fault with a current-limited supply. Check voltage sag, buck temperature, connector temperature, fuse coordination, regenerative voltage rise and both disconnects. The fuse is not active current control.
5. **Supported full robot:** commission unique IDs/zeros/directions, validate all physical ranges at low speed, test command/sensor loss, verify no movement on restart and no automatic recovery from faults.
6. **Floor tests:** padded flat indoor area and support/tether; stand, short crawl, turn, crouch/rise, then a 20min mixed-duty run. Inspect temperature per joint and printed parts, not just average temperature. Reduce the working envelope or change the actuator if measured limits fail.

No physical endurance, component fit, strength, thermal rating, real bus latency or real-world gait result is claimed by this package.

## Mechanical range findings

See [the saved-geometry review](mechanical_review.md). Revision 02 reruns the 113-pose motor-envelope screen after reducing the track, increasing the hip drop and centring the pitch motor. The final machine-readable report is authoritative. The body cover is also checked against home motor/electronics envelopes. These checks exclude many moving shell, clamp, horn and cable interactions; establish physical limits on a supported leg and neck before using them on hardware. `check_urdf` from ROS is unavailable here; cadgen strict URDF validation and MuJoCo loading are used.

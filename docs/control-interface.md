# Future hardware control contract

The package implements validation and a supervisor model in `simulation/control_contract.py`; it does not implement a physical motor driver or claim real-time hardware performance.

## Data and ownership

Only the control process owns the serial port, servo configuration and motor writes. Expression requests and motion requests enter that process; the eye renderer is an independent, nonblocking consumer. Motor control must not wait for the display, network, speech, file I/O or a sensor-service restart.

A complete command has `{sequence, timestamp_s, positions_rad}`. Positions name exactly all 16 configured joints, use radians in the documented joint frames, and satisfy limits. Timestamps use a shared monotonic clock. Reject nonfinite values, missing/unknown joints, replayed sequence numbers, future timestamps and commands older than 250ms. Ramp and slew-limit accepted position goals to 0.8rad/s for the first prototype.

A sensor sample contains monotonic timestamp, sequence, positions, velocities, raw load/current fields with their actual vendor units, per-servo voltage/temperature when available, and the torso IMU quaternion/angular velocity. Do not equate a vendor load estimate with measured torque or motor current. A missing sensor sample is stale, not zero.

## States

- `disabled`: startup/default; no motion authority. Read-only inspection is allowed.
- `enabled`: explicit enable after fresh sensors and validated offsets; a fresh full command is required before writing.
- `fault`: stale sensors, command timeout, nonfinite state, voltage/thermal fault or bus failure; stop active locomotion and require an explicit clear plus re-enable.

Fault handling on the bench must distinguish a controlled pose ramp from immediate motor-power loss. Do not command an unsupported robot to relax as a generic software timeout response. A hardware motor disconnect is always available and can cause the robot to fall; the first tests use a support stand. Hardware watchdog/servo-loss behavior is not yet implemented or verified by the Python supervisor.

## Buses and wiring

The selected STS3250 is TTL half-duplex at 1Mbps. It does not use Microduck's Dynamixel protocol-2 fast-sync-read instruction. Use the ST vendor SDK or a tested implementation of its actual packet/register protocol. Measure the 16-device read/write cycle before accepting 50Hz; report p50/p95/p99, misses and IMU age. Maintain one port lock and one writer. Keep the IMU on the Pi's 3.3V I²C domain and eye displays on 3.3V SPI with separate chip selects.

Motor power: battery -> main fuse/master -> motor-only disconnect -> four leg branches plus head branch. Logic power has a separate input fuse and buck converter so diagnostics survive motor disconnect. Confirm the adapter's schematic before separating its power pins: no 12V connection to USB 5V or GPIO. The wiring diagram is a system diagram, not a verified pin-level harness drawing.

## Policy/estimation boundary

The separate `learning/` implementation now trains command and recovery policies using joint state, IMU-like signals and commands, with 12 leg actions and four neutral expression joints. Its bounded position-target simulation receives no virtual support wrench. The older `simulation/run.py` baseline still uses privileged ground-truth virtual-wrench control. Microduck's biped policies remain incompatible. Neither simulation validates the real STS3250 transfer function, bus timing or physical state estimator; see [training boundaries](rl-training.md).

## Eye frames

`simulation/eyes.py` produces independent 240 × 240 RGB frames with gaze motion and blinking for the two ST7789 displays. `uv run python -m scripts.render_eyes` writes a paired preview and sample frames. This renderer performs no SPI writes and establishes no display refresh-rate guarantee. A future hardware consumer must buffer frames and drop stale frames without delaying motor control.

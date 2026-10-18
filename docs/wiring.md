# V04 wiring

3S LiFePO4 (9.3V loaded cutoff, 10.8V full) → DC master disconnect → 40A main fuse → separately fused motor and logic distribution. Motor emergency disconnect leaves logic powered. 4 × 10A leg branches feed motors in parallel; 5A head branch feeds neck and regulated 6V ear power. 5A logic branch feeds 5V regulator, Pi and collector. Fuse coordination, pack/BMS pulse current, connector ampacity, regen clamp and loaded voltage sag are not validated.

Two independent signal interfaces: USB RS485 → 12 leg SM-series motors; USB TTL → STS neck/ear motors. Their signal conductors must never be joined. Common reference ground and power branches are explicit; 6V and battery positive rails must never be joined. Configure IDs once, with one motor connected, then verify each actual model's register map, baud, polarity, timeout and torque-disable behavior before joining its bus. No assumption of protocol interchangeability is a driver implementation.

Pi Zero OTG requires a powered/appropriate USB hub for the two adapters. Hub and harness routing remain provisional. Main current does not flow through USB, Pi or small signal headers. Use hardware motor disconnect/deadman independent of Python.

GPIO ownership is in config/robot.yaml and checked by scripts/check_sensor_contract.py. XIAO reads four resistor-coded 12-bit ADC loops and the LSM6DSV16X; UART is 3.3V logic to Pi. Pi owns ToF I2C and round LCD SPI. Camera uses the 22-pin Pi Zero to 15-pin camera FPC. Dedicated >=0.5A 3.3V regulator supplies ToF; do not parallel regulator output with Pi 3V3. Ear regulator is a separate 6V buck; verify current at 9.3V input.

Each contact: 3V3 — 10k pull-up — ADC wire — [10k EOL || (NO switch + 1k)] — ground. Put EOL beyond the foot connector. Ratios: air ≈0.5, contact ≈0.0833, open ≈1, short ≈0. Wiring faults and stale samples invalidate contact, never become confirmed air. Collector packet format and host decoder are in hardware/telemetry.py; firmware and electrical drivers have not been flashed or bench-tested.

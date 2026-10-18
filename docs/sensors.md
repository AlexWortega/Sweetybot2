> V05: current placements, sampled actor and electrical limits are in [the v05 contract](v05.md). Head/ear and support-frame geometry changed; the descriptions below are historical sensor selection notes. Physical firmware/driver commissioning is pending.

# Sensor hardware arrangement

The proposed first build uses **one rigidly mounted torso IMU**, one forehead RGB camera, one chest depth imager and **four direct hoof contacts**. Leg IMUs are deferred: they add wiring and mass, while encoders plus torso orientation and direct contacts address the immediate state-estimation needs. Additional lower-leg IMUs could later help study impact or compliance, with measured benefit required before integration.

| Function | Selected module | Native CAD accommodation | Interface / target |
|---|---|---|---|
| Torso attitude / angular velocity | SparkFun SEN-21325 LSM6DSV16X | rigid tray above Pi; 20.32mm square M2 hole pitch | XIAO I2C, 7-bit 0x6B, SFLP quaternion and gyro target 120Hz |
| RGB | Raspberry Pi Camera Module 3 Wide IMX708 | forehead aperture and serviceable edge-clip carrier | Pi CSI, 102° horizontal / 67° vertical FoV; test acquisition rate on Pi Zero 2 W |
| Depth | SparkFun SEN-18642 VL53L5CX | chest pod; tilted 20° down, 20.32mm mounting pitch | separate Pi I2C, **7-bit 0x29**, 8×8 at 15Hz target |
| Ground contact ×4 | Omron B3SE-1002P on a proposed 12×12mm cassette PCB | guided floating rigid sole, TPU tread/tip, spring seats, hard stops | XIAO 4 ADC channels, target scan 200Hz |
| Sensor collector | Seeed XIAO RP2040 | strapped location on sensor tray | 3.3V UART at proposed 460800 baud |

The depth module's 8×8 cells are not a high-resolution depth camera. Its 60Hz mode is 4×4. SparkFun's 0x52 notation is the 8-bit write address; Linux libraries expecting 7-bit addressing need **0x29**. Neither this sensor nor RGB alone establishes reliable operation on arbitrary terrain.

## Placement and service

Coordinates in config are millimetres, +X forward, +Y left, +Z up. Camera and displays belong to the head frame; depth and IMU belong to the torso. The chest direction remains independent of head yaw. Use the Pi Zero **22-pin 0.5mm → camera 15-pin 1mm** FPC, with a 300mm length allowance, strain relief and service loops across yaw/pitch. Verify cable bend radius and the complete motion range with an actual cable. Lens aperture FoV and reflections still require an optical bench check.

The IMU tray is rigidly tied to the chassis through the deck/cowl fastener locations. No loose foam suspension and no mounting the main estimator IMU in the animated head. Calibrate axis alignment, gyro bias, accelerometer scale and time synchronization. SFLP game rotation does not provide absolute magnetic heading; estimate yaw appropriately.

The LCDs are Waveshare 1.28inch round GC9A01 modules, 240×240, board 40.4×37.5mm, active diameter 32.4mm. Angled side mounting tucks the front board edges into the cheeks; the round bezel is cosmetic, and positive PCB retention awaits the exact module revision. CAD color layers are screen graphics, not additional printed eyeballs. Module thickness and retention are provisional until the exact board revision is measured.

## Pin assignment

| Raspberry Pi BCM | Connection |
|---|---|
| 14 TX / 15 RX | XIAO GPIO1 RX / GPIO0 TX, common ground, **3.3V logic** |
| 2 SDA / 3 SCL | chest ToF I2C |
| 17 / 27 | ToF LPn / interrupt |
| 10 MOSI / 11 SCLK | both LCD SPI buses shared |
| 8 / 7 | left / right LCD chip select |
| 22 / 23 | shared LCD DC / reset |

| XIAO raw RP2040 GPIO | Board marking / connection |
|---|---|
| 26, 27, 28, 29 | D0/A0…D3/A3: FL, FR, RL, RR contact ADC |
| 6 / 7 | D4 SDA / D5 SCL: IMU |
| 0 / 1 | D6 TX / D7 RX: Pi UART |
| 2 | D8: IMU interrupt |

Check actual Pi `/dev/serial0` mapping and disable the serial login console. Preserve the joystick Bluetooth path: select a stable UART configuration for this Pi image and verify Bluetooth + UART together; do not blindly disable Bluetooth. No live GPIO configuration has been applied.

Logic 5V powers Pi and XIAO via their specified inputs. The IMU uses the XIAO 3V3 rail. Provide the ToF with a **separate 3.3V >=0.5A regulator** from logic 5V; its 13×10×5mm mounting envelope is a selection allowance, not a selected vendor model. Do not parallel that output with the Pi 3V3 output. Keep common grounds and verify I2C pull-ups, sequencing and brownout behavior. Existing 5V buck capacity and battery/fuse coordination require a measured combined load test.

Ear motors have a separate **6V** power branch: Pololu D30V30F6 is the selected candidate, with a conservative 22×18.5×9.5mm envelope and wiring mass allowance. Its 3.3A specification is typical at 30V input, not a guaranteed output at our 3S LiFePO4 input. Validate startup/current pulses, regulator thermals and output overshoot at full battery and cutoff. Keep TTL data/common ground with the main bus, but do not daisy-chain the servo **power** wires across the 6V/12V branches. Verify bus protocol/register compatibility and unique IDs on a current-limited bench.

## Contact circuit and mechanism

Per foot, at the collector: 3V3 → 10k pull-up → ADC/contact signal. At the hoof: 10k end-of-line resistor from signal to ground, in parallel with a normally-open B3SE switch in series with 1k. The EOL resistor must be physically at the foot, beyond the connector.

| Condition | Nominal ADC / Vref | Decoded |
|---|---:|---|
| Air, switch open | 0.500 | air |
| Ground contact, switch closed | 0.0833 | contact |
| Broken loop | 1.000 | invalid/open |
| Signal shorted to ground | 0.000 | invalid/short |

Reference thresholds and 10ms debounce are in `hardware/contact_adc.py`. Samples older than 20ms become unknown; faults invalidate immediately. Some shorts/component faults can imitate a valid resistor value, so this is basic continuity detection, not a safety-rated circuit. Configure ADC to the documented normalized 12-bit range and calibrate real cable resistance/noise. A 200Hz target is not a measured MCU rate.

The switch reports **contact only**, not force. Four guide screws and springs return each floating sole. Nominal neutral floor-to-sole gap / hard-stop stroke is 0.8mm; spring pockets give approximately 3.8mm installed spring length. The mechanical floor takes the hoof load when the gap closes. The compliant TPU actuator starts near the switch plunger; adjust its height with an M2 screw/shims. Select springs after measuring the switch and printed tolerances. The B3SE operating force is nominally 1.96N and its pretravel 0.3mm with large tolerances: calibration is necessary before full-weight use. The PCB, guides and wiring need sealing; the switch rating does not waterproof the hoof.

## Software boundary

No hardware is connected. This package adds CAD, wiring choices, electrical/reference decoding checks and mass allowances. Real sensor acquisition, MCU firmware, extrinsic calibration, synchronized timestamps, fusion, watchdog handling and policy observations remain integration work. The existing v02 learner reads simulated proprioception; it does not use these camera, depth or contact streams. Keep it unchanged until a new policy is explicitly evaluated on the revised plant.

## Primary sources

- [SparkFun LSM6DSV16X guide](https://docs.sparkfun.com/SparkFun_6DoF_LSM6DSV16X/single_page/)
- [Seeed XIAO RP2040 pinout](https://wiki.seeedstudio.com/XIAO-RP2040/)
- [Raspberry Pi Camera Module 3 product brief](https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf)
- [SparkFun VL53L5CX module](https://www.sparkfun.com/sparkfun-qwiic-tof-imager-vl53l5cx.html), [ST datasheet](https://www.st.com/resource/en/datasheet/vl53l5cx.pdf)
- [Omron B3SE datasheet](https://omronfs.omron.com/en_US/ecb/products/pdf/en-b3se.pdf)
- [Waveshare round LCD](https://www.waveshare.com/wiki/1.28inch_LCD_Module)
- [Pololu D30V30F6](https://www.pololu.com/product/4893/specs)

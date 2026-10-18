# V04 bill of materials

Hardware candidate; no purchasing or physical assembly performed. Servo allowances: SM105 $120 each, SM80 $85 each, STS3250 $44.99 each; these are unverified planning amounts.

| Item | Qty | Qualification |
|---|---:|---|
|SM80BL-C001|4|RS485|
|SM105BL-C001|8|RS485|
|STS3250-12V|2|TTL SMS/STS; confirm firmware protocol|
|STS3032-C001-6V|2|TTL SMS/STS; confirm firmware protocol|
|Pololu D30V30F6 6V regulator|1|validate current and temperature at 3S LiFePO4 input|
|Dedicated 3V3 >=0.5A regulator|1|depth module supply; 13x10x5mm envelope provisional|
|Raspberry Pi Camera Module 3 Wide visible-light|1|25x24 PCB and 12.4mm optical keepout|
|Pi Zero 22-pin to camera 15-pin CSI FPC|1|300mm allowance; check motion loop and bending|
|SparkFun SEN-18642 VL53L5CX|1|chest 8x8 depth 15Hz target|
|SparkFun SEN-21325 LSM6DSV16X|1|rigid torso IMU|
|Seeed XIAO RP2040|1|ADC contacts and IMU collector; firmware pending|
|Waveshare 1.28inch GC9A01 LCD Module|2|240x240 round active area; exact module depth pending|
|Omron B3SE-1002P|4|contact switch only; no force measurement|
|Contact cassette PCB 12x12mm|4|layout not fabricated; include 10k EOL and series 1k|
|10k 1% pull-up resistor at collector|4|one per ADC contact signal|
|10k 1% EOL resistor at hoof|4|must sit beyond foot connector|
|1k 1% series contact resistor at hoof|4|switch branch parallel to EOL|
|M2 guide/shoulder hardware sets|16|verify stroke and captive retention in single hoof prototype|
|Return spring OD3.5mm ID>=2.2mm free4mm|16|low preload target <=0.1N per spring at installed length|
|Adjustable M2 actuator screw/shims|4|compliant TPU tip and hard stop calibrated on bench|
|Strain relief and foot connectors|4 sets|seal exposed PCB and wire exits|
|Sensor tray|1|rigid IMU mounting|
|Chest depth pod|1|open optics aperture; protective window not specified|
|Camera carrier|1|edge clip fit coupon required|
|LCD bezel|2|cosmetic rim; positive PCB retention requires exact module revision|
|Contact hoof carrier|4|replace v02 hoof|
|Floating sole plate|4|hard stop carries load|
|Hoof tread|4|verify friction and fatigue|
|Actuator tip|4|verify switch actuation and overtravel|
|Raspberry Pi Zero 2 W + microSD|1|logic compute|
|USB RS485 adapter, 3.3V-safe USB logic, SMS protocol support|1|12 leg motors only; differential A/B and common reference|
|USB TTL servo adapter|1|2 neck and 2 ear motors; TTL signal bus separate from RS485|
|USB OTG hub/cable + two adapter cables|1|Pi Zero has one USB data port; envelope and cable routing provisional|
|3S LiFePO4 >=2.5Ah pack with >=40A continuous BMS|1|9.9V nominal, 10.8V max, loaded cutoff 9.3V; real pack sag unmeasured|
|40A main fuse, four 10A leg, 5A head and 5A logic fuses|1|DC disconnect and fuse coordination require loaded tests|
|12AWG main, 16AWG leg branches, rated connectors|1|short feeds; fused parallel power; do not daisy-chain motor load current|
|5V >=3A logic buck + motor-only disconnect|1|logic survives motor emergency stop|
|6061 CNC load parts|1|deck, rails, clamps, yokes, cheek plates, forks, adapters and spacers; not FDM load parts|
|Fasteners, passive output supports, horns and bearing interfaces|1|drawing-derived placeholders; physical fit coupon and loaded joint required|

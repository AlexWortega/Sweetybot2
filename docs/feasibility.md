# V04 actuator screen

Rated catalogue points are screened at 9.3V and 0.8rad/s. They are not thermal certification. A 20% mass contingency, 50% load on one leg and 1.5 torque margin are included.

| Joint | Servo | Required/rated envelope Nm | Transition/burst Nm | Pass |
|---|---|---:|---:|---|
|fl_hip_roll|SM80BL-C001|1.072/1.278|1.072/2.812|True|
|fl_hip_pitch|SM105BL-C001|2.172/3.176|2.965/5.240|True|
|fl_knee|SM105BL-C001|2.172/3.176|2.965/5.240|True|
|fr_hip_roll|SM80BL-C001|1.072/1.278|1.072/2.812|True|
|fr_hip_pitch|SM105BL-C001|2.172/3.176|2.965/5.240|True|
|fr_knee|SM105BL-C001|2.172/3.176|2.965/5.240|True|
|rl_hip_roll|SM80BL-C001|1.072/1.278|1.072/2.812|True|
|rl_hip_pitch|SM105BL-C001|2.172/3.176|2.965/5.240|True|
|rl_knee|SM105BL-C001|2.172/3.176|2.965/5.240|True|
|rr_hip_roll|SM80BL-C001|1.072/1.278|1.072/2.812|True|
|rr_hip_pitch|SM105BL-C001|2.172/3.176|2.965/5.240|True|
|rr_knee|SM105BL-C001|2.172/3.176|2.965/5.240|True|
|head_yaw|STS3250-12V|0.100/1.056|0.150/1.815|True|
|head_pitch|STS3250-12V|0.246/1.056|0.246/1.815|True|
|ear_left|STS3032-C001-6V|0.008/0.103|0.008/0.226|True|
|ear_right|STS3032-C001-6V|0.008/0.103|0.008/0.226|True|

All simultaneously stalled leg motors alone draw a catalogue 74A, above the 40A main fuse. Stall is a fault, not a valid motion. Fuse, BMS, connectors, regenerative overvoltage and joint temperature require bench validation. No load rating is inferred from stall torque alone.

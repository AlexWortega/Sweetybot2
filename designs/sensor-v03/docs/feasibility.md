# Actuator and mass screening

conditional analytical screening; thermal validation outstanding

| Candidate | Nominal mass kg | Design mass kg | Sustained required/allowance Nm | Transition required/allowance Nm | 14 main + 2 ear allowance USD | Screen |
|---|---:|---:|---:|---:|---:|---|
| XL330-M288 | 1.921 | 2.306 | 0.561 / 0.120 | 1.163 / 0.265 | 428.86 | reject |
| XC330-M288 | 1.991 | 2.390 | 0.581 / 0.211 | 1.203 / 0.463 | 1491.46 | reject |
| ST3215-12V | 2.439 | 2.927 | 0.708 / 0.410 | 1.460 / 0.902 | 323.86 | reject |
| STS3250-12V | 2.712 | 3.255 | 0.786 / 0.978 | 1.617 / 1.680 | 673.86 | conditional pass |

Selected: **STS3250-12V**, the cheapest candidate passing this conditional screen.

Load calculation: 20% mass contingency; one leg takes 50% of weight; 1.5 torque margin. Knee lever comes from exact two-link IK at nominal height. Hip roll includes the lateral joint offset. Crouch transition uses a conservative 65 mm moment arm. Head uses a 65 mm lever.

The torque-speed allowance is stall torque × assumed duty fraction × (1 − speed/no-load speed). STS3250 sustained screening uses its manufacturer rated torque point (16 kg.cm at 12V), with low-battery and speed derating. Other candidates use the configured stall fraction. The rated point is not a verified continuous thermal rating for this robot. A loaded bench test must validate or replace both before fabrication release.

Candidate screening uses the same 210 mm chassis and 75/80 mm links to avoid comparing different tasks. Smaller servos could support a separate smaller robot; that alternative is not established by this screen.

Power: 24.75 Wh nominal; 44.1 W assumed average; 25.2 minutes estimated at 75% usable energy. This is a budget, not measured endurance.

All-servo stall is 58.8 A, above the main fuse. Simultaneous stall is a fault condition, not an operating point. Fuse coordination and disconnect behavior require bench testing.

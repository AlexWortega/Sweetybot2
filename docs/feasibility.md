# Actuator and mass screening

conditional analytical screening; thermal validation outstanding

| Candidate | Nominal mass kg | Design mass kg | Sustained required/allowance Nm | Transition required/allowance Nm | 16 servos USD | Screen |
|---|---:|---:|---:|---:|---:|---|
| XL330-M288 | 1.807 | 2.169 | 0.526 / 0.120 | 1.097 / 0.265 | 439.84 | reject |
| XC330-M288 | 1.887 | 2.265 | 0.549 / 0.211 | 1.143 / 0.463 | 1654.24 | reject |
| ST3215-12V | 2.399 | 2.879 | 0.694 / 0.410 | 1.437 / 0.902 | 319.84 | reject |
| STS3250-12V | 2.711 | 3.254 | 0.783 / 0.978 | 1.616 / 1.680 | 719.84 | conditional pass |

Selected: **STS3250-12V**, the cheapest candidate passing this conditional screen.

Load calculation: 20% mass contingency; one leg takes 50% of weight; 1.5 torque margin. Knee lever comes from exact two-link IK at nominal height. Hip roll includes the lateral joint offset. Crouch transition uses a conservative 65 mm moment arm. Head uses a 65 mm lever.

The torque-speed allowance is stall torque × assumed duty fraction × (1 − speed/no-load speed). STS3250 sustained screening uses its manufacturer rated torque point (16 kg.cm at 12V), with low-battery and speed derating. Other candidates use the configured stall fraction. The rated point is not a verified continuous thermal rating for this robot. A loaded bench test must validate or replace both before fabrication release.

Candidate screening uses the same 210 mm chassis and 75/80 mm links to avoid comparing different tasks. Smaller servos could support a separate smaller robot; that alternative is not established by this screen.

Power: 24.75 Wh nominal; 44.6 W assumed average; 25.0 minutes estimated at 75% usable energy. This is a budget, not measured endurance.

All-servo stall is 67.2 A, above the main fuse. Simultaneous stall is a fault condition, not an operating point. Fuse coordination and disconnect behavior require bench testing.

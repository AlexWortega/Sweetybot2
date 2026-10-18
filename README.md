# Sweetybot2

**Inspired by [Sweetie Bot](https://github.com/sweetie-bot-project/sweetie_bot) and [Microduck](https://github.com/pollen-robotics/microduck).** Independent pony robot CAD and MuJoCo policy experiments.

![V05 actual CAD turntable](renders/v05/assembly-turntable.gif)

*V05 CAD orbit: actual saved assembly, cosmetic surface finishes and studio lighting. This is not learned motion.*

![V05 mechanical redesign](renders/v05/01-midnight-hero.png)

**V05 digital mechanical + RL build:** 12 SM105BL-C001 leg motors, two STS3250 neck motors, two STS3032 ears, torso IMU, four contact hooves, chest 8×8 ToF and forehead RGB camera. Estimated mass **7.36kg**, with **8.84kg** design allowance. Physical construction, thermal endurance, firmware/drivers and trained v05 capability remain unverified.

| Head / camera | Profile |
|---|---|
| ![Head](renders/v05/02-portrait.png) | ![Profile](renders/v05/03-violet-profile.png) |

[V05 build and scope](docs/v05.md) · [BOM](docs/bom.md) · [Wiring](docs/wiring.md) · [Mechanical gate](reports/mechanical_gate.json) · [Digital physics validation](reports/v05_validation.json)

V04 remains **withdrawn: do not fabricate it**. Its thigh/shin, knee-support and frame interferences motivated this redesign. [Original failure evidence](reports/legacy-v04-motion-audit.json) · [Withdrawn release](https://github.com/AlexWortega/Sweetybot2/releases/tag/v0.4.0-hardware-preview).

The new front/hind legs fold in opposite directions. Separate rotor planes, supported rear outputs, bearing retainers, external case clamps, rigid seating shims, axial stops and explicit flange screw heads replace the interfering layout. Body 240×136×88mm, 115/110mm leg links, 200mm foot-centre track; assembly envelope approximately **351×246×533mm**. CNC load parts use 6061; shells use PETG and tread TPU. Handed, fitted custom instances are in `cad/manufacturing/`; generic previews do not imply four identical feet/carriers.

The conditional low-voltage actuator screen gives **2.41Nm required / 3.18Nm allowance** for hip pitch/knee, with 20% mass contingency, 50% weight on one leg and 1.5 torque margin. This is an analytical screen, not a measured continuous thermal rating. [Details](docs/feasibility.md).

Digital checks include exact same-link assembly fits; full-assembly surface candidates with exact BRep confirmation at 140 operational poses; STEP round trips of all 270 saved component instances; conservative CAD-derived convex collision coverage and the matching MuJoCo pose cohort. Parent-child brackets remain collidable. Neutral targets stand for 20 seconds on flat, rough, 8° slope and 25mm steps. These deterministic checks do **not** establish learned locomotion, running, recovery, continuous clearance or arbitrary-terrain capability.

The actor has **591 observations**: three 197-value frames from sampled IMU, encoders, command/action history, binary contacts/health and ToF/health. RGB is available for rendering/perception and is not an actor input. True root state/contact forces are reward/evaluation-only. Walk/run rewards penalise slip, work, overload and self-collision; running requires repeated sustained forward flight. Recovery requires a stable feet-supported stand. A dynamics signature binds checkpoints to configuration, mesh bytes and sensor/reward code; v02/v04 weights are incompatible. Solver/arena warnings stop training.

## Historical v02 policy preview

![Historical v02 walking and running/recovery attempts](renders/sweetybot2-training.gif)

*Separate lighter v02 model: walking, running attempt and recovery attempt; no mid-episode resets. This GIF is historical and does not show v05 or establish successful running/self-righting. The original 24-hour v02 cycle is complete; its selected policies did not pass all quality gates.*

[V02 release](https://github.com/AlexWortega/Sweetybot2/releases/tag/v0.2.0-preview) · [Preview provenance](renders/training_preview_20261017/manifest.json)

## Rebuild and verify

CAD: Python 3.13 / cadgen 0.7.21. Learning: Python 3.11/3.12 / `learning/requirements.txt`. Collision build: separate Python 3.11 / `simulation/collision-requirements.txt`. Set the Makefile interpreter variables for your machine.

```sh
make setup setup-learning setup-collision
make cad collisions model
make audit test simulate
make render package
```

See [the ordered workflow and acceptance limits](docs/v05.md). The complete ZIP includes STEP/STL, assembly GLB, MuJoCo/URDF, shared sensor decoder, source and reports. Fabrication approval is outstanding: purchased shaft/horn interfaces, engagement, bearing fits, cable routing, power/BMS/fuses, fatigue and motor temperatures require a physical prototype. Hardware firmware and device commissioning have not been performed. No v05 long-training or successful learned-policy claim is made by a CPU optimizer smoke.

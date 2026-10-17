# Sweetie Mini, revision 02

A compact pony quadruped with four 3-DOF legs, head yaw/pitch, independent ears, animated green eyes and a passive tail. Sixteen STS3250 12V actuators are retained: added mass is accepted only while the conditional torque screen passes. Its internal control and smart-servo approach are informed by Microduck, while the external proportions follow Sweetie Bot more closely.

## What changed from revision 01

The former 180mm foot-centre track is reduced to 140mm. The complete home width falls from 212mm to about 174mm. This changes actual hip placements and kinematics; no mesh is scaled to squeeze the robot.

The rounded chest/rump replaces the flat rectangular lid. The yaw motor is embedded in the chest, 14mm above the torso datum. The pitch axis is 44mm above it, placing it at torso Z=58mm instead of 87mm. Its motor case is centred laterally: shifting the case along its own Y rotation axis preserves the joint axis. The head fork is symmetric about Y=0. A removable white collar conceals the lower neck mechanism.

The head has an internal ear-servo saddle instead of two external rectangular boxes. Bent ear stems bring the pinnae outward; their working angles are restricted to ±0.25rad to avoid mutual contact. The pony now has a unicorn horn, green eyes, a purple curled forelock with a pink insert, a curved tail, white leg fairings and broad hollow hoof boots. Hoof soles are level in the home pose; compliant TPU is still required during motion.

The roll-to-pitch drop increases from 25mm to 42mm so the adjacent motor envelopes clear at the narrower spacing. Stance height increases by the same 17mm, retaining the nominal 75/80mm leg-link geometry and knee lever. The complete mechanical range is still subject to fit testing; [the mechanical review](mechanical_review.md) records the checks performed.

Appearance references: [official Sweetie Bot gallery](https://sweetiebot.net/), [Proto3 body](https://sweetiebot.net/new-proto3-body/). Saved reference images and hashes are in `references/appearance/`. The geometry is newly authored, not a copy of Proto3 CAD. The earlier design is retained in the revision-01 ZIP and selected comparison files in `references/previous_design/`.

## Mechanical datums

`config/robot.yaml` supplies millimetres and radians; dynamics descriptions use metres. +X points toward the muzzle, +Y left and +Z up. Legs are FL, FR, RL and RR. Each hip pitch origin is 18mm lateral and 42mm below its roll origin. All four legs use the same pitch-axis sign convention.

The vendor STEP output axis is Y through X=-25.5mm, Z=0. Translation `(25.5,-9.2,0)` establishes the output-face datum. Only rigid rotations/translations are applied to bought motors. Their geometry is not used to infer mass; the manufacturer mass is allocated explicitly. Horns, passive shafts, inserts and mounting fits remain provisional.

The deck is 210 × 100mm. Cowl clearance pockets encompass the roll clamps at home, and its mounting ribs are kept below their bases. Electronics remain layout envelopes; their shapes are not certified drilling templates. Saved print/assembly bounds are in `reports/cad_parts.json` and `assembly_bounds.json`.

## Loads and simulation

[Actuator screening](feasibility.md) includes the complete mass, 20% uncertainty, one leg carrying 50% of robot weight, a 1.5 torque margin and low-battery/speed derating. STS3250 sustained screening uses its manufacturer's rated 16kg.cm torque point. The motor curve, transient fraction and thermal endurance are uncalibrated; stall torque is not a continuous rating.

Mass uses saved STEP volume at solid-PETG density plus bought-component allowances. Link COMs and box-envelope inertias are engineering approximations, updated for the lower neck and dropped hip motors. They are shared by URDF and MuJoCo, and must be replaced by measured properties for hardware transfer.

The floating-base demonstrator uses a 50Hz control loop and 500Hz physics. It applies joint torques, never root forces, welds or mid-run pose resets. A support-wrench controller maps force targets through foot Jacobians; it receives ground-truth base state. A stock STS3250 position interface does not expose the same ideal torque command. State estimation and a calibrated, bounded transfer to the real position loop remain required.

Actual CAD meshes are visual assets; contact primitives are approximations. A slow crawl covers one metre in roughly ten simulated minutes. The `sit` task is a four-foot crouch and rise. Neither nominal simulation nor the parameter sweeps establish physical walking, thermal endurance, printed strength, wiring reliability or complete mechanical clearance.

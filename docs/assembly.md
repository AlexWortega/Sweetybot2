# Printing, assembly and calibration

Revision 02 supersedes the wide stance and exposed stacked neck. The original revision remains in its ZIP.

## Print and fit first

Print `fit_coupon.stl` before structural parts. It contains M3 clearance holes (3.1/3.2/3.3/3.4mm) and bearing pockets (10.0/10.1/10.2mm). Select the fit allowance from the coupon; update the model before regenerating the complete assembly. Current 10.2mm bearing seats are provisional, not an interference-fit prescription.

Use PETG for structural parts, 0.2mm layers, 5-6 walls and at least 50% infill as a starting process. Use solid local regions around bearings and clamp bosses. Cosmetic shells already model their wall thickness: 3 walls and sparse infill are sufficient as a starting point. TPU or bonded rubber is required at the hoof contact patch; a rigid PETG contact surface changes the friction assumption. Print settings are hypotheses until loaded coupons and a single leg are tested.

| Part | Orientation / assembly note |
|---|---|
| Deck | Broad XY underside on bed; keep strap and screw slots clear |
| Rails, cheek plates, roll yoke | Rotate 90 degrees about X or Y to put the broad face flat; avoid layers separating across the bearing bore |
| Cradles | Base down; clear the side-wall windows; check that the vendor case does not touch a clamp boss |
| Clamp straps | Broad face flat; two straps per cradle; M3 through bolts and locknuts |
| Knee support fork | Orient a cheek flat with support under the crossbar; remove support before fitting bearings |
| Head yoke | Cheek flat; support crossbar; leave shaft/horn access open |
| Body cowl | Open underside toward bed; support vent bridges if required |
| Head shell | Split/support as needed in the slicer; retain screw posts and eye windows; do not fill the internal volume |
| Head service hatch | Flat seam face on bed; bolt into the four shell posts |
| Ears and mane | Orient open/service surface to minimise trapped support; check the moving ear clearance |
| Hooves | Flat sole down; TPU; do not overtighten the centre fastener |
| Passive tail | Support at mounting flange; remove internal support before mounting |
| Neck collar | Open lower rim on the bed; keep top gimbal opening clear |
| Leg fairings | Open rim toward the bed; support only if required by the slicer |
| Horn | Base down; check M3 pilot and tip layers |
| Pink mane insert | Thin curved cosmetic piece; fit the groove before gluing |

All named printable part envelopes fit a 220mm bed after orientation. This does not imply every curved shell is support-free. Slice and inspect the toolpath before printing.

## Assembly order

1. With all power disconnected, verify a single servo's dimensions, horn interface, passive-side support and electrical variant against the STEP reference. Use the **6-12V STS3250**, with the specified 3S LiFePO4 pack (10.8V full). A 3S LiPo reaches 12.6V and must not be substituted on this rail.
2. Build one clamped servo and one supported joint. Verify shaft concentricity, horn engagement and axial clearance. Tighten clamp straps evenly; the clamp must not distort the servo case. Fit adapters to the real horn; do not drill or modify the servo case to make a placeholder interface fit.
3. Assemble the deck and two rails with the new inward mounting feet. Match rail holes at X=±90/Y=±37mm. The cowl uses X=±50/Y=±20mm deck holes and four provisional M3 heat-set inserts; test the 4.2mm pilots on a coupon before installing them. Fit roll-servo cradles, battery straps and electronics mounts; preserve access to main fuse, disconnect and USB. The Pi, IMU, adapter, regulator and display CAD representations are layout envelopes, not certified mounting templates. Use the actual board's mounting drawing for standoffs.
4. Assemble each roll yoke, dropped pitch servo, two thigh cheeks and supported knee. The rear knee support fork passes below the servo case, joining the opposite bearing to the front shin structure. Use spacer tubes so tightening an axle cannot squeeze a moving link. Fit two shin cheeks and the hoof.
5. Mount the four legs. Route motor power as separately fused parallel feeds; route data with short stubs. Leave a loop at each joint and retain it outside the swept mechanism. Turn each joint by hand through the intended working range before connecting the battery.
6. Install head yaw inside the chest. Centre the pitch motor case using its 18.7mm axial placement offset, then install the symmetric head fork and neck collar. Fit displays behind the windows. Retain the two ear motors in the internal saddle with soft shims and nylon ties through its slots. Fit the outward-bent ear stems, then the forelock, pink insert, horn and curved tail. The configured ear range is ±0.25rad. Check cable twist through head yaw and both ears.
7. Complete the low-voltage logic wiring and inspect polarity/continuity. Test 5V regulation from a current-limited supply with motor power disconnected. Only then connect a single fused motor branch and commission one servo at a time.

## Joint IDs and zeros

Legs in order FL, FR, RL, RR use IDs 1-3, 4-6, 7-9, 10-12 (roll, pitch, knee). Head yaw/pitch are 13/14; left/right ears 15/16. Commission each new ID in isolation to avoid duplicate IDs. The vendor-specific encoder range is 4096 counts/revolution; establish each mechanical zero and motor direction on the actual assembly. Never assume a mirrored motor has the same encoder sign.

The canonical zero has downward straight thigh/shin links. The actual standing pose comes from `reports/model.json`, not the illustrative home numbers in a servo tool. Lift/support the robot, torque disabled, set offsets, then enable with a slow bounded ramp. Starting a process must not move the robot automatically.

## Drawing and fit limitations

The exploded STEP and renders show part relationships and functional datums. Horn bolt pattern, exact bearing/shaft adapters, connector shell positions and electronics mounting holes require a physical fit sample. Cable swept volumes and fastener tool access require assembly inspection. Do not treat the exploded illustration as a certified manufacturing drawing.

Revision-02 hoof boots are hollow, with their soles aligned to the nominal home pose. Print them in compliant TPU and verify foot contact and plate clearance throughout the gait; they do not add an articulated ankle. The battery has a 1.8mm nominal pad space above the deck. Electronics mounts and all purchased-part interfaces remain sample-fit items.

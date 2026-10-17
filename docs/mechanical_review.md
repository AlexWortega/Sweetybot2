# Saved geometry review, revision 02

The assembly, internal layout, exploded groups and all twenty-three printable model types are rendered from saved STEP and visually reviewed. `reports/cad_parts.json`, `assembly_bounds.json` and `clearance_screen.json` retain the numeric evidence.

Every print candidate is one valid solid with positive volume and a bounding envelope below 220mm per dimension. Source-to-STEP volume changes must remain below 1%. The rounded body cover omits two detached fore-edge chips totalling less than 10mm³ left by intersecting service pockets; those are not printable features.

The yaw-motor service opening extends through the lower chest so the embedded case does not overlap the cover.

The home width is 174mm, down from 212mm, with a 140mm foot-centre track. The motor case for head pitch is centred to within 0.001mm of Y=0. Its bracket is symmetric and the head yaw/pitch axes lie on the torso centre plane. The battery has a nominal 1.8mm deck-pad gap, and the buck converter moves behind the compute board to make room for the embedded yaw motor.

The motor/electronics screen samples home plus seven positions of each of sixteen joints independently (113 poses), using conservative box envelopes transformed through shared FK. The final revision-02 screen reports zero motor/electronics envelope warnings in all 113 poses, replacing the sixteen extreme-roll warnings of revision 01. A separate home-pose intersection check against the saved body cover also reports zero warnings. These results use the 1mm³ reporting threshold stated in the JSON.

These are limited geometric screens. They exclude clamp-to-case mating fits, most moving cover interactions, horns/bearings, all cable swept volumes, deflection under load and simultaneous joint extremes. No blanket collision-free workspace is claimed. Physical ranges must be established on supported joints, including the new hoof boots, ear stems and neck collar.

The four cowl insert pilots are provisional 4.2mm bores for nominal M3/4mm-long heat-set inserts. Their centres match deck holes at X=±50/Y=±20mm. Rail feet match X=±90/Y=±37mm. Fit the actual insert and fastener revision on a coupon before assembly. Electronics remain layout envelopes rather than verified drilling templates. Eye lenses and inner-ear inserts represent appearance; finished display bezels and trim attachment remain fit-development items.

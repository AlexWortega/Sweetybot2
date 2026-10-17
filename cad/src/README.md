# CAD model catalog

All dimensions are millimetres. Run entrypoints through the project environment from the repository root, e.g. `uv run python cad/src/deck.py --force`. Use `make cad` for a consistent full rebuild with cold CAD processes, saved-STEP checks and mass reconciliation.

`lib/shapes.py` owns parametric factories; `config/robot.yaml` supplies geometry and purchased-component envelopes. `simulation/model.py` supplies shared link origins, axes and forward kinematics. Thin entrypoints export one named printable geometry each as STEP and STL. `assembly.py` exports the assembly as STEP and GLB; `internal.py` omits the outer covers; `exploded.py` separates groups for inspection.

| Model | Qty | Purpose |
|---|---:|---|
| deck | 1 | Bolted chassis base, strap and mounting slots |
| rail | 2 | Chassis side structure |
| body_shell | 1 | Rounded chest/rump with hip clearance pockets and low mounting ribs |
| servo_cradle | 10 | Four roll, four hip pitch, head yaw and pitch clamps |
| servo_strap | 20 | Two removable straps per cradle |
| roll_yoke | 4 | Roll-to-pitch offset bridge |
| axle_adapter | 4 | Provisional passive-shaft support |
| thigh_plate | 8 | Two cheeks per thigh |
| shin_plate | 8 | Two cheeks per shin |
| knee_fork | 4 | Opposed knee support |
| hoof | 4 | TPU/rubber contact body |
| head_shell | 1 | Hollow face, internal ear saddle, display apertures and horn mount |
| head_hatch | 1 | Service seam and support attachment |
| head_yoke | 1 | Symmetric centred pitch fork |
| ear | 2 | Independently moving ear shells |
| mane | 1 | Hollow forelock and curl |
| mane_stripe | 1 | Pink insert for the forelock groove |
| horn | 1 | Unicorn horn with mounting pilot |
| neck_collar | 1 | Short white neck cover |
| thigh_fairing | 4 | Rounded outer thigh skins |
| shin_fairing | 4 | Rounded outer shin skins |
| tail | 1 | Curved hollow passive tail |
| fit_coupon | 1 test print | M3 clearance and 623 bearing trials |

The model quantities exclude bought servos, bearings, electronics and fasteners. `reports/cad_parts.json` measures saved STEP volumes and envelopes; its coupon count is zero in robot mass. `reports/assembly_bounds.json` compares source volumes to STEP round trips. `cad/STEP/imported` contains checksum-tracked catalog inputs. Electronics envelopes and horn interfaces are provisional and documented in the assembly guide.

`cad/STL/links` is generated visual geometry in link coordinates, not a print plate. Its `.msh` files preserve STL triangles in MuJoCo's binary mesh format to avoid the STL reader's face-count limit. Both are visual-only assets; contact primitives and inertial allocations are defined separately in the shared graph.

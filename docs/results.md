# Measured digital results

Nominal acceptance: **PASS**. Config SHA-256 `a6ca4f0c284625ad3258c8200d278afb02d487220a7d7f443dd19b00e9801ba5`.

| Scenario | Pass | Simulated s | Forward m | Yaw deg | Max tilt deg | Max path error m |
|---|---|---:|---:|---:|---:|---:|
| stand | True | 60.00 | -0.0001 | 0.01 | 0.21 | 0.0007 |
| walk | True | 629.00 | 1.0208 | 0.02 | 6.24 | 0.0383 |
| turn | True | 324.20 | -0.0006 | 88.82 | 9.91 | 0.0385 |
| sit | True | 12.00 | -0.0000 | -0.02 | 1.72 | 0.0139 |
| mass_plus_20 | True | 629.00 | 1.0203 | 0.05 | 5.46 | 0.0365 |
| low_friction | True | 629.00 | 1.0203 | -0.50 | 5.67 | 0.0386 |
| delay_60ms | True | 629.00 | 1.0198 | -0.68 | 5.91 | 0.0395 |
| backlash_2deg | True | 629.00 | 1.0192 | -0.28 | 5.87 | 0.0376 |
| combined | True | 629.00 | 1.0187 | 0.03 | 5.86 | 0.0364 |

Largest nominal motor peak: 1.748 Nm; largest per-run RMS: 0.427 Nm. These are model outputs, not measured motor capability.

No root forces, root welds or pose resets were used. Ground-truth base state is available to the controller. Actual CAD meshes are visual assets; contact bodies are simplified. The COTS position-loop hardware transfer and thermal tests remain outstanding.

Raw reports retain per-joint peak/RMS torque and speed, actual duration, failures, contacts and the full path samples. A successful low-speed simulation is not a fabrication release.

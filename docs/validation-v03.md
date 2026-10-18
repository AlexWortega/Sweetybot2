# Validation boundary

Saved digital checks cover native CAD validity, one connected solid per printable component, a 220mm print envelope, mass accounting, conditional servo screening, selected **static** new-component clearances, unique GPIO ownership and resistor/debounce reference behavior. Studio images are rendered from the generated assembly GLB; the supplied appearance reference was not substituted for CAD output.

Before fabrication release:

1. Match exact STS3032 output shaft/spline/horn datum to the proposed saddle and ear pivot; print a fit coupon. Current microservo cases and adapters are envelopes.
2. Verify the exact LCD PCB depth/outline and design positive retention; check camera carrier clip tolerances and cable connector access. Check full camera FoV and chest ToF optical crosstalk through the open apertures before adding any protective optical window.
3. Sweep head, ears and neck across full motion with real CSI/servo cables, ToF pod, LCDs and fasteners. Static clearance checks are not a motion certificate.
4. Build one hoof first. Verify guide clearance, return springs, adjustable actuator gap, switch bounce and all pretravel tolerance extremes. Confirm hard stops carry full single-leg design load with negligible switch overload. Check off-centre contact, mud and repeated impacts.
5. Select the dedicated depth 3V3 regulator and its mounting; load-test all logic rails and the Pololu 6V ear rail at battery maximum/cutoff. Test power branch separation and shared-bus voltage levels.
6. Implement and verify sensor MCU firmware, Pi drivers, synchronized timestamps, bias/extrinsic calibration, fusion and stale/fault behavior. Reference decoding is not a running firmware implementation.
7. Measure leg and neck torque/current/temperature under the intended duty. Prior v02 learned policies fail the full quality gates and do not prove this revised hardware can run.
8. Recompute inertias/COM from measured hardware and evaluate a new simulation revision before reusing policies. The 24-hour v02 training run has not been changed by this candidate.

No fabricated parts, physical I/O, loaded thermals, environmental sealing or revised-policy performance have been validated.

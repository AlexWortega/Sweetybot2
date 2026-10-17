# Servo choices for v03

Use **14 STS3250-12V** for the twelve leg joints and head yaw/pitch; provisionally replace the **two ear servos with STS3032-C001 at 6V**. This saves 109g in motor mass before the added 6V supply, harness and changed printed supports. The head has an envelope-based microservo saddle; the exact shaft position and horn adaptor remain a fit gate.

| Candidate | Case mm | Mass | Manufacturer torque point | Decision |
|---|---|---:|---|---|
| Feetech STS3250-12V | 45.22×24.72×35 | 74.5g | 4.903Nm stall; 1.569Nm rated point at 12V | retain load-bearing joints; loaded thermal testing outstanding |
| Feetech STS3032-C001 | 23.2×12.1×28.5 | 20g | 0.441Nm **stall** at 6V; 0.09s/60° no-load | two ears only, separate 6V rail |
| Dynamixel XC430-W240 | 28.5×46.5×34 | 65g | 1.9Nm **stall** at 12V | no clear load or packaging improvement for legs |
| Dynamixel XM430-W350 | 28.5×46.5×34 | 82g | 4.1Nm **stall** at 12V; current control/feedback | useful control features, larger/heavier case; no automatic thermal win |

[Feetech STS3032 manufacturer specification](https://www.feetechrc.com/Data/feetechrc/upload/file/20200611/6372749220559698399729459.pdf) gives TTL half-duplex bus control, position/temperature/speed/voltage/load feedback, 12-bit position resolution and 1.2A stall current at 6V. It does not establish a continuous torque rating. Its sensor terminology is inconsistent between rows; do not infer encoder performance beyond the listed resolution. CAD is an envelope, unlike the retained STS3250 vendor model.

Sources for comparisons: [STS3250](https://www.feetechrc.com/sts_ttl_series%20servo.html), [XC430-W240](https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/), [XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/). No purchase has been made; pricing in analytical screening is a planning allowance, not a quote.

The local mass/load screen is regenerated for 14 main servos plus two 20g ear servos. It keeps the original 20% mass contingency, 50% weight on one leg, 1.5 torque margin and approximate low-voltage torque-speed derating. An analytical pass does not establish running capability: the prior development policy already exceeded the sustained-torque screen. Resolve actual measured joint loads and temperatures before selecting smaller leg actuators.

Ear load uses the ear CAD mass at a conservative 60mm lever, 3g acceleration and 1.5 margin; it is reported separately. No manufacturer continuous rating or prototype thermal test is available. Fit, voltage overshoot, bus compatibility, current limits and temperature under repeated animation must be checked on a bench.

# Bill of materials

Robot parts allowance: **US$1060.39**, plus US$30 charger. Taxes, shipping, printer and tools excluded. These are planning allowances, not verified local quotes. Quantities of consumables include spares.

| Item | Qty | Unit USD | Total USD | Source / qualification |
|---|---:|---:|---:|---|
| Feetech STS3250 12V TTL smart servo, 50kg.cm peak | 16 | 44.99 | 719.84 | [source](https://www.feetechrc.com/sts_ttl_series%20servo.html); planning allowance; use exact 12V version |
| Raspberry Pi Zero 2 W | 1 | 18.00 | 18.00 | [source](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/); planning allowance |
| 32GB high-endurance microSD | 1 | 12.00 | 12.00 | generic procurement specification; planning allowance |
| Waveshare Bus Servo Adapter (A), USB to ST/SC TTL | 1 | 14.00 | 14.00 | [source](https://www.waveshare.com/bus-servo-adapter-a.htm); planning allowance; verify ST protocol and pinout |
| Adafruit ICM-20948 breakout, 3.3V logic | 1 | 25.00 | 25.00 | [source](https://www.adafruit.com/product/4554); planning allowance; provisional envelope |
| Waveshare 1.3inch LCD Module, ST7789 SPI | 2 | 13.00 | 26.00 | [source](https://www.waveshare.com/1.3inch-lcd-module.htm); planning allowance; display mounting needs actual board |
| Assembled 3S LiFePO4 26650 pack, >=2.5Ah, >=25A BMS, balance connector | 1 | 65.00 | 65.00 | [source](https://lithiumwerks.com/products/lithium-ion-26650-cells/); planning allowance; <=82x70x30mm; 10.8V maximum; verify assembled pack |
| 5V 3A buck converter, input >=16V, e.g. Pololu D24V50F5 | 1 | 32.95 | 32.95 | [source](https://www.pololu.com/product/2851); planning allowance; confirm exact SKU before ordering |
| Battery master disconnect + motor-only latching disconnect, >=30A DC | 1 | 18.00 | 18.00 | generic procurement specification; planning allowance; no small signal pushbutton in motor path |
| 25A main fuse + 4x7.5A leg + 5A head fuses + holders | 1 | 12.00 | 12.00 | generic procurement specification; planning allowance; coordination requires load test |
| 16AWG main, 20AWG branches, 22AWG short servo feeds, connectors | 1 | 24.00 | 24.00 | generic procurement specification; planning allowance; confirm connector ampacity |
| 2200uF 25V low-ESR bus capacitor, secure enclosure | 1 | 3.00 | 3.00 | generic procurement specification; planning allowance; not an overvoltage clamp |
| 623ZZ 3x10x4 bearing | 14 | 1.00 | 14.00 | [source](https://www.step.parts/parts/bearing_623zz); planning allowance; external support interfaces provisional |
| M3 heat-set inserts for cowl, 4mm length; verify 4.2mm pilot by coupon | 4 | 0.40 | 1.60 | generic procurement specification; planning allowance; four matched deck/cowl points |
| M3 screws/nuts/washers/spacers/axles assorted | 1 | 28.00 | 28.00 | generic procurement specification; planning allowance; select lengths after coupon fit |
| Nylon straps, heatshrink, cable sleeves, rubber soles | 1 | 12.00 | 12.00 | generic procurement specification; planning allowance |
| PETG filament, 1 kg spool | 1 | 25.00 | 25.00 | generic procurement specification; planning allowance; solid-volume mass check in cad_parts.json |
| TPU hoof pads, small spool/allocation | 1 | 10.00 | 10.00 | generic procurement specification; planning allowance |
| External LiFe-compatible 3S balance charger | 1 | 30.00 | 30.00 | generic procurement specification; planning allowance; listed separately from robot parts |

Printed component quantities and measured solid volumes are in [the CAD parts report](../reports/cad_parts.json). Print the coupon first; do not buy an entire actuator set before one loaded joint is tested.

Vendor CAD is a catalog reference for the STS3250 family. It does not certify a particular supplier revision or horn screw pattern. Dimensions, horn spline engagement and bearing adapters must be checked on the purchased sample.

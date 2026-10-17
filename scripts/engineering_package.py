"""Generate BOM and wiring artifact from the robot configuration and measured print volumes."""
import json
from scripts.common import ROOT,load,total_mass
ITEMS=[
('actuator','Feetech STS3250 12V TTL smart servo, 50kg.cm peak',16,44.99,'https://www.feetechrc.com/sts_ttl_series%20servo.html','planning allowance; use exact 12V version'),
('compute','Raspberry Pi Zero 2 W',1,18,'https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/','planning allowance'),
('storage','32GB high-endurance microSD',1,12,'','planning allowance'),
('interface','Waveshare Bus Servo Adapter (A), USB to ST/SC TTL',1,14,'https://www.waveshare.com/bus-servo-adapter-a.htm','planning allowance; verify ST protocol and pinout'),
('imu','Adafruit ICM-20948 breakout, 3.3V logic',1,25,'https://www.adafruit.com/product/4554','planning allowance; provisional envelope'),
('eyes','Waveshare 1.3inch LCD Module, ST7789 SPI',2,13,'https://www.waveshare.com/1.3inch-lcd-module.htm','planning allowance; display mounting needs actual board'),
('battery','Assembled 3S LiFePO4 26650 pack, >=2.5Ah, >=25A BMS, balance connector',1,65,'https://lithiumwerks.com/products/lithium-ion-26650-cells/','planning allowance; <=82x70x30mm; 10.8V maximum; verify assembled pack'),
('power','5V 3A buck converter, input >=16V, e.g. Pololu D24V50F5',1,32.95,'https://www.pololu.com/product/2851','planning allowance; confirm exact SKU before ordering'),
('power','Battery master disconnect + motor-only latching disconnect, >=30A DC',1,18,'','planning allowance; no small signal pushbutton in motor path'),
('power','25A main fuse + 4x7.5A leg + 5A head fuses + holders',1,12,'','planning allowance; coordination requires load test'),
('power','16AWG main, 20AWG branches, 22AWG short servo feeds, connectors',1,24,'','planning allowance; confirm connector ampacity'),
('power','2200uF 25V low-ESR bus capacitor, secure enclosure',1,3,'','planning allowance; not an overvoltage clamp'),
('hardware','623ZZ 3x10x4 bearing',14,1,'https://www.step.parts/parts/bearing_623zz','planning allowance; external support interfaces provisional'),
('hardware','M3 heat-set inserts for cowl, 4mm length; verify 4.2mm pilot by coupon',4,.40,'','planning allowance; four matched deck/cowl points'),
('hardware','M3 screws/nuts/washers/spacers/axles assorted',1,28,'','planning allowance; select lengths after coupon fit'),
('hardware','Nylon straps, heatshrink, cable sleeves, rubber soles',1,12,'','planning allowance'),
('printing','PETG filament, 1 kg spool',1,25,'','planning allowance; solid-volume mass check in cad_parts.json'),
('printing','TPU hoof pads, small spool/allocation',1,10,'','planning allowance'),
('tools','External LiFe-compatible 3S balance charger',1,30,'','planning allowance; listed separately from robot parts')]

def main():
 c=load();items=[dict(category=a,item=b,quantity=n,unit_usd=u,total_usd=n*u,url=link,status=note) for a,b,n,u,link,note in ITEMS]
 total=sum(i['total_usd'] for i in items if i['category']!='tools');result={'currency':'USD','price_date':'2026-10-17','robot_parts_allowance':round(total,2),'charger_allowance':30,'tax_shipping_tools_excluded':True,'items':items}
 (ROOT/'reports/bom.json').write_text(json.dumps(result,indent=2)+'\n')
 lines=['# Bill of materials','',f'Robot parts allowance: **US${total:.2f}**, plus US$30 charger. Taxes, shipping, printer and tools excluded. These are planning allowances, not verified local quotes. Quantities of consumables include spares.','', '| Item | Qty | Unit USD | Total USD | Source / qualification |','|---|---:|---:|---:|---|']
 for i in items:
  source=f"[source]({i['url']})" if i['url'] else 'generic procurement specification'
  lines.append(f"| {i['item']} | {i['quantity']} | {i['unit_usd']:.2f} | {i['total_usd']:.2f} | {source}; {i['status']} |")
 lines+=['','Printed component quantities and measured solid volumes are in [the CAD parts report](../reports/cad_parts.json). Print the coupon first; do not buy an entire actuator set before one loaded joint is tested.','', 'Vendor CAD is a catalog reference for the STS3250 family. It does not certify a particular supplier revision or horn screw pattern. Dimensions, horn spline engagement and bearing adapters must be checked on the purchased sample.']
 (ROOT/'docs/bom.md').write_text('\n'.join(lines)+'\n')
 svg='''<svg xmlns="http://www.w3.org/2000/svg" width="1300" height="900" viewBox="0 0 1300 900">
<style>text{font-family:Arial,sans-serif;fill:#1e293b} .title{font-size:28px;font-weight:bold}.h{font-size:18px;font-weight:bold}.s{font-size:15px}.box{fill:#f1f5f9;stroke:#94a3b8;stroke-width:2}.power{stroke:#e07040;stroke-width:4;fill:none}.data{stroke:#6378b6;stroke-width:3;fill:none;stroke-dasharray:8 4}</style>
<rect width="1300" height="900" fill="#fff"/><text x="40" y="50" class="title">Sweetie Mini · power and control architecture</text>
<text x="40" y="80" class="s">Design specification · common ground · STS3250 + 3S LiFePO4 only · physical pinout must be verified</text>
<g class="box"><rect x="40" y="130" width="240" height="90" rx="12"/><rect x="360" y="130" width="240" height="90" rx="12"/><rect x="680" y="130" width="250" height="90" rx="12"/><rect x="1010" y="130" width="240" height="90" rx="12"/>
<rect x="680" y="300" width="250" height="75" rx="12"/><rect x="1010" y="300" width="240" height="75" rx="12"/>
<rect x="40" y="480" width="240" height="90" rx="12"/><rect x="360" y="480" width="240" height="90" rx="12"/><rect x="680" y="480" width="250" height="90" rx="12"/><rect x="1010" y="480" width="240" height="90" rx="12"/>
<rect x="360" y="660" width="240" height="75" rx="12"/><rect x="680" y="660" width="250" height="75" rx="12"/></g>
<g class="h"><text x="60" y="165">3S LiFePO4 · 2500mAh</text><text x="380" y="165">25A fuse + master</text><text x="700" y="165">Motor disconnect</text><text x="1030" y="165">Fused distribution</text><text x="700" y="330">5V buck · 3A budget</text><text x="1030" y="330">12 leg + 4 head servos</text><text x="60" y="515">Robot control process</text><text x="380" y="515">Pi Zero 2 W</text><text x="700" y="515">USB / TTL adapter</text><text x="1030" y="515">ST data bus · 1Mbps</text><text x="380" y="690">ICM-20948 · I²C</text><text x="700" y="690">2 × ST7789 · SPI</text></g>
<g class="s"><text x="60" y="194">8.7V stop / 10.8V full</text><text x="380" y="194">main cable: 16AWG budget</text><text x="700" y="194">latching, DC-current-rated</text><text x="1030" y="194">legs 7.5A / head 5A</text><text x="700" y="355">own 2A input fuse</text><text x="1030" y="355">parallel power injections</text><text x="60" y="545">single writer + deadman</text><text x="380" y="545">logic survives motor stop</text><text x="700" y="545">USB data / logic power</text><text x="1030" y="545">daisy data, short branches</text></g>
<path class="power" d="M280 175H360 M600 175H680 M930 175H1010 M1130 220V300 M630 175V337H680 M805 375V420H480V480"/>
<path class="data" d="M280 525H360 M600 525H680 M930 525H1010 M1130 480V375 M480 570V660 M600 550H640V697H680"/>
<text x="40" y="795" class="h">Power is parallel. Motor current must not pass through the Pi or a long chain of servo headers.</text>
<text x="40" y="830" class="s">USB 5V, SPI 3.3V and I²C 3.3V are logic rails. Never connect battery voltage to a GPIO or logic power input.</text>
<text x="40" y="860" class="s">67.2A total stall exceeds the fuse rating. A simultaneous stall is a fault; fuses are not an active torque limiter.</text></svg>'''
 (ROOT/'docs/wiring.svg').write_text(svg)
 print(f'BOM allowance ${total:.2f}; robot allocation {total_mass(c):.3f} kg')
if __name__=='__main__':main()

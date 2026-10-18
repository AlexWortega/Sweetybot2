"""Generate v05 purchasing/build ledger; all prices remain planning allowances."""
import csv,io,json
from collections import Counter
from scripts.common import ROOT,load,actuator

def main():
 c=load();rows=[]
 def add(item,n,status,notes,url='',unit_usd=None):rows.append(dict(item=item,quantity=n,status=status,notes=notes,url=url,unit_usd=unit_usd))
 for model,n in Counter(c['actuation']['assignment'].values()).items():
  spec=next(actuator(c,j) for j,m in c['actuation']['assignment'].items() if m==model)
  add(model,n,'candidate; not purchased',spec.get('interface','TTL SMS/STS; confirm firmware protocol'),spec['source'],spec.get('unit_usd',12.))
 old=list(csv.DictReader((ROOT/'docs/bom-v03.csv').open()))
 for r in old:
  if r['item'].startswith('Feetech'):continue
  add(r['item'],r['quantity'],r['status'],r['notes'])
 for item,n,note in [('Raspberry Pi Zero 2 W + microSD',1,'logic compute'),('USB RS485 adapter, 3.3V-safe USB logic, SMS protocol support',1,'12 leg motors only; differential A/B and common reference'),('Waveshare Bus Servo Adapter A (TTL only), 42x33mm',1,'2 neck and 2 ear motors; TTL signal bus separate from RS485'),('Adafruit CH334F USB hub 5999 + OTG and adapter cables',1,'Pi Zero has one USB data port; envelope and cable routing provisional'),('3S LiFePO4 >=2.5Ah pack with >=40A continuous BMS',1,'9.9V nominal, 10.8V max, loaded cutoff 9.3V; real pack sag unmeasured'),('40A main fuse, four 10A leg, 5A head and 5A logic fuses',1,'DC disconnect and fuse coordination require loaded tests'),('12AWG main, 16AWG leg branches, rated connectors',1,'short feeds; fused parallel power; do not daisy-chain motor load current'),('5V >=3A logic buck + motor-only disconnect',1,'logic survives motor emergency stop'),('6061 CNC load parts',1,'deck, rails, clamps, yokes, cheek plates, forks, adapters and spacers; not FDM load parts'),('6807 bearings, 35x47x7mm',8,'catalogue envelope; fits, retention and external load rating require bench checks'),('0.75 and 0.80mm rigid shim stock for 14 motor clamps',70,'case and strap seating pads, gauge against purchased cases'),('M3 low socket heads, <=6mm diameter and <=3mm height',160,'8 per driven flange, heads included in CAD; confirm screw lengths and engagement'),('M2 bearing retainers and neck/ear horns',1,'shaft spline datum, screw lengths, shoulder fits and loaded joints remain physical gates')]:add(item,n,'procurement specification',note)
 (ROOT/'reports/bom.json').write_text(json.dumps({'revision':c['revision'],'price_status':'planning allowances, not quotes','items':rows},indent=2)+'\n')
 out=io.StringIO();w=csv.DictWriter(out,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows);(ROOT/'docs/bom-v05.csv').write_text(out.getvalue())
 text=['# V05 bill of materials','','Hardware candidate; no purchasing or physical assembly performed. Servo allowances: 12 SM105 $120 each, two STS3250 $44.99 each; these are unverified planning amounts.','','| Item | Qty | Qualification |','|---|---:|---|']
 for r in rows:text.append(f"|{r['item']}|{r['quantity']}|{r['notes']}|")
 (ROOT/'docs/bom.md').write_text('\n'.join(text)+'\n')
 (ROOT/'docs/wiring.md').write_text('''# V05 wiring

3S LiFePO4 (9.3V loaded cutoff, 10.8V full) → DC master disconnect → 40A main fuse → separately fused motor and logic distribution. Motor emergency disconnect leaves logic powered. 4 × 10A leg branches feed motors in parallel; 5A head branch feeds neck and regulated 6V ear power. 5A logic branch feeds 5V regulator, Pi and collector. Fuse coordination, pack/BMS pulse current, connector ampacity, regen clamp and loaded voltage sag are not validated.

Two independent signal interfaces: USB RS485 → 12 leg SM-series motors; USB TTL → STS neck/ear motors. Their signal conductors must never be joined. Common reference ground and power branches are explicit; 6V and battery positive rails must never be joined. Configure IDs once, with one motor connected, then verify each actual model's register map, baud, polarity, timeout and torque-disable behavior before joining its bus. No assumption of protocol interchangeability is a driver implementation.

Pi Zero OTG requires a powered/appropriate USB hub for the two adapters. The selected CH334F hub has a 24x20.1x5mm envelope. USB RS485 board is a 30x22x14mm allowance; select its exact revision before manufacture. Cable routing remains provisional. Main current does not flow through USB, Pi or small signal headers. Use hardware motor disconnect/deadman independent of Python.

GPIO ownership is in config/robot.yaml and checked by scripts/check_sensor_contract.py. XIAO reads four resistor-coded 12-bit ADC loops and the LSM6DSV16X; UART is 3.3V logic to Pi. Pi owns ToF I2C and round LCD SPI. Camera uses the 22-pin Pi Zero to 15-pin camera FPC. Dedicated >=0.5A 3.3V regulator supplies ToF; do not parallel regulator output with Pi 3V3. Ear regulator is a separate 6V buck; verify current at 9.3V input.

Each contact: 3V3 — 10k pull-up — ADC wire — [10k EOL || (NO switch + 1k)] — ground. Put EOL beyond the foot connector. Ratios: air ≈0.5, contact ≈0.0833, open ≈1, short ≈0. Wiring faults and stale samples invalidate contact, never become confirmed air. Collector packet format and host decoder are in hardware/telemetry.py; firmware and electrical drivers have not been flashed or bench-tested.
''')
 print('Wrote v05 BOM and two-bus wiring specification')
if __name__=='__main__':main()

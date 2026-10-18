"""Inspect saved STEP parts; report print envelopes and conservative solid PETG mass."""
import json
from scripts.common import ROOT
from cadgen import read_step
COUNTS={'deck':1,'rail':2,'body_shell':1,'servo_cradle':10,'servo_strap':20,'roll_yoke':4,'thigh_plate':8,'shin_plate':8,'hoof':4,'head_shell':1,'head_hatch':1,'ear':2,'mane':1,'axle_adapter':4,'knee_fork':4,'head_yoke':1,'tail':1,'fit_coupon':0,'horn':1,'mane_stripe':1,'neck_collar':1,'thigh_fairing':4,'shin_fairing':4}
COUNTS.update(camera_carrier=1,sensor_tray=1,depth_pod=1,hoof_sole_plate=4,hoof_tpu_pad=4,hoof_actuator=4,eye_bezel=2)
COUNTS.update(shin_plate=4,shin_rear_plate=4,servo_cradle=2,servo_strap=4,servo_cradle_sm105=8,servo_cradle_sm80=4,servo_strap_sm105=16,servo_strap_sm80=8)
def main():
 rows=[]
 for name,count in COUNTS.items():
  p=ROOT/'cad/STEP'/f'{name}.step';s=read_step(p);b=s.bounding_box();dim=list(b.size)
  # Printing orientation may rotate the part. A 256 mm cubic envelope is a conservative first check.
  metal=name in ['deck','rail','servo_cradle','servo_strap','servo_cradle_sm105','servo_cradle_sm80','servo_strap_sm105','servo_strap_sm80','roll_yoke','thigh_plate','shin_plate','shin_rear_plate','axle_adapter','knee_fork','head_yoke']
  material='6061' if metal else 'TPU' if name in ['hoof_tpu_pad','hoof_actuator'] else 'PETG'
  rows.append(dict(material=material,material_mass_kg=s.volume*({'6061':2.7,'TPU':1.2,'PETG':1.27}[material])*1e-6,part=name,count=count,volume_mm3=s.volume,solid_petg_mass_kg=s.volume*1.27e-6,
              bounds_mm=dim,valid=bool(s.is_valid),solids=len(s.solids()),fits_256_bed=sum(v>256 for v in dim)==0))
 result={'parts':rows,'printed_solid_mass_kg':sum(r['material_mass_kg']*r['count'] for r in rows if r['material']!='6061'),'metal_solid_mass_kg':sum(r['material_mass_kg']*r['count'] for r in rows if r['material']=='6061'),
         'note':'Solid material allocations; load parts require 6061 manufacture, not FDM. Purchased hardware excluded. Does not certify strength.'}
 (ROOT/'reports/cad_parts.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(result,indent=2))
 assert all(r['valid'] and r['solids']==1 and r['fits_256_bed'] for r in rows)
if __name__=='__main__':main()

"""Inspect saved STEP parts; report print envelopes and conservative solid PETG mass."""
import json
from scripts.common import ROOT
from cadgen import read_step
COUNTS={'deck':1,'rail':2,'body_shell':1,'servo_cradle':10,'servo_strap':20,'roll_yoke':4,'thigh_plate':8,'shin_plate':8,'hoof':4,'head_shell':1,'head_hatch':1,'ear':2,'mane':1,'axle_adapter':4,'knee_fork':4,'head_yoke':1,'tail':1,'fit_coupon':0,'horn':1,'mane_stripe':1,'neck_collar':1,'thigh_fairing':4,'shin_fairing':4}
def main():
 rows=[]
 for name,count in COUNTS.items():
  p=ROOT/'cad/STEP'/f'{name}.step';s=read_step(p);b=s.bounding_box();dim=list(b.size)
  # Printing orientation may rotate the part. A 220 mm cubic envelope is a conservative first check.
  rows.append(dict(part=name,count=count,volume_mm3=s.volume,solid_petg_mass_kg=s.volume*1.27e-6,
              bounds_mm=dim,valid=bool(s.is_valid),solids=len(s.solids()),fits_220_bed=sum(v>220 for v in dim)==0))
 result={'parts':rows,'printed_solid_mass_kg':sum(r['solid_petg_mass_kg']*r['count'] for r in rows),
         'note':'Solid CAD density upper bound; purchased hardware excluded. Does not certify FDM strength.'}
 (ROOT/'reports/cad_parts.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(result,indent=2))
 assert all(r['valid'] and r['solids']==1 and r['fits_220_bed'] for r in rows)
if __name__=='__main__':main()

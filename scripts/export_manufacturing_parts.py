"""Export the actual handed/fitted custom parts for design review, not fabrication approval."""
import hashlib,json,shutil
from scripts.common import ROOT
from scripts.cad_snapshot import geometry_signature

def main():
 meta=json.loads((ROOT/'reports/cad_snapshot.json').read_text());assert meta['geometry_signature']==geometry_signature()
 ledger=json.loads((ROOT/'reports/mass_budget.json').read_text());materials={(r['link'],r['part']):r['material'] for r in ledger['components']}
 out=ROOT/'cad/manufacturing';out.mkdir(exist_ok=True);rows=[]
 for row in meta['parts']:
  material=materials.get((row['link'],row['part']))
  if material not in ['6061 aluminium','PETG','TPU','rigid aluminium shim stock']:continue
  src=ROOT/row['step'];assert hashlib.sha256(src.read_bytes()).hexdigest()==row['step_sha256']
  stem=row['link']+'__'+row['part'];target=out/(stem+'.step');shutil.copyfile(src,target)
  stl=src.with_suffix('.stl')
  if stl.exists():shutil.copyfile(stl,out/(stem+'.stl'))
  rows.append({'link':row['link'],'part':row['part'],'material':material,'step':str(target.relative_to(ROOT)),'sha256':row['step_sha256']})
 (out/'README.md').write_text('# Handed custom parts for design review\n\nUse the exact per-instance geometry here when checking left/right and front/hind parts. The hind legs fold rearward; their hoof seats and motor carriers are handed. Generic part previews are insufficient to select four identical pieces.\n\nFabrication approval is outstanding: actual servo spline/horn and screw fits, bearings, fasteners, cable routing, strength, thermal/power tests and device commissioning remain required. See docs/v05.md and reports/mechanical_gate.json.\n')
 (ROOT/'reports/manufacturing_parts.json').write_text(json.dumps({'geometry_signature':geometry_signature(),'fabrication_approved':False,'parts':rows},indent=2)+'\n');print('Exported',len(rows),'handed custom part instances')
if __name__=='__main__':main()

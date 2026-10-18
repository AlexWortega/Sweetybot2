"""Reproducible v05 digital mechanical audit; see docs/v05.md for scope."""
from scripts.audit_inputs import snapshot
INPUTS=snapshot()
from scripts.cad_snapshot import geometry_signature
GEO_INPUT=geometry_signature()
import sys,json
import numpy as np
from scripts.common import ROOT,load
sys.path.insert(0,str(ROOT/'cad/src'))
from lib.shapes import link_components
from cadgen import build123d as bd
P=link_components();all_parts=P;from lib import actuation_v05 as A
A.COLLISION_MODE=True;CP=link_components();A.COLLISION_MODE=False;clean={s.label:s for vv in CP.values() for s in vv};out=ROOT/'cad/collision/source';out.mkdir(parents=True,exist_ok=True);rows=[]
for link,parts in P.items():
 for s in parts:
  n=s.label
  if any(t in n for t in ['display_','pigment','nostril','ear_inner','eyebrow','smile','iris','pupil','highlight']):continue
  key=link+'__'+n;path=out/(key+'.stl');bd.export_stl(s,path,tolerance=.2,angular_tolerance=.3)
  b=s.bounding_box();T=np.eye(4);tr=clean.get(n,s).location.wrapped.Transformation()
  for i in range(3):
   for j in range(4):T[i,j]=tr.Value(i+1,j+1)
  cs=clean.get(n,s);canonical=cs.moved(cs.location.inverse());cp=out/(key+'__canonical.stl');bd.export_stl(canonical,cp,tolerance=.2,angular_tolerance=.3)
  rows.append({'link':link,'part':n,'path':str(path.relative_to(ROOT)),'min_mm':list(b.min),'max_mm':list(b.max),'canonical_path':str(cp.relative_to(ROOT)),'placement_mm':T.tolist(),'volume_mm3':sum(x.volume for x in s.solids()),'solids':len(s.solids()),'valid':bool(s.is_valid)})
from scripts.cad_snapshot import geometry_signature
saved=[]
for link,parts in all_parts.items():
 for shape in parts:
  key=link+'__'+shape.label;p=out/(key+'.step');bd.export_step(shape,p)
  import hashlib
  original_volume=sum(x.volume for x in shape.solids());cold=bd.import_step(p);saved_volume=sum(x.volume for x in cold.solids());error=abs(saved_volume-original_volume)/max(original_volume,1e-12)
  assert error<.01 and len(cold.solids())==len(shape.solids()), (shape.label,'STEP round-trip changed geometry',error)
  saved.append({'link':link,'part':shape.label,'step':str(p.relative_to(ROOT)),'step_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'original_volume_mm3':original_volume,'saved_volume_mm3':saved_volume,'relative_volume_change':error})
assert snapshot()==INPUTS and geometry_signature()==GEO_INPUT, 'Source changed during export'
(ROOT/'reports/cad_snapshot.json').write_text(json.dumps({'geometry_signature':GEO_INPUT,'parts':saved},indent=2)+'\n')
(ROOT/'reports/collision_source.json').write_text(json.dumps(rows,indent=2)+'\n')
print('EXPORTED',len(rows),flush=True)
for r in rows:
 if not r['valid'] or (r['solids']!=1 and not any(t in r['part'] for t in ['M3_heads','clamp_shims'])):print('BAD',r,flush=True)

assert snapshot()==INPUTS, 'Source changed during export'
(ROOT/'reports/collision_source_inputs.json').write_text(json.dumps({'complete':True,'geometry_signature':GEO_INPUT,'source_sha256':INPUTS},indent=2)+'\n')

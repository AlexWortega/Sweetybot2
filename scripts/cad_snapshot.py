"""Cache exact placed solids; mass-only YAML updates do not change CAD geometry."""
import hashlib,json
from scripts.common import ROOT,load

def geometry_signature():
 c=load();c.pop('mass',None);c.pop('link_mass_properties',None)
 h=hashlib.sha256(json.dumps(c,sort_keys=True).encode())
 for n in ['simulation/model.py','cad/src/lib/shapes.py','cad/src/lib/sensors_face.py','cad/src/lib/actuation_v05.py']:
  h.update(n.encode()+b'\0'+(ROOT/n).read_bytes())
 return h.hexdigest()

def load_parts(require_snapshot=False):
 import sys
 sys.path.insert(0,str(ROOT/'cad/src'))
 p=ROOT/'reports/cad_snapshot.json'
 if p.exists():
  meta=json.loads(p.read_text())
  if meta.get('geometry_signature')==geometry_signature():
   from cadgen import read_step
   parts={}
   for row in meta['parts']:
    path=ROOT/row['step']
    if hashlib.sha256(path.read_bytes()).hexdigest()!=row['step_sha256']:raise ValueError('CAD snapshot corrupted: '+row['step'])
    shape=read_step(path);shape.label=row['part']
    if 'original_volume_mm3' not in row:raise ValueError('CAD snapshot lacks STEP round-trip proof; re-export sources')
    volume=sum(x.volume for x in shape.solids())
    if abs(volume-row['original_volume_mm3'])/max(row['original_volume_mm3'],1e-12)>=.01:raise ValueError('STEP geometry differs from original CAD: '+row['part'])
    parts.setdefault(row['link'],[]).append(shape)
   return parts
 if require_snapshot:raise ValueError('Export and verify collision sources before reconciling mass')
 from lib.shapes import link_components
 return link_components()

"""Hashes bind each mechanical audit to the exact source revision it checked."""
import hashlib
from scripts.common import ROOT
FILES=('config/robot.yaml','simulation/model.py','cad/src/lib/shapes.py','cad/src/lib/sensors_face.py','cad/src/lib/actuation_v05.py')
def snapshot():return {n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES}

def overlap_volume(a,b):
 """Intersect solid pairs, avoiding OCC compound false positives at touching faces."""
 import numpy as np
 volume=0.
 for sa in a.solids():
  aa=sa.bounding_box()
  for sb in b.solids():
   bb=sb.bounding_box()
   if not np.all(np.minimum(list(aa.max),list(bb.max))-np.maximum(list(aa.min),list(bb.min))>.001):continue
   hit=sa&sb
   if hit is not None:volume+=max(0.,float(hit.volume))
 return volume

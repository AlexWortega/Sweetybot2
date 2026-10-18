"""Reproducible v05 digital mechanical audit; see docs/v05.md for scope."""
from scripts.audit_inputs import snapshot,overlap_volume
INPUTS=snapshot()
import sys,json,itertools
import numpy as np
from scripts.common import ROOT
sys.path.insert(0,str(ROOT/'cad/src'))
from scripts.cad_snapshot import load_parts as link_components
P=link_components();rows=[]
for link,parts in P.items():
 for a,b in itertools.combinations(parts,2):
  if any(t in a.label+b.label for t in ['display_','eyebrow','smile','nostril','ear_inner']):continue
  ba,bb=a.bounding_box(),b.bounding_box()
  if not np.all(np.minimum(list(ba.max),list(bb.max))-np.maximum(list(ba.min),list(bb.min))>.02):continue
  v=overlap_volume(a,b)
  if v>.1:rows.append({'link':link,'parts':[a.label,b.label],'overlap_mm3':float(v)});print(link,a.label,b.label,round(v,3),flush=True)
(ROOT/'reports/static_fit.json').write_text(json.dumps({'complete':True,'passed':not rows,'source_sha256':INPUTS,'method':'exact BRep overlap between separate physical components on the same rigid link; appearance pigments excluded','intersections':rows},indent=2)+'\n');print('DONE',len(rows),flush=True)

assert snapshot()==INPUTS, 'Source changed during audit'
if rows:raise SystemExit(1)

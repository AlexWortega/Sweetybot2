"""Diagnostic exact BRep sweep of one leg against its adjacent structure.
A negative result proves interference. A clean sampled leg is not a full-robot
continuous motion certificate; same-link assembly fits are outside this audit.
"""
import sys,json,itertools,time
from pathlib import Path
import numpy as np
from scripts.common import ROOT,load
sys.path.insert(0,str(ROOT/'cad/src'))
from lib.shapes import link_components,matrix_location
from simulation.model import graph,fk,home
c=load();P=link_components();links=graph(c);base=home(c);rows=[]
chosen={k:v for k,v in P.items() if k in ['fl_roll','fl_thigh','fl_shin']}
chosen['torso']=[s for s in P['torso'] if s.label.startswith('fl_') or s.label in ['chassis_deck','body_cowl','rail_1']]
poses=[('home',base)]
for kind,vals in [('hip_pitch',np.linspace(-1.5,1.1,11)),('knee',np.linspace(.1,2.1,11)),('hip_roll',np.linspace(-.4,.4,7))]:
 for v in vals:poses.append((f'{kind}={v:.3f}',{**base,'fl_'+kind:float(v)}))
for h,k in itertools.product([-.9,-.3,.4],[.2,1.0,1.8]):poses.append((f'h={h} k={k}',{**base,'fl_hip_pitch':h,'fl_knee':k}))
for label,q in poses:
 T=fk(links,q,c);items=[];hits=[]
 for link,children in chosen.items():
  M=T[link].copy();M[:3,3]*=1000
  for s in children:
   p=s.moved(matrix_location(M));b=p.bounding_box();items.append((link,s.label,p,np.array(list(b.min)),np.array(list(b.max))))
 for a,b in itertools.combinations(items,2):
  if a[0]==b[0]:continue
  if not np.all(np.minimum(a[4],b[4])-np.maximum(a[3],b[3])>.05):continue
  inter=a[2]&b[2];v=0 if inter is None else inter.volume
  if v>.1:hits.append({'parts':[a[1],b[1]],'links':[a[0],b[0]],'overlap_mm3':float(v)})
 rows.append({'pose':label,'angles':{n:q[n] for n in ['fl_hip_roll','fl_hip_pitch','fl_knee']},'intersections':hits})
 print(label,len(hits),json.dumps(sorted(hits,key=lambda x:-x['overlap_mm3'])[:3]),flush=True)
 (ROOT/'reports/v04_motion_audit.json').write_text(json.dumps({'scope':'One front-left leg: actual CAD parts across rigid bodies versus each other and adjacent torso; sampled configurations, not continuous CCD; same-link overlaps not checked','poses':rows},indent=2)+'\n')
import hashlib
report={'complete':True,'scope':'One front-left leg across rigid bodies and adjacent torso; sampled configurations, not continuous CCD; same-link overlaps not checked','method':'Exact CAD BRep intersection; threshold 0.1mm3','poses':rows,'source_sha256':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ['config/robot.yaml','simulation/model.py','cad/src/lib/shapes.py','cad/src/lib/sensors_face.py','cad/src/lib/actuation_v04.py']}}
(ROOT/'reports/v04_motion_audit.json').write_text(json.dumps(report,indent=2)+'\n')
print('COMPLETE',len(rows),flush=True)

if any(p["intersections"] for p in rows):raise SystemExit(1)

"""Reproducible v05 digital mechanical audit; see docs/v05.md for scope."""
from scripts.audit_inputs import snapshot
INPUTS=snapshot()
import json,itertools,time
import numpy as np,trimesh
from scripts.common import ROOT,load,joint_names
from simulation.model import graph,fk,home,joint_spec
c=load();links=graph(c);base=home(c);rows=json.loads((ROOT/'reports/collision_source.json').read_text());manager=trimesh.collision.CollisionManager();which={}
for i,row in enumerate(rows):
 mesh=trimesh.load(ROOT/row['path'],force='mesh');key=str(i);manager.add_object(key,mesh);which[key]=row
poses=[('home',base)]
for n in joint_names(c):
 for v in np.linspace(*joint_spec(n,c)['limits'],7):poses.append((f'{n}={v:.3f}',{**base,n:float(v)}))
for phase in np.linspace(0,2*np.pi,25):
 q=base.copy()
 for k,leg in enumerate(c['legs']):
  p=phase+([0,np.pi,np.pi,0][k]);q[leg+'_hip_pitch']+=.25*np.sin(p);q[leg+'_knee']+=.35*np.sin(p+.6);q[leg+'_hip_roll']+=.06*np.sin(p)
 poses.append((f'gait={phase:.3f}',q))
for kind,h,k in [('folded_crouch',-1.2,2.05),('near_goal_crouch',-.9,1.75)]:
 q=base.copy()
 for leg in c['legs']:q[leg+'_hip_pitch']=h;q[leg+'_knee']=k
 poses.append((kind,q))
# Coordinated abduction during recovery, including crossing-inward limits.
# Single-joint sweeps alone miss opposing shins meeting near the centreline.
for posture,h,k in [('standing',None,None),('folded',-1.2,2.05),('extended',-.6,1.2)]:
 for outward_degrees in (-6,0,15,30,45):
  for pattern,selected in [('all',tuple(c['legs'])),('left',('fl','rl')),('diagonal',('fl','rr'))]:
   q=base.copy()
   for leg,side in c['legs'].items():
    if h is not None:q[leg+'_hip_pitch']=h;q[leg+'_knee']=k
    if leg in selected:q[leg+'_hip_roll']=float(np.deg2rad(outward_degrees)*side['y_sign'])
   poses.append((f'recovery_abduction/{posture}/{pattern}/{outward_degrees}',q))
for i in range(100):
 rng=np.random.default_rng(505+i);q={n:float(rng.uniform(*joint_spec(n,c)['limits'])) for n in joint_names(c)};poses.append((f'random={i}',q))
result=[]
for label,q in poses:
 transforms=fk(links,q,c)
 for key,row in which.items():
  M=transforms[row['link']].copy();M[:3,3]*=1000;manager.set_transform(key,M)
 hit,pairs=manager.in_collision_internal(return_names=True)
 dynamic=[]
 for a,b in pairs:
  x,y=which[a],which[b]
  if x['link']==y['link']:continue
  dynamic.append([x['part'],y['part']])
 result.append({'pose':label,'q':q,'pairs':dynamic})
 if dynamic:print(label,len(dynamic),dynamic[:8],flush=True)
 (ROOT/'reports/fcl_motion_candidates.json').write_text(json.dumps({'complete':len(result)==len(poses),'source_sha256':INPUTS,'method':'triangle BVH FCL, touching counts as collision; exact BRep confirmation required','poses':result},indent=2)+'\n')
print('DONE',len(result),'hit poses',sum(bool(x['pairs']) for x in result),flush=True)

assert snapshot()==INPUTS, 'Source changed during audit'

"""Saved-artifact bounds, STEP round trips, and conservative motor-box sweeps.

Envelope interference is a warning, not a physical clearance certificate. The
motor envelopes include shafts/ears; mating hardware is deliberately excluded.
"""
import itertools,json,sys
import numpy as np
from cadgen import read_step
from scripts.common import ROOT,load
from simulation.model import graph,fk,home,joint_spec
from scripts.build_cad import NAMES,FITTED
sys.path.insert(0,str(ROOT/'cad/src'))
from lib.shapes import FACTORIES,box,matrix_location
from scripts.cad_snapshot import load_parts as link_components

def main():
 c=load();roundtrips=[];components=link_components();fitted={s.label:s for vv in components.values() for s in vv}
 for n in NAMES:
  original=fitted[FITTED[n]] if n in FITTED else FACTORIES[n]();saved=read_step(ROOT/'cad/STEP'/f'{n}.step')
  err=abs(saved.volume-original.volume)/original.volume
  roundtrips.append(dict(part=n,relative_volume_change=err,saved_solids=len(saved.solids())))
  assert err<.01 and len(saved.solids())==1,(n,err)
 a=read_step(ROOT/'cad/STEP/assembly.step');bounds=a.bounding_box()
 (ROOT/'reports/assembly_bounds.json').write_text(json.dumps({'units':'mm','min':list(bounds.min),'max':list(bounds.max),'dimensions':list(bounds.size),'valid':bool(a.is_valid),'step_roundtrips':roundtrips},indent=2)+'\n')
 # Only motor/electronics envelopes: excludes all intended clamp/shaft fits.
 parts=[]
 pitch=next(s for s in components['neck_yaw'] if s.label=='head_pitch_servo')
 centre_y=float(pitch.bounding_box().center().Y)
 assert abs(centre_y)<.001,'Head pitch case is laterally offset'
 for link,children in components.items():
  for s in children:
   if 'servo' not in s.label and 'envelope' not in s.label:continue
   b=s.bounding_box();parts.append((link,s.label,box(tuple(b.size),tuple(b.center()))))
 q=home(c);poses=[('home',q)]
 for name in q:
  for i,value in enumerate(np.linspace(*joint_spec(name,c)['limits'],7)):
   poses.append((f'{name}_{i}',{**q,name:float(value)}))
 warnings=[];cover_warnings=[]
 cowl=read_step(ROOT/'cad/STEP/body_shell.step')
 for label,angles in poses:
  transforms=fk(graph(c),angles,c);items=[]
  for link,n,s in parts:
   m=transforms[link].copy();m[:3,3]*=1000
   shape=s.moved(matrix_location(m));items.append((link,n,shape,shape.bounding_box()))
  if label=='home':
   cb=cowl.bounding_box()
   for link,n,s,b in items:
    if np.all(np.minimum(list(cb.max),list(b.max))-np.maximum(list(cb.min),list(b.min))>0):
     intersection=cowl&s;v=0 if intersection is None else intersection.volume
     if v>1:cover_warnings.append(dict(component=n,overlap_mm3=v))
  for (l,n,s,b),(l2,n2,s2,b2) in itertools.combinations(items,2):
   if l==l2 and label!='home':continue
   lo=np.maximum(list(b.min),list(b2.min));hi=np.minimum(list(b.max),list(b2.max))
   if not np.all(hi-lo>.15):continue
   intersection=s&s2;overlap=0 if intersection is None else intersection.volume
   if overlap>1: warnings.append(dict(pose=label,parts=[n,n2],conservative_box_overlap_mm3=overlap))
 result={'poses_sampled':len(poses),'envelopes':len(parts),'threshold_mm3':1,'warnings':warnings,'home_cowl_warnings':cover_warnings,'head_pitch_case_center_y_mm':centre_y,'foot_centre_track_mm':2*(c['geometry']['hip_y']+c['geometry']['abduction_offset']),'scope':'Conservative axis-aligned-in-link motor/electronics boxes transformed through FK; home cowl checked against these boxes. Full configured ranges are NOT mechanically certified. Clamp, horn, shell, cable and bearing fits require physical checks.'}
 (ROOT/'reports/clearance_screen.json').write_text(json.dumps(result,indent=2)+'\n')
 assert not any(w['pose']=='home' for w in warnings),'Home motor/electronics envelopes intersect'
 assert not cover_warnings,'Home motor/electronics envelope intersects the cowl'
 print(json.dumps({'assembly_dimensions_mm':list(bounds.size),'roundtrips_passed':len(roundtrips),'sweep_poses':len(poses),'envelope_warnings':len(warnings),'home_warnings':[x for x in warnings if x['pose']=='home']},indent=2))
if __name__=='__main__':main()

"""Exact home-pose motor versus structural-part intersections; deliberate clamps excluded."""
import sys,json,itertools
import numpy as np
from scripts.common import ROOT,load
sys.path.insert(0,str(ROOT/'cad/src'))
from lib.shapes import link_components,matrix_location
from simulation.model import graph,fk,home

def main():
 c=load();T=fk(graph(c),home(c),c);parts=link_components();items=[];rows=[];floor=[]
 for link,children in parts.items():
  M=T[link].copy();M[:3,3]*=1000
  for s in children:
   if link.startswith(('ear','head','neck')):continue
   if any(t in s.label for t in ['display','pigment','nostril']):continue
   p=s.moved(matrix_location(M))
   if p.bounding_box().min.Z+(c['geometry']['stance_height']+c['geometry']['foot_radius'])<-.05:floor.append({'part':s.label,'lowest_world_z_mm':p.bounding_box().min.Z+(c['geometry']['stance_height']+c['geometry']['foot_radius'])})
   items.append((link,s.label,p,p.bounding_box()))
 for (l,n,s,b),(l2,n2,s2,b2) in itertools.combinations(items,2):
  if not(n.endswith('_servo') or n2.endswith('_servo')):continue
  if l==l2 and any(t in n+n2 for t in ['cradle','strap']):continue
  if not np.all(np.minimum(list(b.max),list(b2.max))-np.maximum(list(b.min),list(b2.min))>.1):continue
  intersection=s&s2;v=intersection.volume if intersection is not None else 0
  if v>.1:rows.append({'parts':[n,n2],'links':[l,l2],'overlap_mm3':v});print(n,n2,round(v,3),flush=True)
 (ROOT/'reports/actuation_clearances.json').write_text(json.dumps({'pose':'home','method':'exact CAD BRep intersection; excludes intended clamp contacts','intersections':rows,'below_floor_at_home':floor},indent=2)+'\n')
 assert not floor,floor
 assert not rows,'Motor/structure intersections at home'
if __name__=='__main__':main()

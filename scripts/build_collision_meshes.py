"""Build CAD-derived convex colliders; fastener bores are conservatively filled.
Run in the optional collision-build environment. No learned control is run here.
"""
import os,json,hashlib,time
from pathlib import Path
import numpy as np,trimesh,coacd
from scripts.common import ROOT

def main():
 from scripts.cad_snapshot import geometry_signature
 source_inputs=json.loads((ROOT/'reports/collision_source_inputs.json').read_text());assert source_inputs.get('geometry_signature')==geometry_signature(), 'Collision source is stale'
 rows=json.loads((ROOT/'reports/collision_source.json').read_text());out=ROOT/'cad/collision/convex';out.mkdir(exist_ok=True);cache=ROOT/'.run-cache/coacd';cache.mkdir(exist_ok=True)
 params=dict(max_convex_hull=32,threshold=1.,real_metric=True,preprocess_mode='off',resolution=1500,mcts_nodes=10,mcts_iterations=40,mcts_max_depth=3,merge=True,seed=505)
 coacd.set_log_level('warn');records=[]
 for i,row in enumerate(rows):
  src=ROOT/row['canonical_path'];mesh=trimesh.load(src,force='mesh');placement=np.array(row['placement_mm']);part=row['part']
  # Catalogue servo assemblies are visual surface models; their full enclosing case is physical.
  if part in ['head_yaw_servo','head_pitch_servo']:
   mesh=trimesh.creation.box(extents=mesh.extents,transform=trimesh.transformations.translation_matrix(mesh.bounds.mean(0)))
  # A coarse hull cap bridges the ear recess and invents collisions at home.
  # Preserve the nominal concavity bound around articulated head interfaces.
  part_params={**params,'max_convex_hull':-1,'threshold':.75 if part=='pony_head_shell' else 1.} if part in ['pony_head_shell','body_cowl'] else params
  key=hashlib.sha256(mesh.vertices.tobytes()+mesh.faces.tobytes()+json.dumps(part_params,sort_keys=True).encode()).hexdigest();cp=cache/(key+'.npz')
  special=[]
  if part.startswith('ear_shell_'):
   side=-1 if part.endswith('_-1') else 1
   lobe=trimesh.creation.icosphere(subdivisions=3);lobe.vertices=lobe.vertices*np.array([8.08,13.13,24.24])+np.array([0,side*16,33])
   angles=np.linspace(0,2*np.pi,64,endpoint=False);base=np.c_[5.03*np.cos(angles),4.03*np.sin(angles),np.zeros(64)]
   stem=trimesh.convex.convex_hull(np.r_[base+np.array([0,-side*.02,-.02]),base+np.array([0,side*16.02,18.02])])
   C=trimesh.transformations.rotation_matrix(np.pi/2,[0,1,0]);root=trimesh.creation.cylinder(radius=6.03,height=12.04,sections=64,transform=C)
   special=[lobe.convex_hull,stem,root]
  elif part.endswith('_servo') and part not in ['head_yaw_servo','head_pitch_servo']:
   if part.startswith('micro_ear'):special=[trimesh.creation.box(extents=mesh.extents,transform=trimesh.transformations.translation_matrix(mesh.bounds.mean(0)))]
   else:
    # Reflection may be baked into a CAD solid rather than its placement.
    # Recover the actual case/rotor frame from this mesh, never assume axes.
    obb=mesh.bounding_box_oriented;dims=np.asarray(obb.primitive.extents);basis=np.asarray(obb.primitive.transform)[:3,:3];C=obb.primitive.transform[:3,3]
    long=basis[:,int(np.argmin(abs(dims-58.5)))];axial=basis[:,int(np.argmin(abs(dims-49.1)))];short=np.cross(long,axial)
    projection=(mesh.vertices-C)@axial;ends=mesh.vertices[np.abs(projection)>np.max(np.abs(projection))-.02];E=ends.mean(0);E+=axial*np.dot(C-E,axial)
    B=np.eye(4);B[:3,:3]=np.stack([long,axial,short],axis=1);B[:3,3]=C
    special=[trimesh.creation.box(extents=[58.5,44,33.5],transform=B)]
    for direction in [-1,1]:
     T=np.eye(4);T[:3,:3]=np.stack([long,-short,axial],axis=1);T[:3,3]=E+direction*23.275*axial
     special.append(trimesh.creation.cylinder(radius=16.03,height=2.55,sections=64,transform=T))
  elif 'M3_heads' in part or '_clamp_shims' in part:special=[m.convex_hull for m in mesh.split(only_watertight=False)]
  elif part in ['passive_tail','mane_crest','pink_mane_stripe']:
   actual=trimesh.load(ROOT/row['path'],force='mesh');actual.apply_transform(np.linalg.inv(placement));special=[actual.convex_hull]
  if special:hulls=[(h.vertices,h.faces) for h in special]
  elif cp.exists():
   data=np.load(cp);hulls=[(data[f'v{k}'],data[f'f{k}']) for k in range(int(data['n']))]
  else:
   if mesh.is_convex:hulls=[(mesh.vertices,mesh.faces)]
   else:hulls=coacd.run_coacd(coacd.Mesh(np.asarray(mesh.vertices),np.asarray(mesh.faces)),**part_params)
   values={'n':len(hulls)}
   for k,(v,f) in enumerate(hulls):values[f'v{k}']=v;values[f'f{k}']=f
   np.savez_compressed(cp,**values)
  # The filled shell keepouts omit positive mounting bosses that project
  # outside the smooth outer skin. Restore those drawing-defined features.
  bosses=[]
  if part=='body_cowl':
   for x in [-50,50]:
    for y in [-20,20]:bosses.append((4.,4.3,[x,y,-22.65]))
  elif part=='pony_head_shell':bosses.append((4.,10.,[40,0,111]))
  for radius,height,pos in bosses:
   T=np.eye(4);T[:3,3]=pos;h=trimesh.creation.cylinder(radius=radius+.03,height=height+.04,sections=64,transform=T);hulls.append((h.vertices,h.faces))
  if part=='pony_head_shell':
   # CoACD can sit inside a curved skin. Conservative Minkowski padding
   # covers its sampled surface; articulated clearances are rechecked below.
   import itertools
   corners=np.array(list(itertools.product([-.9,.9],repeat=3)));padded=[]
   for v,f in hulls:
    h=trimesh.convex.convex_hull((np.asarray(v)[:,None,:]+corners).reshape(-1,3));padded.append((h.vertices,h.faces))
   hulls=padded
  # Internal rotating couplings/bearings are constrained by the hinge. Their
  # near-zero running fits must not become approximate self-contact constraints.
  role='interface' if any(t in part for t in ['rotor_hub','rotor_spacer','passive_bearing','roll_axle','ear_output_adapter','passive_tail','mane_crest','pink_mane_stripe']) else 'structure'
  if part.endswith(('TPU_pad','floating_sole')):role='sole_visual' # combined sole/pad box is the native contact sensor geom
  files=[]
  for k,(v,f) in enumerate(hulls):
   vertices=np.c_[v,np.ones(len(v))]@placement.T;h=trimesh.Trimesh(vertices[:,:3],f,process=False);p=out/(row['link']+'__'+part+f'__{k}.stl');h.export(p);files.append(str(p.relative_to(ROOT)))
  records.append({'link':row['link'],'part':part,'role':role,'hulls':files,'source':row['path'],'params':part_params,'conservative_padding_cube_mm':.9 if part=='pony_head_shell' else 0.,'cache_key':key,'canonical_source_sha256':hashlib.sha256(src.read_bytes()).hexdigest()})
  print(i+1,len(rows),part,len(files),flush=True)
  (ROOT/'reports/collision_meshes.json').write_text(json.dumps({'method':'CoACD CAD-derived convex pieces, nominal 1mm concavity; fastener bores and selected sealed cavities filled conservatively','params':params,'parts':records,'complete':len(records)==len(rows)},indent=2)+'\n')
 active={ROOT/p for r in records for p in r['hulls']}
 for p in out.glob('*.stl'):
  if p not in active:p.unlink()
if __name__=='__main__':main()

"""Generate both descriptions from the user-authorized parametric configuration."""
import json, math, xml.etree.ElementTree as E
from scripts.common import ROOT, load, joint_names,total_mass
from simulation.model import graph,joint_spec,COLORS,home

def nums(v):return ' '.join(f'{float(x):.10g}' for x in v)
def sub(p,tag,**attrs):return E.SubElement(p,tag,{k:str(v) for k,v in attrs.items()})
def write(tree,path):E.indent(tree,space='  ');E.ElementTree(tree).write(path,encoding='unicode',xml_declaration=True)

def main():
 c=load();links=graph(c); a=c['actuator_candidates'][c['selection']['selected']]
 root=E.Element('mujoco',model=c['name'])
 sub(root,'compiler',angle='radian',inertiafromgeom='false',autolimits='true')
 sub(root,'option',timestep=1/c['control']['physics_hz'],gravity='0 0 -9.81',integrator='implicitfast',iterations=70)
 visual=sub(root,'visual');sub(visual,'global',offwidth=1200,offheight=900)
 default=sub(root,'default');sub(default,'joint',damping=c['control']['joint_viscous_friction'],armature='.00006',frictionloss=c['control']['joint_coulomb_friction'])
 sub(default,'geom',friction=f"{c['control']['friction_coefficient']} .015 .001",solref='.008 1',solimp='.95 .99 .001',condim=4)
 world=sub(root,'worldbody');sub(world,'light',pos='0 -1 2',dir='0 0 -1');sub(world,'light',pos='1 1 1.8',diffuse='.5 .5 .5')
 sub(world,'geom',name='floor',type='plane',size='5 5 .1',rgba='.87 .88 .91 1')
 asset=sub(root,'asset')
 mesh_ready=all((ROOT/'cad/STL/links'/f'{l["name"]}.msh').exists() for l in links)
 if mesh_ready:
  for l in links:sub(asset,'mesh',name=l['name']+'_visual',file=f'../../cad/STL/links/{l["name"]}.msh',scale='.001 .001 .001')
 bodies={}
 for l in links:
  origin=l['origin'] if l['parent'] else [0,0,(c['geometry']['stance_height']+c['geometry']['foot_radius'])/1000]
  b=sub(bodies[l['parent']] if l['parent'] else world,'body',name=l['name'],pos=nums(origin));bodies[l['name']]=b
  if not l['parent']:sub(b,'freejoint',name='root')
  if l['joint']:
   j=joint_spec(l['joint'],c);sub(b,'joint',name=l['joint'],type='hinge',axis=nums(j['axis']),range=nums(j['limits']))
  sub(b,'inertial',mass=l['mass'],pos=nums(l['com']),diaginertia=nums(l['inertia']))
  if mesh_ready:sub(b,'geom',name=l['name']+'_cad',type='mesh',mesh=l['name']+'_visual',contype=0,conaffinity=0,group=1,rgba=nums(COLORS['body']))
  for idx,g in enumerate(l['geoms']):
   kind=g['kind'];size=g['size'];attrs=dict(name=f"{l['name']}_g{idx}",type=kind,rgba=nums(COLORS[g['color']]),pos=nums(g['pos']))
   if kind=='box':size=[v/2 for v in size]
   if kind=='capsule':attrs.pop('pos');attrs['fromto']=nums(g['pos']+g['end']);size=size[:1]
   attrs['size']=nums(size)
   if mesh_ready:attrs['group']=3;attrs['rgba']='0 0 0 0'
   if not g['collision']:attrs.update(contype=0,conaffinity=0,group=1)
   sub(b,'geom',**attrs)
  if l['name'].endswith('_shin'):sub(b,'site',name=l['name'][:2]+'_foot',pos=f"0 0 {-c['geometry']['shin']/1000}",size='.003')
 actuators=sub(root,'actuator')
 for name in joint_names(c):sub(actuators,'motor',name=name,joint=name,gear=1,ctrllimited='true',ctrlrange=nums([-a['stall_torque'],a['stall_torque']]))
 key=sub(root,'keyframe');sub(key,'key',name='stand',qpos=nums([0,0,(c['geometry']['stance_height']+c['geometry']['foot_radius'])/1000,1,0,0,0]+list(home(c).values())))
 write(root,ROOT/'simulation/generated/sweetie_mini.xml')
 urdf=E.Element('robot',name=c['name'])
 urdf.append(E.Comment(''' DESIGN LEDGER: Generated from config/robot.yaml and simulation/model.py by explicit user instruction.
 Frames: metres, radians, +X forward/muzzle, +Y left, +Z up. Torso is floating base.
 Joint origins are expressed in parent frames; axes in joint frames; zero has straight downward legs.
 Knee positive bends feet toward -X. Same axes on all four legs. Left/right roll offsets change sign.
 Masses are engineering allocations, not measured. Diagonal inertias use box-envelope formulas at assigned COM.
 URDF visual meshes are CAD-generated link-local millimetres, scaled 0.001; collision primitives are SI.
 Ear and head collision envelopes are simplified; electronics and servo internals are not collision-resolved.
 Parametric regeneration is authoritative; hand edits to this generated file are overwritten. '''))
 for k,col in COLORS.items():m=sub(urdf,'material',name=k);sub(m,'color',rgba=nums(col))
 for l in links:
  b=sub(urdf,'link',name=l['name']+'_link');inertia=sub(b,'inertial');sub(inertia,'origin',xyz=nums(l['com']),rpy='0 0 0');sub(inertia,'mass',value=l['mass'])
  xx,yy,zz=l['inertia'];sub(inertia,'inertia',ixx=xx,iyy=yy,izz=zz,ixy=0,ixz=0,iyz=0)
  # CAD exports these meshes into link frames, independent of assembly pose.
  vis=sub(b,'visual');sub(vis,'origin',xyz='0 0 0',rpy='0 0 0');vg=sub(vis,'geometry');sub(vg,'mesh',filename=f'../../cad/STL/links/{l["name"]}.stl',scale='.001 .001 .001')
  sub(vis,'material',name='body')
  for g in (x for x in l['geoms'] if x['collision']):
   v=sub(b,'collision');kind=g['kind'];pos=g['pos'];size=g['size']
   if kind=='capsule':
    # Cylinder plus foot sphere already represented separately; URDF has no capsule primitive.
    pos=[(x+y)/2 for x,y in zip(pos,g['end'])]
   sub(v,'origin',xyz=nums(pos),rpy='0 0 0');geom=sub(v,'geometry')
   if kind=='box':sub(geom,'box',size=nums(size))
   elif kind=='sphere':sub(geom,'sphere',radius=size[0])
   elif kind=='capsule':sub(geom,'cylinder',radius=size[0],length=size[1])
   else:sub(geom,'box',size=nums([2*v for v in size]))
  if l['parent']:
   j=joint_spec(l['joint'],c);joint=sub(urdf,'joint',name=l['joint'],type='revolute');sub(joint,'parent',link=l['parent']+'_link');sub(joint,'child',link=l['name']+'_link');sub(joint,'origin',xyz=nums(l['origin']),rpy='0 0 0');sub(joint,'axis',xyz=nums(j['axis']));sub(joint,'limit',lower=j['limits'][0],upper=j['limits'][1],effort=a['stall_torque']*(c['power']['battery']['cutoff_voltage']/a['voltage'])*c['selection']['transient_fraction_of_stall'],velocity=a['no_load_speed']*(c['power']['battery']['cutoff_voltage']/a['voltage']));sub(joint,'dynamics',damping=c['control']['joint_viscous_friction'],friction=c['control']['joint_coulomb_friction'])
 write(urdf,ROOT/'simulation/generated/sweetie_mini.urdf')
 (ROOT/'reports/model.json').write_text(json.dumps({'links':links,'joint_names':joint_names(c),'home':home(c),'mass_kg':sum(l['mass'] for l in links),'inertia_method':'assigned COM + box-envelope diagonal; unmeasured'},indent=2)+'\n')
 assert math.isclose(sum(l['mass'] for l in links),total_mass(c),abs_tol=1e-12)
 print(f'Generated {len(links)} links, 16 joints, mass {total_mass(c):.3f} kg')
if __name__=='__main__':main()

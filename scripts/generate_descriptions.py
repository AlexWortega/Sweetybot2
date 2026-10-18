"""Generate both descriptions from the user-authorized parametric configuration."""
import json, math, xml.etree.ElementTree as E
import numpy as np
from scripts.common import ROOT, load, joint_names,total_mass,actuator
from simulation.model import graph,joint_spec,COLORS,home

def nums(v):return ' '.join(f'{float(x):.10g}' for x in v)
def sub(p,tag,**attrs):return E.SubElement(p,tag,{k:str(v) for k,v in attrs.items()})
def write(tree,path):E.indent(tree,space='  ');E.ElementTree(tree).write(path,encoding='unicode',xml_declaration=True)

def main():
 c=load();links=graph(c);collision=json.loads((ROOT/'reports/collision_meshes.json').read_text());assert collision['complete'];colliders={}
 for row in collision['parts']:colliders.setdefault(row['link'],[]).append(row)
 a=c['actuator_candidates'][c['selection']['selected']]
 root=E.Element('mujoco',model=c['name'])
 sub(root,'size',memory='64M')
 custom=sub(root,'custom');specs=[]
 for name in joint_names(c):
  motor=actuator(c,name);kp,kd=(18.,.35) if name.endswith(('hip_pitch','knee')) else (12.,.22) if name.endswith('hip_roll') else (4.,.08) if name=='head_pitch' else (2.,.06) if name=='head_yaw' else (.5,.01)
  specs += [motor['stall_torque'],motor.get('rated_torque',motor['stall_torque']*.25),motor['no_load_speed'],motor['voltage'],kp,kd,6. if name.startswith('ear_') else 0.,c['actuation']['thermal_time_constant_s']]
 sub(custom,'numeric',name='motor_specs',data=nums(specs))
 sub(custom,'numeric',name='tof_half_tangent',data=math.tan(math.radians(c['sensing']['depth']['diagonal_fov_deg']/2))/math.sqrt(2))
 sub(root,'compiler',angle='radian',inertiafromgeom='false',autolimits='true')
 option=sub(root,'option',timestep=1/c['control']['physics_hz'],gravity='0 0 -9.81',integrator='implicitfast',iterations=70);sub(option,'flag',filterparent='disable')
 visual=sub(root,'visual');sub(visual,'global',offwidth=1200,offheight=900)
 default=sub(root,'default');sub(default,'joint',damping=c['control']['joint_viscous_friction'],armature='.00006',frictionloss=c['control']['joint_coulomb_friction'])
 sub(default,'geom',friction=f"{c['control']['friction_coefficient']} .015 .001",solref='.008 1',solimp='.95 .99 .001',condim=4)
 world=sub(root,'worldbody');sub(world,'light',pos='0 -1 2',dir='0 0 -1');sub(world,'light',pos='1 1 1.8',diffuse='.5 .5 .5')
 sub(world,'geom',name='floor',type='plane',size='5 5 .1',rgba='.87 .88 .91 1',contype=1,conaffinity=7)
 asset=sub(root,'asset')
 for records in colliders.values():
  for row in records:
   if row['role']=='sole_visual':continue
   for k,path in enumerate(row['hulls']):sub(asset,'mesh',name=row['link']+'__'+row['part']+f'__{k}',file='../../'+path,scale='.001 .001 .001')
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
  I=np.array(l['inertia_matrix']);sub(b,'inertial',mass=l['mass'],pos=nums(l['com']),fullinertia=nums([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
  if mesh_ready:sub(b,'geom',name=l['name']+'_cad',type='mesh',mesh=l['name']+'_visual',contype=0,conaffinity=0,group=1,rgba=nums(COLORS['body']))
  for row in colliders.get(l['name'],[]):
   if row['role']=='sole_visual':continue
   for k,path in enumerate(row['hulls']):
    name=row['link']+'__'+row['part']+f'__{k}';interface=row['role']=='interface'
    sub(b,'geom',name=name,type='mesh',mesh=name,contype=2 if interface else 4,conaffinity=1 if interface else 5,group=0,rgba=nums(COLORS['frame']))
  if l['name'].endswith('_shin'):
   from simulation.model import rotation
   leg=l['name'][:2];angle=-joint_spec(leg+'_hip_pitch',c)['axis'][1]*(home(c)[leg+'_hip_pitch']+home(c)[leg+'_knee']);R=rotation([0,1,0],angle);o=np.array([0,0,-c['geometry']['shin']/1000])
   quat=nums([math.cos(angle/2),0,math.sin(angle/2),0])
   sub(b,'geom',name=leg+'_sole_contact',type='box',size='.015 .015 .0021',pos=nums(o+R@np.array([0,0,-.0189])),quat=quat,rgba=nums(COLORS['hoof']),contype=4,conaffinity=5,friction='1 .02 .001',solref='.004 1')
   sub(b,'site',name=leg+'_foot',pos=nums(o+R@np.array([0,0,-.021])),size='.002')
   sub(b,'site',name=leg+'_touch',type='box',pos=nums(o+R@np.array([0,0,-.0189])),quat=quat,size='.016 .016 .003',rgba='0 1 0 0')
 imu=c['sensing']['torso_imu'];sub(bodies['torso'],'site',name='torso_imu',pos=nums([v/1000 for v in imu['center_torso_mm']]),quat=nums(imu['mount_quaternion_wxyz']),size='.003')
 depth=c['sensing']['depth'];pitch=math.radians(depth['downward_pitch_deg']);axes=nums([0,-1,0,math.sin(pitch),0,math.cos(pitch)])
 sub(bodies['torso'],'site',name='chest_tof',pos=nums([v/1000 for v in depth['opening_torso_mm']]),xyaxes=axes,size='.003')
 sub(bodies['torso'],'camera',name='chest_tof_camera',pos=nums([v/1000 for v in depth['opening_torso_mm']]),xyaxes=axes,fovy=2*math.degrees(math.atan(math.tan(math.radians(depth['diagonal_fov_deg']/2))/math.sqrt(2))))
 camera=c['sensing']['camera'];sub(bodies['head'],'camera',name='forehead_rgb',pos=nums([v/1000 for v in camera['lens_axis_head_mm']]),xyaxes='0 -1 0 0 0 1',fovy=camera['fov_vertical_deg'])
 sensors=sub(root,'sensor');sub(sensors,'gyro',name='imu_gyro',site='torso_imu');sub(sensors,'accelerometer',name='imu_accel',site='torso_imu');sub(sensors,'framequat',name='imu_quat',objtype='site',objname='torso_imu')
 for leg in c['legs']:sub(sensors,'touch',name=leg+'_contact_force',site=leg+'_touch')
 actuators=sub(root,'actuator')
 for name in joint_names(c):
  motor=actuator(c,name);sub(actuators,'motor',name=name,joint=name,gear=1,ctrllimited='true',ctrlrange=nums([-motor['stall_torque']*.55,motor['stall_torque']*.55]))
 key=sub(root,'keyframe');sub(key,'key',name='stand',qpos=nums([0,0,(c['geometry']['stance_height']+c['geometry']['foot_radius'])/1000,1,0,0,0]+list(home(c).values())))
 write(root,ROOT/'simulation/generated/sweetie_mini.xml')
 urdf=E.Element('robot',name=c['name'])
 urdf.append(E.Comment(''' DESIGN LEDGER: Generated from config/robot.yaml and simulation/model.py by explicit user instruction.
 Frames: metres, radians, +X forward/muzzle, +Y left, +Z up. Torso is floating base.
 Joint origins are expressed in parent frames; axes in joint frames; zero has straight downward legs.
 Knee positive bends feet toward -X. Same axes on all four legs. Left/right roll offsets change sign.
 Masses/inertias derive from CAD BRep, catalogue masses and unweighed allowances; hardware internals are uniform envelopes.
 URDF visual meshes are CAD-generated link-local millimetres, scaled 0.001; collision primitives are SI.
 Ear and head collision envelopes are simplified; electronics and servo internals are not collision-resolved.
 Parametric regeneration is authoritative; hand edits to this generated file are overwritten. '''))
 for k,col in COLORS.items():m=sub(urdf,'material',name=k);sub(m,'color',rgba=nums(col))
 for l in links:
  b=sub(urdf,'link',name=l['name']+'_link');inertia=sub(b,'inertial');sub(inertia,'origin',xyz=nums(l['com']),rpy='0 0 0');sub(inertia,'mass',value=l['mass'])
  I=np.array(l['inertia_matrix']);sub(inertia,'inertia',ixx=I[0,0],iyy=I[1,1],izz=I[2,2],ixy=I[0,1],ixz=I[0,2],iyz=I[1,2])
  # CAD exports these meshes into link frames, independent of assembly pose.
  vis=sub(b,'visual');sub(vis,'origin',xyz='0 0 0',rpy='0 0 0');vg=sub(vis,'geometry');sub(vg,'mesh',filename=f'../../cad/STL/links/{l["name"]}.stl',scale='.001 .001 .001')
  sub(vis,'material',name='body')
  for row in colliders.get(l['name'],[]):
   if row['role']=='sole_visual':continue
   for path in row['hulls']:
    v=sub(b,'collision');sub(v,'origin',xyz='0 0 0',rpy='0 0 0');sub(sub(v,'geometry'),'mesh',filename='../../'+path,scale='.001 .001 .001')
  if l['name'].endswith('_shin'):
   angle=-(home(c)[l['name'][:2]+'_hip_pitch']+home(c)[l['name'][:2]+'_knee']);o=np.array([0,0,-c['geometry']['shin']/1000]);v=sub(b,'collision');sub(v,'origin',xyz=nums(o+rotation([0,1,0],angle)@np.array([0,0,-.0189])),rpy=nums([0,angle,0]));sub(sub(v,'geometry'),'box',size='.03 .03 .0042')
  if l['parent']:
   j=joint_spec(l['joint'],c);a=actuator(c,l['joint']);vf=(6. if l['joint'].startswith('ear_') else c['power']['battery']['cutoff_voltage'])/a['voltage'];joint=sub(urdf,'joint',name=l['joint'],type='revolute');sub(joint,'parent',link=l['parent']+'_link');sub(joint,'child',link=l['name']+'_link');sub(joint,'origin',xyz=nums(l['origin']),rpy='0 0 0');sub(joint,'axis',xyz=nums(j['axis']));sub(joint,'limit',lower=j['limits'][0],upper=j['limits'][1],effort=a['stall_torque']*min(1,vf)*c['selection']['transient_fraction_of_stall'],velocity=a['no_load_speed']*min(1,vf));sub(joint,'dynamics',damping=c['control']['joint_viscous_friction'],friction=c['control']['joint_coulomb_friction'])
 for name,parent,pos,rpy in [('torso_imu','torso',imu['center_torso_mm'],[0,0,0]),('chest_tof','torso',depth['opening_torso_mm'],[0,pitch,0]),('forehead_rgb','head',camera['lens_axis_head_mm'],[0,0,0])]:
  sub(urdf,'link',name=name+'_frame');j=sub(urdf,'joint',name=name+'_mount',type='fixed');sub(j,'parent',link=parent+'_link');sub(j,'child',link=name+'_frame');sub(j,'origin',xyz=nums([v/1000 for v in pos]),rpy=nums(rpy))
 write(urdf,ROOT/'simulation/generated/sweetie_mini.urdf')
 (ROOT/'reports/model.json').write_text(json.dumps({'links':links,'joint_names':joint_names(c),'home':home(c),'mass_kg':sum(l['mass'] for l in links),'inertia_method':'placed CAD BRep tensors + catalogue masses + unweighed allowances; unmeasured'},indent=2)+'\n')
 assert math.isclose(sum(l['mass'] for l in links),total_mass(c),abs_tol=1e-12)
 print(f'Generated {len(links)} links, 16 joints, mass {total_mass(c):.3f} kg')
if __name__=='__main__':main()

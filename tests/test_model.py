import hashlib,json,math,xml.etree.ElementTree as ET
import mujoco
import numpy as np
import pytest
from scripts.common import ROOT,load,total_mass,joint_names
from scripts.feasibility import screen
from simulation.model import graph,fk,home,ik,joint_spec
from simulation.control_contract import JointCommand,Supervisor,validate

@pytest.fixture(scope='module')
def c():return load()
@pytest.fixture(scope='module')
def model():return mujoco.MjModel.from_xml_path(str(ROOT/'simulation/generated/sweetie_mini.xml'))

def test_ids_and_power_compatibility(c):
 ids=[i for l in c['legs'].values() for i in l['ids']]+list(c['expression_ids'].values())
 assert sorted(ids)==list(range(1,17));assert len(set(joint_names(c)))==16
 lo,hi=c['actuator_candidates'][c['selection']['selected']]['voltage_range']
 assert lo<=c['power']['battery']['cutoff_voltage']<c['power']['battery']['max_voltage']<=hi
 assert c['power']['battery']['chemistry']=='LiFePO4'

def test_forward_kinematics_against_engine(c,model):
 rng=np.random.default_rng(472);d=mujoco.MjData(model);names=joint_names(c)
 for _ in range(60):
  q={n:rng.uniform(*joint_spec(n,c)['limits']) for n in names}
  mujoco.mj_resetDataKeyframe(model,d,0)
  d.qpos[7:]=[q[n] for n in names];mujoco.mj_forward(model,d)
  transforms=fk(graph(c),q,c)
  for leg in c['legs']:
   pt=transforms[leg+'_shin']@np.array([0,0,-c['geometry']['shin']/1000,1])
   world=pt[:3]+d.qpos[:3]
   np.testing.assert_allclose(world,d.site_xpos[model.site(leg+'_foot').id],atol=1e-9)

def test_ik_roundtrip(c):
 rng=np.random.default_rng(291);g=c['geometry']
 for side in [-1,1]:
  for _ in range(100):
   q=rng.uniform([-.25,-.9,.3],[.25,-.2,1.7]);vals=home(c)
   vals.update(dict(zip(['fl_hip_roll','fl_hip_pitch','fl_knee'],q)))
   cc=json.loads(json.dumps(c));cc['legs']['fl']['y_sign']=side
   trans=fk(graph(cc),vals,cc)
   pt=trans['fl_shin']@np.array([0,0,-g['shin']/1000,1]);pt[:3]-=np.array([g['hip_x']/1000,side*g['hip_y']/1000,0])
   np.testing.assert_allclose(ik(pt[:3],side,c),q,atol=1e-9)

def test_mass_inertia_parity(c,model):
 assert model.njnt==17;assert model.nu==16
 assert math.isclose(model.body_mass.sum(),total_mass(c),abs_tol=1e-9)
 root=ET.parse(ROOT/'simulation/generated/sweetie_mini.urdf').getroot()
 for link in graph(c):
  u=root.find(f"link[@name='{link['name']}_link']/inertial")
  assert math.isclose(float(u.find('mass').attrib['value']),model.body(link['name']).mass[0],abs_tol=1e-10)
  diag=[float(u.find('inertia').attrib[k]) for k in ['ixx','iyy','izz']]
  np.testing.assert_allclose(diag,model.body(link['name']).inertia,atol=1e-12)
  assert max(diag)<=sum(diag)-max(diag)+1e-12

def test_reference_checksums():
 manifest=json.loads((ROOT/'references/upstream/manifest.json').read_text())
 for name,repo in manifest.items():
  assert len(repo['commit'])==40
  for f in repo['files']:
   assert 'error' not in f
   assert hashlib.sha256((ROOT/'references/upstream'/name/f['path']).read_bytes()).hexdigest()==f['sha256']
 catalog=json.loads((ROOT/'references/catalog_parts.json').read_text())
 for key,part in catalog.items():assert hashlib.sha256((ROOT/'cad/STEP/imported'/f'{key}.step').read_bytes()).hexdigest()==part['sha256']

def test_screening_uses_complete_mass_and_selects_affordable_feasible(c):
 candidates=screen(c);passing=sorted([r for r in candidates if r['passes_screen']],key=lambda x:(x['actuator_cost_usd'],x['mass_kg']))
 assert passing and passing[0]['candidate']==c['selection']['selected']
 assert passing[0]['design_mass_kg']==pytest.approx(total_mass(c)*1.2)
 assert passing[0]['conditional_sustained_allowance_nm']>passing[0]['sustained_requirement_nm']

def test_command_gate(c):
 q=home(c);s=Supervisor(c);command=JointCommand(1,1.,q)
 with pytest.raises(ValueError):s.accept(command,1.)
 s.enable(True);s.accept(command,1.);assert s.tick(1.1)['motor_write_permitted']
 with pytest.raises(ValueError):s.accept(command,1.1)
 assert s.tick(1.26)['state']=='fault'
 with pytest.raises(ValueError):s.enable(True)
 s.clear_fault();assert s.tick(2.)['state']=='disabled'
 bad=dict(q);bad['fl_knee']=float('nan')
 with pytest.raises(ValueError):validate(JointCommand(1,2.,bad),c)
 with pytest.raises(ValueError):validate(JointCommand(1,2.,{**q,'unknown':0}),c)
 bad=dict(q);bad['head_yaw']=99
 with pytest.raises(ValueError):validate(JointCommand(1,2.,bad),c)

def test_clock_and_sensor_faults(c):
 for clock in [float('nan'),float('inf')]:
  s=Supervisor(c);s.enable(True)
  with pytest.raises(ValueError):s.accept(JointCommand(1,1.,home(c)),clock)
  assert not s.tick(1.)['motor_write_permitted']
 for now,fresh in [(.9,True),(1.1,False),(float('nan'),True)]:
  s=Supervisor(c);s.enable(True);s.accept(JointCommand(1,1.,home(c)),1.)
  assert s.tick(now,fresh)['state']=='fault'

def test_mesh_conversion_preserves_geometry():
 from mujoco.msh2obj import Msh
 for row in json.loads((ROOT/'reports/link_meshes.json').read_text()):
  path=ROOT/row['path']
  triangles=np.fromfile(path,dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')]),offset=84)
  mesh=Msh.create(ROOT/row['mujoco_path'])
  assert len(mesh.face_vertex_indices)==len(triangles)==row['triangles']
  np.testing.assert_allclose(mesh.vertex_positions.min(axis=0),triangles['v'].min(axis=(0,1)),atol=1e-5)
  np.testing.assert_allclose(mesh.vertex_positions.max(axis=0),triangles['v'].max(axis=(0,1)),atol=1e-5)

def test_requested_narrow_stance_and_centred_neck():
 b=json.loads((ROOT/'reports/assembly_bounds.json').read_text())
 g=json.loads((ROOT/'reports/clearance_screen.json').read_text())
 assert b['dimensions'][1]<=180
 assert g['foot_centre_track_mm']==140
 assert abs(g['head_pitch_case_center_y_mm'])<.001
 assert not g['home_cowl_warnings']

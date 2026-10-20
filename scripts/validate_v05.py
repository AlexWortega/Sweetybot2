"""Full collision/sensor digital acceptance; no learned-capability claims."""
import hashlib,json
import numpy as np
import mujoco
from scripts.common import ROOT,load,total_mass,continuous_limits,require_mechanical_gate,joint_names
from learning.env import SweetieVecEnv,make_model
from learning.readiness import dynamics_signature,DYNAMICS_FILES

def main():
 require_mechanical_gate();rows=[]
 # Every physical structure remains active, including parent-child brackets.
 path,_=make_model();m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m)
 candidates=json.loads((ROOT/'reports/fcl_motion_candidates.json').read_text());failures=[]
 for row in candidates['poses']:
  if row['pose'].startswith('random'):continue
  mujoco.mj_resetDataKeyframe(m,d,0);d.qpos[2]=2.;d.qpos[7:]=[row['q'][n] for n in joint_names(load())];mujoco.mj_forward(m,d)
  hits=[]
  for contact in d.contact:
   if contact.dist<-.00001 and m.geom_bodyid[contact.geom1]>0 and m.geom_bodyid[contact.geom2]>0:
    hits.append({'geoms':[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,g) for g in [contact.geom1,contact.geom2]],'penetration_mm':-contact.dist*1000})
  if hits:failures.append({'pose':row['pose'],'hits':hits})
 rows.append({'scenario':'CAD-derived MuJoCo self-collision proxies','poses':sum(not x['pose'].startswith('random') for x in candidates['poses']),'pass':not failures,'failures':failures})
 if failures:
  (ROOT/'reports/v05_validation.json').write_text(json.dumps({'all_passed':False,'cases':rows},indent=2)+'\n');raise ValueError('Collision proxies invent operational interferences; refine them before training')
 for terrain in ['flat','rough','slope','steps']:
  e=SweetieVecEnv(num_envs=4,threads=1,randomize=False,auto_reset=False,terrain=terrain,difficulty=1.,seed=1_000_091)
  e.set_command(0,0,0);peak=np.zeros(16);rms2=np.zeros((4,16));failed=np.zeros(4,bool);self_count=np.zeros(4,int)
  for _ in range(1000):
   o,r,done,info=e.step(np.zeros((4,12)));failed|=done&~info['truncated'];peak=np.maximum(peak,e.state['torque_peak'].max(0));rms2+=e.state['torque_rms']**2;self_count+=e.state['self_contacts']
   assert np.isfinite(o).all() and np.isfinite(r).all()
  rms=np.sqrt(rms2/1000).max(0);valid=e.sensors.frame();passed=bool((e.rot[:,2,2]>.9).all() and (e.height>.8*e.nominal_height).all() and not failed.any() and not e.state['bad_contacts'].any() and not self_count.any() and (rms<=continuous_limits(load())).all())
  rows.append({'scenario':'zero-action home target stand','terrain':terrain,'episodes':4,'seconds':20,'pass':passed,'heights_m':e.height.tolist(),'up_z':e.rot[:,2,2].tolist(),'rms_torque_nm':rms.tolist(),'peak_torque_nm':peak.tolist(),'self_contact_ticks':self_count.tolist(),'contact_valid':valid[4].tolist(),'depth_valid_pixels':valid[8].sum(1).tolist(),'motor_heat_max':float(e.state['motor_heat'].max())})
 e=SweetieVecEnv(task='recovery',num_envs=1,threads=1,randomize=False,auto_reset=False)
 resets=[]
 for pose in e.reset_kind_names:
  e.reset_pose(pose);resets.append({'pose':pose,'self_contacts':int(e.state['self_contacts'][0]),'root_height_m':float(e.height[0])})
 rows.append({'scenario':'all five recovery reset kinds start clear','pass':all(r['self_contacts']==0 for r in resets),'resets':resets})
 e.reset_pose('stand');q=e.state['qpos'].copy();q[:,2]=1.5;e.native.reset(np.array([0]),q,np.zeros((1,22)),e.params);e._read_state()
 for _ in range(5):e.step(np.zeros((1,12)))
 rows.append({'scenario':'unsupported freefall and real IMU specific force','pass':bool(np.linalg.norm(e.sensors.accel[0])<.3 and e.state['qvel'][0,2]<-.8),'specific_force_norm_m_s2':float(np.linalg.norm(e.sensors.accel[0])),'vertical_velocity_m_s':float(e.state['qvel'][0,2])})
 compact=SweetieVecEnv(num_envs=2,threads=1,randomize=False,auto_reset=False,observation_mode='compact')
 compact.set_command(0,0,0)
 for _ in range(100):
  actor,_,_,_=compact.step(np.zeros((2,12)));critic=compact.observe_critic()
  assert actor.shape==(2,63) and critic.shape==(2,81) and np.isfinite(actor).all() and np.isfinite(critic).all()
 rows.append({'scenario':'compact actor and privileged critic neutral stand','pass':bool((compact.rot[:,2,2]>.9).all() and not compact.state['self_contacts'].any()),'actor_dimension':compact.obs_dim,'critic_dimension':compact.critic_obs_dim})
 report={'revision':load()['revision'],'nominal_mass_kg':total_mass(load()),'observation_dimension':compact.obs_dim,'observation_contract':compact.observation_contract,'critic_observation_dimension':compact.critic_obs_dim,'sensor_transport_frame_dimension':197,'legacy_policy_dimension':591,'reward_version':e.reward_version,'all_passed':all(r['pass'] for r in rows),'cases':rows,'input_sha256':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in DYNAMICS_FILES},'dynamics_signature':dynamics_signature(),'policy_quality':'No trained compact policy evaluated; neutral targets and physics checks only','physical_validation':'not run','root_support_forces':False,'mid_episode_resets':False}
 (ROOT/'reports/v05_validation.json').write_text(json.dumps(report,indent=2)+'\n')
 ready={'passed':report['all_passed'],'dynamics_signature':report['dynamics_signature'],'reason':'Digital collision/sensor checks passed; new task policies still require training and independent evaluation' if report['all_passed'] else 'Digital validation failed','report':'reports/v05_validation.json'}
 (ROOT/'reports/rl_readiness.json').write_text(json.dumps(ready,indent=2)+'\n');print(json.dumps(ready,indent=2))
 if not report['all_passed']:raise SystemExit(1)
if __name__=='__main__':main()

"""Digital build validation, without claiming learned or physical capability."""
import hashlib,json
import numpy as np
from scripts.common import ROOT,load,total_mass,continuous_limits
from learning.env import SweetieVecEnv

def main():
 rows=[]
 for terrain in ['flat','rough','slope','steps']:
  e=SweetieVecEnv(num_envs=4,threads=1,randomize=False,auto_reset=False,terrain=terrain,difficulty=1.,seed=1_000_091)
  e.set_command(0,0,0);peak=np.zeros(16);rms2=np.zeros((4,16));frames=[];failed=np.zeros(4,bool)
  for _ in range(1000):
   o,r,done,info=e.step(np.zeros((4,12)));failed|=done&~info['truncated'];peak=np.maximum(peak,e.state['torque_peak'].max(0));rms2+=e.state['torque_rms']**2
   assert np.isfinite(o).all() and np.isfinite(r).all()
  rms=np.sqrt(rms2/1000).max(0);valid=e.sensors.frame();passed=bool((e.rot[:,2,2]>.9).all() and (e.height>.8*e.nominal_height).all() and not failed.any() and not e.state['bad_contacts'].any() and (rms<=continuous_limits(load())).all())
  rows.append({'scenario':'zero-action home target stand','terrain':terrain,'episodes':4,'seconds':20,'pass':passed,'heights_m':e.height.tolist(),'up_z':e.rot[:,2,2].tolist(),'rms_torque_nm':rms.tolist(),'peak_torque_nm':peak.tolist(),'contact_valid':valid[4].tolist(),'depth_valid_pixels':valid[8].sum(1).tolist(),'motor_heat_max':float(e.state['motor_heat'].max())})
 # Freefall distinguishes real IMU acceleration from truth-orientation substitutes.
 e=SweetieVecEnv(num_envs=1,threads=1,randomize=False,auto_reset=False);q=e.state['qpos'].copy();q[:,2]=1.5;e.native.reset(np.array([0]),q,np.zeros((1,22)),e.params);e._read_state()
 for _ in range(5):e.step(np.zeros((1,12)))
 assert np.linalg.norm(e.sensors.accel[0])<.3 and e.state['qvel'][0,2]<-.8
 rows.append({'scenario':'unsupported freefall','pass':True,'specific_force_norm_m_s2':float(np.linalg.norm(e.sensors.accel[0])),'vertical_velocity_m_s':float(e.state['qvel'][0,2])})
 inputs=['config/robot.yaml','simulation/model.py','simulation/generated/sweetie_mini.xml','learning/env.py','learning/native.cpp','learning/rewards.py','learning/sensors.py','hardware/observation.py']
 report={'revision':load()['revision'],'nominal_mass_kg':total_mass(load()),'observation_dimension':591,'observation_contract':e.observation_contract,'reward_version':e.reward_version,'all_passed':all(r['pass'] for r in rows),'cases':rows,'input_sha256':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in inputs},'policy_quality':'No trained v04 policy evaluated; zero-action checks only','physical_validation':'not run','root_support_forces':False,'mid_episode_resets':False}
 (ROOT/'reports/v04_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ['cases','input_sha256']},indent=2))
 if not report['all_passed']:raise SystemExit(1)
if __name__=='__main__':main()

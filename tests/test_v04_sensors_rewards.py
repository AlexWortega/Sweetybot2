"""Counterexamples for v04 sensor realism and reward shortcuts."""
import numpy as np
import pytest
from learning.sensors import SensorSuite,gravity_up
from learning.rewards import locomotion_terms,recovery_terms,recovery_potential

def samples(force=10.):
 return {'imu_gyro_samples':np.zeros((1,10,3)),'imu_accel_samples':np.tile([0.,0,9.81],(1,10,1)),
 'foot_force_samples':np.full((1,10,4),force),'depth_ranges':np.full((1,64),.8),'depth_age':np.array([.01])}

def test_contact_switch_threshold_debounce_fault_and_release():
 s=SensorSuite(1,randomize=False);s.reset([0],[[1,0,0,0]])
 s.update(samples(1.));assert not s.contacts.any();assert s.contact_valid.all()
 s.update(samples(3.));assert s.contacts.all()
 s.force_fault[0,1]=True;s.update(samples(3.));assert s.contacts[0,1]==0 and s.contact_valid[0,1]==0
 s.update(samples(0.));assert not s.contacts.any()

def test_imu_specific_force_freefall_and_stale_validity():
 s=SensorSuite(1,randomize=False);s.reset([0],[[1,0,0,0]]);s.update(samples())
 np.testing.assert_allclose(s.frame()[2],[[0,0,1]])
 free=samples();free['imu_accel_samples'][:]=0;s.update(free)
 np.testing.assert_allclose(s.frame()[2],0);np.testing.assert_allclose(s.frame()[0],[[0,0,-1]])
 bad=samples();bad['imu_accel_samples'][:]=np.nan
 s.update(bad);s.update(bad);f=s.frame();assert not f[5].any();assert np.isfinite(f[0]).all()

def test_imu_gyro_rotation_and_accelerometer_tilt_correction():
 s=SensorSuite(1,randomize=False);s.reset([0],[[1,0,0,0]]);raw=samples();raw['imu_accel_samples'][:]=0;raw['imu_gyro_samples'][:,:,0]=1.
 for _ in range(50):s.update(raw)
 np.testing.assert_allclose(gravity_up(s.q)[0],[0,np.sin(1),np.cos(1)],atol=.01)
 # A small wrong roll estimate must converge with stationary gravity.
 s.reset([0],[[np.cos(.1),np.sin(.1),0,0]])
 for _ in range(200):s.update(samples())
 assert abs(gravity_up(s.q)[0,1])<.01

def test_tof_missing_and_stale_returns_carry_health_masks():
 s=SensorSuite(1,randomize=False);s.reset([0],[[1,0,0,0]]);raw=samples();raw['depth_ranges'][0,:3]=[-1,np.nan,5.];s.update(raw)
 f=s.frame();assert (f[8][0,:3]==0).all();assert (f[7][0,:3]==1).all()
 raw['depth_age'][:]=.21;s.update(raw);assert not s.frame()[8].any()

def reward_args():
 return dict(command=np.array([[.5,0,0]]),velocity=np.array([[.5,0,0]+[0]*12]),omega=np.zeros((1,3)),up=np.ones(1),height=np.array([.241]),nominal_height=.241,
 torque=np.zeros((1,16)),continuous=np.ones((1,16)),actions=np.zeros((1,12)),previous_actions=np.zeros((1,12)),joints=np.zeros((1,12)),home=np.zeros(12),contacts=np.zeros((1,4),bool),foot_velocity=np.zeros((1,4,3)),bad_contacts=np.zeros(1),task='running',flight_seconds=np.array([.04]),previous_flight=np.array([.02]))

@pytest.mark.parametrize('shortcut',['still','no_command','jump','wrong_height','tilted','mistracking','repeat'])
def test_running_flight_event_rejects_shortcuts(shortcut):
 a=reward_args();assert locomotion_terms(**a)['forward_flight_event'][0]>0
 if shortcut=='still':a['velocity'][0,0]=0
 if shortcut=='no_command':a['command'][:]=0
 if shortcut=='jump':a['velocity'][0,2]=.8
 if shortcut=='wrong_height':a['height'][:]=.4
 if shortcut=='tilted':a['up'][:]=.5
 if shortcut=='mistracking':a['velocity'][0,1]=.3
 if shortcut=='repeat':a['previous_flight'][:]=.04
 assert locomotion_terms(**a)['forward_flight_event'][0]==0

def test_slip_overload_and_energy_reduce_reward():
 a=reward_args();base=locomotion_terms(**a);a['contacts'][:]=True;a['foot_velocity'][:,:,:2]=1;a['torque'][:]=2.;a['velocity'][:,3:]=1
 bad=locomotion_terms(**a)
 for name in ['foot_slip','overload','mechanical_power']:assert bad[name][0]<base[name][0]

def test_inverted_stationary_recovery_cannot_farm_positive_reward():
 up=np.array([-1.]);height=np.array([.08]);pose=np.ones(1);phi=recovery_potential(up,height,.241,pose)
 terms,_=recovery_terms(up,height,.241,pose,phi,np.zeros(1),np.zeros(1,bool),np.zeros((1,12)),np.zeros((1,12)),np.zeros((1,16)),np.ones((1,16)),np.ones(1))
 assert sum(terms.values())[0]<0

def test_old_checkpoint_contract_is_rejected_even_if_dimensions_are_forged():
 from learning.ppo import validate_environment_contract
 class E:observation_contract='v04-sensor-suite-1'
 with pytest.raises(ValueError):validate_environment_contract({'obs_dim':591},E())
 from learning.readiness import dynamics_signature
 validate_environment_contract({'environment_metadata':{'observation_contract':E.observation_contract,'dynamics_signature':dynamics_signature()}},E())

def test_collector_crc_reordering_and_broken_contact_wire_are_invalid():
 from hardware.telemetry import Decoder,pack
 d=Decoder()
 def packet(seq,us,adc):return pack(seq,us,[0,0,0],[0,0,9.81],adc)
 p=packet(0,1000,[2048]*4);d.decode(p)
 for seq in [1,2,3]:r=d.decode(packet(seq,1000+seq*5000,[2048]*4))
 assert r['contact_valid']==[1]*4 and r['contact']==[0]*4
 with pytest.raises(ValueError):d.decode(packet(3,20000,[2048]*4))
 bad=bytearray(packet(4,21000,[2048]*4));bad[12]^=1
 with pytest.raises(ValueError):d.decode(bytes(bad))
 r=d.decode(packet(4,21000,[4095,0,2048,2048]));assert r['contact_valid']==[0,0,1,1]
 assert d.contact_state(.05)==([0]*4,[0]*4)

def test_actor_shared_hardware_encoder_matches_simulation_layout():
 from hardware.observation import encode,SLICES
 s=SensorSuite(1,randomize=False);s.reset([0],[[1,0,0,0]]);s.update(samples())
 frame=encode(s.frame(),np.ones((1,16)),np.full((1,16),2.),np.zeros((1,12)),np.array([[.2,0,.3]]),np.array([0.]))
 assert frame.shape==(1,197)
 np.testing.assert_allclose(frame[:,22:38],.2);np.testing.assert_allclose(frame[:,50:53],[[.2,0,.3]])
 assert SLICES['depth_valid']==(132,196)

def test_native_tof_center_rays_match_floor_geometry_and_extrinsics():
 import mujoco
 from learning.env import SweetieVecEnv
 e=SweetieVecEnv(num_envs=1,threads=1,randomize=False,auto_reset=False)
 model=e._reset_model;d=e._reset_data;d.qpos[:]=e.state['qpos'][0];mujoco.mj_forward(model,d)
 site=model.site('chest_tof').id;R=d.site_xmat[site].reshape(3,3);origin=d.site_xpos[site]
 half=np.tan(np.deg2rad(65/2))/np.sqrt(2)
 for row,col in [(3,3),(3,4),(4,3),(4,4)]:
  local=np.array([((col+.5)/4-1)*half,(1-(row+.5)/4)*half,-1.]);ray=R@local;ray/=np.linalg.norm(ray)
  expected=-origin[2]/ray[2]
  assert e.state['depth_ranges'][0,row*8+col]==pytest.approx(expected,abs=1e-8)
 assert origin[0]==pytest.approx(.135)


def test_recovery_requires_feet_support_and_rejects_body_bracing():
 from learning.env import SweetieVecEnv
 e=SweetieVecEnv(task='recovery',num_envs=1,threads=1,randomize=False,auto_reset=False)
 e.reset_pose('supine')
 for _ in range(40):e.step(np.zeros((1,12)))
 assert not e.success[0]
 e.reset_pose('stand')
 for _ in range(40):e.step(np.clip((e.home[:12]-e.state['qpos'][:,7:19])/.35,-1,1))
 assert e.success[0] and e.stable[0]>=35

def test_tof_frame_is_held_between_15hz_measurements():
 s=SensorSuite(1,seed=7,randomize=True);s.reset([0],[[1,0,0,0]]);raw=samples();s.update(raw);first=s.depth.copy()
 raw['depth_age'][:]=.03;s.update(raw);np.testing.assert_array_equal(s.depth,first)
 raw['depth_age'][:]=.001;s.update(raw);assert np.any(s.depth!=first)

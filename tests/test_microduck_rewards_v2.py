"""Counterexamples for stepping, velocity goals, dense get-up and episode state."""
import numpy as np
import pytest
from learning.rewards import FootMotion,RiseMotion,locomotion_terms,recovery_terms,recovery_potential,joint_limit_cost
from learning.curriculum import recovery_progress_cap


def walking_args():
 return dict(command=np.array([[.2,0,0.]]),velocity=np.array([[.2,0,0.]+[0.]*12]),omega=np.zeros((1,3)),
  up=np.ones(1),height=np.array([.296]),nominal_height=.296,torque=np.zeros((1,16)),continuous=np.ones((1,16)),
  actions=np.zeros((1,12)),previous_actions=np.zeros((1,12)),joints=np.zeros((1,12)),home=np.zeros(12),
  contacts=np.array([[1,0,1,0]],bool),foot_velocity=np.tile([.15,0,0.],(1,4,1)),bad_contacts=np.zeros(1),
  task='locomotion',flight_seconds=np.zeros(1),previous_flight=np.zeros(1),self_contacts=np.zeros(1),
  gait=dict(landing_air_time=np.array([[.2,0,.2,0]]),landing_peak=np.full((1,4),.025),clearance=np.full((1,4),.025),dt=.02))


@pytest.mark.parametrize('speed', [0., .05, -.2, .6])
def test_commanded_speed_beats_still_slow_wrong_direction_and_overspeed(speed):
 a=walking_args();good=sum(locomotion_terms(**a).values())[0]
 a['velocity'][0,0]=speed
 assert sum(locomotion_terms(**a).values())[0]<good
 if speed in (0.,.05):assert locomotion_terms(**a)['under_speed'][0]<0
 if speed==.6:assert locomotion_terms(**a)['over_speed'][0]<0


def test_directional_speed_and_yaw_rewards_match_sideways_and_turn_commands():
 a=walking_args();a['command'][:]=[0,.2,0];a['velocity'][0,:3]=[0,.2,0]
 good=sum(locomotion_terms(**a).values())[0]
 a['velocity'][0,:3]=[.2,0,0]
 assert sum(locomotion_terms(**a).values())[0]<good
 a=walking_args();a['command'][:]=[0,0,.5];a['velocity'][:]=0
 a['omega'][:]=[0,0,.5];turn=locomotion_terms(**a)
 a['omega'][:]=0;still=locomotion_terms(**a)
 assert turn['yaw_progress'][0]>still['yaw_progress'][0]
 assert sum(turn.values())[0]>sum(still.values())[0]


def test_small_velocity_commands_are_not_punished_for_exact_tracking():
 a=walking_args();a['command'][:]=[0,.06,0];a['velocity'][0,:3]=[0,.06,0]
 terms=locomotion_terms(**a)
 assert terms['under_speed'][0]==pytest.approx(0.)
 assert terms['over_speed'][0]==pytest.approx(0.)
 assert terms['command_progress'][0]==pytest.approx(2.)


@pytest.mark.parametrize('shortcut',['idle','still','backward','body_support','self_contact','inverted'])
def test_foot_rewards_require_actual_commanded_motion_and_clean_posture(shortcut):
 a=walking_args();good=locomotion_terms(**a)
 for name in ('foot_air_time','foot_clearance','foot_swing_height'):assert good[name][0]>0
 if shortcut=='idle':a['command'][:]=0
 if shortcut=='still':a['velocity'][:]=0
 if shortcut=='backward':a['velocity'][0,0]=-.2
 if shortcut=='body_support':a['bad_contacts'][:]=1
 if shortcut=='self_contact':a['self_contacts'][:]=1
 if shortcut=='inverted':a['up'][:]=-1
 for name in ('foot_air_time','foot_clearance','foot_swing_height'):
  assert locomotion_terms(**a)[name][0]==0


def test_clearance_prefers_small_useful_swing_over_dragging_and_high_flailing():
 a=walking_args();good=locomotion_terms(**a)
 a['gait']['clearance'][:]=0;drag=locomotion_terms(**a)
 assert drag['foot_clearance'][0]<good['foot_clearance'][0]
 assert drag['foot_drag'][0]<good['foot_drag'][0]
 a['gait']['clearance'][:]=.15;high=locomotion_terms(**a)
 assert high['foot_clearance'][0]<good['foot_clearance'][0] and high['excess_foot_lift'][0]<0


def test_air_time_is_a_single_landing_event_and_reset_cannot_create_one():
 f=FootMotion(2);ground=np.ones((2,4),bool);height=np.zeros((2,4))
 # Initial free fall is not a learned step.
 for _ in range(10):f.update(~ground,np.full((2,4),.03))
 assert not f.update(ground,height)['landing_air_time'].any()
 for _ in range(10):f.update(~ground,np.full((2,4),.025))
 landing=f.update(ground,height)
 np.testing.assert_allclose(landing['landing_air_time'],.2)
 assert not f.update(ground,height)['landing_air_time'].any()
 for _ in range(10):f.update(~ground,np.full((2,4),.025))
 f.reset([0]);landing=f.update(ground,height)
 assert not landing['landing_air_time'][0].any() and landing['landing_air_time'][1].any()


def test_mid_swing_body_or_self_contact_invalidates_the_landing_bonus():
 f=FootMotion(1);contact=np.ones((1,4),bool)
 f.update(contact,np.zeros((1,4)))
 for i in range(10):f.update(~contact,np.full((1,4),.025),clean=np.array([i!=5]))
 assert not f.update(contact,np.zeros((1,4)),clean=np.array([True]))['landing_air_time'].any()


def test_rise_reward_rebases_after_initial_fall_and_cannot_pay_for_bobbing():
 r=RiseMotion(1);r.reset([0],np.array([.35]),np.zeros(1))
 rise,acc=r.update(np.array([.2]),np.array([-.5]),np.array([False]),.296)
 assert rise[0]==0 and acc[0]==0
 rise,_=r.update(np.array([.1]),np.array([-.1]),np.array([True]),.296)
 assert rise[0]==0
 rise,_=r.update(np.array([.2]),np.array([.2]),np.array([True]),.296)
 assert rise[0]>0
 r.update(np.array([.1]),np.array([-.2]),np.array([True]),.296)
 rise,_=r.update(np.array([.2]),np.array([.2]),np.array([True]),.296)
 assert rise[0]==0
 r.reset([0],np.array([.1]),np.zeros(1))
 _,acc=r.update(np.array([.1]),np.array([1.]),np.array([True]),.296)
 assert acc[0]==0 # no cross-episode impact spike


def recovery_args(up=1.,height=.296,pose=0.,feet=4,bad=0):
 u=np.array([up]);h=np.array([height]);p=np.array([pose])
 return dict(up=u,height=h,nominal_height=.296,pose_error=p,
  previous_potential=recovery_potential(u,h,.296,p),stable=np.array([35 if feet>=3 and bad==0 and height==.296 and up==1 else 0]),
  just_success=np.zeros(1,bool),actions=np.zeros((1,12)),previous_actions=np.zeros((1,12)),
  torque=np.zeros((1,16)),continuous=np.ones((1,16)),bad_contacts=np.array([bad]),
  feet_contacts=np.array([[i<feet for i in range(4)]]),self_contacts=np.zeros(1),
  rise_velocity=np.zeros(1),vertical_accel=np.zeros(1))


@pytest.mark.parametrize('args',[dict(up=-1.,height=.1,feet=0),dict(up=0.,height=.15,feet=0),
 dict(up=1.,height=.15,pose=.4),dict(up=1.,height=.296,feet=0,bad=1)])
def test_static_bad_states_have_negative_total_and_cannot_collect_goal_stack(args):
 terms,_=recovery_terms(**recovery_args(**args))
 assert sum(terms.values())[0]<0
 assert terms['standing_composite'][0]==0 and terms['height_stand_sharp'][0]==0


def test_dense_goal_rewards_require_feet_and_rise_is_directly_encouraged():
 a=recovery_args();goal,_=recovery_terms(**a)
 assert goal['standing_composite'][0]>0 and goal['pose_stand'][0]>0 and sum(goal.values())[0]>0
 a=recovery_args(up=.8,height=.2,pose=.3,feet=2)
 still,_=recovery_terms(**a);a['rise_velocity'][:]=.15;rising,_=recovery_terms(**a)
 assert rising['upward_motion'][0]>0 and sum(rising.values())[0]>sum(still.values())[0]


def test_hard_starts_unlock_by_measured_success_not_elapsed_time():
 assert recovery_progress_cap(None)==.19
 def champion(**rates):return {'metrics':{'per_orientation_success_rate':rates}}
 assert recovery_progress_cap(champion(folded_crouch=.3))==.49
 assert recovery_progress_cap(champion(folded_crouch=.3,left=.3,right=.3))==.84
 assert recovery_progress_cap(champion(folded_crouch=.3,left=.3,right=.3,supine=.3))==1


def test_joint_limit_and_torque_change_costs_have_safe_sign():
 limits=(np.full(12,-1.),np.ones(12))
 assert joint_limit_cost(np.zeros((1,12)),limits)[0]==0
 assert joint_limit_cost(np.ones((1,12)),limits)[0]<0
 a=recovery_args();a['previous_torque']=np.ones((1,16));a['vertical_accel'][:]=100
 early,_=recovery_terms(**a,progress=0.);late,_=recovery_terms(**a,progress=1.)
 for name in ('impact','torque_change'):assert late[name][0]<early[name][0]<=0

import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

from learning.env import make_model,stair_profile,stair_height,SweetieVecEnv
from learning.evaluate_navigation import heading_delta,heading_command,heading_success
from learning.speed_profile import SpeedTargetEnv,forward_ceiling


def test_stair_flights_landings_and_real_collider_heights():
    profile=stair_profile(1.)
    assert profile['riser_m']==pytest.approx(.04)
    np.testing.assert_allclose(stair_height(np.array([-.5,0,.5]),profile),0)
    np.testing.assert_allclose(stair_height(np.arange(.9,1.9,.2),profile),np.arange(1,6)*.04)
    np.testing.assert_allclose(stair_height(np.arange(2.7,3.7,.2),profile),np.arange(4,-1,-1)*.04,atol=1e-12)
    model_path,_=make_model('stairs',1.,991);model=mujoco.MjModel.from_xml_path(str(model_path));data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,0);data.qpos[2]=3;mujoco.mj_forward(model,data)
    for a,b,top in profile['segments_m']:
        x=(a+b)/2;geom=np.zeros(1,np.int32)
        distance=mujoco.mj_ray(model,data,np.array([x,.5,1.]),np.array([0.,0.,-1.]),None,1,-1,geom)
        assert 1.-distance==pytest.approx(top,abs=1e-7)
        assert stair_height(x,profile)==pytest.approx(stair_height(-x,profile))
    with pytest.raises(ValueError):stair_profile(float('nan'))


def test_unwrapped_heading_and_turn_goal_cannot_credit_wrong_way_extra_laps_or_drift():
    assert heading_delta(-math.pi+.1,math.pi-.1)==pytest.approx(.2)
    assert heading_success(np.array([math.pi]),math.pi,[20],[True],[.1],[0])[0]
    for angle,ticks,alive,drift in [(0,20,True,.1),(-math.pi,20,True,.1),(3*math.pi,20,True,.1),(math.pi,19,True,.1),(math.pi,20,False,.1),(math.pi,20,True,.4)]:
        assert not heading_success(np.array([angle]),math.pi,[ticks],[alive],[drift],[0])[0]
    assert heading_success([2*math.pi],2*math.pi,[20],[True],[.1],[0])[0]
    assert not heading_success([math.pi/2],math.pi/2,[20],[True],[0],[.1],arc=True)[0]
    command=heading_command(np.array([0,math.pi,math.pi+.3]),math.pi)
    assert command[0,2]>0 and command[1,2]==0 and command[2,2]<0


def test_navigation_sampler_retains_idle_and_trains_both_pivots_and_arcs():
    e=SpeedTargetEnv.__new__(SpeedTargetEnv);e.navigation=True;e.speed_stage=1
    e.task='locomotion';e.observation_mode='compact';e.manual_command=False;e.progress=1.
    e.rng=np.random.default_rng(91);e.steps=np.zeros(20000,int);e.commands=np.zeros((20000,3));e.command_targets=e.commands.copy()
    e._sample_commands(np.arange(20000));v=e.commands
    for sign in (-1,1):
        assert ((v[:,0]==0)&(v[:,1]==0)&(sign*v[:,2]>.2)).mean()>.10
        assert ((v[:,0]>.05)&(sign*v[:,2]>.2)).mean()>.10
    assert (v==0).all(1).mean()>.08 and (v[:,0]<0).any()
    assert np.abs(v[:,2]).max()<=1.2
    assert forward_ceiling('running','navigation-v1',1)==.8


def test_stair_resets_keep_compact_sensors_finite_without_assistance():
    env=SweetieVecEnv(terrain='stairs',difficulty=.5,num_envs=4,threads=1,randomize=True,auto_reset=True,observation_mode='compact')
    try:
        assert env.observe().shape==(4,63) and env.observe_critic().shape==(4,81)
        assert not env.state['self_contacts'].any()
        for _ in range(10):
            obs,reward,_,_=env.step(np.zeros((4,12)))
            assert np.isfinite(obs).all() and np.isfinite(reward).all()
    finally:env.close()


def test_navigation_schedule_includes_stairs_without_forcing_running_or_recovery_onto_them():
    from learning.driver import training_surface
    assert training_surface('locomotion',0,'navigation-v1')==('stairs',.12)
    assert 'stairs' not in [training_surface('running',i,'navigation-v1')[0] for i in range(20)]
    assert training_surface('recovery',5,'navigation-v1',{'quality_gate_passed':False})==('flat',0.)


def test_navigation_reward_penalizes_wrong_way_overspin_and_pivot_translation():
    from tests.test_microduck_rewards_v2 import walking_args
    from learning.rewards import locomotion_terms
    a=walking_args();a['command'][:]=[0,0,.8];a['velocity'][:]=0;a['omega'][:]=[0,0,.8];a['navigation']=True
    good=sum(locomotion_terms(**a).values())[0]
    for yaw in (-.8,0.,2.):
        a['omega'][0,2]=yaw
        assert sum(locomotion_terms(**a).values())[0]<good
    a['omega'][0,2]=.8;a['velocity'][0,0]=.3
    assert locomotion_terms(**a)['pivot_drift'][0]<0


def test_stair_clearance_target_rewards_riser_clearance_without_rewarding_static_lifting():
    from tests.test_microduck_rewards_v2 import walking_args
    from learning.rewards import locomotion_terms
    a=walking_args();a['gait']['clearance'][:]=.065;a['gait']['landing_peak'][:]=.065
    baseline=locomotion_terms(**a)['foot_clearance'][0]
    a['foot_clearance_target']=.065
    assert locomotion_terms(**a)['foot_clearance'][0]>baseline
    a['velocity'][:]=0
    assert locomotion_terms(**a)['foot_clearance'][0]==0


def test_explicit_running_actor_to_walking_transfer_preserves_actor_only(tmp_path):
    import torch
    from learning.ppo import _checkpoint,policy_for_env,TrainConfig
    from learning.recipe_bootstrap import export
    from scripts.common import ROOT
    torch.set_num_threads(1);torch.manual_seed(99)
    env=SweetieVecEnv(task='running',num_envs=1,threads=1,randomize=False,observation_mode='compact')
    try:
        model=policy_for_env(env);optimizer=torch.optim.Adam(model.parameters())
        donor=tmp_path/'donor.pt';_checkpoint(donor,model,optimizer,TrainConfig(task='running',device='cpu'),32,1,1.,env)
        with pytest.raises(ValueError):export(donor,ROOT,tmp_path/'bad','locomotion')
        path=export(donor,ROOT,tmp_path/'walk','locomotion',running_to_walk=True)
        saved=torch.load(path,weights_only=False)
        assert saved['config']['task']=='locomotion' and saved['steps']==0 and not saved['optimizer']['state']
        assert saved['initialization']['cross_task']=='running_to_locomotion_same_home_relative_actions'
        for name,value in model.state_dict().items():
            if name=='log_std' or name.startswith(('actor.','normalizer.')):assert torch.equal(value,saved['policy'][name])
    finally:env.close()

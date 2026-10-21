"""Milestones must reward actual, persistent recovery rather than reward loops."""
import numpy as np
import pytest
from learning.rewards import RecoveryWaypoints,recovery_terms
from tests.test_microduck_rewards_v2 import recovery_args


def state(up=-1.,height=.12,feet=0,bad=1,self_contact=0,stable=0,spin=0.):
    return dict(up=np.array([up]),height=np.array([height]),nominal_height=.296,
        contacts=np.array([[i<feet for i in range(4)]]),bad_contacts=np.array([bad]),
        self_contacts=np.array([self_contact]),omega=np.full((1,3),spin),stable=np.array([stable]))


def ledger(**kwargs):
    s=state(**kwargs);w=RecoveryWaypoints(1)
    w.reset([0],s['up'],s['height'],s['nominal_height'],s['contacts'])
    return w


def integrate(w,s,ticks=20):
    return sum(sum(w.update(**s).values())[0]*w.dt for _ in range(ticks))


def test_supine_side_belly_crouch_stand_sequence_is_paid_once():
    w=ledger();total=0.
    for level,s in enumerate([state(up=0.),state(up=.65,feet=1),
                             state(up=.9,height=.20,feet=2,bad=0),
                             state(up=1.,height=.296,feet=4,bad=0,stable=35)],1):
        total+=integrate(w,s);assert w.level[0]==level
        assert integrate(w,s)==0
    assert w.earned[0].all() and total>0
    assert integrate(w,state(),100)==0
    assert integrate(w,state(up=1.,height=.296,feet=4,bad=0,stable=35),100)==0


def test_threshold_chatter_cannot_farm_milestone_or_progress_forever():
    w=ledger();s=state(up=-.19)
    # Single ticks do not satisfy dwell. High-water progress is bounded.
    rewards=[]
    for _ in range(20):
        rewards.append(integrate(w,s,1));integrate(w,state(up=-.21),1)
    assert w.level[0]==0 and not w.earned.any()
    assert sum(rewards)<=2. and sum(rewards[1:])==0
    integrate(w,state(up=0.),6);assert w.level[0]==1


@pytest.mark.parametrize('bad', ['airborne','self_contact','spin'])
def test_roll_milestones_require_ground_clean_geometry_and_controlled_motion(bad):
    w=ledger();s=state(up=0.)
    if bad=='airborne':s['bad_contacts'][:]=0
    if bad=='self_contact':s['self_contacts'][:]=1
    if bad=='spin':s['omega'][:]=10
    integrate(w,s);assert w.level[0]==0 and not w.earned.any()


def test_initial_upright_pose_gets_no_roll_or_crouch_credit():
    w=ledger(up=1.,height=.296,feet=4,bad=0)
    integrate(w,state(up=1.,height=.296,feet=4,bad=0),100)
    assert w.initial_level[0]==3 and not w.earned.any()
    integrate(w,state(up=1.,height=.296,feet=4,bad=0,stable=35))
    assert w.earned[0].tolist()==[False,False,False,True]


def test_async_reset_clears_only_the_finished_episode_ledger():
    w=RecoveryWaypoints(2)
    w.level[:]=2;w.earned[:]=True;w.peak[:]=.8;w.dwell[:]=3
    w.reset([1],np.array([-1.]),np.array([.1]),.296,np.zeros((1,4),bool))
    assert w.level.tolist()==[2,0] and w.dwell.tolist()==[3,0]
    assert w.earned[0].all() and not w.earned[1].any() and w.peak[0]==.8


def test_waypoint_mode_reduces_roll_height_penalty_but_preserves_safety_costs():
    a=recovery_args(up=-.5,height=.1,feet=0,bad=1)
    a['torque'][:]=2
    old,_=recovery_terms(**a);new,_=recovery_terms(**a,waypoints={'waypoint_progress':np.zeros(1)})
    assert old['height_stand_l1'][0]<new['height_stand_l1'][0]<0
    assert old['overload'][0]==new['overload'][0]<0
    assert sum(new.values())[0]<0


def test_native_supine_rollout_reports_waypoints_without_assistance():
    from learning.env import SweetieVecEnv
    e=SweetieVecEnv(task='recovery',num_envs=2,threads=1,randomize=False,auto_reset=False,observation_mode='compact')
    e.reset_pose('supine');assert (e.rot[:,2,2]<-.99).all()
    e.max_episode_steps=15
    assert e.recovery_waypoints.level.tolist()==[0,0]
    for _ in range(15):
        obs,reward,done,info=e.step(np.zeros((2,12)))
        assert obs.shape==(2,63) and np.isfinite(reward).all()
        assert 'waypoint_side' in info['reward_terms']
    assert e.steps.tolist()==[15,15] and not e.recovery_waypoints.earned.any()
    assert done.all() and 'reached_side' in info['episode']
    assert info['episode']['recovery_initial_waypoint'].tolist()==[0,0]


def test_recipe_export_preserves_actor_exactly_and_resets_learning_state(tmp_path):
    import torch
    from learning.env import SweetieVecEnv
    from learning.ppo import policy_for_env,_checkpoint,TrainConfig
    from learning.recipe_bootstrap import export
    env=SweetieVecEnv(task='recovery',num_envs=1,threads=1,randomize=False,auto_reset=False,observation_mode='compact')
    model=policy_for_env(env);optimizer=torch.optim.Adam(model.parameters(),lr=.0002)
    obs=env.observe()
    with torch.no_grad():model.actor[-1].bias.fill_(.25);model.critic[-1].bias.fill_(123.)
    donor=tmp_path/'donor.pt'
    _checkpoint(donor,model,optimizer,TrainConfig(task='recovery',observation_mode='compact'),99,8,20.,env,curriculum_env_steps=5000)
    out=export(donor,None,tmp_path/'export','recovery');saved=torch.load(out,weights_only=False)
    assert saved['steps']==saved['updates']==0 and not saved['optimizer']['state']
    assert saved['curriculum_env_steps']==5000
    assert saved['initialization']['donor_steps']==99
    for k,v in model.state_dict().items():
        if k=='log_std' or k.startswith(('actor.','normalizer.')):
            torch.testing.assert_close(saved['policy'][k],v,rtol=0,atol=0)
    assert saved['policy']['critic.6.bias'].item()!=123.

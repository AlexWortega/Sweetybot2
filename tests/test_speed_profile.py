import numpy as np
import pytest
from learning.speed_profile import SpeedTargetEnv,run_ceiling,speed_stage_ready
from learning.evaluate import locomotion_failure


def sampler(task,stage=0):
    e=SpeedTargetEnv.__new__(SpeedTargetEnv);e.speed_stage=stage
    e.task=task;e.observation_mode='compact';e.manual_command=False;e.progress=1.
    e.rng=np.random.default_rng(8);e.steps=np.zeros(10000,int)
    e.commands=np.zeros((10000,3));e.command_targets=e.commands.copy()
    e._sample_commands(np.arange(10000));return e


@pytest.mark.parametrize('stage',[0,1,2])
def test_running_targets_expand_but_keep_slow_anchors_idle_and_fixed_lateral_bounds(stage):
    e=sampler('running',stage);v=e.commands[:,0]
    assert .08<(v==0).mean()<.12 and .4<((v>=.35)&(v<=.55)).mean()<.5
    assert (v>.55).sum()>4000 and v.max()<=run_ceiling(stage) and v.max()>run_ceiling(stage)-.01
    assert np.abs(e.commands[:,1]).max()<=.05 and np.abs(e.commands[:,2]).max()<=.35


def test_walking_keeps_all_command_modes_and_adds_fast_forward_examples():
    e=sampler('locomotion');v=e.commands[:,0]
    assert .18<(e.commands==0).all(1).mean()<.22
    assert (v<0).any() and (v>.30).sum()>1000 and .39<v.max()<=.40
    assert (np.count_nonzero(e.commands,axis=1)==3).mean()>.23
    assert (v[(v>0)&(v<=.30)]).size>1000


def test_resampling_changes_target_only_and_manual_commands_are_not_resampled():
    e=sampler('running');e.steps[:]=200;e.commands[:]=.1
    e._sample_commands(np.arange(10000));assert (e.commands==.1).all()
    e.manual_command=True;before=e.command_targets.copy()
    e._sample_commands(np.arange(10000));np.testing.assert_array_equal(before,e.command_targets)


def good_gate():
    metrics=dict(command_coverage_complete=True,survival_fraction=1.,worst_command_running_survival_fraction=1.,torque_gate_passed=True,self_collision_gate_passed=True)
    row=dict(survival_fraction=1.,linear_command_rmse=.08,mean_local_vx=.62,command=[.65,0,0])
    probe=dict(survival_fraction=1.,self_contact_fraction=0.,rms_joint_torque_nm=[1.]*16,
               continuous_torque_limits_nm=[2.]*16,per_command={'profile_forward':row.copy(),'profile_max':row.copy()})
    return metrics,probe


@pytest.mark.parametrize('bad',['fall','under_speed','overload','collision','coverage','nan_limit'])
def test_speed_unlock_requires_controlled_measured_current_ceiling_without_hiding_falls(bad):
    m,p=good_gate();assert speed_stage_ready(m,p)
    if bad=='fall':p['per_command']['profile_max']['survival_fraction']=.5
    if bad=='under_speed':p['per_command']['profile_max']['mean_local_vx']=.2
    if bad=='overload':p['rms_joint_torque_nm'][0]=3.
    if bad=='collision':m['self_collision_gate_passed']=False
    if bad=='coverage':m['worst_command_running_survival_fraction']=0.
    if bad=='nan_limit':p['continuous_torque_limits_nm'][0]=float('nan')
    assert not speed_stage_ready(m,p)


def test_fast_motion_past_old_arena_does_not_count_as_fall():
    up=np.ones(3);height=np.full(3,.296);q=np.zeros((3,23));q[:,0]=[4.,8.,9.6]
    assert locomotion_failure(up,height,.296,q,10.).tolist()==[False,False,True]
    up[0]=0.;assert locomotion_failure(up,height,.296,q,10.)[0]


def test_profile_manual_command_honors_recorded_stage():
    e=SpeedTargetEnv(task='running',speed_stage=2,num_envs=1,threads=1,randomize=False,auto_reset=False,observation_mode='compact')
    e.set_command(1.,0,0);assert e.commands[0,0]==1.
    assert e.observe()[0,50]==1.
    assert e.manual_command

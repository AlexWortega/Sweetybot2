import copy
import pytest
from learning.driver import migrate_schedule,score_development,POLICY_FILES


def metrics():
    return {'survival_fraction':1.,'linear_command_rmse':.10,'yaw_command_rmse':.2,
        'running_success_rate':.75,'rms_joint_torque_nm':[.8]*16,'continuous_torque_limits_nm':[1.5691]*16,
        'per_command':{name:{'running_success_rate':.75,'survival_fraction':1.,'mean_local_vx':.40}
                       for name in ('forward','fast_forward','run_turn')}}


def test_migration_preserves_budget_checkpoints_and_runs_once():
    state={'started_epoch':10.,'deadline_epoch':86410.,'budget_hours':24.,'round':7,
        'latest_training':{'locomotion':'saved_walk.pt','recovery':'saved_recovery.pt'},
        'champions':{'locomotion':{'checkpoint':'best.pt'}},
        'consecutive_failures':{'locomotion':1,'recovery':0},
        'children':[{'kind':'training','task':'locomotion','status':'completed'},
                    {'kind':'training','task':'locomotion','status':'failed'},
                    {'kind':'training','task':'recovery','status':'completed'}]}
    before=copy.deepcopy(state);migrate_schedule(state)
    for key in ('started_epoch','deadline_epoch','budget_hours','round','latest_training','champions','children'):
        assert state[key]==before[key]
    assert state['schedule_index']==2 and state['completed_chunks_by_task']=={'locomotion':1,'recovery':1,'running':0}
    assert state['consecutive_failures']['running']==0
    assert len(state['migration_log'])==1
    migrate_schedule(state)
    assert len(state['migration_log'])==1
    assert set(POLICY_FILES.values())=={'walk.pt','recover.pt','run.pt'}


def test_running_gate_rejects_fast_shuffle_missing_commands_and_overload():
    good=metrics();assert score_development('running',[good])[1]
    shuffle=metrics();shuffle['running_success_rate']=0.
    for row in shuffle['per_command'].values():row['running_success_rate']=0.
    assert not score_development('running',[shuffle])[1]
    missing=metrics();missing['per_command'].pop('run_turn')
    assert not score_development('running',[missing])[1]
    overload=metrics();overload['rms_joint_torque_nm'][3]=1.57
    assert not score_development('running',[overload])[1]
    assert score_development('running',[good])[0]>score_development('running',[shuffle])[0]


def test_running_gate_checks_each_surface_and_real_velocity():
    slow=metrics();slow['per_command']['forward']['mean_local_vx']=.10
    assert not score_development('running',[metrics(),slow])[1]
    fallen=metrics();fallen['per_command']['fast_forward']['survival_fraction']=.5
    assert not score_development('running',[metrics(),fallen])[1]
    with pytest.raises(ValueError):score_development('running',[{}])

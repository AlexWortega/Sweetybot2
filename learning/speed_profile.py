"""Configurable command distribution; robot physics and actor inputs unchanged."""
import numpy as np
from learning.env import SweetieVecEnv

PROFILE='higher-v1'
RUN_CEILINGS=(.65,.80,1.00)
WALK_CEILING=.40


def run_ceiling(stage):
    if stage not in range(len(RUN_CEILINGS)):raise ValueError('Invalid speed stage')
    return RUN_CEILINGS[stage]


def forward_ceiling(task,profile='baseline',stage=0):
    if profile not in ('baseline',PROFILE):raise ValueError('Unknown speed profile')
    run_ceiling(stage)
    if task=='recovery':return 0.
    if task=='running':return run_ceiling(stage) if profile==PROFILE else .55
    return WALK_CEILING if profile==PROFILE else .30


class SpeedTargetEnv(SweetieVecEnv):
    def __init__(self,*args,speed_stage=0,**kwargs):
        self.speed_stage=int(speed_stage);run_ceiling(self.speed_stage)
        super().__init__(*args,**kwargs)

    def _sample_commands(self,ids):
        super()._sample_commands(ids)
        if self.manual_command or self.observation_mode!='compact' or self.task=='recovery':return
        target=self.command_targets[ids].copy()
        # Half of forward commands retain their original modest range. Idle,
        # backward, lateral and turn coverage is retained, including mixed turns.
        fast=(target[:,0]>.05)&(self.rng.random(len(ids))<.5)
        low,high=(.55,run_ceiling(self.speed_stage)) if self.task=='running' else (.30,WALK_CEILING)
        target[fast,0]=self.rng.uniform(low,high,fast.sum())
        self.command_targets[ids]=target
        reset=self.steps[ids]==0
        self.commands[ids[reset]]=target[reset]

    def set_command(self,vx,vy,yaw):
        if self.task!='running':return super().set_command(vx,vy,yaw)
        if not np.isfinite([vx,vy,yaw]).all():raise ValueError('Finite command required')
        self.manual_command=True
        self.commands[:]=np.clip([vx,vy,yaw],[0.,-.08,-.6],[run_ceiling(self.speed_stage),.08,.6])
        self.command_targets[:]=self.commands
        self.history[:,-1]=self._frame()


def speed_stage_ready(metrics,probe):
    """A faster target requires survival, tracking, load and clean-contact proof.

The unchanged four-surface suite and a separate current-ceiling probe must both
pass. A single fast transient or speed before falling cannot unlock a stage.
"""
    if not (metrics.get('command_coverage_complete') and metrics.get('survival_fraction',0)>=.90 and
            metrics.get('worst_command_survival_fraction',metrics.get('worst_command_running_survival_fraction',0))>=.90 and
            metrics.get('torque_gate_passed') and metrics.get('self_collision_gate_passed')):return False
    rows=list(probe.get('per_command',{}).values())
    if set(probe.get('per_command',{}))!={'profile_forward','profile_max'} or probe.get('survival_fraction',0)<.90 or probe.get('self_contact_fraction',1)>0:return False
    rms=np.asarray(probe.get('rms_joint_torque_nm',[]),float)
    limits=np.asarray(probe.get('continuous_torque_limits_nm',[]),float)
    if rms.shape!=(16,) or limits.shape!=(16,) or not np.isfinite(rms).all() or not np.isfinite(limits).all() or (limits<=0).any() or (rms>limits).any():return False
    return all(r.get('survival_fraction',0)>=.90 and r.get('linear_command_rmse',float('inf'))<=.15 and
               r.get('mean_local_vx',0)>=.8*r.get('command',[float('inf')])[0] for r in rows)

"""No-reset stair and heading-goal evaluation; zero-action baselines are valid.

Goals affect joystick velocity commands only. No root forces, teleportation,
heading observations or trajectory tracking targets are supplied to the actor.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import torch

from learning.evaluate import locomotion_failure
from learning.ppo import load_policy,validate_environment_contract
from learning.speed_profile import SpeedTargetEnv
from scripts.common import load,continuous_limits

CASES=('left_180','right_180','left_360','right_360','arc_left','arc_right','stairs_up','stairs_down')


def heading_delta(current,previous):
    return np.arctan2(np.sin(current-previous),np.cos(current-previous))


def heading_command(turned,goal,arc=False):
    sign=np.sign(goal);remaining=np.abs(goal)-sign*np.asarray(turned)
    # Reverse a small overshoot instead of crediting another full revolution.
    yaw=sign*np.clip(1.7*remaining,-.35,1.1)
    yaw=np.where(np.abs(remaining)<.06,0.,yaw)
    vx=np.where(remaining>.15,.20,0.) if arc else np.zeros_like(yaw)
    return np.column_stack((vx,np.zeros_like(yaw),yaw))


def heading_success(turned,goal,stable_ticks,alive,drift,path_length,arc=False):
    good=(np.abs(np.asarray(turned)-goal)<.20)&(np.asarray(stable_ticks)>=20)&np.asarray(alive)
    return good & (np.asarray(path_length)>=.20 if arc else np.asarray(drift)<.30)


def stair_success(progress,ground_change,riser,descending,alive,clean,up,height_ratio,contacts,vx):
    sign=-1 if descending else 1
    return (np.asarray(alive)&np.asarray(clean)&(np.asarray(progress)>=1.2)&
            (sign*np.asarray(ground_change)>=3.5*riser)&(np.asarray(up)>.85)&
            (np.asarray(height_ratio)>.65)&(np.asarray(height_ratio)<1.4)&
            (np.asarray(contacts)>=2)&(np.abs(vx)<.35))


def place_initial(env,x):
    """Episode initialization only. Resolve actual collider penetration once."""
    ids=np.arange(env.num_envs);q=env.state['qpos'].copy();q[:,0]=x
    q[:,1]=np.linspace(-.04,.04,env.num_envs);q[:,2]=env.ground_height(q[:,0],q[:,1])+env.nominal_height
    env._clear_reset_penetration(q)
    env.native.reset(ids,q,np.zeros((env.num_envs,22)),env.params);env._read_state()
    env.sensors.reset(ids,env.state['imu_quat']);env.previous_feet[:]=env.state['feet']
    env.targets[:]=q[:,7:];env.history[:]=env._frame()[:,None,:]


def evaluate_case(checkpoint,case,episodes=2,seconds=12.,difficulty=.2,seed=1_700_000):
    if case not in CASES:raise ValueError(case)
    if episodes<1 or not np.isfinite(seconds) or seconds<=0:raise ValueError('Invalid evaluation length')
    policy,metadata=(load_policy(checkpoint,device='cpu') if checkpoint else (None,None))
    stairs=case.startswith('stairs_');arc=case.startswith('arc_')
    env=SpeedTargetEnv(task='locomotion',navigation=True,speed_stage=1,num_envs=episodes,
                      terrain='stairs' if stairs else 'flat',difficulty=difficulty,seed=seed,
                      threads=min(4,episodes),randomize=False,auto_reset=False,observation_mode='compact')
    try:
        if metadata:
            validate_environment_contract(metadata,env)
            if metadata['config']['task']!='locomotion':raise ValueError('Navigation requires the locomotion policy')
        env.manual_command=True;env.reset_pose('stand')
        place_initial(env,2.4 if case=='stairs_down' else .5 if stairs else 0.)
        start=env.state['qpos'][:,:3].copy();ground_start=env.ground_height(start[:,0],start[:,1])
        previous=env.state['qpos'][:,:2].copy();previous_yaw=np.arctan2(env.rot[:,1,0],env.rot[:,0,0])
        turned=np.zeros(episodes);path=np.zeros(episodes);alive=np.ones(episodes,bool)
        stable=np.zeros(episodes,int);torque2=np.zeros((episodes,16));self_ticks=np.zeros(episodes,int);body_ticks=np.zeros(episodes,int)
        goal=(-1 if 'right' in case else 1)*(math.pi/2 if arc else 2*math.pi if '360' in case else math.pi)
        ticks=round(seconds/env.dt)
        for _ in range(ticks):
            command=np.tile([.15,0,0],(episodes,1)) if stairs else heading_command(turned,goal,arc)
            env.commands[:]=command;env.command_targets[:]=command;env.history[:,-1]=env._frame()
            with torch.inference_mode():action=policy.act(env.observe(),deterministic=True).cpu().numpy() if policy else np.zeros((episodes,12))
            env.step(action)
            q=env.state['qpos'];yaw=np.arctan2(env.rot[:,1,0],env.rot[:,0,0])
            turned+=heading_delta(yaw,previous_yaw);previous_yaw=yaw
            path+=np.linalg.norm(q[:,:2]-previous,axis=1);previous=q[:,:2].copy()
            alive &= ~locomotion_failure(env.rot[:,2,2],env.height,env.nominal_height,q,env.terrain['extent_m'])
            self_ticks+=env.state['self_contacts']>0;body_ticks+=env.state['bad_contacts']>0
            torque2+=env.state['torque_rms']**2
            near=(np.abs(turned-goal)<.20)&(np.abs(env.local_angular[:,2])<.25)&(env.rot[:,2,2]>.85)&(env.state['contacts'].sum(1)>=2)
            stable=np.where(near&alive,stable+1,0)
        limits=np.asarray(continuous_limits(load()));rms=np.sqrt(torque2/max(1,ticks))
        clean=(self_ticks==0)&(body_ticks==0)&(rms<=limits).all(1)
        drift=np.linalg.norm(env.state['qpos'][:,:2]-start[:,:2],axis=1)
        ground_change=env.ground_height(env.state['qpos'][:,0],env.state['qpos'][:,1])-ground_start
        progress=env.state['qpos'][:,0]-start[:,0]
        if stairs:
            success=stair_success(progress,ground_change,env.terrain['riser_m'],case=='stairs_down',alive,clean,
                                  env.rot[:,2,2],env.height/env.nominal_height,env.state['contacts'].sum(1),env.local_velocity[:,0])
        else:success=heading_success(turned,goal,stable,alive,drift,path,arc)&clean
        return dict(case=case,episodes=episodes,successes=int(success.sum()),success_rate=float(success.mean()),
                    survival_fraction=float(alive.mean()),heading_goal_rad=None if stairs else goal,
                    heading_change_rad=turned.tolist(),path_length_m=path.tolist(),drift_m=drift.tolist(),
                    forward_progress_m=progress.tolist(),ground_height_change_m=ground_change.tolist(),
                    rms_joint_torque_nm=rms.max(0).tolist(),continuous_torque_limits_nm=limits.tolist(),
                    self_contact_ticks=self_ticks.tolist(),body_contact_ticks=body_ticks.tolist(),
                    terrain={k:v for k,v in env.terrain.items() if k!='height_samples'},seed=seed,seconds=seconds,
                    auto_reset=False,root_support=False,initial_pose_randomized=False)
    finally:env.close()


def evaluate(checkpoint=None,episodes=2,seconds=12.,difficulty=.2,seed=1_700_000):
    torch.set_num_threads(1)
    cases=[evaluate_case(checkpoint,case,episodes,seconds,difficulty,seed+i) for i,case in enumerate(CASES)]
    from learning.readiness import ROOT,dynamics_signature
    return dict(protocol='navigation-stairs-heading-v2',dynamics_signature=dynamics_signature(),
                source_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in ('learning/evaluate_navigation.py','learning/env.py','learning/speed_profile.py')},checkpoint=str(checkpoint) if checkpoint else None,
                checkpoint_sha256=hashlib.sha256(Path(checkpoint).read_bytes()).hexdigest() if checkpoint else None,
                cases=cases,all_passed=all(row['success_rate']>=.9 for row in cases),
                limitation='Fixed initial poses and small lateral offsets; not arbitrary stairs/falls',
                actor_inputs_unchanged=True,auto_reset=False,root_support=False)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--checkpoint',type=Path)
    p.add_argument('--out',required=True,type=Path);p.add_argument('--episodes',type=int,default=2)
    p.add_argument('--seconds',type=float,default=12.);p.add_argument('--difficulty',type=float,default=.2)
    p.add_argument('--seed',type=int,default=1_700_000);a=p.parse_args()
    result=evaluate(a.checkpoint,a.episodes,a.seconds,a.difficulty,a.seed)
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({r['case']:r['success_rate'] for r in result['cases']}))


if __name__=='__main__':main()

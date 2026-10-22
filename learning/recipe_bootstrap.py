"""Explicit actor-only recipe handoff with source/physical parity verification.

The donor remains immutable. Export a fresh critic and PPO optimizer, preserving
the actor, its normalizer and exploration scale exactly. No learned transitions
are claimed by an export. The existing campaign deadline is managed by driver.
"""
import argparse
import hashlib
import io
from pathlib import Path

import torch

from learning.env import SweetieVecEnv
from learning.ppo import TrainConfig, _checkpoint, policy_for_env
from learning.transfer import validate_initializer


def export(donor, source, output, task, *, joint_limit_transfer=False,running_to_walk=False):
    raw=Path(donor).read_bytes()
    saved=torch.load(io.BytesIO(raw),map_location='cpu',weights_only=False)
    donor_task=saved.get('config',{}).get('task')
    if running_to_walk and (donor_task,task)!=('running','locomotion'):
        raise ValueError('Running-to-walk transfer requires running donor and locomotion target')
    if donor_task!=task and not running_to_walk:
        raise ValueError('Recipe handoff requires the same task')
    if saved.get('environment_metadata',{}).get('observation_mode')!='compact':
        raise ValueError('Recipe handoff expects the compact actor')
    torch.set_num_threads(1);torch.manual_seed(37)
    env=SweetieVecEnv(task=task,num_envs=1,threads=1,randomize=False,
                     auto_reset=False,observation_mode='compact')
    provenance=validate_initializer(saved,env,source,joint_limit_transfer=joint_limit_transfer)
    model=policy_for_env(env)
    actor_keys={k:v for k,v in saved['policy'].items()
                if k=='log_std' or k.startswith(('actor.','normalizer.'))}
    own=model.state_dict()
    expected={k for k in own if k=='log_std' or k.startswith(('actor.','normalizer.'))}
    if set(actor_keys)!=expected:raise ValueError('Actor architecture changed')
    own.update(actor_keys);model.load_state_dict(own)
    optimizer=torch.optim.Adam(model.parameters(),lr=.0003,eps=1e-5)
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    config=TrainConfig(task=task,out=str(output),device='cpu',observation_mode='compact')
    initialization=dict(mode='actor_and_normalizer_only_fresh_critic_optimizer',
        donor_path=str(Path(donor).resolve()),donor_sha256=hashlib.sha256(raw).hexdigest(),
        donor_steps=saved.get('steps'),donor_updates=saved.get('updates'),
        recipe_transfer=provenance,ppo_transitions=0,
        cross_task='running_to_locomotion_same_home_relative_actions' if running_to_walk else None)
    _checkpoint(output/'latest.pt',model,optimizer,config,0,0,0.,env,initialization,
                curriculum_env_steps=saved.get('curriculum_env_steps',0.))
    env.close()
    return output/'latest.pt'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--donor',required=True);p.add_argument('--source',required=True)
    p.add_argument('--out',required=True);p.add_argument('--task',choices=('locomotion','recovery','running'),required=True)
    p.add_argument('--joint-limit-transfer',action='store_true',help='Explicit actor-only transfer after joint-limit-only mechanical revalidation')
    p.add_argument('--running-to-walk',action='store_true',help='Explicit compact actor transfer; both tasks use the same home-relative joint targets')
    a=p.parse_args();print(export(a.donor,a.source,a.out,a.task,joint_limit_transfer=a.joint_limit_transfer,running_to_walk=a.running_to_walk),flush=True)


if __name__=='__main__':main()

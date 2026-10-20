"""CPU warm start of a compact actor from an explicitly verified frozen policy.

Collects real MuJoCo transitions with teacher/student roll-ins. Only 63 sampled
sensor values enter the student; old sensor history is used for labels only.
The critic is fresh and the exported PPO optimizer has no BC momentum.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import torch

from learning.ppo import TrainConfig, _checkpoint, load_policy, policy_for_env
from learning.transfer import validate_teacher


def distill(env, teacher, metadata, checkpoint, source, out, *, seconds=60.,
            max_steps=8192, rollout_steps=64, epochs=5, seed=37):
    if (not np.isfinite(seconds) or seconds<=0 or max_steps<env.num_envs
            or rollout_steps<1 or epochs<1):raise ValueError('Positive bounded distillation budget required')
    if getattr(env,'observation_mode',None)!='compact':raise ValueError('Compact student environment required')
    if env.task not in ('locomotion','running'):raise ValueError('No learned get-up expert is supplied by walking distillation')
    donor_task=metadata.get('config',{}).get('task')
    if donor_task!=env.task and (donor_task,env.task)!=('locomotion','running'):
        raise ValueError('Teacher action semantics do not match the motion task')
    if teacher.device.type!='cpu':raise ValueError('This preparation tool is CPU-only')
    provenance=validate_teacher(metadata,env,source)
    torch.set_num_threads(2);torch.manual_seed(seed)
    rng=np.random.default_rng(seed)
    model=policy_for_env(env)
    teacher.requires_grad_(False);teacher.eval()
    optimizer=torch.optim.Adam(model.actor.parameters(),lr=3e-4)
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    if (out/'latest.pt').exists():raise FileExistsError('Choose a fresh distillation output')
    obs=env.reset();samples=0;updates=0;last={};start=time.monotonic()
    with (out/'metrics.jsonl').open('w',buffering=1) as log:
        while samples+env.num_envs<=max_steps and time.monotonic()-start<seconds:
            observations=[];critics=[];labels=[]
            for _ in range(min(rollout_steps,(max_steps-samples)//env.num_envs)):
                if time.monotonic()-start>=seconds:break
                raw=env.teacher_observation() if teacher.obs_dim!=model.obs_dim else obs
                with torch.inference_mode():
                    target=teacher.act(raw).cpu().numpy()
                    student=model.act(obs).cpu().numpy()
                if target.shape!=(env.num_envs,12) or not np.isfinite(target).all():
                    raise FloatingPointError('Invalid frozen teacher labels')
                observations.append(obs.copy());critics.append(env.observe_critic());labels.append(target.copy())
                teacher_fraction=1. if updates==0 else .7
                use_teacher=rng.random(env.num_envs)<teacher_fraction
                actions=np.where(use_teacher[:,None],target,student)
                obs,_,_,_=env.step(actions)
                samples+=env.num_envs
            if not observations:break
            x=np.concatenate(observations);c=np.concatenate(critics);y=torch.as_tensor(np.concatenate(labels))
            model.update_norm(x,c);x=torch.as_tensor(x)
            with torch.no_grad():before=float((model.actor(model.normalize(x)).tanh()-y).square().mean())
            for _ in range(epochs):
                for ids in torch.randperm(len(x)).split(2048):
                    if time.monotonic()-start>=seconds:break
                    loss=(model.actor(model.normalize(x[ids])).tanh()-y[ids]).square().mean()
                    if not torch.isfinite(loss):raise FloatingPointError('Nonfinite BC objective')
                    optimizer.zero_grad(set_to_none=True);loss.backward()
                    torch.nn.utils.clip_grad_norm_(model.actor.parameters(),1.);optimizer.step()
            updates+=1
            with torch.no_grad():after=float((model.actor(model.normalize(x)).tanh()-y).square().mean())
            last=dict(samples=samples,bc_updates=updates,training_mse_before=before,training_mse_after=after,
                      teacher_rollin_fraction=teacher_fraction,wall_seconds=time.monotonic()-start)
            log.write(json.dumps(last,allow_nan=False)+'\n');print(json.dumps(last),flush=True)
    if not samples:raise RuntimeError('Distillation collected no training transitions')
    fresh_optimizer=torch.optim.Adam(model.parameters(),lr=3e-4,eps=1e-5)
    receipt=dict(mode='compact_actor_distillation',teacher_checkpoint=str(Path(checkpoint).resolve()),
                 teacher_sha256=hashlib.sha256(Path(checkpoint).read_bytes()).hexdigest(),
                 teacher_source=provenance,samples=samples,bc_updates=updates,critic='fresh',
                 optimizer_restored=False,quality='training fit only; independent evaluation required')
    config=TrainConfig(task=env.task,out=str(out),nenvs=env.num_envs,device='cpu',seed=seed,
                       observation_mode='compact')
    _checkpoint(out/'latest.pt',model,fresh_optimizer,config,0,0,0.,env,receipt,0.)
    result=dict(initialization=receipt,metrics=last,checkpoint=str((out/'latest.pt').resolve()))
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--teacher',required=True,type=Path)
    parser.add_argument('--teacher-source',type=Path)
    parser.add_argument('--out',required=True,type=Path)
    parser.add_argument('--task',choices=('locomotion','running'),default='locomotion')
    parser.add_argument('--seconds',type=float,default=60.)
    parser.add_argument('--max-steps',type=int,default=8192)
    parser.add_argument('--nenvs',type=int,default=8)
    parser.add_argument('--threads',type=int,default=2)
    parser.add_argument('--seed',type=int,default=37)
    args=parser.parse_args()
    if not 1<=args.threads<=16 or not 1<=args.nenvs<=128:parser.error('CPU preparation caps: 16 threads / 128 envs')
    from learning.readiness import require_training_ready
    require_training_ready()
    from learning.env import SweetieVecEnv
    teacher,metadata=load_policy(args.teacher,'cpu')
    env=SweetieVecEnv(task=args.task,num_envs=args.nenvs,threads=args.threads,seed=args.seed,observation_mode='compact')
    try:
        print(json.dumps(distill(env,teacher,metadata,args.teacher,args.teacher_source,args.out,
              seconds=args.seconds,max_steps=args.max_steps,seed=args.seed),indent=2))
    finally:env.close()


if __name__=='__main__':main()

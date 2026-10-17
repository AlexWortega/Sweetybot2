"""Pull scored policies and progress from a configured SSH host."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import shlex
import os
import time
from datetime import datetime,timezone


def atomic_text(path,value):
    temporary=path.with_name(path.name+f'.{os.getpid()}.tmp')
    temporary.write_text(value)
    temporary.replace(path)


def final_summary(results):
    summary={}
    for task,terrains in results.items():
        summary[task]={}
        for terrain,result in terrains.items():
            if result.get('status') in {'failed','budget_exhausted'}:
                summary[task][terrain]={'status':result['status'],'error':result.get('error')}
            elif task=='running':
                summary[task][terrain]={'running_successes':result['running_successes'],'attempts':result['running_episodes'],
                    'survivals':round(result['episodes']*result['survival_fraction']),
                    'linear_rmse_mps':result['linear_command_rmse'],'yaw_rmse_rad_s':result['yaw_command_rmse'],
                    'per_command':{name:{'successes':row['running_successes'],'attempts':row['episodes'],
                        'mean_vx_mps':row['mean_local_vx'],'flight_intervals':row['mean_valid_flight_intervals']}
                        for name,row in result['per_command'].items()}}
            elif task=='locomotion':
                summary[task][terrain]={'episodes':result['episodes'],'survivals':round(result['episodes']*result['survival_fraction']),
                    'linear_rmse_mps':result['linear_command_rmse'],'yaw_rmse_rad_s':result['yaw_command_rmse']}
            else:
                summary[task][terrain]={'recoveries':result['recovery_successes'],'attempts':result['recovery_episodes'],
                    'per_orientation':{name:{'successes':row['successes'],'attempts':row['episodes']} for name,row in result['per_orientation'].items() if name!='stand'}}
    return summary


def pull_once(remote, destination):
    destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    status_path=destination/'status.json'
    result=subprocess.run(['rsync','-az','--timeout=30',remote+'/status.json',str(status_path)],capture_output=True,text=True,timeout=45)
    if result.returncode:raise RuntimeError(result.stderr.strip())
    state=json.loads(status_path.read_text())
    downloaded={}
    for task,champion in state.get('champions',{}).items():
        name={'locomotion':'walk.pt','recovery':'recover.pt','running':'run.pt'}[task];target=destination/name
        wanted=champion['checkpoint_sha256']
        actual=hashlib.sha256(target.read_bytes()).hexdigest() if target.exists() else None
        if actual!=wanted:
            subprocess.run(['rsync','-azL','--timeout=30',remote+'/'+name,str(target)+'.tmp'],check=True,capture_output=True,timeout=60)
            temporary=Path(str(target)+'.tmp')
            if hashlib.sha256(temporary.read_bytes()).hexdigest()!=wanted:
                temporary.unlink(missing_ok=True)
                raise RuntimeError('Policy changed during sync; will retry next poll')
            temporary.replace(target)
        downloaded[task]={'file':name,'sha256':wanted,'quality_gate_passed':champion['quality_gate_passed'],'metrics':champion['metrics']}
    finals=None
    if state.get('complete') and state.get('final_results') and state.get('champions'):
        subprocess.run(['rsync','-az','--timeout=30',remote+'/final_results.json',str(destination/'final_results.json')],check=True,capture_output=True,timeout=60)
        finals=final_summary(json.loads((destination/'final_results.json').read_text()))
    active_metrics=None
    active=state.get('active_run',{}).get('directory')
    if active and ':' in remote:
        host=remote.split(':',1)[0]
        tail=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10',host,'tail -n 1 '+shlex.quote(active+'/metrics.jsonl')],capture_output=True,text=True,timeout=20)
        if tail.returncode==0 and tail.stdout.strip():
            active_metrics=json.loads(tail.stdout.strip())
            atomic_text(destination/'active_metrics.json',json.dumps(active_metrics,indent=2)+'\n')
    charged=state.get('charged_wall_seconds',0)
    if not state.get('complete') and state.get('started_epoch'):
        charged=min(state['budget_hours']*3600,max(charged,time.time()-state['started_epoch']))
    summary={'synced_utc':datetime.now(timezone.utc).isoformat(),'remote':remote,'phase':state.get('phase'),'complete':state.get('complete',False),
             'round':state.get('round'),'active_run':state.get('active_run'),'active_metrics':active_metrics,'charged_gpu_hours':charged/3600,
             'budget_gpu_hours':state.get('budget_hours'),'policies':downloaded,'independent_final_results':finals,
             'note':'Quality gate and held-out results establish status; checkpoint presence alone does not mean successful walking/running/recovery.'}
    atomic_text(destination/'progress.json',json.dumps(summary,indent=2)+'\n')
    phase=state.get('phase','unknown');quality='\n'.join(f"{task}: quality_gate={data['quality_gate_passed']}; {data['metrics']}" for task,data in downloaded.items())
    final_text='\nНезависимые финальные проверки:\n'+json.dumps(finals,ensure_ascii=False,indent=2) if finals is not None else ''
    atomic_text(destination/'СТАТУС.txt',f"Обновлено: {summary['synced_utc']}\nЭтап: {phase}; цикл завершён: {state.get('complete',False)}\nЦикл: {state.get('round',0)}\nУчтено GPU-часов: {summary['charged_gpu_hours']:.3f} / {state.get('budget_hours')}\n{quality}\nНаличие checkpoint не означает, что навык успешно обучен. quality_gate относится к development; независимые финальные проверки: final_results.json после завершения.\n{final_text}\n")
    return state


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--remote',required=True,help='SSH destination, e.g. gpu-host:/data/sweetybot2/training')
    parser.add_argument('--dest',required=True);parser.add_argument('--hours',type=float,default=25);parser.add_argument('--interval',type=float,default=120)
    parser.add_argument('--once',action='store_true');args=parser.parse_args();end=time.monotonic()+args.hours*3600
    while time.monotonic()<end:
        try:
            state=pull_once(args.remote,args.dest)
            print(json.dumps({'time':time.time(),'phase':state.get('phase'),'round':state.get('round')}),flush=True)
            if state.get('complete') or args.once:return
        except (RuntimeError,subprocess.SubprocessError,OSError,ValueError) as error:
            print(f'Sync pending: {error}',flush=True)
            if args.once:raise
        time.sleep(max(10,args.interval))


if __name__=='__main__':main()

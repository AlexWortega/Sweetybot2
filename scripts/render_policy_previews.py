"""Export labelled, uninterrupted CAD replays of explicitly selected policies.

CPU inference only. Rendering receives copies of native state and never drives
the physics. Preview seeds are development seeds, not held-out evaluation data.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import torch

from learning.env import SweetieVecEnv
from learning.play import render_model
from learning.ppo import load_policy, validate_environment_contract
from learning.readiness import dynamics_signature


def render(job, output, seconds=10., seed=1_000_090):
    checkpoint=Path(job['checkpoint'])
    policy,metadata=load_policy(checkpoint,device='cpu')
    assert metadata['config']['task']==job['task']
    env=SweetieVecEnv(task=job['task'],num_envs=1,threads=1,randomize=False,
                     auto_reset=False,terrain='flat',difficulty=0.,seed=seed)
    validate_environment_contract(metadata,env)
    env.manual_command=True
    env.reset_pose(job.get('pose','stand'))
    command=np.array(job['command'],dtype=float)
    env.set_command(*command)
    model=render_model(env,cad=True)
    data=mujoco.MjData(model)
    # Display-only palette distinguishes covers and joints on the saved CAD.
    for i in range(model.ngeom):
        name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) or ''
        if name.endswith('_cad'):
            if 'roll_cad' in name or name=='neck_yaw_cad':model.geom_rgba[i]=[.30,.28,.38,1.]
            elif name.startswith('ear_'):model.geom_rgba[i]=[.62,.45,.78,1.]
            else:model.geom_rgba[i]=[.90,.91,.96,1.]
    camera=mujoco.MjvCamera();camera.distance=1.20;camera.azimuth=45;camera.elevation=-18
    options=mujoco.MjvOption();options.geomgroup[:]=0;options.geomgroup[:2]=1
    title_font=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial Bold.ttf',18)
    font=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',14)
    small=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',12)
    mp4=output/(job['name']+'.mp4');gif=mp4.with_suffix('.gif')
    trajectory=[];fell=False;self_ticks=0;frames=0
    renderer=mujoco.Renderer(model,width=640,height=480)
    try:
        with imageio.get_writer(str(mp4),fps=25,codec='libx264',quality=8) as writer:
            for tick in range(round(seconds/env.dt)+1):
                if tick:
                    with torch.inference_mode():actions=policy.act(env.observe(),deterministic=True).cpu().numpy()
                    env.step(actions)
                root=env.state['qpos'][0,:3]
                up=float(env.rot[0,2,2]);vx=float(env.local_velocity[0,0])
                fell |= bool(up<.45 or env.height[0]<.43*env.nominal_height)
                self_ticks+=bool(env.state['self_contacts'][0])
                trajectory.append(dict(t=tick*env.dt,position=root.tolist(),up_z=up,vx=vx,
                    contacts=int(env.state['contacts'][0].sum()),self_contacts=int(env.state['self_contacts'][0]),
                    stable_ticks=int(env.stable[0])))
                if tick%2:continue
                data.qpos[:]=env.state['qpos'][0];data.qvel[:]=env.state['qvel'][0]
                mujoco.mj_forward(model,data)
                camera.lookat[:]=[root[0],root[1],.255]
                renderer.update_scene(data,camera,scene_option=options)
                # 10 cm world grid makes translation visible with a following camera.
                center=np.round(root[:2]/.1)*.1
                for axis in (0,1):
                    for offset in np.arange(-1.2,1.21,.1):
                        a=np.array([center[0]-1.2,center[1]-1.2,.001])
                        b=np.array([center[0]+1.2,center[1]+1.2,.001])
                        a[axis]=b[axis]=center[axis]+offset
                        g=renderer.scene.geoms[renderer.scene.ngeom]
                        mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([.42,.44,.52,.45],dtype=np.float32))
                        mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_LINE,1.,a,b)
                        renderer.scene.ngeom+=1
                canvas=Image.new('RGB',(640,544),'#141625')
                canvas.paste(Image.fromarray(renderer.render()),(0,64))
                draw=ImageDraw.Draw(canvas)
                draw.text((16,8),job['title'],font=title_font,fill='#f4f1ff')
                draw.text((16,35),'MuJoCo · реальное время 1× · без сбросов и поддержки',font=small,fill='#bebad5')
                draw.rectangle((0,515,640,544),fill='#141625')
                draw.text((14,521),f't={tick*env.dt:4.1f} c   команда {command[0]:.2f} м/с   факт {vx:+.2f} м/с   опора {env.state["contacts"][0].sum()}/4',font=font,fill='#e8e4f6')
                writer.append_data(np.asarray(canvas));frames+=1
                if tick==0:canvas.save(output/(job['name']+'-first.png'))
        result=subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(mp4),'-filter_complex',
            '[0:v]fps=20,split[a][b];[a]palettegen=max_colors=128:stats_mode=full[p];[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle','-loop','0',str(gif)],capture_output=True,text=True)
        if result.returncode:raise RuntimeError(result.stderr)
    finally:
        renderer.close();env.close()
    (output/(job['name']+'-trajectory.json')).write_text(json.dumps(trajectory,indent=2)+'\n')
    record={**job,'checkpoint_sha256':hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        'checkpoint_steps':metadata.get('steps'),'checkpoint_updates':metadata.get('updates'),
        'dynamics_signature':dynamics_signature(),'seed':seed,'seconds':seconds,
        'fps':25,'video_frames':frames,'gif':str(gif),'mp4':str(mp4),'fell':fell,
        'self_contact_ticks':self_ticks,'terminal_stable_recovery':bool(env.stable[0]>=35),
        'mean_forward_velocity_mps':float(np.mean([r['vx'] for r in trajectory])),
        'auto_reset':False,'base_support':False,'policy':'deterministic CPU inference',
        'interpretation':'one labelled development-seed preview, not an independent quality evaluation'}
    return record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--only',help='Render one named job for visual QA')
    parser.add_argument('--seconds',type=float,default=10.)
    args=parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds<=0:raise ValueError('Positive finite duration required')
    torch.set_num_threads(1)
    plan=json.loads(args.plan.read_text());output=Path(plan['output']);output.mkdir(parents=True,exist_ok=True)
    manifest={'created_utc':datetime.now(timezone.utc).isoformat(),'jobs':[]}
    for job in plan['jobs']:
        if args.only and job['name']!=args.only:continue
        record=render(job,output,args.seconds)
        manifest['jobs'].append(record)
        (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps({k:record[k] for k in ('name','gif','mean_forward_velocity_mps','fell','terminal_stable_recovery')},ensure_ascii=False),flush=True)


if __name__=='__main__':main()

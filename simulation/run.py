"""Unassisted floating-base, torque-limited MuJoCo experiments. No root forces or pose resets."""
from __future__ import annotations
import argparse, json, math, time
from collections import deque
import numpy as np
import mujoco
from scripts.common import ROOT,load,joint_names
from simulation.model import home,ik,joint_spec

def ease(s):return s*s*s*(10+s*(-15+6*s))
def rz(t):return np.array([[math.cos(t),-math.sin(t),0],[math.sin(t),math.cos(t),0],[0,0,1.]])

def plan(c,task,duration):
    g=c['geometry'];h=(g['stance_height']+g['foot_radius'])/1000
    feet={leg:np.array([s['x_sign']*g['hip_x']/1000,s['y_sign']*(g['hip_y']+g['abduction_offset'])/1000,g['foot_radius']/1000]) for leg,s in c['legs'].items()}
    # Nominal robot COM is forward of torso; remove it when targeting support centroid.
    com_offset=.0168
    body=np.array([0.,0.,h]); yaw=0.; segments=[]
    def append(t,b,f,y,kind):
        nonlocal body,feet,yaw
        segments.append((t,body.copy(),b.copy(),{k:v.copy() for k,v in feet.items()},{k:v.copy() for k,v in f.items()},yaw,y,kind))
        body=b.copy();feet={k:v.copy() for k,v in f.items()};yaw=y
    append(2.,body,feet,yaw,'settle')
    if task=='sit':
        b=body.copy();b[2]=.120
        append(3,b,feet,yaw,'sit_down');append(2,b,feet,yaw,'sit_hold');b[2]=h;append(3,b,feet,yaw,'stand_up');append(2,b,feet,yaw,'settle')
    elif task in ('walk','turn'):
        b=body.copy();b[2]-=.035;append(2.,b,feet,yaw,'lower_for_walk')
        order=['fl','rr','fr','rl'];stride=.015
        # Each completed four-leg cycle advances by stride or turns by 3 degrees.
        cycles=math.ceil(1.03/stride) if task=='walk' else 31
        target_yaw=0.
        for cycle in range(cycles):
            target_yaw=0. if task=='walk' else min(math.pi/2,(cycle+1)*math.radians(3))
            for leg in order:
                support=np.mean([p for k,p in feet.items() if k!=leg],axis=0)
                b=body.copy();b[:2]=support[:2]-(rz(yaw)@np.array([com_offset,0,0]))[:2]
                append(1.0,b,feet,yaw,'shift')
                f={k:v.copy() for k,v in feet.items()}
                if task=='walk':f[leg][0]+=stride
                else:
                    s=c['legs'][leg];p=rz(target_yaw)@np.array([s['x_sign']*g['hip_x']/1000,s['y_sign']*(g['hip_y']+g['abduction_offset'])/1000,0]);f[leg][:2]=p[:2]
                append(1.0,b,f,yaw,'swing_'+leg)
                append(.25,b,feet,yaw,'settle')
            if task=='turn':append(1.2,body,feet,target_yaw,'yaw')
        center=np.mean(list(feet.values()),axis=0);b=body.copy();b[:2]=center[:2]-(rz(yaw)@np.array([com_offset,0,0]))[:2]
        append(2,b,feet,yaw,'recenter');append(2,b,feet,yaw,'settle')
    else:append(max(0,duration-2),body,feet,yaw,'stand')
    return segments

class Trajectory:
    def __init__(self,c,task,duration):
        self.c=c;self.segments=plan(c,task,duration);self.ends=np.cumsum([s[0] for s in self.segments]);self.duration=float(self.ends[-1]);self.unreachable=0
    def sample(self,t):
        i=min(int(np.searchsorted(self.ends,t,side='right')),len(self.segments)-1);s=self.segments[i];start=0 if i==0 else self.ends[i-1];p=float(np.clip((t-start)/max(s[0],1e-9),0,1));v=ease(p)
        _,b0,b1,f0,f1,y0,y1,kind=s;b=(1-v)*b0+v*b1;y=(1-v)*y0+v*y1
        feet={k:(1-v)*f0[k]+v*f1[k] for k in f0}
        if kind.startswith('swing_'):feet[kind[6:]][2]+=.013*math.sin(math.pi*p)**2
        self.last_feet=feet
        q=home(self.c);g=self.c['geometry']
        for leg,sgn in self.c['legs'].items():
            hip=np.array([sgn['x_sign']*g['hip_x']/1000,sgn['y_sign']*g['hip_y']/1000,0.])
            local=rz(y).T@(feet[leg]-b)-hip
            try:qs=ik(local,sgn['y_sign'],self.c)
            except ValueError:
                self.unreachable+=1
                # No silent reach projection; experiment fails and records its first cause.
                raise
            for j,val in zip(('hip_roll','hip_pitch','knee'),qs):q[leg+'_'+j]=val
        q['head_yaw']=.12*math.sin(.6*t);q['head_pitch']=.07*math.sin(.8*t)
        q['ear_left']=.18*math.sin(1.2*t);q['ear_right']=-.18*(math.sin(1.2*t+.4)-math.sin(.4))
        return np.array([q[n] for n in joint_names(self.c)]),b,y,kind

def run(task='stand',seconds=60.,mass_scale=1.,friction=.9,delay=1,backlash=.0174533,record=None,out=None,record_start=4.,record_seconds=16.):
    c=load();names=joint_names(c);model=mujoco.MjModel.from_xml_path(str(ROOT/'simulation/generated/sweetie_mini.xml'));data=mujoco.MjData(model)
    model.body_mass[1:]*=mass_scale;model.body_inertia[1:]*=mass_scale;model.geom_friction[:,0]=friction
    mujoco.mj_setConst(model,data);mujoco.mj_resetDataKeyframe(model,data,0)
    traj=Trajectory(c,task,seconds);duration=traj.duration if task!='stand' else seconds
    a=c['actuator_candidates'][c['selection']['selected']];cfg=c['control'];step=1/cfg['physics_hz'];period=1/cfg['policy_hz'];substeps=round(period/step)
    qids=np.array([model.jnt_qposadr[model.joint(n).id] for n in names]);dids=np.array([model.jnt_dofadr[model.joint(n).id] for n in names]);limits=np.array([joint_spec(n,c)['limits'] for n in names])
    homeq=np.array([home(c)[n] for n in names]);queue=deque([homeq.copy() for _ in range(delay+1)],maxlen=delay+1);target=homeq.copy()
    torque_peak=np.zeros(16);torque_sum2=np.zeros(16);maxspeed=np.zeros(16);max_tracking=0.; max_yaw_error=0.; angle_excess=0.;min_z=99.;max_tilt=0.;nonfoot=[];rows=[];error=None;writer=None;renderer=None;frames=0
    tstart=time.monotonic();initial=data.qpos[:3].copy();previous_q=homeq.copy();command_speed=np.zeros(16);next_frame=record_start
    if record:
        import imageio.v2 as imageio
        writer=imageio.get_writer(str(record),fps=20,codec='libx264',quality=7);renderer=mujoco.Renderer(model,height=720,width=960)
    try:
      for tick in range(math.ceil(duration/period)):
        t=tick*period
        try:desired,bref,yref,phase=traj.sample(t)
        except ValueError as e:error=str(e);break
        Rnow=np.zeros(9);mujoco.mju_quat2Mat(Rnow,data.qpos[3:7]);Rnow=Rnow.reshape(3,3);ynow=math.atan2(Rnow[1,0],Rnow[0,0])
        perr=data.qpos[:3]-bref; yerr=math.atan2(math.sin(ynow-yref),math.cos(ynow-yref))
        max_tracking=max(max_tracking,float(np.linalg.norm(perr[:2])));max_yaw_error=max(max_yaw_error,abs(yerr))
        # Convert world-fixed contact anchors through the measured floating-base pose.
        # This is simulation state feedback, not an externally imposed base trajectory.
        try:
            g=c['geometry']
            for idx,(leg,sgn) in enumerate(c['legs'].items()):
                hip=np.array([sgn['x_sign']*g['hip_x']/1000,sgn['y_sign']*g['hip_y']/1000,0.])
                pose_for_ik=data.qpos[:3].copy();pose_for_ik[2]=bref[2]
                local=Rnow.T@(traj.last_feet[leg]-pose_for_ik)-hip
                desired[idx*3:idx*3+3]=ik(local,sgn['y_sign'],c)
        except ValueError as e:error='State-feedback IK: '+str(e);break
        # Virtual model control: desired support wrench becomes joint torques through foot Jacobians.
        # No wrench is applied to the floating base. Real hardware would need calibrated position offsets.
        active=[leg for leg in c['legs'] if phase!='swing_'+leg]
        A=np.zeros((6,3*len(active)))
        com=data.subtree_com[1]
        for idx,leg in enumerate(active):
            r=data.site_xpos[model.site(leg+'_foot').id]-com
            A[:3,idx*3:idx*3+3]=np.eye(3)
            A[3:,idx*3:idx*3+3]=np.array([[0,-r[2],r[1]],[r[2],0,-r[0]],[-r[1],r[0],0]])
        m=model.body_mass.sum()
        pitch=math.asin(float(np.clip(-Rnow[2,0],-1,1)));roll=math.atan2(Rnow[2,1],Rnow[2,2])
        force=m*(np.array([75.,75.,100.])*(bref-data.qpos[:3])-np.array([14.,14.,18.])*data.qvel[:3])+np.array([0,0,m*9.81])
        moment=np.array([-3.0*roll-.35*data.qvel[3],-3.0*pitch-.35*data.qvel[4],-1.3*yerr-.18*data.qvel[5]])
        wrench=np.r_[force,moment]
        forces=np.linalg.lstsq(A,wrench,rcond=1e-5)[0].reshape(-1,3)
        feedforward=np.zeros(16)
        for idx,leg in enumerate(active):
            f=forces[idx];f[2]=max(0,f[2]);tangent=np.linalg.norm(f[:2]);max_tangent=friction*f[2]*.8
            if tangent>max_tangent and tangent>0:f[:2]*=max_tangent/tangent
            jac=np.zeros((3,model.nv));rotjac=np.zeros_like(jac);mujoco.mj_jacSite(model,data,jac,rotjac,model.site(leg+'_foot').id)
            feedforward-=jac[:,dids].T@f
        feedforward+=data.qfrc_bias[dids]
        kp=np.full(16,cfg['kp']);kd=np.full(16,cfg['kd'])
        for idx,leg in enumerate(c['legs']):
            if leg in active:kp[idx*3:idx*3+3]=.8;kd[idx*3:idx*3+3]=.12
        desired=np.clip(desired,limits[:,0],limits[:,1])
        desired=previous_q+np.clip(desired-previous_q,-.8*period,.8*period)
        angle_excess=max(angle_excess,float(np.max(np.maximum(limits[:,0]-desired,desired-limits[:,1]))))
        desired=np.clip(desired,limits[:,0],limits[:,1]);command_speed=np.maximum(command_speed,np.abs(desired-previous_q)/period);previous_q=desired.copy();queue.append(desired);target=queue[0]
        for substep in range(substeps):
            q=data.qpos[qids];vel=data.qvel[dids];err=target-q;err=np.sign(err)*np.maximum(np.abs(err)-backlash,0)
            # Uncalibrated DC torque-speed envelope at low battery; no ideal position actuators.
            voltage_factor=c['power']['battery'].get('cutoff_voltage',10.5)/a['voltage']
            speed=a['no_load_speed']*voltage_factor
            limit=a['stall_torque']*voltage_factor*c['selection']['transient_fraction_of_stall']*np.maximum(0,1-np.abs(vel)/speed)
            tau=np.clip(kp*err-kd*vel+feedforward,-limit,limit);data.ctrl[:]=tau
            mujoco.mj_step(model,data)
            torque_peak=np.maximum(torque_peak,np.abs(tau));torque_sum2+=tau*tau;maxspeed=np.maximum(maxspeed,np.abs(vel))
        min_z=min(min_z,float(data.qpos[2]));R=np.zeros(9);mujoco.mju_quat2Mat(R,data.qpos[3:7]);tilt=math.acos(float(np.clip(R.reshape(3,3)[2,2],-1,1)));max_tilt=max(max_tilt,tilt)
        for contact in data.contact[:data.ncon]:
            n1=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,contact.geom1);n2=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,contact.geom2)
            if 'floor' in (n1,n2):
                other=n2 if n1=='floor' else n1
                if not other.endswith('_shin_g1') and len(nonfoot)<20:nonfoot.append({'t':t,'geometry':other})
            elif contact.dist<-.002 and len(nonfoot)<20:nonfoot.append({'t':t,'self_collision':[n1,n2],'penetration_m':float(contact.dist)})
        if tick%5==0:rows.append(dict(t=t,x=float(data.qpos[0]),y=float(data.qpos[1]),z=float(data.qpos[2]),tilt_rad=tilt,phase=phase))
        if renderer is not None and t+1e-9>=next_frame and t<record_start+record_seconds:
            camera=mujoco.MjvCamera();camera.lookat[:]=data.qpos[:3]+np.array([0,0,.02]);camera.distance=.72;camera.azimuth=135;camera.elevation=-22
            renderer.update_scene(data,camera);writer.append_data(renderer.render());frames+=1;next_frame+=1/20
        if not np.isfinite(data.qpos).all() or tilt>.8 or data.qpos[2]<.065:
            error=f'Fallen or nonfinite state at {t:.2f}s';break
    finally:
        if writer:writer.close()
        if renderer:renderer.close()
    R=np.zeros(9);mujoco.mju_quat2Mat(R,data.qpos[3:7]);R=R.reshape(3,3);yaw=math.atan2(R[1,0],R[0,0]);elapsed=float(data.time)
    complete=elapsed>=duration-period
    result=dict(task=task,parameters=dict(mass_scale=mass_scale,friction=friction,delay_ticks=delay,backlash_rad=backlash),
      requested_duration_s=duration,simulated_duration_s=elapsed,wall_time_s=time.monotonic()-tstart,
      max_tracking_error_m=max_tracking,max_yaw_error_rad=max_yaw_error,displacement_m=(data.qpos[:3]-initial).tolist(),yaw_rad=yaw,min_base_height_m=min_z,max_tilt_rad=max_tilt,
      max_command_limit_excess_rad=angle_excess,max_command_speed_rad_s=dict(zip(names,command_speed.tolist())),
      peak_torque_nm=dict(zip(names,torque_peak.tolist())),rms_torque_nm=dict(zip(names,np.sqrt(torque_sum2/max(1,int(elapsed/step))).tolist())),max_joint_speed_rad_s=dict(zip(names,maxspeed.tolist())),
      unexpected_contacts=nonfoot,error=error,complete=complete,frames=frames,
      recording_window_s=[record_start,record_start+record_seconds] if record else None,
      physics='floating base; no applied root forces, welds or pose resets; uncalibrated torque-speed PD + backlash deadband')
    criteria=complete and error is None and angle_excess<1e-6 and not nonfoot and max_tilt<.35 and max_tracking<.06 and max_yaw_error<.15
    if task=='walk':criteria=criteria and result['displacement_m'][0]>=1.0 and abs(result['displacement_m'][1])<.05
    if task=='turn':criteria=criteria and abs(yaw-math.pi/2)<.15
    if task=='stand':criteria=criteria and np.linalg.norm(np.array(result['displacement_m'])[:2])<.025
    result['passed']=bool(criteria)
    if out:
        out=ROOT/out;out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n');out.with_suffix('.trajectory.json').write_text(json.dumps(rows)+'\n')
    print(json.dumps({k:result[k] for k in ['task','passed','simulated_duration_s','displacement_m','yaw_rad','max_tilt_rad','error','unexpected_contacts']},indent=2))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--task',choices=['stand','walk','turn','sit'],default='stand');p.add_argument('--seconds',type=float,default=60);p.add_argument('--mass-scale',type=float,default=1);p.add_argument('--friction',type=float,default=.9);p.add_argument('--delay',type=int,default=1);p.add_argument('--backlash',type=float,default=.0174533);p.add_argument('--record',type=str);p.add_argument('--out',default='reports/sim_stand.json');args=p.parse_args()
    result=run(args.task,args.seconds,args.mass_scale,args.friction,args.delay,args.backlash,args.record,args.out)
    raise SystemExit(0 if result['passed'] else 1)

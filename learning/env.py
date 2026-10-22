"""Batched, unassisted MuJoCo environments with deployable sensor observations.

All root state used below is reward/reset/evaluation-only. The actor sees only
IMU-like signals, encoders, commands, previous actions and an internal gait clock.
"""
from __future__ import annotations
import hashlib
import json
from collections import deque
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scripts.common import ROOT, load
from simulation.model import home


def quaternion_matrix(q):
    w,x,y,z = np.asarray(q).T
    return np.stack((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
                     2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
                     2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)), -1).reshape(-1,3,3)


def running_flight_bonus(velocity, commands, up, height, contacts, bad_contacts):
    """Small forward-flight incentive, never a reward for jumping in place.

    Flight is a training hypothesis, not evidence that a checkpoint can run.
    All inputs here are privileged reward measurements, absent from the actor.
    """
    error=np.square(velocity[:,:2]-commands[:,:2]).sum(1)
    tracking=np.exp(-error/.025)
    valid=((velocity[:,0]>=.35)&(commands[:,0]>=.30)&(up>.8)&
           (height>.15)&(height<.28)&(np.abs(velocity[:,2])<.6)&
           (error<.04)&~np.asarray(contacts,dtype=bool).any(1)&(bad_contacts==0))
    return .2*tracking*valid


def stair_profile(difficulty,extent=10.):
    """Five-riser flights and landings, mirrored around a flat spawn pad."""
    if not np.isfinite(difficulty) or not 0<=difficulty<=1:raise ValueError('Invalid stair difficulty')
    rise=.01+.03*difficulty;tread=.20;landing=.80;n=5
    positive=[(0.,landing,0.)];x=landing
    while x<extent:
        for height,width in ([(rise*(i+1),tread) for i in range(n)]+[(rise*n,landing)]+
                             [(rise*(n-1-i),tread) for i in range(n)]+[(0.,landing)]):
            if x>=extent:break
            end=min(extent,x+width);positive.append((x,end,height));x=end
    segments=[(-b,-a,h) for a,b,h in reversed(positive)]+positive
    return dict(riser_m=rise,tread_m=tread,risers_per_flight=n,landing_m=landing,
                segments_m=[list(row) for row in segments])


def stair_height(x,profile):
    rows=np.asarray(profile['segments_m']);indices=np.searchsorted(rows[:,0],x,side='right')-1
    return rows[np.clip(indices,0,len(rows)-1),2]


def make_model(terrain='flat', difficulty=0., seed=1):
    """Current CAD mass properties, convex assembly collisions and physical sensors."""
    difficulty=float(np.clip(difficulty,0,1))
    if terrain=='mixed': terrain='rough'
    if terrain not in ('flat','rough','slope','steps','stairs'): raise ValueError(terrain)
    source=ROOT/'simulation/generated/sweetie_mini.xml'
    extent=10.;n_samples=321
    fingerprint=hashlib.sha256(source.read_bytes()+f'{terrain}/{difficulty}/{seed}/rl-v05-mechanics-1'.encode()).hexdigest()[:16]
    directory=ROOT/'learning/models'; directory.mkdir(parents=True,exist_ok=True)
    path=directory/f'{terrain}_{fingerprint}.xml'
    metadata={'terrain':terrain,'difficulty':difficulty,'seed':int(seed),'slope_radians':0.,'height_range_m':0.,
              'extent_m':extent,'n_samples':n_samples}
    tree=ET.parse(source); root=tree.getroot()
    for parent in root.iter():
        for item in list(parent):
            if (item.tag=='mesh' and item.get('name','').endswith('_visual')) or (item.tag=='geom' and item.get('name','').endswith('_cad')):
                parent.remove(item)
    for mesh in root.iter('mesh'):
        if mesh.get('file'):mesh.set('file',str((source.parent/mesh.get('file')).resolve()))
    for geom in root.iter('geom'):
        if geom.get('group')=='3':
            geom.set('group','0'); geom.set('rgba','.90 .90 .96 1')
    root.find('option').set('iterations','40')
    world=root.find('worldbody'); floor=world.find("geom[@name='floor']")
    floor.set('size','12 12 .1')
    if terrain=='slope':
        angle=np.deg2rad(8*difficulty); floor.set('quat',f'{np.cos(angle/2)} 0 {-np.sin(angle/2)} 0')
        metadata['slope_radians']=float(angle)
    elif terrain=='rough':
        rng=np.random.default_rng(seed); n=n_samples
        axis=np.linspace(-extent,extent,n); x,y=np.meshgrid(axis,axis)
        h=np.zeros((n,n))
        for period,weight in ((.35,.5),(.17,.3),(.09,.2)):
            h+=weight*np.sin(x*2*np.pi/period+rng.uniform(0,6.28))*np.cos(y*2*np.pi/period+rng.uniform(0,6.28))
        h=(h-h.min())/(h.max()-h.min())
        binary=directory/f'{path.stem}.bin'
        with binary.open('wb') as f:
            np.array([n,n],dtype=np.int32).tofile(f);h.astype(np.float32).tofile(f)
        amplitude=max(.00001,.025*difficulty)
        ET.SubElement(root.find('asset'),'hfield',name='terrain',file=binary.name,size=f'{extent} {extent} {amplitude} .1')
        ET.SubElement(world,'geom',name='terrain',type='hfield',hfield='terrain',rgba='.53 .62 .46 1',friction='.9 .015 .001',contype='1',conaffinity='7')
        floor.set('pos','0 0 -.101')
        metadata.update(height_range_m=amplitude,height_samples=h.tolist())
    elif terrain=='stairs':
        metadata.update(stair_profile(difficulty,extent))
        metadata['height_range_m']=metadata['riser_m']*metadata['risers_per_flight']
        for i,(a,b,top) in enumerate(metadata['segments_m']):
            ET.SubElement(world,'geom',name=f'stair_{i}',type='box',size=f'{(b-a)/2} {extent} {(.1+top)/2}',
                          pos=f'{(a+b)/2} 0 {(top-.1)/2}',rgba='.62 .65 .67 1',contype='1',conaffinity='7')
        floor.set('pos','0 0 -.101')
    elif terrain=='steps':
        height=.025*difficulty; metadata['height_range_m']=height
        metadata['n_steps']=int(2*extent/.1)
        for i in range(metadata['n_steps']):
            x=-extent+(i+.5)*.1; top=height*(i%6)/5
            ET.SubElement(world,'geom',name=f'step_{i}',type='box',size=f'.05 {extent} {(.1+top)/2}',pos=f'{x} 0 {(top-.1)/2}',rgba='.62 .65 .67 1',contype='1',conaffinity='7')
    ET.indent(root,space='  '); tree.write(path,encoding='unicode')
    public={k:v for k,v in metadata.items() if k!='height_samples'}
    path.with_suffix('.json').write_text(json.dumps(public,indent=2)+'\n')
    return path,metadata


class SweetieVecEnv:
    observation_mode='legacy'
    observation_contract="v05-sensor-suite-1"
    reward_version="v05-recovery-waypoints-1"
    action_dim=12
    frame_dim=197
    obs_dim=591
    dt=.02
    reset_kind_names=('stand','folded_crouch','left','right','supine')
    def __init__(self,task='locomotion',nenvs=None,num_envs=128,threads=16,seed=1,terrain='flat',difficulty=0.,randomize=True,auto_reset=True,observation_mode='legacy'):
        if task not in ('locomotion','running','recovery'):raise ValueError(task)
        if observation_mode not in ('legacy','compact'):raise ValueError(observation_mode)
        self.observation_mode=observation_mode
        if observation_mode=='compact':
            from hardware.policy_observation import CONTRACT,DIM,CRITIC_CONTRACT,CRITIC_DIM
            self.observation_contract=CONTRACT;self.obs_dim=DIM
            self.critic_observation_contract=CRITIC_CONTRACT;self.critic_obs_dim=CRITIC_DIM
        self.task=task; self.num_envs=int(nenvs if nenvs is not None else num_envs)
        self.seed=int(seed); self.rng=np.random.default_rng(seed);self.randomize=randomize;self.auto_reset=auto_reset
        self.difficulty=float(difficulty);self.progress=0.;self.noise_enabled=bool(randomize)
        self.model_path,self.terrain=make_model(terrain,difficulty,seed)
        # Reset poses must clear the actual terrain under every link, including
        # folded legs and the horn. A center-point terrain height alone permits
        # deep initial penetrations and artificial get-up impulses.
        import mujoco
        self._reset_model=mujoco.MjModel.from_xml_path(str(self.model_path))
        self._reset_data=mujoco.MjData(self._reset_model)
        from learning.build_native import build
        build()  # A changed servo source must never silently reuse an old binary.
        from learning._sweetie_native import Batch
        self.native=Batch(str(self.model_path),self.num_envs,threads)
        self.home=np.array(list(home().values()))
        c=load();from simulation.model import joint_spec
        self.names=list(home())
        limits=np.array([joint_spec(n,c)['limits'] for n in self.names]);self.low,self.high=limits.T
        n=self.num_envs
        self.commands=np.zeros((n,3));self.steps=np.zeros(n,dtype=int);self.phase=np.zeros(n)
        self.command_targets=np.zeros((n,3))
        self.last_action=np.zeros((n,12));self.targets=np.tile(self.home,(n,1))
        self.history=np.zeros((n,3,self.frame_dim),dtype=np.float32)
        self.encoder_bias=np.zeros((n,16));self.returns=np.zeros(n);self.stable=np.zeros(n,int);self.success=np.zeros(n,bool)
        self.reset_kind=np.zeros(n,int);self.params=np.zeros((n,8));self.velocity_error=np.zeros(n)
        self.nominal_height=(c['geometry']['stance_height']+c['geometry']['foot_radius'])/1000
        from scripts.common import total_mass
        self.weight_newtons=max(.1,total_mass(c)*9.81)
        self.previous_up=np.ones(n);self.previous_height=np.full(n,self.nominal_height)
        from learning.sensors import SensorSuite
        self.sensors=SensorSuite(n,seed+809,bool(randomize));self.previous_potential=np.zeros(n);self.flight_seconds=np.zeros(n);self.previous_feet=np.zeros((n,4,3));self.foot_velocity=np.zeros((n,4,3))
        from learning.rewards import FootMotion,RiseMotion,RecoveryWaypoints
        self.foot_motion=FootMotion(n,self.dt);self.rise_motion=RiseMotion(n,self.dt)
        self.recovery_waypoints=RecoveryWaypoints(n,self.dt)
        self.max_episode_steps=500 if task=='recovery' else 600
        # Keep a bounded diagnostic tail during day-long runs; lifetime and
        # episode disturbance counts remain available in every step's info.
        self.manual_command=False;self.events=deque(maxlen=4096);self.state={}
        self.next_push_step=np.full(n,10**9,dtype=int)
        self.disturbance_count=np.zeros(n,dtype=int);self.total_disturbances=0
        self.reset()

    def set_progress(self,progress):self.progress=float(np.clip(progress,0,1))

    def ground_height(self,x,y):
        x,y=np.broadcast_arrays(x,y); kind=self.terrain['terrain']
        if kind=='slope':return np.tan(self.terrain['slope_radians'])*x
        extent=self.terrain['extent_m']
        if kind=='stairs':return stair_height(x,self.terrain)
        if kind=='steps':
            index=np.clip(np.floor((x+extent)/.1).astype(int),0,self.terrain['n_steps']-1)
            return self.terrain['height_range_m']*(index%6)/5
        if kind=='rough':
            h=np.asarray(self.terrain['height_samples']);last=self.terrain['n_samples']-1
            a=np.clip((x+extent)/(2*extent)*last,0,last);b=np.clip((y+extent)/(2*extent)*last,0,last)
            i=np.minimum(a.astype(int),last-1);j=np.minimum(b.astype(int),last-1);u=a-i;v=b-j
            return ((1-u)*(1-v)*h[j,i]+u*(1-v)*h[j,i+1]+(1-u)*v*h[j+1,i]+u*v*h[j+1,i+1])*self.terrain['height_range_m']
        return np.zeros_like(x,dtype=float)

    def _sample_commands(self,ids):
        if self.manual_command:return
        k=len(ids);r=self.rng
        if self.observation_mode=='compact':
            target=np.zeros((k,3))
            if self.task=='running':
                target[:,0]=r.uniform(.35,.55,k)
                target[:,1]=r.uniform(-.05,.05,k);target[:,2]=r.uniform(-.35,.35,k)
                target[r.random(k)<.1]=0
            elif self.task=='locomotion':
                modes=r.random(k)
                turn=(modes>=.20)&(modes<.35);forward=(modes>=.35)&(modes<.55)
                backward=(modes>=.55)&(modes<.65);side=(modes>=.65)&(modes<.75);mixed=modes>=.75
                target[turn,2]=r.choice([-1.,1.],turn.sum())*r.uniform(.2,1.,turn.sum())
                target[forward,0]=r.uniform(.08,.30,forward.sum())
                target[backward,0]=-r.uniform(.08,.18,backward.sum())
                target[side,1]=r.choice([-1.,1.],side.sum())*r.uniform(.04,.12,side.sum())
                target[mixed,0]=r.uniform(-.15,.30,mixed.sum())
                target[mixed,1]=r.uniform(-.10,.10,mixed.sum());target[mixed,2]=r.uniform(-.6,.6,mixed.sum())
            self.command_targets[ids]=target
            reset=self.steps[ids]==0
            self.commands[ids[reset]]=target[reset]
            return
        if self.task=='running':
            # Start from a forward gait and expand speed, lateral and turn
            # demands together; this is a command target, not a speed claim.
            self.commands[ids,0]=r.uniform(.30,.30+.35*self.progress,k)
            self.commands[ids,1]=r.uniform(-.08,.08,k)*min(1,self.progress*3)
            self.commands[ids,2]=r.uniform(-.6,.6,k)*min(1,self.progress*3)
            self.commands[ids[r.random(k)<.1]]=0
            return
        self.commands[ids]=0
        if self.task=='recovery':return
        # Explicit buckets train exact idle, backward, sideways and turn-in-place.
        modes=r.random(k);forward=(modes>=.20)&(modes<.55)
        backward=(modes>=.55)&(modes<.70);side=(modes>=.70)&(modes<.85);turn=modes>=.85
        self.commands[ids[forward],0]=r.uniform(.12,.25+.35*self.progress,forward.sum())
        self.commands[ids[backward],0]=-r.uniform(.08,.15+.15*self.progress,backward.sum())
        self.commands[ids[side],1]=r.choice([-1.,1.],side.sum())*r.uniform(.06,.10+.06*self.progress,side.sum())
        self.commands[ids[turn],2]=r.choice([-1.,1.],turn.sum())*r.uniform(.20,.35+.65*self.progress,turn.sum())

    def _clear_reset_penetration(self,qpos):
        """Lift only episode initializations; never correct an active episode."""
        import mujoco
        model,data=self._reset_model,self._reset_data
        for row in qpos:
            data.qpos[:]=row;data.qvel[:]=0
            for _ in range(32):
                mujoco.mj_forward(model,data)
                lift=0.
                for contact in data.contact:
                    a,b=contact.geom1,contact.geom2
                    if (model.geom_bodyid[a]==0)==(model.geom_bodyid[b]==0):continue
                    if contact.dist < -.000001:
                        # Plane slopes and heightfield facets require a larger
                        # vertical lift than their signed normal penetration.
                        lift=max(lift,-contact.dist/max(abs(contact.frame[2]),.2)+.001)
                if lift==0:break
                # Near-vertical step faces can produce horizontal contact
                # normals; short lifts avoid launching a reset high above them.
                data.qpos[2]+=min(lift,.01)
            else:
                raise RuntimeError('Could not initialize robot clear of terrain')
            row[2]=data.qpos[2]

    def _clear_reset_self_collision(self,qpos,reference):
        """Reject invalid reset jitter; never repair or teleport active episodes.

        Joint limits alone do not certify the Cartesian product of leg angles.
        Shrink jitter toward the requested reset kind, preserving its root pose.
        An invalid reference pose is a model error and stops the run.
        """
        import mujoco
        model,data=self._reset_model,self._reset_data
        for row,base in zip(qpos,reference):
            delta=row[7:]-base[7:]
            for attempt in range(9):
                row[7:]=np.clip(base[7:]+delta*(.5**attempt if attempt<8 else 0.),self.low,self.high)
                data.qpos[:]=row;data.qpos[2]=2.;data.qvel[:]=0
                mujoco.mj_forward(model,data)
                if not any(c.dist<-.00001 and model.geom_bodyid[c.geom1]>0 and model.geom_bodyid[c.geom2]>0 for c in data.contact):break
            else:raise RuntimeError('Requested reset kind self-collides; mechanical/physics acceptance must be repeated')

    def _reset_indices(self,ids,pose=None):
        ids=np.asarray(ids,dtype=np.int64);k=len(ids)
        if not k:return
        r=self.rng;q=np.zeros((k,23));v=np.zeros((k,22));q[:,3]=1;q[:,7:]=self.home
        q[:,:2]=r.uniform(-1.2,1.2,(k,2)) if self.auto_reset else 0
        yaw=r.uniform(-np.pi,np.pi,k) if self.auto_reset else np.zeros(k)
        if self.terrain['terrain']=='stairs' and self.auto_reset:
            q[:,:2]=np.column_stack((r.uniform(-6,6,k),r.uniform(-.6,.6,k)))
            yaw=r.choice([0.,np.pi],k)+r.uniform(-.2,.2,k)
        q[:,3]=np.cos(yaw/2);q[:,6]=np.sin(yaw/2)
        kinds=np.zeros(k,int)
        if self.task=='recovery':
            from learning.curriculum import recovery_stage
            stage=recovery_stage(self.progress)
            kinds=r.choice(5,k,p=stage['probabilities'])
        if pose is not None:kinds[:]=dict(stand=0,folded_crouch=1,prone=1,left=2,right=3,supine=4)[pose]
        for i,kind in enumerate(kinds):
            z=self.nominal_height
            if kind==0 and self.terrain['terrain']=='slope':
                # Initialize a neutral stance tangent to its support plane.
                # This is episode setup, never a mid-episode pose correction.
                tilt=-self.terrain['slope_radians']/2;heading=yaw[i]/2
                ct,st=np.cos(tilt),np.sin(tilt);ch,sh=np.cos(heading),np.sin(heading)
                q[i,3:7]=[ct*ch,st*sh,st*ch,ct*sh]
            if kind==1:
                # Legacy pose name "prone" means this folded upright crouch.
                # It is easier than lying on the torso and must be reported as
                # folded_crouch, separately from side/inverted recovery.
                q[i,7:19]=np.tile([0,-1.2,2.05],4);z=.10
            elif kind in (2,3,4):
                angle={2:np.pi/2,3:-np.pi/2,4:np.pi}[kind]
                if self.randomize and self.auto_reset:
                    # Reverse curriculum: starts partway through the roll.
                    if r.random()>stage['full_fall_probability']:
                        angle*=r.uniform(.15,stage['roll_scale'])
                if self.randomize:
                    # Rz(yaw) Ry(pitch) Rx(roll): diversify falls rather than
                    # overwriting the heading with one exact side orientation.
                    roll=angle+r.uniform(-.3,.3);pitch=r.uniform(-.4,.4);heading=r.uniform(-np.pi,np.pi)
                    cr,sr=np.cos(roll/2),np.sin(roll/2);cp,sp=np.cos(pitch/2),np.sin(pitch/2)
                    cy,sy=np.cos(heading/2),np.sin(heading/2)
                    q[i,3:7]=[cy*cp*cr+sy*sp*sr,cy*cp*sr-sy*sp*cr,
                              cy*sp*cr+sy*cp*sr,sy*cp*cr-cy*sp*sr]
                else:q[i,3:7]=[np.cos(angle/2),np.sin(angle/2),0,0]
                z=.32
            elif self.task=='recovery' and r.random()<.5 and pose is None:
                # Near-goal starts teach the last part of the rise first.
                blend=r.uniform(.1,.6)
                q[i,7:19]=(1-blend)*self.home[:12]+blend*np.tile([0,-1.2,2.05],4)
                z=max(.15,self.nominal_height*(1-.5*blend))
            q[i,2]=z+self.ground_height(q[i,0],q[i,1])
        reference=q.copy()
        if self.randomize:
            jitter=.12 if self.task=='recovery' else .03
            q[:,7:19]+=r.normal(0,jitter,(k,12));q[:,7:]=np.clip(q[:,7:],self.low,self.high)
        self._clear_reset_self_collision(q,reference)
        self._clear_reset_penetration(q)
        # Conservative dynamics randomization; specific STS parameters remain unmeasured.
        p=np.tile([1.,.9,9.9,1.,1.,.00872665,1.,1.],(k,1))
        if self.randomize:
            spread=.3+.7*min(1,self.progress*3)
            p[:,0]=r.uniform(1-.15*spread,1+.2*spread,k)
            p[:,1]=r.uniform(.55,1.15,k);p[:,2]=r.uniform(9.3,10.8,k)
            p[:,3]=r.uniform(.85,1.05,k);p[:,4]=r.uniform(.9,1.1,k)
            p[:,5]=r.uniform(.005,.035,k);p[:,6]=r.integers(0,4,k);p[:,7]=r.uniform(.85,1.15,k)
            self.encoder_bias[ids]=r.uniform(-.015,.015,(k,16))
        self.native.reset(ids,q,v,p);self.params[ids]=p;self.reset_kind[ids]=kinds
        self.steps[ids]=0;self.phase[ids]=r.uniform(0,2*np.pi,k);self.last_action[ids]=0;self.targets[ids]=q[:,7:]
        self.returns[ids]=0;self.stable[ids]=0;self.success[ids]=False;self.velocity_error[ids]=0
        self.disturbance_count[ids]=0
        self.next_push_step[ids]=r.integers(200,501,k) if self.randomize and self.auto_reset else 10**9
        self._sample_commands(ids);self._read_state()
        self.previous_up[ids]=self.rot[ids,2,2];self.previous_height[ids]=self.height[ids]
        self.sensors.reset(ids,self.state['imu_quat'][ids]);self.flight_seconds[ids]=0;self.previous_feet[ids]=self.state['feet'][ids]
        self.foot_motion.reset(ids)
        self.rise_motion.reset(ids,self.height[ids],self.state['qvel'][ids,2])
        self.recovery_waypoints.reset(ids,self.rot[ids,2,2],self.height[ids],self.nominal_height,self.state['contacts'][ids])
        from learning.rewards import recovery_potential
        pose_error=np.square(self.state['qpos'][ids,7:19]-self.home[:12]).mean(1)
        self.previous_potential[ids]=recovery_potential(self.rot[ids,2,2],self.height[ids],self.nominal_height,pose_error)
        frames=self._frame()[ids];self.history[ids]=frames[:,None,:]

    def _read_state(self):
        self.state=self.native.state();q=self.state['qpos'];self.rot=quaternion_matrix(q[:,3:7])
        self.height=q[:,2]-self.ground_height(q[:,0],q[:,1]);self.state['height_above_ground']=self.height.copy()
        self.local_velocity=np.einsum('nji,nj->ni',self.rot,self.state['qvel'][:,:3])
        # Free-joint rotational velocity is already local in MuJoCo.
        self.local_angular=self.state['qvel'][:,3:6]

    def _frame(self):
        q=self.state['qpos'][:,7:]-self.home+self.encoder_bias
        dq=self.state['qvel'][:,6:]
        gravity,gyro,accel,contact,contact_valid,imu_valid,imu_age,depth,depth_valid,depth_age=self.sensors.frame()
        if self.noise_enabled:
            q=q+self.rng.normal(0,.003,q.shape);dq=dq+self.rng.normal(0,.15,dq.shape)
        from hardware.observation import encode
        frame=encode(self.sensors.frame(),q,dq,self.last_action,self.commands,self.phase)
        assert frame.shape==(self.num_envs,self.frame_dim),frame.shape
        return frame.astype(np.float32)

    def observe(self):
        if self.observation_mode=='compact':
            from hardware.policy_observation import from_sensor_frame
            return from_sensor_frame(self.history[:,-1]).copy()
        return self.history.reshape(self.num_envs,-1).copy()

    def teacher_observation(self):
        """Legacy teacher input is training-only, never passed to compact actor."""
        return self.history.reshape(self.num_envs,-1).copy()

    def observe_critic(self):
        if self.observation_mode!='compact':return self.observe()
        from hardware.policy_observation import encode,CRITIC_DIM
        n=self.num_envs
        clean=(-self.rot[:,2,:],self.local_angular*.25,None,
               self.state['contacts'].astype(float),np.ones((n,4)),np.ones((n,1)),
               np.zeros((n,1)),None,None,None,None)
        core=encode(clean,self.state['qpos'][:,7:]-self.home,self.state['qvel'][:,6:],self.last_action,self.commands)
        feet=self.state['feet']
        clearance=feet[:,:,2]-self.ground_height(feet[:,:,0],feet[:,:,1])
        forces=np.clip(self.state['foot_force_samples'][:,-1,:]/self.weight_newtons,0,10)
        collision=np.stack((self.state['bad_contacts'],self.state['self_contacts']),1)/10.
        obs=np.concatenate((core,self.local_velocity,(self.height/self.nominal_height)[:,None],
                            forces,clearance,self.foot_motion.air,collision),1).astype(np.float32)
        if obs.shape!=(n,CRITIC_DIM) or not np.isfinite(obs).all():raise ValueError('Invalid critic observation')
        return obs

    def reset(self):
        self._reset_indices(np.arange(self.num_envs));return self.observe()

    def reset_pose(self,pose='stand'):
        """Initialize a pose; 'prone' is the legacy folded-crouch name."""
        self._reset_indices(np.arange(self.num_envs),pose);return self.observe()

    def set_command(self,vx,vy,yaw):
        self.manual_command=True
        low,high=([0.,-.08,-.6],[.65,.08,.6]) if self.task=='running' else ([-.45,-.2,-1.5],[.6,.2,1.5])
        self.commands[:]=np.clip([vx,vy,yaw],low,high)
        self.command_targets[:]=self.commands
        self.history[:,-1,:]=self._frame()

    def apply_impulse(self,dv):
        dv=np.broadcast_to(np.asarray(dv,float),(self.num_envs,6)).copy()
        self.native.push(dv);self.events.append({'event':'velocity_impulse','delta_velocity':dv.tolist(),'step':self.steps.tolist()});self._read_state()

    def fall(self,direction='left'):
        self.apply_impulse([0,0,0,9. if direction=='left' else -9.,0,0])

    def _training_disturbances(self):
        """Zero-mean physical velocity kicks; no vertical lift or base support."""
        if not (self.auto_reset and self.randomize and self.progress>.2):return
        ids=np.flatnonzero(self.steps>=self.next_push_step)
        if not len(ids):return
        dv=np.zeros((self.num_envs,6))
        dv[ids,:2]=self.rng.uniform(-.15,.15,(len(ids),2))*self.progress
        dv[ids,3:5]=self.rng.uniform(-.4,.4,(len(ids),2))*self.progress
        self.native.push(dv)
        self.disturbance_count[ids]+=1;self.total_disturbances+=len(ids)
        self.next_push_step[ids]=self.steps[ids]+self.rng.integers(200,501,len(ids))
        self.events.append({'event':'training_velocity_impulse','count':int(len(ids)),
                            'mean_abs_delta_velocity':np.abs(dv[ids]).mean(0).tolist(),
                            'total_count':int(self.total_disturbances)})

    def step(self,actions):
        actions=np.asarray(actions,dtype=float)
        if actions.shape!=(self.num_envs,12) or not np.isfinite(actions).all():raise ValueError('Expected finite [N,12] actions')
        actions=np.clip(actions,-1,1)
        desired=np.tile(self.home,(self.num_envs,1))
        if self.task=='recovery':desired[:,:12]=self.state['qpos'][:,7:19]+.35*actions
        else:desired[:,:12]+=.65*actions
        self.targets+=np.clip(desired-self.targets,-.11,.11)
        self.targets=np.clip(self.targets,self.low,self.high)
        previous_torque=self.state['torque_rms'].copy()
        self._training_disturbances()
        self.native.step(self.targets);self.steps+=1;self._read_state()
        if self.state['physics_warnings'].any():raise RuntimeError('MuJoCo solver/arena warning; invalid physics must not enter training')
        self.sensors.update(self.state)
        self.foot_velocity=(self.state["feet"]-self.previous_feet)/self.dt;self.previous_feet[:]=self.state["feet"]
        feet=self.state['feet']
        clearance=np.maximum(0,feet[:,:,2]-self.ground_height(feet[:,:,0],feet[:,:,1]))
        gait=self.foot_motion.update(self.state['contacts'],clearance,(self.state['bad_contacts']==0)&(self.state['self_contacts']==0))
        rise_velocity,vertical_accel=self.rise_motion.update(self.height,self.state['qvel'][:,2],self.state['contacts'].any(1)|(self.state['bad_contacts']>0),self.nominal_height)
        up=self.rot[:,2,2];vel=self.local_velocity;omega=self.local_angular
        error=np.square(vel[:,:2]-self.commands[:,:2]).sum(1)
        torque2=np.square(self.state['torque_rms']).sum(1)
        speed2=np.square(self.state['qvel'][:,6:18]).sum(1)
        action_rate=np.square(actions-self.last_action).mean(1)
        pose=np.square(self.state['qpos'][:,7:19]-self.home[:12]).mean(1)
        self.stable=np.where((up>.90)&(self.height>.84*self.nominal_height)&(np.linalg.norm(vel[:,:2],axis=1)<.25)&(self.height<1.4*self.nominal_height)&(self.state['contacts'].sum(1)>=3)&(self.state['bad_contacts']==0)&(self.state.get('self_contacts',np.zeros(self.num_envs))==0),self.stable+1,0)
        just_success=(self.stable>=35)&~self.success;self.success|=just_success
        from learning.rewards import locomotion_terms,recovery_terms
        previous_flight=self.flight_seconds.copy()
        clean_flight=(~self.state['contacts'].any(1))&(self.state['bad_contacts']==0)&(self.state['self_contacts']==0)&(up>.8)&(self.height>.75*self.nominal_height)&(self.height<1.4*self.nominal_height)&(vel[:,0]>=.35)&(self.commands[:,0]>=.35)&(error<.04)&(np.abs(vel[:,2])<.6)
        self.flight_seconds=np.where(clean_flight,self.flight_seconds+self.dt,0.)
        if self.task in ('locomotion','running'):
            velocity_reward=np.concatenate((vel,self.state['qvel'][:,6:18]),axis=1)
            terms=locomotion_terms(self.commands,velocity_reward,omega,up,self.height,self.nominal_height,
                self.state['torque_rms'],self.state['motor_continuous'],actions,self.last_action,
                self.state['qpos'][:,7:19],self.home[:12],self.state['contacts'],self.foot_velocity,
                self.state['bad_contacts'],self.task,self.flight_seconds,previous_flight,progress=self.progress,
                gait=gait,previous_torque=previous_torque,joint_limits=(self.low[:12],self.high[:12]),self_contacts=self.state['self_contacts'],
                navigation=getattr(self,'navigation',False),foot_clearance_target=.025+self.terrain.get('riser_m',0.))
            failure=(up<.45)|(self.height<.43*self.nominal_height)
        else:
            waypoint_terms=self.recovery_waypoints.update(up,self.height,self.nominal_height,
                self.state['contacts'],self.state['bad_contacts'],self.state['self_contacts'],omega,self.stable)
            terms,self.previous_potential=recovery_terms(up,self.height,self.nominal_height,pose,self.previous_potential,
                self.stable,just_success,actions,self.last_action,self.state['torque_rms'],self.state['motor_continuous'],self.state['bad_contacts'],progress=self.progress,omega=omega,
                feet_contacts=self.state['contacts'],self_contacts=self.state['self_contacts'],rise_velocity=rise_velocity,vertical_accel=vertical_accel,
                joints=self.state['qpos'][:,7:19],joint_limits=(self.low[:12],self.high[:12]),previous_torque=previous_torque,waypoints=waypoint_terms)
            failure=np.zeros(self.num_envs,bool)
        terms['self_collision']=-.2*np.minimum(self.state.get('self_contacts',np.zeros(self.num_envs)),10)
        reward=sum(terms.values())*self.dt
        if self.task in ('locomotion','running'):reward[failure]-=3.
        failure|=~np.isfinite(self.state['qpos']).all(1)|(np.abs(self.state['qpos'][:,:2])>self.terrain['extent_m']-.5).any(1)
        self.previous_up=up.copy();self.previous_height=self.height.copy();self.returns+=reward;self.velocity_error+=np.sqrt(error)
        clock_base=1.8 if self.task=='running' else 1.2
        self.phase+=self.dt*2*np.pi*(clock_base+1.5*np.linalg.norm(self.commands[:,:2],axis=1))
        if self.auto_reset and self.task in ('locomotion','running'):
            # Reward the action against the old command, then expose the new
            # command immediately in the observation for the following action.
            ids=np.flatnonzero((self.steps%200==0)&(self.steps>0)&~failure&(self.steps<self.max_episode_steps))
            self._sample_commands(ids)
            if self.observation_mode=='compact' and not self.manual_command:
                delta=self.command_targets-self.commands
                norm=np.linalg.norm(delta[:,:2],axis=1)
                delta[:,:2]*=np.minimum(1.,.6*self.dt/np.maximum(norm,1e-12))[:,None]
                delta[:,2]=np.clip(delta[:,2],-.6*self.dt,.6*self.dt)
                self.commands+=delta
        self.last_action[:]=actions
        self.history[:,:-1]=self.history[:,1:];self.history[:,-1]=self._frame()
        terminal_obs=self.observe();truncated=(self.steps>=self.max_episode_steps)&~failure;done=failure|truncated
        terminal_critic_obs=self.observe_critic() if self.observation_mode=='compact' else None
        episode={}
        if done.any():
            episode={'return':self.returns[done].copy(),'length_s':self.steps[done]*self.dt,
                     'recovery_success':self.success[done].astype(float),'command_rmse':self.velocity_error[done]/self.steps[done],
                     'reset_kind':self.reset_kind[done].copy(),'fall':failure[done].astype(float),
                     'disturbance_count':self.disturbance_count[done].copy()}
            if self.task=='recovery':
                episode['recovery_initial_waypoint']=self.recovery_waypoints.initial_level[done].copy()
                episode['recovery_final_waypoint']=self.recovery_waypoints.level[done].copy()
                for i,name in enumerate(self.recovery_waypoints.names):
                    episode['reached_'+name]=self.recovery_waypoints.earned[done,i].astype(float)
        info={'truncated':truncated,'terminal_observation':terminal_obs,'episode':episode,'success':self.success.copy(),
              'total_disturbances':int(self.total_disturbances)}
        if terminal_critic_obs is not None:info['terminal_critic_observation']=terminal_critic_obs
        info['reward_terms']={k:v.copy() for k,v in terms.items()}
        if self.auto_reset and done.any():self._reset_indices(np.flatnonzero(done))
        return self.observe(),reward.astype(np.float32),done,info

    def close(self):pass

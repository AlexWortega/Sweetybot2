"""V05 rewards: privileged measurements shape training only, never actor input.
Tests cover stationary flight farming, sliding and overload counterexamples.
"""
import numpy as np

class RecoveryWaypoints:
 """Episode-local pose milestones; no repeated payment for holding or cycling.

 Levels describe geometry/support rather than a commanded joint animation.
 Initial poses receive no milestone credit. The ledger is reward-only: neither
 privileged pose measurements nor milestone IDs are added to actor inputs.
 """
 names=('side','belly_support','supported_crouch','stable_stand')
 def __init__(self,n,dt=.02):
  self.dt=dt;self.level=np.zeros(n,int);self.initial_level=np.zeros(n,int)
  self.dwell=np.zeros(n,int);self.peak=np.zeros(n);self.earned=np.zeros((n,4),bool)

 def _score(self,up,height,nominal_height,contacts,stable,levels=None):
  count=np.asarray(contacts).sum(1);h=height/nominal_height
  scores=np.stack((np.clip((up+1)/.8,0,1),
   .75*np.clip((up+.2)/.75,0,1)+.25*np.minimum(count,1),
   .4*np.clip((up-.55)/.30,0,1)+.3*np.clip((h-.20)/.30,0,1)+.3*np.minimum(count/2,1),
   .25*np.clip((up-.85)/.10,0,1)+.25*np.clip((h-.5)/.34,0,1)+.25*np.minimum(count/3,1)+.25*np.minimum(stable/35,1)),axis=1)
  return scores[np.arange(len(up)),np.minimum(self.level if levels is None else levels,3)]

 def reset(self,ids,up,height,nominal_height,contacts):
  # Recognize poses already present at reset, even before the first contact.
  # Dropping an initially upright robot onto its feet cannot earn roll bonuses.
  self.level[ids]=((up>=-.20).astype(int)+(up>=.55).astype(int)+
                   ((up>=.85)&(height>=.5*nominal_height)).astype(int))
  self.initial_level[ids]=self.level[ids];self.dwell[ids]=0;self.earned[ids]=False
  self.peak[ids]=self._score(up,height,nominal_height,contacts,np.zeros(len(ids)),self.level[ids])

 def update(self,up,height,nominal_height,contacts,bad_contacts,self_contacts,omega,stable):
  count=np.asarray(contacts).sum(1);h=height/nominal_height
  grounded=(count>0)|(bad_contacts>0)
  clean=(self_contacts==0)&np.isfinite(up)&np.isfinite(height)&np.isfinite(omega).all(1)
  controlled=np.linalg.norm(omega,axis=1)<4.
  conditions=np.stack((up>=-.20,(up>=.55)&(count>=1),
   (up>=.85)&(h>=.5)&(count>=2)&(bad_contacts==0),
   (stable>=35)&(count>=3)&(bad_contacts==0)&(up>.90)&(h>.84)&(h<1.4)),axis=1)
  index=np.minimum(self.level,3);active=self.level<4
  qualified=conditions[np.arange(len(up)),index]&clean&grounded&controlled&active
  self.dwell=np.where(qualified,self.dwell+1,0)
  # Require a pose to persist: transient tumbles are not milestones.
  durations=np.array([.12,.16,.20,self.dt])
  reached=qualified&(self.dwell*self.dt+1e-9>=durations[index])
  events=np.zeros_like(self.earned)
  rows=np.flatnonzero(reached);events[rows,index[rows]]=True
  self.earned|=events
  score=self._score(up,height,nominal_height,contacts,stable)
  progress=np.maximum(0,score-self.peak)*clean*grounded*active
  self.peak=np.where(clean&grounded,np.maximum(self.peak,score),self.peak)
  bonuses=np.array([.5,.75,1.,1.5])
  terms={'waypoint_progress':2.*progress/self.dt}
  for i,name in enumerate(self.names):terms['waypoint_'+name]=bonuses[i]*events[:,i]/self.dt
  self.level[rows]+=1;self.dwell[rows]=0
  if len(rows):self.peak[rows]=self._score(up,height,nominal_height,contacts,stable)[rows]
  return terms

class FootMotion:
 """Per-foot landing events; an initial fall and contact chatter earn nothing."""
 def __init__(self,n,dt=.02):
  self.dt=dt;self.air=np.zeros((n,4));self.peak=np.zeros((n,4));self.seen_ground=np.zeros((n,4),bool);self.clean_air=np.ones((n,4),bool)
 def reset(self,ids):
  self.air[ids]=0;self.peak[ids]=0;self.seen_ground[ids]=False;self.clean_air[ids]=True
 def update(self,contacts,clearance,clean=None):
  contacts=np.asarray(contacts,bool);clearance=np.asarray(clearance,float)
  airborne=~contacts & self.seen_ground
  if clean is not None:self.clean_air &= np.asarray(clean,bool)[:,None]
  self.air=np.where(airborne,np.minimum(2.,self.air+self.dt),self.air)
  self.peak=np.where(airborne,np.maximum(self.peak,clearance),self.peak)
  landing=contacts & (self.air>0) & self.clean_air
  result={'landing_air_time':np.where(landing,self.air,0.),'landing_peak':np.where(landing,self.peak,0.),'clearance':clearance.copy(),'dt':self.dt}
  self.air=np.where(contacts,0.,self.air);self.peak=np.where(contacts,0.,self.peak)
  self.clean_air=np.where(contacts,True,self.clean_air)
  self.seen_ground|=contacts
  return result


class RiseMotion:
 """Pay only newly reached heights after floor contact, preventing bobbing loops."""
 def __init__(self,n,dt=.02):
  self.dt=dt;self.peak=np.zeros(n);self.grounded=np.zeros(n,bool);self.previous_vz=np.zeros(n);self.age=np.zeros(n,int)
 def reset(self,ids,height,vz):
  self.peak[ids]=height;self.grounded[ids]=False;self.previous_vz[ids]=vz;self.age[ids]=0
 def update(self,height,vz,ground_contact,nominal_height):
  first=np.asarray(ground_contact,bool)&~self.grounded
  self.grounded|=ground_contact
  target_height=np.minimum(height,nominal_height)
  self.peak=np.where(first,target_height,self.peak)
  rise=np.maximum(0,target_height-self.peak)/self.dt*self.grounded
  self.peak=np.where(self.grounded,np.maximum(self.peak,target_height),target_height)
  accel=np.where(self.age>0,(vz-self.previous_vz)/self.dt,0.)
  self.previous_vz=vz.copy();self.age+=1
  return rise,accel


def joint_limit_cost(joints,limits):
 if limits is None:return np.zeros(len(joints))
 low,high=limits
 margin=np.abs(2*(joints-low)/np.maximum(high-low,1e-6)-1)
 return -.1*np.square(np.clip((margin-.9)/.1,0,1)).mean(1)


def torque_change_cost(torque,previous,continuous,polish):
 if previous is None:return np.zeros(len(torque))
 return -.002*polish*np.square((torque-previous)/np.maximum(.05,continuous)).mean(1)


def locomotion_terms(command,velocity,omega,up,height,nominal_height,torque,continuous,actions,previous_actions,joints,home,contacts,foot_velocity,bad_contacts,task,flight_seconds,previous_flight,progress=1.,gait=None,previous_torque=None,joint_limits=None,self_contacts=None):
 from learning.curriculum import polish_weight
 polish=polish_weight(progress) if task=='running' else 1.
 error=np.square(velocity[:,:2]-command[:,:2]).sum(1);yaw_error=np.square(omega[:,2]-command[:,2])
 tracking=np.exp(-error/.10);yaw=np.exp(-yaw_error/.50)
 speed=np.linalg.norm(command[:,:2],axis=1);moving=speed>.05
 turning=np.abs(command[:,2])>.10;active=moving|turning
 slip=(np.square(foot_velocity[:,:,:2]).sum(2)*contacts).sum(1)/np.maximum(1,contacts.sum(1))
 overload=np.square(np.maximum(0,np.abs(torque)/np.maximum(.05,continuous)-.8)).mean(1)
 action_rate=np.square(actions-previous_actions).mean(1)
 upright=np.clip(up,0,1);height_score=np.exp(-np.square(height-nominal_height)/(.13*nominal_height)**2)
 pose_error=np.square(joints-home).mean(1)
 terms={'velocity_tracking':3.*tracking,'yaw_tracking':2.*yaw,'upright':.4*upright,'height':.4*height_score,
        'pose':-np.where(active,.02,.06)*polish*pose_error,'action_rate':-.04*polish*action_rate,'foot_slip':-.25*slip,
        'overload':-.8*overload,'body_contact':-.12*np.minimum(bad_contacts,5),
        'angular_wobble':-.04*polish*np.square(omega[:,:2]).sum(1),
        'mechanical_power':-.015*np.abs(torque[:,:12]*velocity[:,3:15]).sum(1) if velocity.shape[1]>=15 else np.zeros(len(up))}
 # A directional progress signal remains informative far from the Gaussian's
 # target. It saturates at the command; wrong-way motion cannot earn it.
 along=(velocity[:,:2]*command[:,:2]).sum(1)/np.maximum(.0025,np.square(command[:,:2]).sum(1))
 # Keep the cleanliness expression explicit: all positive gait/speed shaping
 # requires a body clear of terrain and clear of self-contact.
 clear=(bad_contacts==0)&(np.ones(len(up),bool) if self_contacts is None else self_contacts==0)
 upright_clear=(up>.75)&(height>.65*nominal_height)&clear
 terms['command_progress']=2.*np.clip(along,0,1)*moving*upright_clear
 terms['under_speed']=-1.5*np.square(np.clip(1-along,0,2))*moving
 terms['over_speed']=-.5*np.square(np.clip(along-1,0,2))*moving
 yaw_along=omega[:,2]*np.sign(command[:,2])/np.maximum(.1,np.abs(command[:,2]))
 terms['yaw_progress']=.5*np.clip(yaw_along,0,1)*turning*upright_clear
 terms['under_turn']=-.5*np.square(np.clip(1-yaw_along,0,2))*turning
 terms['command_pose']=.35*np.exp(-pose_error/np.where(active,.6,.15)**2)*upright_clear
 terms['joint_limit']=joint_limit_cost(joints,joint_limits)
 terms['torque_change']=torque_change_cost(torque,previous_torque,continuous,polish)
 if gait is not None:
  directional_speed=(velocity[:,:2]*command[:,:2]).sum(1)/np.maximum(.05,speed)
  actual_motion=(moving&(directional_speed>.02))|(turning&(yaw_along>.2)&(np.abs(omega[:,2])>.08))
  gate=active&actual_motion&upright_clear
  air=gait['landing_air_time'];peak=gait['landing_peak'];clearance=gait['clearance']
  lo,hi=(.08,.30) if task=='running' else (.10,.35)
  landing=(air>=lo)&(air<=hi)&(peak>=.012)
  timing=np.exp(-np.square((air-.20)/.10))*landing
  # Event unit: at most 0.08 reward per foot landing, after dt integration.
  terms['foot_air_time']=.08/max(1e-6,gait['dt'])*timing.sum(1)*gate
  horizontal=np.linalg.norm(foot_velocity[:,:,:2],axis=2)
  swing=(~contacts)*np.clip(horizontal/.10,0,1)
  terms['foot_clearance']=.6*(np.exp(-np.square((clearance-.025)/.015))*swing).mean(1)*gate
  terms['foot_swing_height']=.04/max(1e-6,gait['dt'])*(np.exp(-np.square((peak-.025)/.02))*landing).sum(1)*gate
  terms['foot_drag']=-.2*(np.maximum(0,.015-clearance)/.015*swing).mean(1)*active*upright_clear
  terms['excess_foot_lift']=-.2*np.square(np.maximum(0,clearance-.07)/.07).mean(1)*active
 if task=='running':
  clean=(up>.8)&(height>.75*nominal_height)&(height<1.4*nominal_height)&(bad_contacts==0)&(velocity[:,0]>=.35)&(command[:,0]>=.35)&(error<.04)&(np.abs(velocity[:,2])<.6)
  # One event per clean >=40ms forward flight, not a per-tick reward for hanging.
  event=(flight_seconds>=.04)&(previous_flight<.04)&clean
  terms['forward_flight_event']=event.astype(float)*8.
  terms['moving_tracking']=.8*tracking*moving
 return terms

def recovery_potential(up,height,nominal_height,pose_error):
 # Inverted plateaus receive low potential and no positive survival reward.
 return 2.*np.clip((up+1)/2,0,1)+1.2*np.exp(-np.square(height-nominal_height)/(.2*nominal_height)**2)*(up>.5)+.2*np.exp(-pose_error/.4)*(up>.8)

def recovery_terms(up,height,nominal_height,pose_error,previous_potential,stable,just_success,actions,previous_actions,torque,continuous,bad_contacts,progress=1.,omega=None,feet_contacts=None,self_contacts=None,rise_velocity=None,vertical_accel=None,joints=None,joint_limits=None,previous_torque=None,waypoints=None):
 from learning.curriculum import polish_weight
 polish=polish_weight(progress)
 potential=recovery_potential(up,height,nominal_height,pose_error)
 overload=np.square(np.maximum(0,np.abs(torque)/np.maximum(.05,continuous)-.8)).mean(1)
 count=np.zeros(len(up)) if feet_contacts is None else np.asarray(feet_contacts).sum(1)
 clear=(bad_contacts==0)&(np.ones(len(up),bool) if self_contacts is None else self_contacts==0)
 supported=(count>=1)&clear&(up>.5)
 goal_support=(count>=3)&clear&(up>.9)&(height>.84*nominal_height)
 height_error=(height-nominal_height)/nominal_height
 broad=np.exp(-np.square(height_error/.135));sharp=np.exp(-np.square(height_error/.05))
 tilt=np.arccos(np.clip(up,-1,1));pose_score=np.exp(-pose_error/.5**2)
 terms={'potential_progress':(.995*potential-previous_potential)*10.,'time_cost':np.full(len(up),-.15),
        'stable_support':2.*(stable>0),'completed_stand':40.*just_success.astype(float),
        'overload':-.5*overload,'action_rate':-.01*polish*np.square(actions-previous_actions).mean(1),
        'upright_body_contact':-.15*np.minimum(bad_contacts,5)*(up>.85)}
 # Offset cosine is zero at upright, negative while tipped. This avoids paying
 # a positive survival reward to an upright but permanently folded robot.
 terms.update(upright_linear=1.5*(up-1),
  height_stand=broad*supported,height_stand_sharp=sharp*goal_support,
  height_stand_l1=-6.*np.abs(height_error),
  pose_stand=2.*pose_score*np.clip((height/nominal_height-.5)/.5,0,1)*supported,
  upright_sharp=1.5*np.exp(-np.square(tilt/.3))*goal_support,
  standing_composite=3.75*broad*np.exp(-np.square(tilt/.4))*np.exp(-pose_error/.4**2)*goal_support,
  insufficient_support=-.3*(count<3)*(height>.84*nominal_height)*(up>.8),
  torque_change=torque_change_cost(torque,previous_torque,continuous,polish))
 if rise_velocity is not None:terms['upward_motion']=1.5*np.maximum(0,rise_velocity)/nominal_height
 if vertical_accel is not None:terms['impact']=-.002*polish*np.abs(vertical_accel)
 if joints is not None:terms['joint_limit']=joint_limit_cost(joints,joint_limits)
 if omega is not None:
  # Brake near arrival only; do not tax the rotation needed to leave the back.
  terms['arrival_damping']=-.025*polish*np.square(omega[:,:2]).sum(1)*(up>.85)*(height>.8*nominal_height)
 if waypoints is not None:
  terms.update(waypoints)
  # Lowering the torso and folding limbs is useful during a roll. Full-height
  # and stand-pose shaping takes over only after the torso faces upright.
  terms['height_stand_l1']*=np.where(up>.55,1.,.1)
  terms['upright_linear']*=np.where(up>.55,1.,.25)
 return terms,potential

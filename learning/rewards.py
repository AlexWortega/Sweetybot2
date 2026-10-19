"""V05 rewards: privileged measurements shape training only, never actor input.
Tests cover stationary flight farming, sliding and overload counterexamples.
"""
import numpy as np

def locomotion_terms(command,velocity,omega,up,height,nominal_height,torque,continuous,actions,previous_actions,joints,home,contacts,foot_velocity,bad_contacts,task,flight_seconds,previous_flight,progress=1.):
 from learning.curriculum import polish_weight
 polish=polish_weight(progress) if task=='running' else 1.
 error=np.square(velocity[:,:2]-command[:,:2]).sum(1);yaw_error=np.square(omega[:,2]-command[:,2])
 tracking=np.exp(-error/.025);yaw=np.exp(-yaw_error/.20)
 moving=np.linalg.norm(command[:,:2],axis=1)>.10
 slip=(np.square(foot_velocity[:,:,:2]).sum(2)*contacts).sum(1)/np.maximum(1,contacts.sum(1))
 overload=np.square(np.maximum(0,np.abs(torque)/np.maximum(.05,continuous)-.8)).mean(1)
 action_rate=np.square(actions-previous_actions).mean(1)
 upright=np.clip(up,0,1);height_score=np.exp(-np.square(height-nominal_height)/(.13*nominal_height)**2)
 terms={'velocity_tracking':3.*tracking,'yaw_tracking':2.*yaw,'upright':.4*upright,'height':.4*height_score,
        'pose':-.06*polish*np.square(joints-home).mean(1),'action_rate':-.04*polish*action_rate,'foot_slip':-.7*slip,
        'overload':-.8*overload,'body_contact':-.12*np.minimum(bad_contacts,5),
        'angular_wobble':-.04*polish*np.square(omega[:,:2]).sum(1),
        'mechanical_power':-.015*np.abs(torque[:,:12]*velocity[:,3:15]).sum(1) if velocity.shape[1]>=15 else np.zeros(len(up))}
 # A directional progress signal remains informative far from the Gaussian's
 # target. It saturates at the command; wrong-way motion cannot earn it.
 along=(velocity[:,:2]*command[:,:2]).sum(1)/np.maximum(.01,np.square(command[:,:2]).sum(1))
 terms['command_progress']=np.clip(along,0,1)*moving*(up>.75)*(bad_contacts==0)
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

def recovery_terms(up,height,nominal_height,pose_error,previous_potential,stable,just_success,actions,previous_actions,torque,continuous,bad_contacts,progress=1.,omega=None):
 from learning.curriculum import polish_weight
 polish=polish_weight(progress)
 potential=recovery_potential(up,height,nominal_height,pose_error)
 overload=np.square(np.maximum(0,np.abs(torque)/np.maximum(.05,continuous)-.8)).mean(1)
 terms={'potential_progress':(.995*potential-previous_potential)*25.,'time_cost':np.full(len(up),-.15),
        'stable_support':2.*(stable>0),'completed_stand':40.*just_success.astype(float),
        'overload':-.5*overload,'action_rate':-.01*polish*np.square(actions-previous_actions).mean(1),
        'upright_body_contact':-.15*np.minimum(bad_contacts,5)*(up>.85)}
 if omega is not None:
  # Brake near arrival only; do not tax the rotation needed to leave the back.
  terms['arrival_damping']=-.025*polish*np.square(omega[:,:2]).sum(1)*(up>.85)*(height>.8*nominal_height)
 return terms,potential

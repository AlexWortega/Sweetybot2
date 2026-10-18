"""V04 sampled sensor model. Actor channels match IMU, contact switches and ToF.
True root/foot states remain reward/evaluation only. Initial tilt is a noisy
fused-IMU boot prior, representing the estimator already running before a fall;
thereafter orientation is propagated from physical gyro/accelerometer samples.
"""
from __future__ import annotations
import numpy as np


def quat_mul(a,b):
 w,x,y,z=a.T;v,u,t,s=b.T
 return np.stack((w*v-x*u-y*t-z*s,w*u+x*v+y*s-z*t,w*t-x*s+y*v+z*u,w*s+x*t-y*u+z*v),1)
def gravity_up(q):
 w,x,y,z=q.T
 return np.stack((2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)),1)

class SensorSuite:
 def __init__(self,n,seed=1,randomize=True):
  self.n=n;self.rng=np.random.default_rng(seed);self.randomize=randomize
  self.q=np.tile([1.,0,0,0],(n,1));self.gyro=np.zeros((n,3));self.accel=np.tile([0.,0,9.81],(n,1));self.imu_age=np.zeros(n)
  self.contacts=np.zeros((n,4));self.contact_valid=np.zeros((n,4));self.candidate=np.zeros((n,4),bool);self.dwell=np.zeros((n,4));self.switch_previous=np.zeros((n,4),bool);self.bounce_left=np.zeros((n,4))
  self.time=np.zeros(n);self.imu_clock=np.zeros(n);self.contact_clock=np.zeros(n)
  self.gyro_bias=np.zeros((n,3));self.accel_bias=np.zeros((n,3));self.contact_threshold=np.full((n,4),2.36)
  self.depth=np.ones((n,64));self.depth_valid=np.zeros((n,64));self.depth_age=np.ones(n);self.previous_depth_age=np.full(n,np.inf);self.depth_seen=np.zeros(n,bool);self.force_fault=np.zeros((n,4),bool)
 def reset(self,ids,quat):
  ids=np.asarray(ids,int);q=np.asarray(quat,float).copy();q/=np.linalg.norm(q,axis=1,keepdims=True)
  # Relative-heading IMU: no global heading enters the actor, only gravity.
  if self.randomize:
   delta=self.rng.normal(0,.005,(len(ids),4));delta[:,0]=1.;q=quat_mul(q,delta);q/=np.linalg.norm(q,axis=1,keepdims=True)
  self.q[ids]=q;self.gyro[ids]=0;self.accel[ids]=gravity_up(q)*9.81;self.imu_age[ids]=0
  for a in [self.contacts,self.contact_valid,self.dwell,self.bounce_left]:a[ids]=0
  self.candidate[ids]=False;self.switch_previous[ids]=False;self.time[ids]=self.imu_clock[ids]=self.contact_clock[ids]=0
  self.gyro_bias[ids]=self.rng.normal(0,.003,(len(ids),3)) if self.randomize else 0
  self.accel_bias[ids]=self.rng.normal(0,.03,(len(ids),3)) if self.randomize else 0
  self.contact_threshold[ids]=self.rng.uniform(1.87,2.85,(len(ids),4)) if self.randomize else 2.36
  self.depth[ids]=1.;self.depth_valid[ids]=0;self.depth_age[ids]=1.;self.previous_depth_age[ids]=np.inf;self.depth_seen[ids]=False;self.force_fault[ids]=False
 def update(self,state,dt=.002):
  gyros=np.asarray(state['imu_gyro_samples']);accels=np.asarray(state['imu_accel_samples']);forces=np.asarray(state['foot_force_samples'])
  assert gyros.shape==(self.n,10,3) and forces.shape==(self.n,10,4)
  for k in range(10):
   self.time+=dt;self.imu_age+=dt;self.imu_clock+=dt;self.contact_clock+=dt
   sample=self.imu_clock>=1/120-1e-10
   if sample.any():
    self.imu_clock[sample]-=1/120
    gyro=gyros[sample,k]+self.gyro_bias[sample];accel=accels[sample,k]+self.accel_bias[sample]
    if self.randomize:gyro+=self.rng.normal(0,.007,gyro.shape);accel+=self.rng.normal(0,.08,accel.shape)
    finite=np.isfinite(gyro).all(1)&np.isfinite(accel).all(1);ix=np.flatnonzero(sample);good=ix[finite]
    if len(good):
     w=gyro[finite];a=accel[finite];norm=np.linalg.norm(a,axis=1);pred=gravity_up(self.q[good]);measured=a/np.maximum(norm[:,None],1e-6)
     # Reject large impact/flight acceleration instead of interpreting it as tilt.
     correction=np.cross(measured,pred);correction*=((norm>7.8)&(norm<11.8))[:,None]
     w=w+1.2*correction
     interval=1/120
     dq=np.concatenate((np.zeros((len(good),1)),w),1)
     self.q[good]+=.5*interval*quat_mul(self.q[good],dq);self.q[good]/=np.linalg.norm(self.q[good],axis=1,keepdims=True)
     self.gyro[good]=gyro[finite];self.accel[good]=a;self.imu_age[good]=0
   sample=self.contact_clock>=.005-1e-10
   if sample.any():
    self.contact_clock[sample]-=.005;ix=np.flatnonzero(sample);force=forces[sample,k]
    pressed=force>=self.contact_threshold[sample];changed=pressed!=self.switch_previous[sample]
    self.bounce_left[sample]=np.where(changed,.005,np.maximum(0,self.bounce_left[sample]-.005));self.switch_previous[sample]=pressed
    observed=pressed.copy()
    if self.randomize:
     bounce=(self.bounce_left[sample]>0)&(self.rng.random(pressed.shape)<.5);observed^=bounce
    fault=self.force_fault[sample]|~np.isfinite(force)
    if self.randomize:fault|=self.rng.random(force.shape)<.0005
    self.dwell[sample]=np.where(fault,0.,np.where(observed==self.candidate[sample],self.dwell[sample]+.005,0.))
    self.candidate[sample]=observed
    valid=(self.dwell[sample]>=.010-1e-10)&~fault
    self.contact_valid[sample]=valid;self.contacts[sample]=np.where(valid,observed,0.)
  raw=np.asarray(state['depth_ranges']);age=np.asarray(state['depth_age'])
  valid=np.isfinite(raw)&(raw>=.03)&(raw<=4.)
  depth=raw.copy()
  if self.randomize:
   depth+=self.rng.normal(0,.005+.005*np.maximum(0,raw),raw.shape);valid&=self.rng.random(raw.shape)>.005
  fresh=(~self.depth_seen)|(age<self.previous_depth_age-1e-9)
  self.depth[fresh]=np.where(valid,np.clip(depth/4,0,1),1.)[fresh];self.depth_valid[fresh]=valid[fresh];self.depth_seen[fresh]=True
  self.depth_age=age.copy();self.previous_depth_age=age.copy()
 def frame(self):
  valid=np.isfinite(self.q).all(1)&(self.imu_age<.03)
  gravity=np.where(valid[:,None],-gravity_up(self.q),0.)
  gyro=np.where(valid[:,None],self.gyro,0.)*.25;accel=np.where(valid[:,None],self.accel,0.)/9.81
  depth_valid=self.depth_valid*(self.depth_age<.2)[:,None]
  return gravity,gyro,accel,self.contacts,self.contact_valid,valid[:,None].astype(float),np.minimum(self.imu_age[:,None]/.03,1),self.depth,depth_valid,np.minimum(self.depth_age[:,None]/.2,1)

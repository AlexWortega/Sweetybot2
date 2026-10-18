"""Shared deployable actor frame: no world position, velocity or simulator truth."""
import numpy as np
CONTRACT='v04-sensor-suite-1'
FRAME_DIM=197
SLICES={'gravity':(0,3),'gyro':(3,6),'joint_delta':(6,22),'joint_velocity':(22,38),'previous_action':(38,50),'command':(50,53),'phase':(53,55),'accel':(55,58),'contact':(58,62),'contact_valid':(62,66),'imu_valid':(66,67),'imu_age':(67,68),'depth':(68,132),'depth_valid':(132,196),'depth_age':(196,197)}

def encode(sensor_frame,joint_delta,joint_velocity,previous_action,command,phase):
 gravity,gyro,accel,contact,contact_valid,imu_valid,imu_age,depth,depth_valid,depth_age=sensor_frame
 frame=np.concatenate((gravity,gyro,joint_delta,joint_velocity*.1,previous_action,command,np.sin(phase)[:,None],np.cos(phase)[:,None],accel,contact,contact_valid,imu_valid,imu_age,depth,depth_valid,depth_age),axis=1).astype(np.float32)
 if frame.shape[1]!=FRAME_DIM or not np.isfinite(frame).all():raise ValueError('Invalid v04 sensor frame')
 return frame

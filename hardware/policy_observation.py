"""Compact deployable policy input, inspired by MicroDuck's proprioceptive actor.

The 197-value sensor transport stays unchanged. No simulator state, depth,
camera, accelerometer or gait clock enters this single-frame actor contract.
"""
import numpy as np

CONTRACT = 'v05-compact-proprio-1'
DIM = 63
CRITIC_CONTRACT = 'v05-privileged-value-1'
CRITIC_DIM = 81
SLICES = {'gravity': (0, 3), 'gyro': (3, 6), 'joint_delta': (6, 22),
          'joint_velocity': (22, 38), 'previous_action': (38, 50),
          'command': (50, 53), 'contact': (53, 57), 'contact_valid': (57, 61),
          'imu_valid': (61, 62), 'imu_age': (62, 63)}


def _checked(frame):
    frame = np.asarray(frame, dtype=np.float32)
    if frame.ndim != 2 or frame.shape[1] != DIM or not np.isfinite(frame).all():
        raise ValueError('Invalid compact policy observation')
    return frame


def encode(sensor_frame, joint_delta, joint_velocity, previous_action, command):
    gravity, gyro, _, contact, contact_valid, imu_valid, imu_age, *_ = sensor_frame
    return _checked(np.concatenate((gravity, gyro, joint_delta, joint_velocity*.1,
                                   previous_action, command, contact, contact_valid,
                                   imu_valid, imu_age), axis=1))


def from_sensor_frame(frame):
    """Project a sampled sensor frame without resampling sensor noise."""
    frame = np.asarray(frame)
    if frame.ndim != 2 or frame.shape[1] != 197:
        raise ValueError('Expected the 197-value sensor transport')
    return _checked(np.concatenate((frame[:, :53], frame[:, 58:68]), axis=1))

from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]
def load():
    return yaml.safe_load((ROOT/'config/robot.yaml').read_text())
def joint_names(c):
    return [f'{leg}_{kind}' for leg in c['legs'] for kind in ('hip_roll','hip_pitch','knee')]+['head_yaw','head_pitch','ear_left','ear_right']
def total_mass(c,servo_mass=None):
    m=c['mass']; s=servo_mass if servo_mass is not None else c['actuator_candidates'][c['selection']['selected']]['mass']
    return m['torso_without_servos']+4*(m['roll_link_without_servo']+m['thigh_without_servo']+m['shin_and_foot'])+m['head_yaw_without_servo']+m['head_without_servos']+2*m['ear_without_servo']+16*s

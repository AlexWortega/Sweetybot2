from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]
def load():
    return yaml.safe_load((ROOT/'config/robot.yaml').read_text())
def joint_names(c):
    return [f'{leg}_{kind}' for leg in c['legs'] for kind in ('hip_roll','hip_pitch','knee')]+['head_yaw','head_pitch','ear_left','ear_right']
def actuator(c,name):
    model=c['actuation']['assignment'][name]
    return c['ear_actuator'] if model==c['ear_actuator']['model'] else c['actuator_candidates'][model]

def actuator_masses(c):
    return {n:actuator(c,n)['mass'] for n in joint_names(c)}

def total_mass(c,servo_mass=None):
    # Candidate sweeps may replace the eight pitch/knee motors only.
    m=c['mass'];motors=actuator_masses(c)
    if c.get('link_mass_properties'):
        return sum(p['mass'] for p in c['link_mass_properties'].values())+(0. if servo_mass is None else 8*(servo_mass-actuator(c,'fl_knee')['mass']))
    if servo_mass is not None:
        for n in motors:
            if n.endswith(('hip_pitch','knee')):motors[n]=servo_mass
    return m['torso_without_servos']+4*(m['roll_link_without_servo']+m['thigh_without_servo']+m['shin_and_foot'])+m['head_yaw_without_servo']+m['head_without_servos']+2*m['ear_without_servo']+sum(motors.values())

def continuous_limits(c, speed=0.):
    limits=[]
    for name in joint_names(c):
        a=actuator(c,name);vf=min(1.,(6. if name.startswith('ear_') else c['power']['battery']['cutoff_voltage'])/a['voltage'])
        limits.append(a.get('rated_torque',a['stall_torque']*.25)*vf*max(0.,1-speed/(a['no_load_speed']*vf)))
    return limits

def require_mechanical_gate():
    import json,hashlib
    p=ROOT/'reports/mechanical_gate.json'
    if not p.exists():raise ValueError('Mechanical release gate is missing; no fabrication package permitted')
    g=json.loads(p.read_text())
    if not g.get('passed') or not g.get('all_link_pairs_checked'):
        raise ValueError(g.get('reason','Full mechanical audit has not passed'))
    for name,digest in g.get('source_sha256',{}).items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise ValueError('Mechanical audit is stale: '+name)
    return g

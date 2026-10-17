"""Reproducible, conditional actuator screening. Never labels stall derating a rating."""
import json, math
from pathlib import Path
from scripts.common import ROOT, load, total_mass

def screen(c):
    g=c['geometry']; sel=c['selection']; out=[]
    l1,l2,h=(g[k]/1000 for k in ('thigh','shin','stance_height'))
    h-=g['hip_drop']/1000
    knee=math.acos((h*h-l1*l1-l2*l2)/(2*l1*l2))
    hip=-math.atan2(l2*math.sin(knee),l1+l2*math.cos(knee))
    knee_arm=abs(l2*math.sin(hip+knee))
    for name,a in c['actuator_candidates'].items():
        mass=total_mass(c,a['mass']); design_mass=mass*(1+c['mass']['uncertainty_fraction'])
        f=design_mass*9.81*sel['load_share_worst']
        knee_t=f*knee_arm+(c['mass']['shin_and_foot'])*9.81*knee_arm/2
        roll_t=f*g['abduction_offset']/1000
        # A heavily loaded leg in a crouched transition; transient, not indefinite sitting.
        transition_t=f*0.065+0.04
        head_t=(c['mass']['head_without_servos']+2*c['ear_actuator']['mass']+2*c['mass']['ear_without_servo'])*9.81*0.065
        sustained=sel['torque_margin']*max(knee_t,roll_t,head_t)
        transient=sel['torque_margin']*transition_t
        # First-order torque-speed envelope; not a measured motor curve.
        voltage_factor=(c['power']['battery']['cutoff_voltage']/a['voltage']) if name.startswith(('ST3215','STS3250')) else 1.0
        derate=voltage_factor*max(0,1-sel['required_joint_speed']/(a['no_load_speed']*voltage_factor))
        allow=a.get('rated_torque',a['stall_torque']*sel['sustained_fraction_of_stall'])*derate
        burst=a['stall_torque']*sel['transient_fraction_of_stall']*derate
        out.append(dict(candidate=name,mass_kg=mass,design_mass_kg=design_mass,
                        knee_arm_m=knee_arm,stance_knee_rad=knee,stance_hip_rad=hip,
                        sustained_requirement_nm=sustained,transient_requirement_nm=transient,
                        conditional_sustained_allowance_nm=allow,conditional_transient_allowance_nm=burst,
                        actuator_cost_usd=14*a['unit_usd']+2*22.0,passes_screen=sustained<=allow and transient<=burst,
                        price_status=a['price_status']))
    return out

def main():
    c=load(); rows=screen(c)
    passed=sorted((r for r in rows if r['passes_screen']),key=lambda r:(r['actuator_cost_usd'],r['mass_kg']))
    if not passed:raise RuntimeError('No candidate passes conditional screen; revise dimensions or loads')
    winner=passed[0]['candidate']
    if winner!=c['selection']['selected']:raise RuntimeError(f'Configuration must select {winner}')
    p=c['power']; a=c['actuator_candidates'][winner]
    mean_w=14*p['assumed_average_servo_current_a']*p['battery']['nominal_voltage']+p['electronics_w']+2.0
    result={'status':'conditional analytical screening; thermal validation outstanding','selected':winner,
            'candidates':rows,'assumptions':{'continuous_duty':'STS3250 uses manufacturer rated torque point with low-voltage/speed derating; thermal bench test still required',
             'torque_speed':'linear screening envelope, uncalibrated','prices':'USD, taxes/shipping excluded; ST3215 is an allowance'},
            'power':{'ear_6v_stall_a':2*c['ear_actuator']['stall_current'],'main_servo_rail_stall_a':14*a['stall_current'],'nominal_energy_wh':p['battery']['nominal_voltage']*p['battery']['capacity_ah'],
             'assumed_average_w':mean_w,'estimated_runtime_min':60*p['usable_energy_fraction']*p['battery']['nominal_voltage']*p['battery']['capacity_ah']/mean_w}}
    (ROOT/'reports/feasibility.json').write_text(json.dumps(result,indent=2)+'\n')
    text=['# Actuator and mass screening','',result['status'],'',
      '| Candidate | Nominal mass kg | Design mass kg | Sustained required/allowance Nm | Transition required/allowance Nm | 14 main + 2 ear allowance USD | Screen |',
      '|---|---:|---:|---:|---:|---:|---|']
    for r in rows:text.append(f"| {r['candidate']} | {r['mass_kg']:.3f} | {r['design_mass_kg']:.3f} | {r['sustained_requirement_nm']:.3f} / {r['conditional_sustained_allowance_nm']:.3f} | {r['transient_requirement_nm']:.3f} / {r['conditional_transient_allowance_nm']:.3f} | {r['actuator_cost_usd']:.2f} | {'conditional pass' if r['passes_screen'] else 'reject'} |")
    text+=['',f'Selected: **{winner}**, the cheapest candidate passing this conditional screen.',
     '', 'Load calculation: 20% mass contingency; one leg takes 50% of weight; 1.5 torque margin. Knee lever comes from exact two-link IK at nominal height. Hip roll includes the lateral joint offset. Crouch transition uses a conservative 65 mm moment arm. Head uses a 65 mm lever.',
     '', 'The torque-speed allowance is stall torque × assumed duty fraction × (1 − speed/no-load speed). STS3250 sustained screening uses its manufacturer rated torque point (16 kg.cm at 12V), with low-battery and speed derating. Other candidates use the configured stall fraction. The rated point is not a verified continuous thermal rating for this robot. A loaded bench test must validate or replace both before fabrication release.',
     '', 'Candidate screening uses the same 210 mm chassis and 75/80 mm links to avoid comparing different tasks. Smaller servos could support a separate smaller robot; that alternative is not established by this screen.',
     '',f"Power: {result['power']['nominal_energy_wh']:.2f} Wh nominal; {mean_w:.1f} W assumed average; {result['power']['estimated_runtime_min']:.1f} minutes estimated at 75% usable energy. This is a budget, not measured endurance.",
     '', f"All-servo stall is {result['power']['main_servo_rail_stall_a']:.1f} A, above the main fuse. Simultaneous stall is a fault condition, not an operating point. Fuse coordination and disconnect behavior require bench testing."]
    (ROOT/'docs/feasibility.md').write_text('\n'.join(text)+'\n')
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()

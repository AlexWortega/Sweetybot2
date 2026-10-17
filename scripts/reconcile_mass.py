"""Reconcile nominal link allocations to saved STEP volumes and explicit hardware allowances."""
import json,yaml
from scripts.common import ROOT,load,total_mass

def main():
 c=load();d=json.loads((ROOT/'reports/cad_parts.json').read_text());p={x['part']:x['solid_petg_mass_kg'] for x in d['parts']}
 groups={
 'torso_without_servos':{'deck':p['deck'],'rails':2*p['rail'],'cowl':p['body_shell'],'five_cradles':5*p['servo_cradle'],'ten_straps':10*p['servo_strap'],'tail':p['tail'],'battery_pack':c['power']['battery']['pack_mass_kg'],'compute_storage_imu':.016,'adapter_regulator':.020,'switch_fuses_wiring':.075,'fasteners':.035},
 'roll_link_without_servo':{'cradle':p['servo_cradle'],'two_straps':2*p['servo_strap'],'yoke':p['roll_yoke'],'adapter':p['axle_adapter'],'bearings_fasteners':.010},
 'thigh_without_servo':{'two_cheeks':2*p['thigh_plate'],'outer_fairing':p['thigh_fairing'],'spacers':.005,'bearings_fasteners':.010},
 'shin_and_foot':{'two_cheeks':2*p['shin_plate'],'outer_fairing':p['shin_fairing'],'support_fork':p['knee_fork'],'hoof':p['hoof'],'fasteners':.008},
 'head_yaw_without_servo':{'cradle':p['servo_cradle'],'two_straps':2*p['servo_strap'],'bearings_fasteners':.006,'collar':p['neck_collar']},
 'head_without_servos':{'shell':p['head_shell'],'hatch':p['head_hatch'],'mane':p['mane'],'horn':p['horn'],'pink_stripe':p['mane_stripe'],'support_yoke':p['head_yoke'],'displays_wire':.021,'bearings_fasteners':.016},
 'ear_without_servo':{'shell':p['ear'],'fasteners':.0015}}
 for k,v in groups.items():c['mass'][k]=round(sum(v.values())+.001,6)
 (ROOT/'config/robot.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
 report={'nominal_mass_kg':total_mass(c),'design_mass_kg':total_mass(c)*(1+c['mass']['uncertainty_fraction']), 'printed_solid_petg_kg':d['printed_solid_mass_kg'],'servo_mass_kg':16*c['actuator_candidates'][c['selection']['selected']]['mass'],'per_link_allowances':groups,'method':'saved STEP volume x 1270 kg/m3, explicit purchased hardware allowances, rounded up 1g per link group'}
 (ROOT/'reports/mass_budget.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()

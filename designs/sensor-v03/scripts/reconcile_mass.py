"""v03 mass budget from saved native CAD plus explicit purchased-part allowances."""
import json,yaml
from scripts.common import ROOT,load,total_mass

def main():
 c=load();d=json.loads((ROOT/'reports/cad_parts.json').read_text());p={x['part']:x['solid_petg_mass_kg'] for x in d['parts']};a=c['sensing']['mass_allowances_kg']
 # TPU pads/tips use 1200kg/m3; all other printed components use solid PETG1270.
 tpu=lambda key:p[key]*1200/1270
 groups={
 'torso_without_servos':{'deck':p['deck'],'rails':2*p['rail'],'cowl':p['body_shell'],'five_cradles':5*p['servo_cradle'],'ten_straps':10*p['servo_strap'],'tail':p['tail'],'battery_pack':c['power']['battery']['pack_mass_kg'],'Pi_storage':.013,'adapter_5V_regulator':.020,'switch_fuses_wiring':.075,'fasteners':.035,'sensor_tray':p['sensor_tray'],'depth_pod':p['depth_pod'],'depth_with_harness':a['depth_with_harness'],'imu':a['imu_breakout'],'collector':a['collector_with_connector'],'sensor_harness_fasteners':a['extra_torso_wiring_fasteners'],'ear_6V_buck_allowance':.012,'depth_3V3_buck':a['depth_3v3_buck']},
 'roll_link_without_servo':{'cradle':p['servo_cradle'],'two_straps':2*p['servo_strap'],'yoke':p['roll_yoke'],'adapter':p['axle_adapter'],'bearings_fasteners':.010},
 'thigh_without_servo':{'two_cheeks':2*p['thigh_plate'],'outer_fairing':p['thigh_fairing'],'spacers':.005,'bearings_fasteners':.010},
 'shin_and_foot':{'two_cheeks':2*p['shin_plate'],'outer_fairing':p['shin_fairing'],'support_fork':p['knee_fork'],'hoof_carrier':p['hoof'],'sole_plate':p['hoof_sole_plate'],'TPU_pad':tpu('hoof_tpu_pad'),'TPU_tip':tpu('hoof_actuator'),'foot_electronics':a['foot_pcb_switch_wire_each'],'guides_springs':a['foot_guides_springs_each'],'fasteners':.008},
 'head_yaw_without_servo':{'cradle':p['servo_cradle'],'two_straps':2*p['servo_strap'],'bearings_fasteners':.006,'collar':p['neck_collar']},
 'head_without_servos':{'shell':p['head_shell'],'hatch':p['head_hatch'],'mane':p['mane'],'horn':p['horn'],'pink_stripe':p['mane_stripe'],'support_yoke':p['head_yoke'],'two_round_LCDs_wiring_allowance':.030,'two_bezels':2*p['eye_bezel'],'camera_carrier':p['camera_carrier'],'camera_FPC':a['camera_with_fpc'],'bearings_fasteners':.016},
 'ear_without_servo':{'shell':p['ear'],'fasteners':.0015}}
 for k,v in groups.items():c['mass'][k]=round(sum(v.values())+.001,6)
 (ROOT/'config/robot.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
 report={'nominal_mass_kg':total_mass(c),'design_mass_kg':total_mass(c)*(1+c['mass']['uncertainty_fraction']),'printed_solid_mass_kg':d['printed_solid_mass_kg']-4*(p['hoof_tpu_pad']+p['hoof_actuator'])*(1-1200/1270),'servo_mass_kg':14*c['actuator_candidates'][c['selection']['selected']]['mass']+2*c['ear_actuator']['mass'],'per_link_allowances':groups,'method':'saved STEP volumes x PETG1270 / TPU1200kg/m3, explicit unweighed purchased-part allowances, 1g contingency per link group; not a scale measurement'}
 (ROOT/'reports/mass_budget.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()

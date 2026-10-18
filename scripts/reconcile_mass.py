"""Link mass properties from placed CAD solids and explicit purchased allowances.
CNC 6061 load parts, PETG cosmetics, TPU soles. Hardware interiors are modelled
as uniform envelope density; wiring/fasteners are point allowances. Not measured.
"""
import json,sys,yaml
import numpy as np
from scripts.common import ROOT,load,actuator,actuator_masses
sys.path.insert(0,str(ROOT/'cad/src'))
from scripts.cad_snapshot import load_parts as link_components
from cadgen import build123d as bd

def main():
 c=load();a=c['sensing']['mass_allowances_kg'];parts=link_components(require_snapshot=True);props={};ledger=[]
 fixed={'battery_envelope':c['power']['battery']['pack_mass_kg'],'compute_envelope':.013,
 'adapter_envelope':.016,'rs485_adapter':.012,'usb_hub':.0022,'regulator_envelope':.010,'imu_board':a['imu_breakout'],
 'collector':a['collector_with_connector'],'depth_board':a['depth_with_harness']-.001,
 'depth_optic':.001,'ear_6V_buck':.012,'depth_3V3_buck':a['depth_3v3_buck'],
 'camera_board':a['camera_with_fpc']-.002,'camera_lens':.002}
 for link,children in parts.items():
  rows=[]
  for s in children:
   n=s.label;material='PETG';density=1.27e-6;mass=None
   if 'M3_heads' in n:mass=0.;material='purchased fastener heads, mass included in fastener_wiring_allowance'
   elif '_clamp_shims' in n:density=2.7e-6;material='rigid aluminium shim stock'
   elif any(t in n for t in ['display_','eyebrow_','smile_pigment','nostril_','ear_inner_']):mass=0.;material='appearance only'
   elif n.endswith('_servo'):
    joint=n.removesuffix('_servo').replace('_roll','_hip_roll').replace('_pitch','_hip_pitch') if n[:2] in c['legs'] else n.removesuffix('_servo')
    mass=actuator(c,joint)['mass'];material='manufacturer mass / uniform envelope'
   elif n.startswith('micro_ear_servo_'):mass=c['ear_actuator']['mass'];material='manufacturer mass / uniform envelope'
   elif n in fixed:mass=fixed[n];material='purchased allowance / uniform envelope'
   elif n.endswith('_passive_bearing'):mass=.027;material='NSK 6807 catalogue allowance'
   elif n.startswith('eye_board_'):mass=.015;material='LCD + harness allowance'
   elif n.startswith('ear_output_adapter_'):mass=.001;material='purchased horn allowance'
   elif n.endswith('_switch_PCB'):mass=a['foot_pcb_switch_wire_each']-.001;material='PCB + switch harness allowance'
   elif n.endswith('_switch'):mass=.001;material='switch allowance'
   elif any(t in n for t in ['TPU_pad','TPU_actuator']):density=1.2e-6;material='TPU'
   elif any(t in n for t in ['chassis_deck','rail_','cradle','strap_','roll_yoke','roll_axle','cheek_','spacer_','knee_support_fork','head_support_yoke','rotor_hub','rotor_spacer','bearing_retainer','knee_cross_spacer']):density=2.7e-6;material='6061 aluminium'
   volume=float(sum(x.volume for x in s.solids()))
   if mass is None:mass=volume*density
   center=np.array(list(s.center(bd.CenterOf.MASS)))/1000
   inertia=np.asarray(s.matrix_of_inertia)*mass/max(volume,1e-12)*1e-6
   rows.append((mass,center,inertia));ledger.append({'link':link,'part':n,'material':material,'mass_kg':mass,'com_m':center.tolist(),'volume_mm3':volume})
  # Fasteners, flexible wiring, fuse/switches and unmodelled sensor springs.
  extra=.123 if link=='torso' else .016 if link=='head' else .006 if link=='neck_yaw' else .0015 if link.startswith('ear_') else (.008+a['foot_guides_springs_each'] if link.endswith('_shin') else .010)
  center=np.array([0.,0.,-.04 if link.endswith(('_thigh','_shin')) else 0.])
  rows.append((extra,center,np.eye(3)*extra*1e-6));ledger.append({'link':link,'part':'fastener_wiring_allowance','material':'unweighed allowance','mass_kg':extra,'com_m':center.tolist()})
  m=sum(r[0] for r in rows);com=sum(r[0]*r[1] for r in rows)/m;I=np.zeros((3,3))
  for mi,ci,Ii in rows:
   d=ci-com;I+=Ii+mi*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
  assert np.linalg.eigvalsh(I).min()>0
  props[link]={'mass':float(m),'com':com.tolist(),'inertia_matrix':I.tolist()}
 # Keep explicit nonmotor group totals for analytical screening.
 motors=actuator_masses(c)
 for key,link,names in [('torso_without_servos','torso',[f'{l}_hip_roll' for l in c['legs']]+['head_yaw']),('roll_link_without_servo','fl_roll',['fl_hip_pitch']),('thigh_without_servo','fl_thigh',['fl_knee']),('shin_and_foot','fl_shin',[]),('head_yaw_without_servo','neck_yaw',['head_pitch']),('head_without_servos','head',['ear_left','ear_right']),('ear_without_servo','ear_left',[])]:
  c['mass'][key]=props[link]['mass']-sum(motors[n] for n in names)
 c['link_mass_properties']=props;c['mass']['method']='placed CAD BRep volume/inertia, 6061/PETG/TPU; catalogue motor masses and unweighed electronics/fastener allowances'
 (ROOT/'config/robot.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
 nominal=sum(p['mass'] for p in props.values());report={'nominal_mass_kg':nominal,'design_mass_kg':nominal*(1+c['mass']['uncertainty_fraction']),'servo_mass_kg':sum(motors.values()),'method':c['mass']['method'],'hardware_inertia_assumption':'uniform CAD envelopes; actual component mass distribution and temperatures unmeasured','components':ledger,'links':props}
 (ROOT/'reports/mass_budget.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ('components','links')},indent=2))
if __name__=='__main__':main()

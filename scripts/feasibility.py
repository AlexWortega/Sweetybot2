"""Per-axis engineering screen using the configured motor set."""
import json,math
from scripts.common import ROOT,load,total_mass,joint_names,actuator,continuous_limits

def screen(c):
 g=c['geometry'];s=c['selection'];mass=total_mass(c);design=mass*(1+c['mass']['uncertainty_fraction'])
 l1,l2,h=g['thigh']/1000,g['shin']/1000,(g['stance_height']-g['hip_drop'])/1000
 knee=math.acos((h*h-l1*l1-l2*l2)/(2*l1*l2));hip=-math.atan2(l2*math.sin(knee),l1+l2*math.cos(knee));arm=abs(l2*math.sin(hip+knee))
 F=design*9.81*s['load_share_worst'];margin=s['torque_margin'];rows=[]
 limits=continuous_limits(c,s['required_joint_speed'])
 for i,n in enumerate(joint_names(c)):
  a=actuator(c,n);vf=min(1.,(6. if n.startswith('ear_') else c['power']['battery']['cutoff_voltage'])/a['voltage'])
  speed_derate=max(0.,1-s['required_joint_speed']/(a['no_load_speed']*vf))
  if n.endswith('hip_roll'):required=margin*F*g['abduction_offset']/1000;transient=required
  elif n.endswith(('hip_pitch','knee')):required=margin*(F*arm+c['mass']['shin_and_foot']*9.81*arm/2);transient=margin*(F*.065+.04)
  elif n=='head_pitch':required=margin*(c['link_mass_properties']['head']['mass']+2*c['mass']['ear_without_servo'])*9.81*.065;transient=required
  elif n=='head_yaw':required=.10;transient=.15
  else:required=margin*c['mass']['ear_without_servo']*9.81*.04;transient=required
  peak=.55*a['stall_torque']*vf*speed_derate
  rows.append({'joint':n,'candidate':a.get('model',c['actuation']['assignment'][n]),'mass_kg':mass,'design_mass_kg':design,'sustained_requirement_nm':required,'conditional_sustained_allowance_nm':limits[i],'transient_requirement_nm':transient,'conditional_transient_allowance_nm':peak,'passes_screen':required<=limits[i] and transient<=peak,'actuator_cost_usd':a.get('unit_usd',0),'price_status':a.get('price_status','planning allowance')})
 return rows

def main():
 c=load();rows=screen(c);result={'status':'conditional per-axis analytical screen; temperatures and dynamics unmeasured','nominal_mass_kg':total_mass(c),'design_mass_kg':total_mass(c)*1.2,'all_pass':all(r['passes_screen'] for r in rows),'joints':rows,'assumptions':{'loaded_leg_weight_fraction':.5,'torque_margin':1.5,'transition_moment_arm_m':.065,'battery_v':c['power']['battery']['cutoff_voltage'],'joint_speed_rad_s':c['selection']['required_joint_speed'],'rated_point':'manufacturer rated point is not a proven continuous thermal rating','torque_speed':'uncalibrated linear envelope'},'power':{'main_fuse_a':40,'leg_fuse_a':10,'bms_continuous_a':40,'catalogue_12_leg_stall_a':12*6.3,'all_stall_exceeds_main_fuse':True,'fuse_coordination_verified':False}}
 (ROOT/'reports/feasibility.json').write_text(json.dumps(result,indent=2)+'\n')
 text=['# V05 actuator screen','','Rated catalogue points are screened at 9.3V and 0.8rad/s. They are not thermal certification. A 20% mass contingency, 50% load on one leg and 1.5 torque margin are included.','','| Joint | Servo | Required/rated envelope Nm | Transition/burst Nm | Pass |','|---|---|---:|---:|---|']
 for r in rows:text.append(f"|{r['joint']}|{r['candidate']}|{r['sustained_requirement_nm']:.3f}/{r['conditional_sustained_allowance_nm']:.3f}|{r['transient_requirement_nm']:.3f}/{r['conditional_transient_allowance_nm']:.3f}|{r['passes_screen']}|")
 text+=['','All simultaneously stalled leg motors alone draw a catalogue 75.6A, above the 40A main fuse. Stall is a fault, not a valid motion. Fuse, BMS, connectors, regenerative overvoltage and joint temperature require bench validation. No load rating is inferred from stall torque alone.']
 (ROOT/'docs/feasibility.md').write_text('\n'.join(text)+'\n');print(json.dumps(result,indent=2))
 if not result['all_pass']:raise SystemExit(1)
if __name__=='__main__':main()

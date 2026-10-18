"""Exact static BRep checks of new interfaces; not a complete swept-motion audit."""
import sys,json
from scripts.common import ROOT
sys.path.insert(0,str(ROOT/'cad/src'))
from lib.shapes import link_components
from lib.sensors_face import head_outer

def main():
 p=link_components();head={s.label:s for s in p['head']};torso={s.label:s for s in p['torso']};rows=[]
 pairs=[('eye_board_1','eye_bezel_1'),('camera_carrier','pony_head_shell'),('camera_board','micro_ear_servo_1'),('camera_carrier','micro_ear_servo_1'),('eye_board_1','micro_ear_servo_1'),('eye_board_-1','micro_ear_servo_-1'),('eye_board_1','pony_head_shell'),('eye_board_-1','pony_head_shell'),('camera_board','pony_head_shell'),('camera_board','camera_carrier'),('micro_ear_servo_1','pony_head_shell'),('sensor_tray','compute_envelope'),('sensor_tray','battery_envelope'),('ear_6V_buck','compute_envelope'),('depth_board','head_yaw_cradle'),('depth_board','chest_tof_pod')]
 for names in pairs:
  d=head if names[0] in head else torso;a,b=(d[n] for n in names);v=(a&b).volume
  rows.append({'pair':names,'overlap_mm3':v,'pass':v<.1});print(names,round(v,4),flush=True)
 for label in ['eye_board_1','eye_board_-1','camera_board','micro_ear_servo_1','micro_ear_servo_-1']:
  v=(head[label]-head_outer()).volume;rows.append({'inside_outer_head':label,'outside_mm3':v,'pass':v<.1});print(label,'outside',v,flush=True)
 report={'scope':'New interfaces at neutral head/ear pose; not full articulated motion or tolerance certification','checks':rows,'pass':all(r['pass'] for r in rows)}
 (ROOT/'reports/new_clearances.json').write_text(json.dumps(report,indent=2)+'\n')
 if not report['pass']:raise SystemExit(1)
if __name__=='__main__':main()

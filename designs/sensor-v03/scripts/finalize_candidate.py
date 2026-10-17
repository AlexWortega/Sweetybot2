"""Summarize measured CAD data and conservative catalog-based screening."""
import hashlib,json,math,argparse
from scripts.common import ROOT,load,total_mass

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--v02-root',type=__import__('pathlib').Path,default=ROOT.parents[1]);args=parser.parse_args()
 c=load();base=json.loads((ROOT/'reports/v02-baseline-sha256.json').read_text());original=args.v02_root
 baseline={name:{'expected':sha,'actual':hashlib.sha256((original/name).read_bytes()).hexdigest()} for name,sha in base.items()}
 assert all(r['expected']==r['actual'] for r in baseline.values())
 f=json.loads((ROOT/'reports/feasibility.json').read_text());selected=next(r for r in f['candidates'] if r['candidate']==c['selection']['selected'])
 ear=c['ear_actuator'];required=c['mass']['ear_without_servo']*9.81*.060*3*1.5
 allow=ear['stall_torque']*.25*(1-.8/ear['no_load_speed'])
 report={'status':'digital candidate; no hardware validated','baseline_hashes':baseline,'mass_kg':total_mass(c),'design_mass_kg':total_mass(c)*1.2,'main_joints_screen':selected,'ear_screen':{'required_nm':required,'conditional_allowance_nm':allow,'assumptions':'60mm lever, 3g acceleration, 1.5 torque margin; 25% stall fraction, 0.8rad/s; no manufacturer continuous rating','conditional_pass':required<=allow},'gross_ear_motor_saving_g':2*(.0745-.020)*1000,'unchanged_v02_training':True}
 (ROOT/'reports/candidate_summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()

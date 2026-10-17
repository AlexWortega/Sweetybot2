"""Repeatable software/artifact verification; physical gates remain open."""
import hashlib,json,subprocess,sys
from scripts.common import ROOT

def main():
 checks=[]
 for label,args in [('pytest',[sys.executable,'-m','pytest','-q']),('urdf',[str(ROOT/'.venv/bin/cadgen'),'urdf','validate','simulation/generated/sweetie_mini.urdf','--strict','--json'])]:
  r=subprocess.run(args,cwd=ROOT,text=True,capture_output=True)
  checks.append(dict(name=label,exit_code=r.returncode,stdout=r.stdout,stderr=r.stderr));print(r.stdout,flush=True)
  if r.returncode:
   (ROOT/'reports/verification.json').write_text(json.dumps({'passed':False,'checks':checks},indent=2)+'\n');raise SystemExit(r.returncode)
 validation=json.loads((ROOT/'reports/validation.json').read_text())
 assert validation['nominal_all_passed']
 assert validation['config_sha256']==hashlib.sha256((ROOT/'config/robot.yaml').read_bytes()).hexdigest()
 for p,digest in validation.get('input_sha256',{}).items():assert hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==digest,p
 parts=json.loads((ROOT/'reports/cad_parts.json').read_text())
 assert all(p['valid'] and p['solids']==1 and p['fits_220_bed'] for p in parts['parts'])
 assertions={'simulation_config_matches':True,'nominal_scenarios':len(validation['nominal']),'sensitivity_scenarios':len(validation['sensitivity']),'print_types':len(parts['parts']),'physical_validation':'not run','external_ROS_check_urdf':'unavailable'}
 (ROOT/'reports/verification.json').write_text(json.dumps({'passed':True,'checks':checks,'assertions':assertions},indent=2)+'\n')
 print('Software and artifact checks passed; physical validation is not run.')
if __name__=='__main__':main()

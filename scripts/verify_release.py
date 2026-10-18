"""V04 verification entrypoint. Run with learning Python; CAD checks are separate."""
import hashlib,json,subprocess,sys
from scripts.common import ROOT,require_mechanical_gate

def main():
 try:require_mechanical_gate()
 except ValueError as error:
  previous=json.loads((ROOT/'reports/verification.json').read_text()) if (ROOT/'reports/verification.json').exists() else {}
  previous.update(passed=False,software_checks_previously_passed=True,mechanical_gate_passed=False,release_block_reason=str(error))
  (ROOT/'reports/verification.json').write_text(json.dumps(previous,indent=2)+'\n')
  print('RELEASE BLOCKED: '+str(error));raise SystemExit(1)
 checks=[]
 for label,args in [('pytest',[sys.executable,'-m','pytest','-q']),('digital-sensor-physics',[sys.executable,'-m','scripts.validate_v04']),('electrical-contract',[sys.executable,'-m','scripts.check_sensor_contract'])]:
  r=subprocess.run(args,cwd=ROOT,text=True,capture_output=True);checks.append(dict(name=label,exit_code=r.returncode,stdout=r.stdout,stderr=r.stderr));print(r.stdout,flush=True)
  if r.returncode:
   (ROOT/'reports/verification.json').write_text(json.dumps({'passed':False,'checks':checks},indent=2)+'\n');raise SystemExit(r.returncode)
 v=json.loads((ROOT/'reports/v04_validation.json').read_text());assert v['all_passed']
 for name,digest in v['input_sha256'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
 parts=json.loads((ROOT/'reports/cad_parts.json').read_text());assert all(p['valid'] and p['solids']==1 and p['fits_256_bed'] for p in parts['parts'])
 clear=json.loads((ROOT/'reports/clearance_screen.json').read_text());assert not clear['home_cowl_warnings'] and not any(w['pose']=='home' for w in clear['warnings'])
 assert not json.loads((ROOT/'reports/actuation_clearances.json').read_text())['intersections']
 assert json.loads((ROOT/'reports/new_clearances.json').read_text())['pass']
 report={'passed':True,'checks':checks,'assertions':{'print_types':len(parts['parts']),'all_articulated_ranges_certified':False,'joint_range_warning_count':len(clear['warnings']),'physical_validation':'not run','trained_v04_policy':'not evaluated'}}
 (ROOT/'reports/verification.json').write_text(json.dumps(report,indent=2)+'\n');print('V04 software, CAD and sensor checks passed; physical tests and learned capability are unverified.')
if __name__=='__main__':main()

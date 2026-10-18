"""V05 verification entrypoint. Run with learning Python; CAD checks are separate."""
import hashlib,json,subprocess,sys
from scripts.common import ROOT,require_mechanical_gate

def main():
 try:require_mechanical_gate()
 except ValueError as error:
  previous=json.loads((ROOT/'reports/verification.json').read_text()) if (ROOT/'reports/verification.json').exists() else {}
  previous.update(passed=False,software_checks_previously_passed=bool(previous.get('checks') and all(x.get('exit_code')==0 for x in previous['checks'])),mechanical_gate_passed=False,release_block_reason=str(error))
  (ROOT/'reports/verification.json').write_text(json.dumps(previous,indent=2)+'\n')
  print('RELEASE BLOCKED: '+str(error));raise SystemExit(1)
 checks=[]
 for label,args in [('pytest',[sys.executable,'-m','pytest','-q']),('digital-sensor-physics',[sys.executable,'-m','scripts.validate_v05']),('electrical-contract',[sys.executable,'-m','scripts.check_sensor_contract'])]:
  r=subprocess.run(args,cwd=ROOT,text=True,capture_output=True);checks.append(dict(name=label,exit_code=r.returncode,stdout=r.stdout,stderr=r.stderr));print(r.stdout,flush=True)
  if r.returncode:
   (ROOT/'reports/verification.json').write_text(json.dumps({'passed':False,'checks':checks},indent=2)+'\n');raise SystemExit(r.returncode)
 v=json.loads((ROOT/'reports/v05_validation.json').read_text());assert v['all_passed']
 for name,digest in v['input_sha256'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
 parts=json.loads((ROOT/'reports/cad_parts.json').read_text());assert all(p['valid'] and p['solids']==1 and p['fits_256_bed'] for p in parts['parts'])
 from learning.readiness import require_training_ready,dynamics_signature
 require_training_ready()
 bounds=json.loads((ROOT/'reports/assembly_bounds.json').read_text());assert bounds['valid'] and all(x['relative_volume_change']<.01 for x in bounds['step_roundtrips'])
 coverage=json.loads((ROOT/'reports/collision_coverage.json').read_text());assert coverage['passed'] and coverage['dynamics_signature']==dynamics_signature()
 assert coverage['mesh_manifest_sha256']==hashlib.sha256((ROOT/'reports/collision_meshes.json').read_bytes()).hexdigest()
 from scripts.cad_snapshot import geometry_signature
 custom=json.loads((ROOT/'reports/manufacturing_parts.json').read_text());assert custom['geometry_signature']==geometry_signature()
 for row in custom['parts']:assert hashlib.sha256((ROOT/row['step']).read_bytes()).hexdigest()==row['sha256'],row['part']
 saved=json.loads((ROOT/'reports/cad_snapshot.json').read_text());assert all(x['relative_volume_change']<.01 for x in saved['parts'])
 export=json.loads((ROOT/'reports/assembly_exports.json').read_text());assert export['geometry_signature']==geometry_signature()
 assert export['config_sha256']==hashlib.sha256((ROOT/'config/robot.yaml').read_bytes()).hexdigest()
 assert json.loads((ROOT/'reports/v05-urdf-validation.json').read_text())['ok']
 report={'passed':True,'checks':checks,'assertions':{'print_types':len(parts['parts']),'all_articulated_ranges_certified':False,'mechanical_audit':'reports/mechanical_gate.json','physical_validation':'not run','trained_v05_policy':'not evaluated'}}
 (ROOT/'reports/verification.json').write_text(json.dumps(report,indent=2)+'\n');print('V05 software, CAD and sensor checks passed; physical tests and learned capability are unverified.')
if __name__=='__main__':main()

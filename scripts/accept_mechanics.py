"""Accept sampled digital geometry only; never certify a fabricated robot."""
import hashlib,json
from scripts.common import ROOT
from scripts.audit_inputs import snapshot

def main():
 inputs=snapshot();static=json.loads((ROOT/'reports/static_fit.json').read_text());motion=json.loads((ROOT/'reports/full_motion_audit.json').read_text())
 for name,r in [('static assembly',static),('articulated motion',motion)]:
  if not r.get('complete') or not r.get('passed') or r.get('source_sha256')!=inputs:raise ValueError(name+' audit failed, incomplete or stale')
 if len(motion['poses'])<140:raise ValueError('Missing full operational pose cohort')
 assert not static['intersections'] and not any(x['intersections'] for x in motion['poses'])
 files={**inputs}
 for n in ['reports/static_fit.json','reports/full_motion_audit.json','reports/fcl_motion_candidates.json','scripts/audit_inputs.py','scripts/find_motion_candidates.py','scripts/check_assembly_fits.py','scripts/check_motion_candidates.py','scripts/cad_snapshot.py','reports/cad_snapshot.json']:
  files[n]=hashlib.sha256((ROOT/n).read_bytes()).hexdigest()
 report={'passed':True,'status':'SAMPLED_DIGITAL_GEOMETRY_ACCEPTED','all_link_pairs_checked':True,'sampled_poses':len(motion['poses']),'scope':'All separate physical components at fixed-link assembly and all cross-link surface pairs in the declared home, individual joint, coordinated gait and recovery-reset cohort. Exact solid intersections confirm FCL triangle candidates. This is sampled geometry, not continuous collision detection or all joint combinations.','fabrication_approved':False,'physical_validation':'not run','bench_gates':['shaft/horn spline and screw engagement','bearing fits and load test','fastener access and cable routing on actual assembly','fatigue and servo thermal endurance','power/BMS/fuse coordination','real firmware, drivers and sensor timing'],'source_sha256':files}
 (ROOT/'reports/mechanical_gate.json').write_text(json.dumps(report,indent=2)+'\n');print(report['status'],report['sampled_poses'])
if __name__=='__main__':main()

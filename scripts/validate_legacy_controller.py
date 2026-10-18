"""Full nominal acceptance and explicit sensitivity cases; preserves all failures."""
import json,hashlib
from scripts.common import ROOT,load
from simulation.run import run

def main():
 inputs=['config/robot.yaml','simulation/generated/sweetie_mini.xml','simulation/model.py','simulation/run.py']
 def fingerprints():return {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in inputs}
 frozen=fingerprints()
 def unchanged():
  if fingerprints()!=frozen:raise RuntimeError('Validation inputs changed during the suite; do not publish mixed results')
 c=load();cases=[]
 for task,seconds in [('stand',60),('walk',60),('turn',60),('sit',60)]:
  unchanged();r=run(task,seconds,out=f'reports/sim_{task}.json');unchanged();cases.append(r)
 variations=[('mass_plus_20',dict(mass_scale=1.2)),('low_friction',dict(friction=.55)),('delay_60ms',dict(delay=3)),('backlash_2deg',dict(backlash=.0349066)),('combined',dict(mass_scale=1.2,friction=.55,delay=3,backlash=.0349066))]
 sensitivity=[]
 for label,params in variations:
  unchanged();r=run('walk',out=f'reports/sensitivity_{label}.json',**params);unchanged();r['case']=label;sensitivity.append(r)
 result={'config_sha256':frozen['config/robot.yaml'],'input_sha256':frozen,'nominal_all_passed':all(x['passed'] for x in cases),'nominal':cases,'sensitivity':sensitivity,'physical_validation':'not run','actuator_model':'uncalibrated torque-speed envelope and virtual support-wrench controller; hardware position-loop transfer unverified'}
 (ROOT/'reports/validation.json').write_text(json.dumps(result,indent=2)+'\n')
 lines=['# Measured digital results','',f"Nominal acceptance: **{'PASS' if result['nominal_all_passed'] else 'FAIL'}**. Config SHA-256 `{result['config_sha256']}`.",'','| Scenario | Pass | Simulated s | Forward m | Yaw deg | Max tilt deg | Max path error m |','|---|---|---:|---:|---:|---:|---:|']
 for r in cases+sensitivity:
  lines.append(f"| {r.get('case',r['task'])} | {r['passed']} | {r['simulated_duration_s']:.2f} | {r['displacement_m'][0]:.4f} | {r['yaw_rad']*57.2958:.2f} | {r['max_tilt_rad']*57.2958:.2f} | {r['max_tracking_error_m']:.4f} |")
  if r['error']:lines.append(f"\nFailure ({r.get('case',r['task'])}): `{r['error']}`\n")
 peak=max(max(r['peak_torque_nm'].values()) for r in cases);rms=max(max(r['rms_torque_nm'].values()) for r in cases)
 lines+=['',f'Largest nominal motor peak: {peak:.3f} Nm; largest per-run RMS: {rms:.3f} Nm. These are model outputs, not measured motor capability.','', 'No root forces, root welds or pose resets were used. Ground-truth base state is available to the controller. Actual CAD meshes are visual assets; contact bodies are simplified. The COTS position-loop hardware transfer and thermal tests remain outstanding.','', 'Raw reports retain per-joint peak/RMS torque and speed, actual duration, failures, contacts and the full path samples. A successful low-speed simulation is not a fabrication release.']
 (ROOT/'docs/results.md').write_text('\n'.join(lines)+'\n')
 if not result['nominal_all_passed']:raise SystemExit(1)
if __name__=='__main__':main()

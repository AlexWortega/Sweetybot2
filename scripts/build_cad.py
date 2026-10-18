"""Write thin entrypoints and build their independently cached outputs."""
from pathlib import Path
import os,subprocess,sys
from concurrent.futures import ThreadPoolExecutor
from scripts.common import ROOT
NAMES=['deck','rail','body_shell','servo_cradle','roll_yoke','thigh_plate','shin_plate','hoof','head_shell','head_hatch','ear','mane','fit_coupon','axle_adapter','knee_fork','head_yoke','tail','servo_strap','horn','mane_stripe','neck_collar','thigh_fairing','shin_fairing']
NAMES += ['camera_carrier', 'sensor_tray', 'depth_pod', 'hoof_sole_plate', 'hoof_tpu_pad', 'hoof_actuator', 'eye_bezel']
NAMES += ['thigh_rear_plate','bearing_retainer','rotor_spacer','rear_hip_hub','rear_knee_hub','knee_front_hub','shin_rear_plate','servo_cradle_sm105','servo_strap_sm105']
FITTED={'tail':'passive_tail','mane':'mane_crest','mane_stripe':'pink_mane_stripe','horn':'unicorn_horn'}
HEADER='''from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES
'''
def main():
 for name in NAMES:
  p=ROOT/'cad/src'/f'{name}.py'
  p.write_text(HEADER+f"\n@step(out='../STEP/{name}.step')\n@stl(out='../STL/{name}.stl',mesh_tolerance=0.0005)\ndef {name}():return FACTORIES['{name}']()\n\nif __name__=='__main__':{name}()\n")
 def build(name):
  # Cold builds avoid warm-worker module reuse for this dynamic factory registry.
  subprocess.run([sys.executable,str(ROOT/'cad/src'/f'{name}.py'),'--force'],cwd=ROOT,env={**os.environ,'CADGEN_DAEMON':'0'},check=True)
 with ThreadPoolExecutor(max_workers=3) as pool:list(pool.map(build,NAMES))
 # Surface-seated cosmetics have assembly-specific mating reliefs. Export
 # their actual fitted solids rather than the untrimmed concept primitives.
 sys.path.insert(0,str(ROOT/'cad/src'))
 from lib.shapes import link_components
 from cadgen import build123d as bd
 fitted={s.label:s for vv in link_components().values() for s in vv}
 for name,label in FITTED.items():
  s=fitted[label];bd.export_step(s,ROOT/'cad/STEP'/f'{name}.step');bd.export_stl(s,ROOT/'cad/STL'/f'{name}.stl',tolerance=.15,angular_tolerance=.25)
 # Verified snapshot-based assembly export follows mass reconciliation in make cad.
if __name__=='__main__':main()

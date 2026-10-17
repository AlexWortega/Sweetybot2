"""Write thin entrypoints and build their independently cached outputs."""
from pathlib import Path
import os,subprocess,sys
from scripts.common import ROOT
NAMES=['deck','rail','body_shell','servo_cradle','roll_yoke','thigh_plate','shin_plate','hoof','head_shell','head_hatch','ear','mane','fit_coupon','axle_adapter','knee_fork','head_yoke','tail','servo_strap','horn','mane_stripe','neck_collar','thigh_fairing','shin_fairing']
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
 for name in NAMES+['assembly','internal','exploded']:
  # Cold builds avoid warm-worker module reuse for this dynamic factory registry.
  subprocess.run([sys.executable,str(ROOT/'cad/src'/f'{name}.py'),'--force'],cwd=ROOT,env={**os.environ,'CADGEN_DAEMON':'0'},check=True)
if __name__=='__main__':main()

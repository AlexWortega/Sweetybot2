from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/servo_strap.step')
@stl(out='../STL/servo_strap.stl',mesh_tolerance=0.0005)
def servo_strap():return FACTORIES['servo_strap']()

if __name__=='__main__':servo_strap()

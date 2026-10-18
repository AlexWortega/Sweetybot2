from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/servo_strap_sm105.step')
@stl(out='../STL/servo_strap_sm105.stl',mesh_tolerance=0.0005)
def servo_strap_sm105():return FACTORIES['servo_strap_sm105']()

if __name__=='__main__':servo_strap_sm105()

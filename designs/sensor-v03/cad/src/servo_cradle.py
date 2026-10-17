from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/servo_cradle.step')
@stl(out='../STL/servo_cradle.stl',mesh_tolerance=0.0005)
def servo_cradle():return FACTORIES['servo_cradle']()

if __name__=='__main__':servo_cradle()

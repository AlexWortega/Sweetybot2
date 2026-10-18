from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/servo_cradle_sm80.step')
@stl(out='../STL/servo_cradle_sm80.stl',mesh_tolerance=0.0005)
def servo_cradle_sm80():return FACTORIES['servo_cradle_sm80']()

if __name__=='__main__':servo_cradle_sm80()

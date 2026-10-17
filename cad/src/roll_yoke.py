from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/roll_yoke.step')
@stl(out='../STL/roll_yoke.stl',mesh_tolerance=0.0005)
def roll_yoke():return FACTORIES['roll_yoke']()

if __name__=='__main__':roll_yoke()

from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/neck_collar.step')
@stl(out='../STL/neck_collar.stl',mesh_tolerance=0.0005)
def neck_collar():return FACTORIES['neck_collar']()

if __name__=='__main__':neck_collar()

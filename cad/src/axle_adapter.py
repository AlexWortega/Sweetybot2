from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/axle_adapter.step')
@stl(out='../STL/axle_adapter.stl',mesh_tolerance=0.0005)
def axle_adapter():return FACTORIES['axle_adapter']()

if __name__=='__main__':axle_adapter()

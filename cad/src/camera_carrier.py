from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/camera_carrier.step')
@stl(out='../STL/camera_carrier.stl',mesh_tolerance=0.0005)
def camera_carrier():return FACTORIES['camera_carrier']()

if __name__=='__main__':camera_carrier()

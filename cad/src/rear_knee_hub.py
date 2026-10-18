from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/rear_knee_hub.step')
@stl(out='../STL/rear_knee_hub.stl',mesh_tolerance=0.0005)
def rear_knee_hub():return FACTORIES['rear_knee_hub']()

if __name__=='__main__':rear_knee_hub()

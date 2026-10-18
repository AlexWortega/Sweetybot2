from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/rear_hip_hub.step')
@stl(out='../STL/rear_hip_hub.stl',mesh_tolerance=0.0005)
def rear_hip_hub():return FACTORIES['rear_hip_hub']()

if __name__=='__main__':rear_hip_hub()

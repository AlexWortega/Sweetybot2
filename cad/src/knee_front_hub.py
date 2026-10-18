from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/knee_front_hub.step')
@stl(out='../STL/knee_front_hub.stl',mesh_tolerance=0.0005)
def knee_front_hub():return FACTORIES['knee_front_hub']()

if __name__=='__main__':knee_front_hub()

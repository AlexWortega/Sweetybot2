from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/depth_pod.step')
@stl(out='../STL/depth_pod.stl',mesh_tolerance=0.0005)
def depth_pod():return FACTORIES['depth_pod']()

if __name__=='__main__':depth_pod()

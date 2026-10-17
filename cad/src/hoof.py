from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/hoof.step')
@stl(out='../STL/hoof.stl',mesh_tolerance=0.0005)
def hoof():return FACTORIES['hoof']()

if __name__=='__main__':hoof()

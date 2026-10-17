from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/horn.step')
@stl(out='../STL/horn.stl',mesh_tolerance=0.0005)
def horn():return FACTORIES['horn']()

if __name__=='__main__':horn()

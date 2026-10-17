from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/head_hatch.step')
@stl(out='../STL/head_hatch.stl',mesh_tolerance=0.0005)
def head_hatch():return FACTORIES['head_hatch']()

if __name__=='__main__':head_hatch()

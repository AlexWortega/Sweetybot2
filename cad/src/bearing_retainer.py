from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/bearing_retainer.step')
@stl(out='../STL/bearing_retainer.stl',mesh_tolerance=0.0005)
def bearing_retainer():return FACTORIES['bearing_retainer']()

if __name__=='__main__':bearing_retainer()

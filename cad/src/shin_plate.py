from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/shin_plate.step')
@stl(out='../STL/shin_plate.stl',mesh_tolerance=0.0005)
def shin_plate():return FACTORIES['shin_plate']()

if __name__=='__main__':shin_plate()

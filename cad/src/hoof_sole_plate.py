from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/hoof_sole_plate.step')
@stl(out='../STL/hoof_sole_plate.stl',mesh_tolerance=0.0005)
def hoof_sole_plate():return FACTORIES['hoof_sole_plate']()

if __name__=='__main__':hoof_sole_plate()

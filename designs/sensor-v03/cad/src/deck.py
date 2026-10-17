from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/deck.step')
@stl(out='../STL/deck.stl',mesh_tolerance=0.0005)
def deck():return FACTORIES['deck']()

if __name__=='__main__':deck()

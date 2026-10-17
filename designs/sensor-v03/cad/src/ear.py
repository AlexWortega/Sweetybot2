from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/ear.step')
@stl(out='../STL/ear.stl',mesh_tolerance=0.0005)
def ear():return FACTORIES['ear']()

if __name__=='__main__':ear()

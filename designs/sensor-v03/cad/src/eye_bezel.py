from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/eye_bezel.step')
@stl(out='../STL/eye_bezel.stl',mesh_tolerance=0.0005)
def eye_bezel():return FACTORIES['eye_bezel']()

if __name__=='__main__':eye_bezel()

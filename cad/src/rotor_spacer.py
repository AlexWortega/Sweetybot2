from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/rotor_spacer.step')
@stl(out='../STL/rotor_spacer.stl',mesh_tolerance=0.0005)
def rotor_spacer():return FACTORIES['rotor_spacer']()

if __name__=='__main__':rotor_spacer()

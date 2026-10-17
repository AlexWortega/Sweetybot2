from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/mane_stripe.step')
@stl(out='../STL/mane_stripe.stl',mesh_tolerance=0.0005)
def mane_stripe():return FACTORIES['mane_stripe']()

if __name__=='__main__':mane_stripe()

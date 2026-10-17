from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/fit_coupon.step')
@stl(out='../STL/fit_coupon.stl',mesh_tolerance=0.0005)
def fit_coupon():return FACTORIES['fit_coupon']()

if __name__=='__main__':fit_coupon()

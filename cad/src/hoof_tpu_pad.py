from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/hoof_tpu_pad.step')
@stl(out='../STL/hoof_tpu_pad.stl',mesh_tolerance=0.0005)
def hoof_tpu_pad():return FACTORIES['hoof_tpu_pad']()

if __name__=='__main__':hoof_tpu_pad()

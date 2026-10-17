from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/knee_fork.step')
@stl(out='../STL/knee_fork.stl',mesh_tolerance=0.0005)
def knee_fork():return FACTORIES['knee_fork']()

if __name__=='__main__':knee_fork()

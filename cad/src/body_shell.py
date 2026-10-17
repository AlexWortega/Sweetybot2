from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/body_shell.step')
@stl(out='../STL/body_shell.stl',mesh_tolerance=0.0005)
def body_shell():return FACTORIES['body_shell']()

if __name__=='__main__':body_shell()

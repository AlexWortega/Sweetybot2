from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/head_shell.step')
@stl(out='../STL/head_shell.stl',mesh_tolerance=0.0005)
def head_shell():return FACTORIES['head_shell']()

if __name__=='__main__':head_shell()

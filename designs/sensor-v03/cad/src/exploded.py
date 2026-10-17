from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step
from lib.shapes import assembly_shape
@step(out='../STEP/exploded.step')
def exploded():return assembly_shape(exploded=True)
if __name__=='__main__':exploded()

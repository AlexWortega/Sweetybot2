from __future__ import annotations
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import build123d as bd,step,glb
from lib.shapes import assembly_shape

@step(out='../STEP/assembly.step')
@glb(out='../GLB/assembly.glb')
def assembly():return assembly_shape()

if __name__=='__main__':assembly()

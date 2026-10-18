from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from cadgen import step,stl
from lib.shapes import FACTORIES

@step(out='../STEP/sensor_tray.step')
@stl(out='../STL/sensor_tray.stl',mesh_tolerance=0.0005)
def sensor_tray():return FACTORIES['sensor_tray']()

if __name__=='__main__':sensor_tray()

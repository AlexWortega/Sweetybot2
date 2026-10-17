"""Hardware-neutral command validation and fail-closed state machine; no serial I/O."""
from dataclasses import dataclass
import math
from scripts.common import load,joint_names
from simulation.model import joint_spec

@dataclass(frozen=True)
class JointCommand:
    sequence:int
    timestamp_s:float
    positions_rad:dict[str,float]

def validate(command:JointCommand,c=None):
    c=c or load();names=set(joint_names(c))
    if set(command.positions_rad)!=names:raise ValueError('A command must name exactly all 16 joints')
    if not isinstance(command.sequence,int) or command.sequence<0:raise ValueError('Invalid sequence')
    if not math.isfinite(command.timestamp_s):raise ValueError('Nonfinite timestamp')
    for name,value in command.positions_rad.items():
        lo,hi=joint_spec(name,c)['limits']
        if not math.isfinite(value) or not lo<=value<=hi:raise ValueError('Invalid joint position: '+name)
    return command

class Supervisor:
    """Startup and faults never enable motion. Explicit reset+enable is required."""
    def __init__(self,c=None):self.c=c or load();self.state='disabled';self.last=None;self.reason=None
    def enable(self,sensors_fresh:bool):
        if self.state=='fault' or not sensors_fresh:raise ValueError('Clear fault and obtain fresh sensors first')
        self.state='enabled'
    def disable(self):self.state='disabled';self.last=None
    def clear_fault(self):self.state='disabled';self.reason=None;self.last=None
    def fault(self,reason):self.state='fault';self.reason=reason
    def accept(self,command,now_s):
        if self.state!='enabled':raise ValueError('Not enabled')
        if not math.isfinite(now_s):self.fault('invalid_clock');raise ValueError('Nonfinite clock')
        validate(command,self.c)
        if command.timestamp_s>now_s or now_s-command.timestamp_s>self.c['control']['command_timeout_s']:self.fault('stale_command');raise ValueError('Stale/future command')
        if self.last and command.sequence<=self.last.sequence:raise ValueError('Replay/out-of-order command')
        self.last=command
    def tick(self,now_s,sensors_fresh=True):
        if self.state=='enabled':
            if not math.isfinite(now_s) or (self.last is not None and now_s<self.last.timestamp_s):self.fault('invalid_clock')
            elif not sensors_fresh:self.fault('stale_sensors')
            elif self.last is None or now_s-self.last.timestamp_s>self.c['control']['command_timeout_s']:self.fault('command_timeout')
        return {'state':self.state,'reason':self.reason,'motor_write_permitted':self.state=='enabled' and self.last is not None}

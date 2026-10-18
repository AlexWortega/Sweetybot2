"""Reference host-side contact decoding for the proposed resistor-coded loops.
No ADC, MCU firmware, UART or robot I/O is implemented here.
"""
from dataclasses import dataclass
from enum import Enum
import math
class State(str,Enum):
 CONTACT='contact';AIR='air';SHORT='short';OPEN='open';UNKNOWN='unknown'
def classify(ratio:float)->State:
 if not math.isfinite(ratio) or not 0<=ratio<=1:return State.UNKNOWN
 if ratio<.02:return State.SHORT
 if ratio>.90:return State.OPEN
 if .04<=ratio<=.16:return State.CONTACT
 if .35<=ratio<=.65:return State.AIR
 return State.UNKNOWN
@dataclass
class Debouncer:
 """Require 10ms of stable valid state; any wiring fault invalidates immediately."""
 candidate:State=State.UNKNOWN
 candidate_since:float=0.
 state:State=State.UNKNOWN
 last_time:float=-math.inf
 def update(self,ratio:float,time_s:float)->State:
  if not math.isfinite(time_s) or time_s<=self.last_time:raise ValueError('strictly increasing finite timestamps required')
  gap=time_s-self.last_time;self.last_time=time_s;new=classify(ratio)
  if new not in (State.AIR,State.CONTACT):
   self.state=self.candidate=new;self.candidate_since=time_s;return self.state
  if gap>.020 or new!=self.candidate:
   self.candidate=new;self.candidate_since=time_s;self.state=State.UNKNOWN
  if time_s-self.candidate_since>=.010-1e-9:self.state=new
  return self.state
 def current(self,now_s:float)->State:
  if not math.isfinite(now_s) or now_s<self.last_time or now_s-self.last_time>.020:return State.UNKNOWN
  return self.state

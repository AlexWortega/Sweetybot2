"""Versioned, CRC-protected collector packet decoder. No serial or MCU driver.
Units: gyro rad/s, accelerometer specific force m/s2, ADC counts 0..4095.
MCU monotonic microseconds are extended to uint64. One stream per collector.
"""
from dataclasses import dataclass,field
import struct,zlib,math
from hardware.contact_adc import Debouncer,State
HEADER=struct.Struct('<2sBIQ6f4H')
MAGIC=b'SB';VERSION=1

def pack(sequence,time_us,gyro,accel,adc):
 payload=HEADER.pack(MAGIC,VERSION,sequence,time_us,*gyro,*accel,*adc)
 return payload+struct.pack('<I',zlib.crc32(payload))

@dataclass
class Decoder:
 sequence:int=-1
 time_us:int=-1
 contacts:list=field(default_factory=lambda:[Debouncer() for _ in range(4)])
 def decode(self,packet):
  if len(packet)!=HEADER.size+4:raise ValueError('collector packet size')
  payload=packet[:-4]
  if zlib.crc32(payload)!=struct.unpack('<I',packet[-4:])[0]:raise ValueError('collector CRC')
  magic,version,seq,ts,*values=HEADER.unpack(payload)
  if magic!=MAGIC or version!=VERSION:raise ValueError('collector version')
  if self.sequence>=0 and ((seq-self.sequence)&0xffffffff) not in range(1,1001):raise ValueError('repeated/reordered collector packet')
  if ts<=self.time_us:raise ValueError('nonmonotonic collector timestamp')
  if not all(math.isfinite(v) for v in values[:6]) or any(v>4095 for v in values[6:]):raise ValueError('invalid collector sample')
  # Commit stream state only after all validation has passed.
  self.sequence,self.time_us=seq,ts;t=ts/1e6
  states=[d.update(v/4095,t) for d,v in zip(self.contacts,values[6:])]
  return {'timestamp_s':t,'gyro_rad_s':values[:3],'specific_force_m_s2':values[3:6],
    'contact':[int(s==State.CONTACT) for s in states],
    'contact_valid':[int(s in (State.CONTACT,State.AIR)) for s in states]}
 def contact_state(self,now_s):
  states=[d.current(now_s) for d in self.contacts]
  return [int(s==State.CONTACT) for s in states],[int(s in (State.CONTACT,State.AIR)) for s in states]

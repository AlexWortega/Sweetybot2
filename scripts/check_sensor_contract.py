"""Check pin ownership, resistor tolerances and fail-invalid contact semantics."""
import itertools,json,sys
from pathlib import Path
from scripts.common import ROOT,load
sys.path.insert(0,str(ROOT))
from hardware.contact_adc import classify,State,Debouncer

def main():
 c=load();s=c['sensing'];checks=[]
 for bus in ['pi_gpio_bcm','xiao_gpio']:
  pins=list(s[bus].values());assert len(pins)==len(set(pins)),bus
  checks.append({'check':bus+' unique ownership','pass':True})
 ratios={}
 for contact in (False,True):
  vals=[]
  for up,end,closed in itertools.product([.99,1.01],repeat=3):
   ru=10000*up;re=10000*end;rc=1000*closed;r=1/(1/re+1/rc) if contact else re
   for error in [-.01,.01]:
    v=r/(ru+r)+error;vals.append(v);assert classify(v)==(State.CONTACT if contact else State.AIR)
  ratios['contact' if contact else 'air']=[min(vals),max(vals)]
 for v,expected in [(0,State.SHORT),(1,State.OPEN),(float('nan'),State.UNKNOWN),(.25,State.UNKNOWN)]:assert classify(v)==expected
 b=Debouncer();assert b.update(.5,0)==State.UNKNOWN;assert b.update(.5,.005)==State.UNKNOWN;assert b.update(.5,.010)==State.AIR
 assert b.update(1,.015)==State.OPEN;assert b.current(.1)==State.UNKNOWN
 b=Debouncer();b.update(.5,0);b.update(.5,.010);assert b.update(.5,.100)==State.UNKNOWN
 checks+= [{'check':'1% resistor corners plus 1% Vref ADC error','pass':True,'voltage_ratio_ranges':ratios},{'check':'fault immediate; 10ms debounce; >20ms stale invalid','pass':True}]
 report={'status':'offline electrical model and reference decoder only; no hardware drivers or firmware verified','checks':checks}
 (ROOT/'reports/sensor_contract.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()

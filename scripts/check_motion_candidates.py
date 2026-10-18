"""Reproducible v05 digital mechanical audit; see docs/v05.md for scope."""
from scripts.audit_inputs import snapshot,overlap_volume
INPUTS=snapshot()
import sys,json,itertools
import numpy as np
from scripts.common import ROOT,load
sys.path.insert(0,str(ROOT/'cad/src'))
from lib.shapes import matrix_location
from scripts.cad_snapshot import load_parts as link_components
from simulation.model import graph,fk
P=link_components();lookup={s.label:(link,s) for link,ss in P.items() for s in ss};c=load();L=graph(c);R=json.loads((ROOT/'reports/fcl_motion_candidates.json').read_text());assert R['complete'] and R['source_sha256']==INPUTS, 'Candidate audit is stale';results=[];memo={}
for row in R['poses']:
 if row['pose'].startswith('random'):continue
 T=fk(L,row['q'],c);hits=[]
 for a,b in row['pairs']:
  if a not in lookup or b not in lookup:continue
  la,sa=lookup[a];lb,sb=lookup[b];rel=np.linalg.inv(T[la])@T[lb];key=(a,b,tuple(np.round(rel,7).ravel()))
  if key in memo:v=memo[key]
  else:
   M=rel.copy();M[:3,3]*=1000;s2=sb.moved(matrix_location(M));v=overlap_volume(sa,s2);memo[key]=v
  if v>.1:hits.append({'parts':[a,b],'overlap_mm3':float(v)})
 results.append({'pose':row['pose'],'q':row['q'],'intersections':hits})
 if hits:print(row['pose'],json.dumps(hits[:10]),flush=True)
 (ROOT/'reports/full_motion_audit.json').write_text(json.dumps({'source_sha256':INPUTS,'complete':False,'method':'FCL broad phase with exact BRep confirmation; cross-link physical parts, axis sweeps and gait; random poses excluded from acceptance','poses':results},indent=2)+'\n')
print('DONE',len(results),'hit poses',sum(bool(x['intersections']) for x in results),'exact tests',len(memo),flush=True)

assert snapshot()==INPUTS, 'Source changed during audit'
report=json.loads((ROOT/'reports/full_motion_audit.json').read_text());report.update(complete=True,passed=not any(x['intersections'] for x in results));(ROOT/'reports/full_motion_audit.json').write_text(json.dumps(report,indent=2)+'\n')
if not report['passed']:raise SystemExit(1)

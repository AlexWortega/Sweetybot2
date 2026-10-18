"""Check placed convex proxies cover sampled real CAD surfaces within mesh tolerance."""
import json,hashlib
import numpy as np,trimesh
from scipy.spatial import ConvexHull
from scripts.common import ROOT

def main():
 manifest=json.loads((ROOT/'reports/collision_meshes.json').read_text());assert manifest['complete'];rows=[]
 for row in manifest['parts']:
  if row['role']=='sole_visual':continue
  src=trimesh.load(ROOT/row['source'],force='mesh');v=src.vertices
  ids=np.linspace(0,len(v)-1,min(1024,len(v)),dtype=int);points=v[ids];covered=np.zeros(len(points),bool);tight=np.zeros(len(points),bool)
  bounds=[]
  for name in row['hulls']:
   mesh=trimesh.load(ROOT/name,force='mesh');h=ConvexHull(mesh.vertices);eq=h.equations
   bounds.append(mesh.bounds)
   candidate=(points>=mesh.bounds[0]-1.).all(1)&(points<=mesh.bounds[1]+1.).all(1)&~tight
   if candidate.any():
    violation=(points[candidate]@eq[:,:3].T+eq[:,3]).max(1);covered[candidate]|=violation<=1.;tight[candidate]|=violation<=.3
  rows.append({'link':row['link'],'part':row['part'],'sampled_vertices':len(points),'covered_within_03mm':int(tight.sum()),'covered_within_1mm':int(covered.sum()),'uncovered':int((~covered).sum()),'uncovered_points_mm':points[~covered][:64].tolist(),'source_bounds_mm':src.bounds.tolist(),'proxy_bounds_mm':[np.min(np.array(bounds)[:,0],axis=0).tolist(),np.max(np.array(bounds)[:,1],axis=0).tolist()]})
 from learning.readiness import dynamics_signature
 report={'dynamics_signature':dynamics_signature(),'mesh_manifest_sha256':hashlib.sha256((ROOT/'reports/collision_meshes.json').read_bytes()).hexdigest(),'method':'up to 1024 deterministic surface vertices per part; point in union of convex hull half-spaces, 1mm proxy tolerance (0.3mm coverage also reported); sampled coverage, not a certified Hausdorff bound','passed':all(r['uncovered']==0 for r in rows),'parts':rows}
 (ROOT/'reports/collision_coverage.json').write_text(json.dumps(report,indent=2)+'\n')
 print('Coverage',report['passed'],[(r['part'],r['uncovered']) for r in rows if r['uncovered']])
 if not report['passed']:raise SystemExit(1)
if __name__=='__main__':main()

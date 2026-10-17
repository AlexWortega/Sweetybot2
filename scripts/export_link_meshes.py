"""Export actual CAD link-local visual meshes for both descriptions (millimetres)."""
import sys,json,struct
import numpy as np
from scripts.common import ROOT,load
sys.path.insert(0,str(ROOT/'cad/src'))
from cadgen import build123d as bd
from lib.shapes import link_components

def export_msh(stl,target):
 # MuJoCo's STL reader has a 200k-face limit. MSH preserves all triangles,
 # deduplicates vertices, and avoids that reader limit without decimation.
 faces=np.fromfile(stl,dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')]),offset=84)
 vertices,index=np.unique(faces['v'].reshape(-1,3),axis=0,return_inverse=True)
 with target.open('wb') as f:
  f.write(struct.pack('<4i',len(vertices),0,0,len(faces)))
  f.write(vertices.astype('<f4').tobytes());f.write(index.astype('<i4').tobytes())
 return len(faces)

def main():
 target=ROOT/'cad/STL/links';target.mkdir(exist_ok=True)
 results=[]
 for name,children in link_components().items():
  shape=bd.Compound(children=children,label=name)
  p=target/f'{name}.stl'
  if not bd.export_stl(shape,p,tolerance=.15,angular_tolerance=.25):raise RuntimeError(str(p))
  faces=export_msh(p,p.with_suffix('.msh'))
  b=shape.bounding_box()
  results.append({'link':name,'path':str(p.relative_to(ROOT)),'mujoco_path':str(p.with_suffix('.msh').relative_to(ROOT)),'triangles':faces,'bounds_mm':list(b.size),'volume_mm3':shape.volume})
 (ROOT/'reports/link_meshes.json').write_text(json.dumps(results,indent=2)+'\n')
 print(f'Exported {len(results)} actual CAD link meshes')
if __name__=='__main__':main()

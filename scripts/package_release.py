"""Package editable sources, pinned references, CAD, simulation and evidence."""
import hashlib,json,zipfile
from scripts.common import ROOT,load

def files():
 paths=[ROOT/x for x in ['README.md','pyproject.toml','uv.lock','Makefile','.gitignore']]
 for folder in ['config','cad/src','cad/STEP','cad/STL','simulation','scripts','tests','docs','references','reports','renders']:
  for p in (ROOT/folder).rglob('*'):
   if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ['.pyc','.log'] and p.name not in ['artifact_manifest.json','desktop_delivery.json']:paths.append(p)
 paths.append(ROOT/'cad/GLB/assembly.glb')
 return sorted(set(paths))

def main():
 c=load();v=json.loads((ROOT/'reports/validation.json').read_text());verified=json.loads((ROOT/'reports/verification.json').read_text())
 assert verified['passed'] and v['nominal_all_passed']
 assert v['config_sha256']==hashlib.sha256((ROOT/'config/robot.yaml').read_bytes()).hexdigest()
 for required in ['renders/assembly.png','renders/internal.png','renders/parts_review.jpg','renders/walk_preview.mp4','docs/results.md','reports/clearance_screen.json']:
  assert (ROOT/required).is_file(),required
 selected=files();entries=[]
 for p in selected:entries.append({'path':str(p.relative_to(ROOT)),'bytes':p.stat().st_size,'sha256':hashlib.file_digest(p.open('rb'),'sha256').hexdigest()})
 manifest=ROOT/'reports/artifact_manifest.json';manifest.write_text(json.dumps({'revision':c['revision'],'status':'digital prototype; physical validation not run','files':entries},indent=2)+'\n')
 selected.append(manifest);target=ROOT/'dist'/f"sweetie-mini-{c['revision']}.zip";target.parent.mkdir(exist_ok=True)
 with zipfile.ZipFile(target,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
  for p in sorted(selected):z.write(p,'sweetie-mini/'+str(p.relative_to(ROOT)))
 with zipfile.ZipFile(target) as z:assert z.testzip() is None
 digest=hashlib.file_digest(target.open('rb'),'sha256').hexdigest();target.with_suffix('.zip.sha256').write_text(digest+'  '+target.name+'\n')
 print(json.dumps({'archive':str(target.relative_to(ROOT)),'bytes':target.stat().st_size,'files':len(selected),'sha256':digest},indent=2))
if __name__=='__main__':main()

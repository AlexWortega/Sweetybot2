"""Package v05 source, saved CAD, sensor contract, references and evidence."""
import hashlib,json,zipfile
from scripts.common import ROOT,load,require_mechanical_gate

def files():
 paths=[ROOT/x for x in ['README.md','pyproject.toml','uv.lock','Makefile','.gitignore','.gitattributes']]
 for folder in ['config','cad/src','cad/STEP','cad/STL','cad/GLB','cad/manufacturing','cad/collision','simulation','learning','hardware','scripts','tests','docs','references','reports','renders']:
  for p in (ROOT/folder).rglob('*'):
   if not p.is_file() or any(k in p.parts for k in ['__pycache__','models','runs','results','artifacts','checkpoints']):continue
   if p.suffix in ['.pyc','.log','.so'] or p.name in ['artifact_manifest.json','desktop_delivery.json']:continue
   paths.append(p)
 return sorted(set(paths))

def main():
 require_mechanical_gate()
 c=load();v=json.loads((ROOT/'reports/v05_validation.json').read_text());check=json.loads((ROOT/'reports/verification.json').read_text());assert v['all_passed'] and check['passed']
 for n,digest in v['input_sha256'].items():assert hashlib.sha256((ROOT/n).read_bytes()).hexdigest()==digest,n
 selected=files();entries=[]
 for p in selected:entries.append({'path':str(p.relative_to(ROOT)),'bytes':p.stat().st_size,'sha256':hashlib.file_digest(p.open('rb'),'sha256').hexdigest()})
 manifest=ROOT/'reports/artifact_manifest.json';manifest.write_text(json.dumps({'revision':c['revision'],'status':'digital hardware and learning candidate; physical validation and trained v05 policies pending','files':entries},indent=2)+'\n');selected.append(manifest)
 target=ROOT/'dist'/'Sweetybot2-v05.zip';target.parent.mkdir(exist_ok=True)
 with zipfile.ZipFile(target,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
  for p in selected:z.write(p,'Sweetybot2-v05/'+str(p.relative_to(ROOT)))
 with zipfile.ZipFile(target) as z:assert z.testzip() is None
 digest=hashlib.file_digest(target.open('rb'),'sha256').hexdigest();target.with_suffix('.zip.sha256').write_text(digest+'  '+target.name+'\n')
 print(json.dumps({'archive':str(target),'bytes':target.stat().st_size,'files':len(selected),'sha256':digest},indent=2))
if __name__=='__main__':main()

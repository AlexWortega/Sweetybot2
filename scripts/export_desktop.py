"""Deliver a verified release to Desktop, preserving the previous folder."""
from pathlib import Path
from urllib.parse import urlencode
from datetime import datetime
import hashlib,json,shutil,zipfile,subprocess
from scripts.common import ROOT,load
from scripts.build_cad import NAMES

def digest(path):
 with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def main():
 revision=load()['revision'];desktop=Path.home()/'Desktop';target=desktop/'SweetieMini'
 stamp=datetime.now().strftime('%Y%m%d-%H%M%S');stage=desktop/('.SweetieMini-stage-'+stamp)
 source=ROOT/'dist'/f'sweetie-mini-{revision}.zip';archive=desktop/f'SweetieMini-{revision}.zip'
 assert source.is_file() and not stage.exists()
 stage.mkdir()
 with zipfile.ZipFile(source) as z:
  assert z.testzip() is None
  for entry in z.infolist():
   rel=Path(entry.filename).relative_to('sweetie-mini');assert '..' not in rel.parts
   dest=stage/rel;dest.parent.mkdir(parents=True,exist_ok=True)
   with z.open(entry) as src,dest.open('wb') as dst:shutil.copyfileobj(src,dst)
 viewer=subprocess.run([str(ROOT/'.venv/bin/cadgen'),'viewer','--host','127.0.0.1','--json','--detach'],text=True,capture_output=True,check=True)
 viewer_info=next(json.loads(line) for line in viewer.stdout.splitlines() if line.startswith('{') and '"url"' in line)
 lines=['# CAD gallery','','These viewer links open this Desktop delivery. Rebuild with `make render` after moving it.','','| Saved file | Viewer |','|---|---|']
 for name in ['assembly','internal','exploded']+NAMES:
  url=viewer_info['url']+'?'+urlencode({'file':str(target/'cad/STEP'/f'{name}.step')})
  lines.append(f'| [{name}.step](../cad/STEP/{name}.step) | [Open CAD]({url}) |')
 (stage/'docs/cad-gallery.md').write_text('\n'.join(lines)+'\n')
 manifest=stage/'reports/artifact_manifest.json';data=json.loads(manifest.read_text())
 for entry in data['files']:
  p=stage/entry['path']
  if entry['path']=='docs/cad-gallery.md':entry.update(bytes=p.stat().st_size,sha256=digest(p))
  assert p.stat().st_size==entry['bytes'] and digest(p)==entry['sha256'],entry['path']
 manifest.write_text(json.dumps(data,indent=2)+'\n')
 backup=None
 if target.exists():
  previous=target/'reports/artifact_manifest.json';assert previous.is_file(),'Destination is not an identified delivery; preserve it'
  old=json.loads(previous.read_text())['revision'];backup=desktop/f'SweetieMini_{old}'
  if backup.exists():backup=desktop/(backup.name+'_'+stamp)
  target.rename(backup)
 stage.rename(target)
 if archive.exists():archive.rename(desktop/(archive.stem+'_'+stamp+archive.suffix))
 paths=sorted(p for p in target.rglob('*') if p.is_file())
 temporary=archive.with_suffix('.zip.tmp')
 with zipfile.ZipFile(temporary,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
  for p in paths:z.write(p,'SweetieMini/'+str(p.relative_to(target)))
 with zipfile.ZipFile(temporary) as z:assert z.testzip() is None
 temporary.rename(archive);sha=digest(archive);archive.with_suffix('.zip.sha256').write_text(sha+'  '+archive.name+'\n')
 result=dict(folder=str(target),archive=str(archive),previous_folder=str(backup) if backup else None,files=len(paths),archive_bytes=archive.stat().st_size,sha256=sha,manifest_verified=True)
 (ROOT/'reports/desktop_delivery.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()

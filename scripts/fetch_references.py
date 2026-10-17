"""Download a small, revision-pinned reference snapshot; never execute upstream code."""
import concurrent.futures, hashlib, json, urllib.request
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
SOURCES = {
 'sweetie_bot': ('sweetie-bot-project/sweetie_bot','master',['README.md','.gitmodules','LICENSE','config/sweetie_bot_proto2_deploy/README.md']),
 'sweetie_proto2': ('sweetie-bot-project/sweetie_bot_proto2_description','master',['README.md','urdf/sweetie_bot_proto2_description_dynamics.urdf']),
 'microduck': ('pollen-robotics/microduck','main',['README.md','LICENSE','docs/design/architecture.md','docs/design/robotd-design.md']),
 'microduck_rl': ('pollen-robotics/microduck_rl','main',['README.md','LICENSE','src/mjlab_microduck/robot/microduck/config_mjcf_walk.json'])}
def fetch(key, spec):
 repo, branch, paths = spec
 def read(u): return urllib.request.urlopen(u,timeout=45).read()
 commit=json.loads(read(f'https://api.github.com/repos/{repo}/commits/{branch}'))['sha']
 files=[]
 for p in paths:
  u=f'https://raw.githubusercontent.com/{repo}/{commit}/{p}'
  try: data=read(u)
  except Exception as e: files.append({'path':p,'error':str(e)}); continue
  dst=ROOT/'references/upstream'/key/p; dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
  files.append({'path':p,'url':u,'sha256':hashlib.sha256(data).hexdigest()})
 return key, {'repository':repo,'requested_branch':branch,'commit':commit,'files':files}
if __name__=='__main__':
 with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
  result=dict(pool.map(lambda kv:fetch(*kv),SOURCES.items()))
 (ROOT/'references/upstream/manifest.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({k:v['commit'] for k,v in result.items()},indent=2))

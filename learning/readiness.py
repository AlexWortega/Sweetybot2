"""Robot training must use a mechanically accepted, matching collision model."""
import hashlib,json
from scripts.common import ROOT,load,require_mechanical_gate
PHYSICAL_FILES=('config/robot.yaml','simulation/model.py','simulation/generated/sweetie_mini.xml','reports/collision_meshes.json','learning/native.cpp','learning/sensors.py','hardware/observation.py')
DYNAMICS_FILES=PHYSICAL_FILES+('learning/env.py','learning/rewards.py','learning/curriculum.py')
LEGACY_DYNAMICS_FILES=('config/robot.yaml','simulation/model.py','simulation/generated/sweetie_mini.xml','reports/collision_meshes.json','learning/env.py','learning/native.cpp','learning/sensors.py','learning/rewards.py','hardware/observation.py')
def source_signature(root,files):
 h=hashlib.sha256()
 for name in files:h.update(name.encode()+b'\0'+(root/name).read_bytes()+b'\0')
 manifest=json.loads((root/'reports/collision_meshes.json').read_text())
 for part in manifest['parts']:
  for name in part['hulls']:h.update(name.encode()+b'\0'+(root/name).read_bytes()+b'\0')
 return h.hexdigest()
def dynamics_signature():return source_signature(ROOT,DYNAMICS_FILES)
def physical_signature(root=ROOT):return source_signature(root,PHYSICAL_FILES)
def require_training_ready():
 require_mechanical_gate()
 path=ROOT/'reports/rl_readiness.json'
 if not path.exists():raise ValueError('RL collision-model acceptance is missing')
 r=json.loads(path.read_text())
 if not r.get('passed'):raise ValueError(r.get('reason','RL physics acceptance did not pass'))
 if r.get('dynamics_signature')!=dynamics_signature():raise ValueError('RL physics acceptance is stale; revalidate the current model')
 return r

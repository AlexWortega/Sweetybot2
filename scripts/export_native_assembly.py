"""Export assemblies from the exact STEP round-trip-verified component snapshot."""
import hashlib,json,sys
from scripts.common import ROOT,load
from scripts.cad_snapshot import load_parts,geometry_signature
sys.path.insert(0,str(ROOT/'cad/src'))
from lib import shapes
from cadgen import build123d as bd

def finish(name):
 if any(x in name for x in ['display_pupil','display_highlight','display_glint','nostril','eyebrow','smile','camera_lens','depth_optic']):return 'body' if 'highlight' in name or 'glint' in name else 'eye'
 if 'display_iris' in name:return 'iris'
 if 'ear_inner' in name or 'pink_mane' in name or 'TPU_actuator' in name:return 'pink'
 if any(x in name for x in ['mane_crest','passive_tail']):return 'mane'
 if 'TPU_pad' in name:return 'hoof'
 if any(x in name for x in ['_servo','passive_bearing','ear_output','rotor_','roll_axle']):return 'servo'
 if any(x in name for x in ['cradle','strap','spacer','retainer','_M3_','rail_','chassis_deck','yoke','cheek_rear','board','collector','adapter','regulator','usb_hub','buck','sensor_tray','camera_carrier','floating_sole']):return 'frame'
 return 'body'

def main():
 parts=load_parts(require_snapshot=True)
 for children in parts.values():
  for shape in children:
   color=list(shapes.COLORS[finish(shape.label)])
   if finish(shape.label)=='iris':color=[.53,.24,.78,1.]
   shape.color=bd.Color(*color)
 # assembly_shape performs exactly the shared FK and root-height transform;
 # the verified components are reused, without rebuilding or changing geometry.
 shapes.link_components=lambda:parts
 records=[]
 for name,kw in [('assembly',{}),('internal',{'internal':True}),('exploded',{'exploded':True})]:
  shape=shapes.assembly_shape(**kw);target=ROOT/'cad/STEP'/f'{name}.step';bd.export_step(shape,target)
  expected=sum(s.volume for s in shape.solids());cold=bd.import_step(target);observed=sum(s.volume for s in cold.solids());error=abs(expected-observed)/expected
  assert error<.001,(name,error)
  record={'name':name,'step':str(target.relative_to(ROOT)),'relative_volume_change':error,'bounds_mm':[list(shape.bounding_box().min),list(shape.bounding_box().max)]}
  if name=='assembly':
   path=ROOT/'cad/GLB/assembly.glb';path.parent.mkdir(exist_ok=True);assert bd.export_gltf(shape,path,binary=True,linear_deflection=.15,angular_deflection=.25);record['glb_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
  records.append(record);print('EXPORTED',name,error,flush=True)
 report={'geometry_signature':geometry_signature(),'config_sha256':hashlib.sha256((ROOT/'config/robot.yaml').read_bytes()).hexdigest(),'cad_snapshot_sha256':hashlib.sha256((ROOT/'reports/cad_snapshot.json').read_bytes()).hexdigest(),'method':'verified placed component STEP solids, shared FK, semantic surface finishes; geometry preserved','assemblies':records}
 (ROOT/'reports/assembly_exports.json').write_text(json.dumps(report,indent=2)+'\n')
if __name__=='__main__':main()

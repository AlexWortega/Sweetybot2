"""Render saved STEP documents and a compact, labelled review sheet."""
import json,subprocess,sys,math
from urllib.parse import urlencode
from PIL import Image,ImageDraw
from scripts.common import ROOT
from scripts.build_cad import NAMES

def main():
 target=ROOT/'renders';target.mkdir(exist_ok=True)
 jobs=[]
 for name,cameras in [('assembly',['iso','front','left']),('internal',['iso']),('exploded',['iso'])]:
  jobs.append({'input':f'cad/STEP/{name}.step','display':'render','outputs':[{'path':f'renders/{name}'+('' if camera=='iso' else '_'+camera)+'.png','camera':camera,'width':1200,'height':900} for camera in cameras]})
 for name in NAMES:
  jobs.append({'input':f'cad/STEP/{name}.step','display':'solid','outputs':[{'path':f'renders/parts/{name}.png','camera':'iso','width':600,'height':450}]})
 (target/'parts').mkdir(exist_ok=True)
 packet=target/'jobs.json';packet.write_text(json.dumps({'jobs':jobs},indent=2)+'\n')
 subprocess.run([str(ROOT/'.venv/bin/cadgen'),'step','snapshot','--job',str(packet),'--json'],cwd=ROOT,check=True)
 sheet=Image.new('RGB',(1800,math.ceil(len(NAMES)/3)*490),'#f4f5f9');draw=ImageDraw.Draw(sheet)
 for i,name in enumerate(NAMES):
  im=Image.open(target/'parts'/f'{name}.png').convert('RGB');x=i%3*600;y=i//3*490;sheet.paste(im,(x,y));draw.text((x+18,y+458),name,fill='#18263b',font_size=23)
 sheet.save(target/'parts_review.jpg',quality=90)
 # Capture the viewer's announced address instead of assuming a port.
 viewer=subprocess.run([str(ROOT/'.venv/bin/cadgen'),'viewer','--host','127.0.0.1','--json','--detach'],cwd=ROOT,text=True,capture_output=True,check=True)
 info=next(json.loads(line) for line in viewer.stdout.splitlines() if line.startswith('{') and '"url"' in line)
 lines=['# CAD gallery','', 'Viewer links below refer to this checkout on this machine. Rebuild with `make render` after moving the folder.','', '| Saved file | Viewer |','|---|---|']
 for name in ['assembly','internal','exploded']+NAMES:
  path=ROOT/'cad/STEP'/f'{name}.step';url=info['url']+'?'+urlencode({'file':str(path)})
  lines.append(f'| [{name}.step](../cad/STEP/{name}.step) | [Open CAD]({url}) |')
 (ROOT/'docs/cad-gallery.md').write_text('\n'.join(lines)+'\n')
 print('Review renders/assembly.png, internal.png, exploded.png and parts_review.jpg')
if __name__=='__main__':main()

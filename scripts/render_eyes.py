"""Create a preview and two exact-resolution display frames without hardware I/O."""
from PIL import Image
from scripts.common import ROOT
from simulation.eyes import eye_frame

def main():
 frames=[]
 for i in range(80):
  frame=Image.new('RGB',(500,240),'#121020')
  for side,x in [(1,0),(-1,260)]:frame.paste(eye_frame(i/20,side),(x,0))
  frames.append(frame)
 target=ROOT/'renders';target.mkdir(exist_ok=True)
 frames[0].save(target/'eyes_demo.gif',save_all=True,append_images=frames[1:],duration=50,loop=0)
 for name,side in [('left',1),('right',-1)]:eye_frame(0,side).save(target/f'eye_{name}_240.png')
 print('Generated 4s paired-eye preview; no display driver or hardware timing claimed')
if __name__=='__main__':main()

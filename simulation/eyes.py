"""240px ST7789 eye-frame generator; no GPIO or display-bus I/O."""
import math
from PIL import Image,ImageDraw

def eye_frame(t,side=1,size=240):
 image=Image.new('RGB',(size,size),(18,16,32));d=ImageDraw.Draw(image)
 gaze=math.sin(t*.8)*15*side;gy=math.sin(t*.6)*8
 # Blink once per 4 seconds. Eyelids are rendered into the independent frame.
 phase=t%4;openness=1-.98*math.exp(-((phase-2)/.10)**2)
 cx=size/2+gaze;cy=size/2+gy;ry=83*openness
 d.ellipse((cx-67,cy-ry,cx+67,cy+ry),fill=(56,224,82))
 d.ellipse((cx-31,cy-ry*.90,cx+31,cy+ry*.90),fill=(13,12,24))
 if openness>.3:
  d.ellipse((cx+12,cy-ry*.6,cx+28,cy-ry*.6+22),fill=(244,248,255))
 return image

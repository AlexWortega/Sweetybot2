"""2D-drawing-derived Feetech envelopes and new leg load paths, millimetres.
No vendor STEP was supplied for these motors; fit and strength remain bench gates.
"""
import numpy as np
from cadgen import build123d as bd
G={}
def box(*a,**k):return G['box'](*a,**k)
def hole(*a,**k):return G['hole'](*a,**k)
def paint(*a,**k):return G['paint'](*a,**k)
# Output mounting face at y=0, long case dimension x, width z.
SPECS={'SM105BL-C001':(58.5,44.,33.5,-19.75,2.55,16.),'SM80BL-C001':(46.5,34.,28.5,-11.5,2.5,5.0)}
def motor(model):
 a,b,c,xmin,gap,r=SPECS[model]
 s=box((a,b,c),(xmin+a/2,-gap-b/2,0),radius=1.)
 s=s+hole(r,gap,(0,-gap/2,0),'y')+hole(r,gap,(0,-gap-b-gap/2,0),'y')
 return paint(s,model+'_drawing_envelope','servo')
def cradle(model='SM105BL-C001'):
 a,b,c,xmin,gap,r=SPECS[model];cx=xmin+a/2;cy=-gap-b/2
 s=box((a+8,b+5,3.2),(cx,cy,-c/2-2.4),radius=.8)
 for x in [xmin-2.4,xmin+a+2.4]:
  s=s+box((3.2,b+5,c+4.8),(x,cy,0))
  for y in [cy-b/2+5,cy+b/2-5]:s=s-hole(1.65,c+12,(x,y,0))
 # Case is clamped from outside; housing assembly screws are not used as mounts.
 for x in [cx-12,cx+12]:s=s-box((6,b-7,8),(x,cy,-c/2-2.4),radius=.6)
 return paint(s,model+'_clamp_cradle','frame')
def strap(model='SM105BL-C001'):
 a,b,c,xmin,gap,r=SPECS[model];cx=xmin+a/2
 s=box((a+8,6,3.2),(cx,0,0),radius=.8)
 for x in [xmin-2.4,xmin+a+2.4]:s=s-hole(1.65,8,(x,0,0))
 return paint(s,model+'_clamp_strap','frame')
def plate(length,label):
 s=box((38,3,length+36),(0,0,-length/2),radius=1.)
 for z in [0,-length]:
  s=s-hole(4,8,(0,0,z),'y')
  for angle in np.arange(0,360,45):
   x=13*np.cos(np.deg2rad(angle));zz=z+13*np.sin(np.deg2rad(angle))
   s=s-hole(1.65,8,(x,0,zz),'y')
 s=s-box((16,8,max(8,length-40)),(0,0,-length/2),radius=1.)
 return paint(s,label,'body')
def thigh_plate():return plate(G['C']()['geometry']['thigh'],'v04_thigh_6061_cheek')
def shin_plate():
 L=G['C']()['geometry']['shin']
 vertices=[(-19,-1.5,18),(19,-1.5,18),(19,-1.5,-L+35),(8,-1.5,-L-8),(-8,-1.5,-L-8),(-19,-1.5,-L+35)]
 s=bd.Solid.extrude(bd.Face(bd.Wire.make_polygon(vertices,close=True)),(0,3,0))
 s=s-hole(4,9,(0,0,0),'y')-hole(1.65,9,(0,0,-L),'y')
 for a in np.arange(0,360,45):s=s-hole(1.65,9,(13*np.cos(np.deg2rad(a)),0,13*np.sin(np.deg2rad(a))),'y')
 s=s-box((16,10,L-55),(0,0,-L*.45),radius=1)
 return paint(s,'v04_tapered_shin_6061_cheek','body')
def shin_rear_plate():
 # Offset proximal bearing support returns to the hoof frame distally.
 L=G['C']()['geometry']['shin'];sections=[]
 for z,y,width in [(18,-54.1,38),(-25,-54.1,38),(-70,-5,38),(-L-8,-5,16)]:
  sections.append(bd.Wire.make_rect(width,3,plane=bd.Plane(origin=(0,y,z))))
 s=bd.Solid.make_loft(sections,ruled=True)
 s=s-hole(1.65,12,(0,-5,-L),'y')-hole(4,12,(0,-54.1,0),'y')
 for a in np.arange(0,360,45):s=s-hole(1.65,12,(13*np.cos(np.deg2rad(a)),-54.1,13*np.sin(np.deg2rad(a))),'y')
 return paint(s,'v04_offset_rear_shin_6061','frame')

def roll_yoke():
 g=G['C']()['geometry'];off=g['abduction_offset'];drop=g['hip_drop']
 # Orthogonal output/pitch interfaces: roll axis X, pitch axis Y.
 # Three 3mm aluminium plates joined with angle blocks; CAD is a grouped fit reference.
 s=box((3,30,drop+24),(24.5,off/2,-drop/2))
 s=s+box((49,30,3),(1.5,off/2,-drop/2))
 s=s+box((49,3,50),(1.5,-30,-drop+8))
 # Rear support remains behind the passive output face; no plate occupies the rotating horn plane.
 s=s+box((49,45,3),(1.5,-9,-drop/2))
 s=s-hole(1.65,8,(24.5,0,0),'x')
 for a in np.arange(0,360,45):s=s-hole(1.65,12,(13*np.cos(np.deg2rad(a)),-30,-drop+13*np.sin(np.deg2rad(a))),'y')
 s=s-hole(4,12,(0,-30,-drop),'y')
 return paint(s,'v04_roll_to_pitch_6061_yoke','frame')
def axle_adapter():
 return paint(hole(7,24.5,(12.25,0,0),'x')-hole(1.65,30,(12.25,0,0),'x'),'v04_roll_output_spacer','frame')
def knee_fork():
 # Passive support outside the complete 49.1mm motor envelope.
 s=box((38,3,34),(0,-60.1,-4))+box((3,68.1,5),(17.5,-26.05,-19))
 s=s-hole(4,9,(0,-60.1,0),'y')
 for a in np.arange(0,360,45):s=s-hole(1.65,9,(13*np.cos(np.deg2rad(a)),-60.1,13*np.sin(np.deg2rad(a))),'y')
 return paint(s,'v04_knee_6061_passive_support','frame')
def body_shell():
 c=G['C']();geo=c['geometry'];x,y,z=geo['body'];w=geo['shell_wall']
 outer=G['ellipsoid'](((x+4)/2,(y+4)/2,(z+4)/2));inner=G['ellipsoid'](((x+4)/2-w,(y+4)/2-w,(z+4)/2-w))
 s=outer-inner;s=s-box((x+30,y+30,100),(0,0,-74.8))
 for sx in [-1,1]:
  for sy in [-1,1]:
   R=np.array([[0,sx,0],[-sy,0,0],[0,0,sy*sx]],float)
   pocket=box((56,44,40),(11.75,-19.5,0))
   s=s-G['rigid'](pocket,'roll_motor_service_clearance',(sx*geo['hip_x'],sy*geo['hip_y'],0),R)
 s=s-hole(30,160,(geo['head_origin'][0],0,38))
 for px in [-35,-25,-15,-5,5,15,25]:s=s-box((3.2,24,30),(px,0,44))
 for px in [-50,50]:
  for side in [-1,1]:
   s=s+box((5,32,4.3),(px,side*35,-22.65))+hole(4,4.3,(px,side*20,-22.65))
   s=s-hole(2.1,8,(px,side*20,-22.65))
 # Cosmetic scraps separated by service openings are not installed parts.
 return paint(max(s.solids(),key=lambda a:a.volume),'v04_serviceable_body_cowl','body')
def rail():
 # Notched vertical web clears the larger roll cases; the lower chord remains.
 s=G['original_rail_v04']()
 for x in [-79.5,79.5]:s=s-box((38,12,45),(x,0,17.3))
 return paint(s,'v04_notched_chassis_rail','frame')

def install(g):
 global G;G=g;g['original_rail_v04']=g['rail'];original=g['link_components'];olddepth=g['sensors_face'].depth_pod
 f={'servo_cradle_sm105':lambda:cradle(),'servo_cradle_sm80':lambda:cradle('SM80BL-C001'),'servo_strap_sm105':lambda:strap(),'servo_strap_sm80':lambda:strap('SM80BL-C001'),'thigh_plate':thigh_plate,'shin_plate':shin_plate,'shin_rear_plate':shin_rear_plate,'roll_yoke':roll_yoke,'axle_adapter':axle_adapter,'knee_fork':knee_fork,'body_shell':body_shell,'rail':rail}
 g.update({k:v for k,v in f.items() if k in g});g['FACTORIES'].update(f)
 # Chest module follows the actual configured position in the enlarged body.
 def depth_pod():return olddepth().moved(bd.Location((17,0,0)))
 g['FACTORIES']['depth_pod']=depth_pod;g['sensors_face'].depth_pod=depth_pod
 def components():
  from simulation.model import rotation
  c=g['C']();geo=c['geometry'];parts=original();rigid=g['rigid']
  def add(link,shape,label,pos=(0,0,0),R=None):parts[link].append(rigid(shape,label,pos,R))
  def addmotor(link,model,label,pos,R):
   add(link,motor(model),label+'_servo',pos,R);add(link,cradle(model),label+'_cradle',pos,R)
   a,b,w,xmin,gap,r=SPECS[model];cy=-gap-b/2
   for y in [cy-b/2+5,cy+b/2-5]:add(link,strap(model),label+'_strap_'+str(y),np.array(pos)+R@np.array([0,y,w/2+2.4]),R)
  for leg,sgn in c['legs'].items():
   sx=sgn['x_sign'];side=sgn['y_sign'];o=np.array([sx*geo['hip_x'],side*geo['hip_y'],0.])
   parts['torso']=[s for s in parts['torso'] if not s.label.startswith(leg+'_')]
   R=np.array([[0,sx,0],[-side,0,0],[0,0,side*sx]],float)
   addmotor('torso','SM80BL-C001',leg+'_roll',o,R)
   parts[leg+'_roll']=[]
   Rh=np.array([[0,0,side],[0,side,0],[-1,0,0]],float)
   addmotor(leg+'_roll','SM105BL-C001',leg+'_pitch',[0,side*geo['abduction_offset'],-geo['hip_drop']],Rh)
   yy=roll_yoke() if side==1 else bd.mirror(roll_yoke(),about=bd.Plane.XZ)
   if sx<0:yy=bd.mirror(yy,about=bd.Plane.YZ)
   add(leg+'_roll',yy,leg+'_roll_yoke')
   add(leg+'_roll',axle_adapter() if sx>0 else bd.mirror(axle_adapter(),about=bd.Plane.YZ),leg+'_roll_axle')
   parts[leg+'_thigh']=[]
   Rk=np.array([[0,0,-side],[0,side,0],[1,0,0]],float)
   addmotor(leg+'_thigh','SM105BL-C001',leg+'_knee',[0,0,-geo['thigh']],Rk)
   for y in [side*5,-side*60.1]:add(leg+'_thigh',thigh_plate(),leg+'_thigh_cheek_'+str(y),(0,y,0))
   add(leg+'_thigh',g['thigh_fairing']() if side==1 else bd.mirror(g['thigh_fairing'](),about=bd.Plane.XZ),leg+'_thigh_fairing',(0,side*9,0))
   for z in [-geo['thigh']*.37,-geo['thigh']*.52]:add(leg+'_thigh',paint(hole(3.2,62.1,axis='y')-hole(1.65,68,axis='y'),'6061_spacer','frame'),leg+'_thigh_spacer_'+str(z),(13,-side*27.55,z))
   parts[leg+'_shin']=[s for s in parts[leg+'_shin'] if not any(t in s.label for t in ['shin_cheek','knee_support_fork','shin_fairing'])]
   add(leg+'_shin',shin_plate(),leg+'_shin_cheek_front',(0,side*5,0))
   add(leg+'_shin',shin_rear_plate() if side==1 else bd.mirror(shin_rear_plate(),about=bd.Plane.XZ),leg+'_shin_cheek_rear')
   fork=knee_fork() if side==1 else bd.mirror(knee_fork(),about=bd.Plane.XZ)
   add(leg+'_shin',fork,leg+'_knee_support_fork')
   add(leg+'_shin',g['shin_fairing']() if side==1 else bd.mirror(g['shin_fairing'](),about=bd.Plane.XZ),leg+'_shin_fairing',(0,side*9,0))
  add('torso',paint(box((30,20,9)),'isolated_USB_RS485_adapter_envelope','frame'),'rs485_adapter',(-42,24,15))
  # Move board/optics together with the chest pod; placement comes from config.
  for i,s in enumerate(parts['torso']):
   if s.label in ['depth_board','depth_optic']:parts['torso'][i]=s.moved(bd.Location((17,0,0)))
  # Main roll cases are wider: cowl relief must be cut against actual cases.
  cowl=next(s for s in parts['torso'] if s.label=='body_cowl')
  for s in parts['torso']:
   if s.label.endswith('_roll_servo'):cowl=cowl-s
  cowl.label='body_cowl';parts['torso']=[cowl if s.label=='body_cowl' else s for s in parts['torso']]
  return parts
 g['link_components']=components

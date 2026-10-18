"""V05: independent rotor planes, supported rear outputs and serviceable frame.
Millimetres; drawing-derived servo cases, catalogue 6807 bearing envelopes.
"""
import numpy as np
from functools import lru_cache
from cadgen import build123d as bd
G={}
COLLISION_MODE=False
SPECS={'SM105BL-C001':(58.5,44.,33.5,-19.75,2.55,16.)}
def box(*a,**k):return G['box'](*a,**k)
def hole(*a,**k):
 s=G['hole'](*a,**k)
 # Filling fastener bores is conservative for collision, while bearing bores remain open.
 return s.moved(bd.Location((0,0,100000))) if COLLISION_MODE and a[0]<=4.05 else s
def paint(*a,**k):return G['paint'](*a,**k)
def motor(model='SM105BL-C001'):
 a,b,c,xmin,gap,r=SPECS[model]
 return paint(box((a,b,c),(xmin+a/2,-gap-b/2,0),radius=1)+hole(r,gap,(0,-gap/2,0),'y')+hole(r,gap,(0,-gap-b-gap/2,0),'y'),model+'_drawing_envelope','servo')
def cradle(model='SM105BL-C001'):
 a,b,c,xmin,gap,r=SPECS[model];cx=xmin+a/2;cy=-gap-b/2
 s=box((a+16,b+5,3.2),(cx,cy,-c/2-2.4),radius=.8)
 for x in [xmin-4.25,xmin+a+4.25]:
  s=s+box((7,b+5,c+1.6),(x,cy,0))
  for y in [cy-b/2+5,cy+b/2-5]:s=s-hole(1.65,c+20,(x,y,0))
  # Axial link attachment through external end walls, outside the case.
  for z in [-10,10]:s=s-hole(1.65,b+15,(x,cy,z),'y')
  # Machined lightening window leaves both axial screw lands intact.
  s=s-box((10,22,12),(x,cy,0))
 for y in [-.95,-48.15]:s=s+box((14,3.2,14),(29.5,y,-11))
 return paint(s,'v05_external_clamp','frame')
def strap(model='SM105BL-C001'):
 a,b,c,xmin,gap,r=SPECS[model];s=box((a+16,6,3.2),(xmin+a/2,0,0),radius=.8)
 for x in [xmin-4.25,xmin+a+4.25]:s=s-hole(1.65,10,(x,0,0))
 return paint(s,'v05_clamp_strap','frame')
def bolt_circle(s,z=0,y=0,r=13,length=15):
 for a in np.arange(0,360,45):s=s-hole(1.65,length,(r*np.cos(np.deg2rad(a)),y,z+r*np.sin(np.deg2rad(a))),'y')
 return s
def hub(length=3.5):
 s=hole(17.48,length,(0,-length/2,0),'y')-hole(4,length+4,(0,-length/2,0),'y')
 return paint(bolt_circle(s,y=-length/2,length=length+5),'v05_bolted_rotor_sleeve','frame')
def bearing():return paint(hole(23.5,7,axis='y')-hole(17.5,10,axis='y'),'6807_35x47x7','servo')
def housing(center_y,z=0):
 s=hole(30,9,(0,center_y,z),'y')-hole(17.7,15,(0,center_y,z),'y')
 s=s-hole(23.52,7,(0,center_y-1,z),'y')
 for a in [45,135,225,315]:s=s-hole(1.1,15,(27.5*np.cos(np.deg2rad(a)),center_y,z+27.5*np.sin(np.deg2rad(a))),'y')
 return s
def retainer():
 s=hole(30,1.5,axis='y')-hole(17.7,4,axis='y')
 for a in [45,135,225,315]:s=s-hole(1.1,5,(27.5*np.cos(np.deg2rad(a)),0,27.5*np.sin(np.deg2rad(a))),'y')
 for x in [-10,10]:s=s-hole(1.65,5,(x,0,24),'y')
 return paint(s,'v05_bearing_retainer','frame')
def thigh_plate():
 L=G['C']()['geometry']['thigh'];end=-L+27
 s=box((38,3,18-end),(0,0,(18+end)/2),radius=1.)
 s=s-hole(4,10,(0,0,0),'y');s=bolt_circle(s)
 for x in [-27,27]:s=s+box((18,3,14),(x,0,-L/2),radius=1.)
 for x in [-27,27]:s=s-hole(1.65,12,(x,0,-L/2),'y')
 for x in [-10,10]:s=s-hole(1.65,12,(x,0,-L+43),'y')
 return paint(s,'v05_thigh_front_clear_output','body')
def thigh_rear_plate():
 L=G['C']()['geometry']['thigh'];sections=[]
 for z,y in [(18,-65),(-20,-65),(-60,-56.5),(-L,-56.5)]:sections.append(bd.Wire.make_rect(38,3,plane=bd.Plane(origin=(0,y,z))))
 s=bd.Solid.make_loft(sections,ruled=True)+housing(-56.5,-L)
 s=s-hole(17.7,20,(0,-56.5,-L),'y')-hole(23.52,7,(0,-57.5,-L),'y')
 s=s-hole(4,16,(0,-65,0),'y');s=bolt_circle(s,y=-65)
 # Rear web's two side lugs match the front cross-spacers.
 y=-65+8.5*((-L/2+20)/(-40))
 for x in [-27,27]:s=s+box((18,3,14),(x,y,-L/2),radius=1.)-hole(1.65,16,(x,y,-L/2),'y')
 for x in [-10,10]:s=s-hole(1.65,20,(x,-58,-L+43),'y')
 return paint(s,'v05_thigh_rear_bearing_carrier','frame')
def shin_plate():
 L=G['C']()['geometry']['shin'];v=[(-19,-1.5,18),(19,-1.5,18),(19,-1.5,-L+35),(8,-1.5,-L-3),(-8,-1.5,-L-3),(-19,-1.5,-L+35)]
 s=bd.Solid.extrude(bd.Face(bd.Wire.make_polygon(v,close=True)),(0,3,0));s=s-hole(4,10,axis='y');s=bolt_circle(s)
 for x in [-6,6]:s=s-hole(1.65,10,(x,0,-L),'y')
 for x in [-5,5]:s=s-hole(1.65,10,(x,0,-40),'y')
 return paint(s,'v05_shin_rotor_cheek','body')
def shin_rear_plate():
 L=G['C']()['geometry']['shin'];sections=[]
 for z,y,w in [(18,-67,38),(-42,-67,38),(-75,-5,25),(-L-3,-5,16)]:sections.append(bd.Wire.make_rect(w,3,plane=bd.Plane(origin=(0,y,z))))
 s=bd.Solid.make_loft(sections,ruled=True);s=s-hole(4,16,(0,-67,0),'y');s=bolt_circle(s,y=-67)
 for x in [-5,5]:s=s-hole(1.65,16,(x,-67,-40),'y')
 for x in [-6,6]:s=s-hole(1.65,12,(x,-5,-L),'y')
 return paint(s,'v05_offset_shin_support','frame')
def knee_fork():
 s=box((18,75,8),(0,-28,-40))
 for x in [-5,5]:s=s-hole(1.65,90,(x,-28,-40),'y')
 # Match the sloping rear cheek seat without a buried overlapping corner.
 s=s-shin_rear_plate()
 solids=list(s.solids());main=max(solids,key=lambda a:a.volume)
 assert sum(a.volume for a in solids if a is not main)<3., 'Unexpected disconnected cross-spacer feature'
 s=main
 return paint(s,'v05_distal_knee_cross_spacer','frame')
def axle_adapter():
 # 8xM3 flange transmission, rather than an unkeyed centre-screw tube.
 s=hub(21.5).rotate(bd.Axis.Z,90)
 return paint(s,'v05_roll_flange_extension','frame')
def roll_yoke():
 g=G['C']()['geometry'];drop=g['hip_drop'];cy=G['C']()['mechanics']['hip_rear_bearing_y_roll_mm']
 s=hole(22,6,(24.5,0,0),'x')-hole(4,12,(24.5,0,0),'x')
 for a in np.arange(0,360,45):s=s-hole(1.65,15,(24.5,13*np.cos(np.deg2rad(a)),13*np.sin(np.deg2rad(a))),'x')
 s=s+box((8,16,drop+10),(31.25,0,-drop/2))+box((8,abs(cy)+8,8),(31.25,cy/2,-drop))
 s=s+housing(cy,-drop)
 for a in np.arange(0,360,45):s=s-hole(3.1,12,(33.45,13*np.cos(np.deg2rad(a)),13*np.sin(np.deg2rad(a))),'x')
 # Two long M3 tie bolts attach the pitch stator clamp to the carrier.
 for x in [-10,10]:s=s-hole(1.65,20,(x,cy,-drop+24),'y')
 return paint(s,'v05_supported_roll_to_pitch_yoke','frame')
def fairing(length):
 s=box((42,6,length-70),(0,3,-length/2),radius=1.)-box((38,7,length-74),(0,.5,-length/2),radius=.8)
 return paint(s,'v05_leg_cover','body')
def thigh_fairing():return fairing(G['C']()['geometry']['thigh'])
def shin_fairing():return fairing(G['C']()['geometry']['shin'])
def deck():
 c=G['C']();width=c['geometry']['body'][1];s=box((134,width,3.2),radius=1.)+box((240,44,3.2),radius=1.)
 for x in [-57.45,57.45]:
  for sy in [-1,1]:
   s=s+box((16,38,3.2),(x,sy*(width/2+17),0),radius=1.)
   for y in [sy*27,sy*94]:
    s=s+hole(4.5,4.05,(x,y,3.625));s=s-hole(1.65,20,(x,y,2))
 for x in [-37,37]:
  for y in [-37,37]:s=s-hole(1.65,12,(x,y,0))
 for x in [-50,50]:
  for y in [-20,20]:s=s-hole(1.65,12,(x,y,0))
 for x in [-45,45]:
  for y in [-28,28]:s=s-hole(1.65,12,(x,y,0))
 for x in [-40,20]:
  for y in [-20,20]:s=s-box((3.2,18,12),(x,y,0))
 return paint(s,'v05_spine_and_servo_mount_deck','frame')
def rail():
 s=box((90,3.2,30))
 for x in [-37,37]:s=s+box((14,14,3.2),(x,-5,-13.4))-hole(1.65,14,(x,-6,-13))
 # Leave the battery's full 82mm width free, including the low mounting tabs.
 s=s-box((100,20,40),(0,-12.02,3))
 return paint(s,'v05_short_chassis_rail','frame')
def body_shell():
 g=G['C']()['geometry'];x,y,z=g['body'];w=g['shell_wall'];ell=G['ellipsoid']
 s=ell(((x+4)/2,(y+4)/2,(z+4)/2))
 if not COLLISION_MODE:s=s-ell(((x+4)/2-w,(y+4)/2-w,(z+4)/2-w))
 s=s-box((x+40,y+40,100),(0,0,-74.8))
 for sx in [-1,1]:
  for sy in [-1,1]:
   s=s-box((92,112,120),(sx*g['hip_x'],sy*g['hip_y'],-40))
   R=np.array([[0,sx,0],[-sy,0,0],[0,0,sy*sx]],float)
   s=s-G['rigid'](box((78,53,49),(9.5,-24.55,0)),'roll_fixed_keepout',(sx*g['hip_x'],sy*g['hip_y'],0),R)
 s=s-hole(38,160,(g['head_origin'][0],0,38))
 for px in [-35,-25,-15,-5,5,15,25]:s=s-box((3.2,24,30),(px,0,44))
 for px in [-50,50]:
  for side in [-1,1]:s=s+box((5,40,4.3),(px,side*39,-22.65))+hole(4,4.3,(px,side*20,-22.65))-hole(2.1,8,(px,side*20,-22.65))
 for x in [-45,45]:
  for y in [-28,28]:s=s-G['hole'](3.8,70,(x,y,0))
 # Apply the motor pockets after adding the cowl ribs as well.
 for sx in [-1,1]:
  for sy in [-1,1]:
   R=np.array([[0,sx,0],[-sy,0,0],[0,0,sy*sx]],float)
   s=s-G['rigid'](box((78,53,49),(9.5,-24.55,0)),'final_roll_keepout',(sx*g['hip_x'],sy*g['hip_y'],0),R)
 return paint(max(s.solids(),key=lambda a:a.volume),'v05_cowl_with_motion_windows','body')

def neck_collar():
 sections=[(4,34,34),(40,34,34)]
 def loft(w):return bd.Solid.make_loft([bd.Wire.make_ellipse(rx-w,ry-w,plane=bd.Plane(origin=(0,0,z))) for z,rx,ry in sections])
 s=loft(0) if COLLISION_MODE else loft(0)-loft(1.8)
 from simulation.model import rotation
 Rp=np.array([[0,0,1],[0,1,0],[-1,0,0]],float)
 s=s-G['rigid'](box((53,43,34),(12.5,-20,0)),'pitch_case_pocket',(0,18.7,G['C']()['geometry']['neck_pitch_rise']),Rp)
 return paint(s,'v05_clear_neck_collar','body')
def head_yoke():
 s=box((48,4,42),(14,40,13))+box((48,4,42),(14,-40,13))+box((8,84,4),(34,0,31))
 s=s-hole(1.65,100,(0,0,0),'y')
 for y in [-20,20]:s=s-hole(1.65,12,(34,y,31))
 return paint(s,'v05_head_output_yoke','frame')
def ear():
 # Affine-sphere boolean trims changed complement on STEP round-trip. Native
 # elliptical loft surfaces preserve the cut and its connected root exactly.
 def ell(r,p):
  return bd.Solid.make_loft([bd.Wire.make_ellipse(r[0]*np.sin(t),r[1]*np.sin(t),plane=bd.Plane(origin=(p[0],p[1],p[2]-r[2]*np.cos(t)))) for t in np.linspace(.01,np.pi-.01,17)])
 s=ell((8,13,24),(0,16,33))-ell((5,9,18),(5,16,35))
 stem=bd.Solid.make_loft([bd.Wire.make_ellipse(5,4,plane=bd.Plane(origin=(0,0,0))),bd.Wire.make_ellipse(5,4,plane=bd.Plane(origin=(0,16,18)))])
 if COLLISION_MODE:s=ell((8,13,24),(0,16,33))
 return paint(s+stem+hole(6,12,axis='x')-hole(3.7,20,axis='x'),'v05_offset_ear_shell','body')
@lru_cache(None)
def neck_sweep():
 from simulation.model import rotation
 solids=[]
 for q in np.linspace(-.35,.35,15):solids.append(G['rigid'](box((46,50,74),(.55,-1.3,-12.5)),'fixed_neck_clearance',(0,0,0),rotation([0,1,0],-q)))
 return bd.Compound(children=solids)
@lru_cache(None)
def ear_clearance():
 from simulation.model import rotation
 solids=[]
 for side in [-1,1]:
  for q in np.linspace(-.25,.25,9):
   shape=ear() if side>0 else bd.mirror(ear(),about=bd.Plane.XZ)
   # A 3mm radial safety envelope around the narrow stem and root.
   stem=bd.Solid.make_loft([bd.Wire.make_ellipse(8,7,plane=bd.Plane(origin=(0,-side*8/3,-3))),bd.Wire.make_ellipse(8,7,plane=bd.Plane(origin=(0,side*56/3,21)))])
   shape=shape+stem+hole(9,18,axis='x')+G['ellipsoid']((11,16,27),(0,side*16,33))
   solids.append(G['rigid'](shape,'ear_motion_clearance',(45,side*12,102),rotation([1,0,0],q)))
 return bd.Compound(children=solids)
def head_shell():
 sf=G['sensors_face']
 if COLLISION_MODE:return paint(sf.head_outer().moved(bd.Location((0,0,24)))-neck_sweep()-ear_clearance(),'v05_head_outer_collision','body')
 old=G['old_head_shell_v05']()
 cut=box((36,65,2.4),(20,0,46.5))
 for py in [-20,20]:cut=cut+G['hole'](3,29,(20,py,32))
 for py in [-29.5,-14.5,14.5,29.5]:cut=cut+box((33,2,14),(20,py,54.5))
 s=(old-(cut & sf.head_inner())).moved(bd.Location((0,0,24)))
 # New ear saddle above the display modules, attached to the hatch bosses.
 base=box((36,48,2.4),(20,0,37.2))+box((36,44,2.4),(20,0,89.2))
 for x in [8,32]:
  for y in [-12,12]:base=base+G['hole'](3,49.6,(x,y,63.2))
 for y in [-19.8,-4.2,4.2,19.8]:base=base+box((33,2,14),(20,y,102))
 s=s+base
 for y in [-20,20]:
  for x in [20,34]:s=s-G['hole'](1.65,14,(x,y,37.2))
 s=s-neck_sweep()-ear_clearance()
 for side in [-1,1]:
  s=s-G['rigid'](G['sensors_face'].eye_bezel(),'bezel_seat',(53,side*42,68),G['sensors_face'].eye_rotation(side))
  s=s-box((29.1,12.7,23.8),(20,side*12,102))
 return paint(max(s.solids(),key=lambda p:p.volume),'v05_head_shell_with_swept_clearances','body')
def head_hatch():
 s=G['old_head_hatch_v05']().moved(bd.Location((0,0,24)))-neck_sweep()
 for y in [-20,20]:s=s-G['hole'](1.65,14,(34,y,34))
 return paint(s,'v05_clear_head_hatch','body')

def sensor_tray():
 # Narrow tray and rear pillars clear the neck clamp and cowl mounting bosses.
 s=box((100,62,2.4),(-2,0,24))
 for x in [-45,45]:
  for y in [-28,28]:s=s+G['hole'](3.5,47.6,(x,y,-1.0))-hole(1.65,60,(x,y,0))
 for x in [-.16,20.16]:
  for y in [-10.16,10.16]:s=s+G['hole'](2.8,3,(x,y,26.7))-hole(1.1,10,(x,y,26))
 for x in [-39,-21]:s=s-box((2.2,20,8),(x,0,24))
 return paint(s,'v05_neck_clear_sensor_tray','frame')

def hoof():
 # A closed carrier ring, four guide seats and a separate floating sole.
 s=box((34,34,24),(0,0,-4),radius=2)-box((30,30,24),(0,0,0),radius=1)
 s=s+box((30,23,6),(0,0,0))
 for x in [-6,6]:s=s-hole(1.65,45,(x,0,0),'y')
 # Switch PCB is edge supported at its bottom, not buried in a shelf.
 s=s+box((14,3,1),(0,-13.5,-11.4))
 for x in [-5.5,5.5]:s=s+box((2,12,1),(x,-8.5,-11.4))
 s=s-box((12.4,12.4,1.8),(0,-8.5,-10.0))-box((6.6,6.6,2.4),(0,-8.5,-11.9))
 s=s-G['hole'](3.25,7,(0,-8.5,-15))
 for x in [-10.5,10.5]:
  for y in [-10.5,10.5]:s=s-hole(1.2,9,(x,y,-13.5))-hole(1.8,3.5,(x,y,-14.75))
 from simulation.model import rotation,home
 c=G['C']();q=home(c);angle=q['fl_hip_pitch']+q['fl_knee'];R=rotation([0,1,0],angle)
 for y in [-11,-5,5,11]:s=s-G['rigid'](box((28,3.4,18),(0,0,4.5)),'ankle_cheek_seat',(0,y,0),R)
 for x in [-6,6]:s=s-G['rigid'](hole(1.65,45,(x,0,0),'y'),'ankle_bolt',(0,0,0),R)
 return paint(s,'v05_contact_hoof_carrier','body')

def servo_cradle():
 # Purchased STS case includes side tabs; external clamps must clear those too.
 s=box((60.721,43,3.2),(12.5,-20,-14.7605))
 for x in [-14.3605,39.3605]:
  s=s+box((7,43,29.521),(x,-20,-1.6))
  for y in [-7,-33]:s=s-hole(1.65,55,(x,y,0))
 return paint(s,'v05_STS_external_cradle','frame')

def servo_strap():
 s=box((60.721,7,3.2),(12.5,0,0))
 for x in [-14.3605,39.3605]:s=s-hole(1.65,12,(x,0,0))
 return paint(s,'v05_STS_clamp_strap','frame')

def neck_servo(rotation=None):
 # Upstream STEP is a split surface assembly: its boolean intersection can
 # extend outside its own bounding box. Use its measured enclosing case as a
 # closed bought-part keepout, retaining the vendor file only as a reference.
 s=box((45.221,37.401,24.721),(12.5,-18.7005,0))
 if rotation is not None:s=G['rigid'](s,'STS3250_case_envelope',(0,0,0),rotation)
 return paint(s,'STS3250_closed_case_keepout','servo')

def flange_heads(axis='y',center=(0,0,0),r=13,radius=3.,height=3.):
 cs=[]
 for a in np.arange(0,360,45):
  u,v=r*np.cos(np.deg2rad(a)),r*np.sin(np.deg2rad(a));p=np.array(center)+np.array([u,0,v] if axis=='y' else [0,u,v])
  cs.append(G['hole'](radius,height,p,axis))
 return paint(bd.Compound(children=cs),'M3_socket_head_envelopes','frame')

def clamp_shims():
 cs=[box((58.5,44,.8),(9.5,-24.55,-17.15))]
 for x in [-20.125,39.125]:cs.append(box((.75,40,32),(x,-24.55,0)))
 for y in [-41.55,-7.55]:cs.append(box((58.5,6,.8),(9.5,y,17.15)))
 return paint(bd.Compound(children=cs),'gauged_075_080mm_rigid_shim_stock','frame')

def neck_shims():
 cs=[box((45.221,37.401,.8),(12.5,-18.7005,-12.7605))]
 for x in [-10.4855,35.4855]:cs.append(box((.75,34,23),(x,-18.7005,0)))
 for y in [-7,-33]:cs.append(box((45.221,7,.8),(12.5,y,12.7605)))
 return paint(bd.Compound(children=cs),'STS_rigid_shim_stock','frame')

def install(g):
 global G;G=g;original=g['link_components'];olddepth=g['sensors_face'].depth_pod
 g['old_head_shell_v05']=g['head_shell'];g['old_head_hatch_v05']=g['head_hatch'];g['old_hoof_v05']=g['hoof']
 f={'servo_cradle_sm105':cradle,'servo_strap_sm105':strap,'thigh_plate':thigh_plate,'thigh_rear_plate':thigh_rear_plate,'shin_plate':shin_plate,'shin_rear_plate':shin_rear_plate,'roll_yoke':roll_yoke,'axle_adapter':axle_adapter,'knee_fork':knee_fork,'body_shell':body_shell,'neck_collar':neck_collar,'head_yoke':head_yoke,'head_shell':head_shell,'head_hatch':head_hatch,'ear':ear,'rail':rail,'deck':deck,'thigh_fairing':thigh_fairing,'shin_fairing':shin_fairing,'bearing_retainer':retainer,'rotor_spacer':lambda:hub(3.5),'rear_hip_hub':lambda:hub(14.4),'rear_knee_hub':lambda:hub(16.4),'knee_front_hub':lambda:hub(9.5)}
 g.update({k:v for k,v in f.items() if k in g});g['FACTORIES'].update(f)
 g.update(hoof=hoof,servo_cradle=servo_cradle,servo_strap=servo_strap,servo=neck_servo)
 g['FACTORIES'].update(hoof=hoof,sensor_tray=sensor_tray,servo_cradle=servo_cradle,servo_strap=servo_strap)
 g['sensors_face'].sensor_tray=sensor_tray
 def depth_pod():return olddepth().moved(bd.Location((17,0,0)))
 g['FACTORIES']['depth_pod']=depth_pod;g['sensors_face'].depth_pod=depth_pod
 def components():
  c=g['C']();geo=c['geometry'];parts=original();rigid=g['rigid']
  for lk in ['torso','neck_yaw']:
   for i,ss in enumerate(parts[lk]):
    if ss.label.startswith('head_') and '_cradle_strap_' in ss.label:
     n=ss.label;R0=np.array([[-1,0,0],[0,0,1],[0,1,0]],float) if lk=='torso' else np.array([[0,0,1],[0,1,0],[-1,0,0]],float);ss=ss.moved(bd.Location(tuple(R0@np.array([0,0,-2.3395]))));ss.label=n;parts[lk][i]=ss
  parts['torso']=[s for s in parts['torso'] if s.label!='adapter_envelope']
  parts['torso'].append(rigid(paint(box((42,33,16)),'Waveshare_Bus_Servo_Adapter_A_keepout','frame'),'adapter_envelope',(-66,0,-4)))
  def add(link,shape,label,pos=(0,0,0),R=None):parts[link].append(rigid(shape,label,pos,R))
  add('torso',neck_shims(),'head_yaw_clamp_shims',geo['head_origin'],np.array([[-1,0,0],[0,0,1],[0,1,0]],float))
  add('neck_yaw',neck_shims(),'head_pitch_clamp_shims',(0,18.7,geo['neck_pitch_rise']),np.array([[0,0,1],[0,1,0],[-1,0,0]],float))
  def mirror(s,side):return s if side==1 else bd.mirror(s,about=bd.Plane.XZ)
  def addmotor(link,label,pos,R):
   add(link,motor(),label+'_servo',pos,R);add(link,cradle(),label+'_cradle',pos,R);add(link,clamp_shims(),label+'_clamp_shims',pos,R)
   for y in [-41.55,-7.55]:add(link,strap(),label+'_strap_'+str(y),np.array(pos)+R@np.array([0,y,19.15]),R)
  for leg,sgn in c['legs'].items():
   sx=sgn['x_sign'];side=sgn['y_sign'];parts['torso']=[s for s in parts['torso'] if not s.label.startswith(leg+'_')]
   R=np.array([[0,sx,0],[-side,0,0],[0,0,side*sx]],float);addmotor('torso',leg+'_roll',[sx*geo['hip_x'],side*geo['hip_y'],0],R)
   parts[leg+'_roll']=[];Rh=np.array([[0,0,side],[0,side,0],[-1,0,0]],float)
   addmotor(leg+'_roll',leg+'_pitch',[0,side*geo['abduction_offset'],-geo['hip_drop']],Rh)
   yy=roll_yoke()
   if sx<0 and side<0:yy=yy.rotate(bd.Axis.Z,180)
   elif side<0:yy=bd.mirror(yy,about=bd.Plane.XZ)
   elif sx<0:yy=bd.mirror(yy,about=bd.Plane.YZ)
   add(leg+'_roll',yy,leg+'_roll_yoke');add(leg+'_roll',flange_heads('x',(sx*29.,0,0)),leg+'_roll_M3_heads');ax=axle_adapter() if sx>0 else bd.mirror(axle_adapter(),about=bd.Plane.YZ);add(leg+'_roll',ax,leg+'_roll_axle')
   cy=c['mechanics']['hip_rear_bearing_y_roll_mm'];add(leg+'_roll',bearing(),leg+'_hip_passive_bearing',(0,side*(cy-1),-geo['hip_drop']))
   add(leg+'_roll',retainer(),leg+'_hip_bearing_retainer',(0,side*(cy-5.25),-geo['hip_drop']))
   for x in [-10,10]:
    pad=G['hole'](3.2,.95,axis='y')-hole(1.65,5,axis='y')
    add(leg+'_roll',pad,leg+'_pitch_mount_spacer_'+str(x),(x,-side*19.525,-geo['hip_drop']+24))
   parts[leg+'_thigh']=[];Rk=np.array([[0,0,-side],[0,side,0],[1,0,0]],float);addmotor(leg+'_thigh',leg+'_knee',[0,0,-geo['thigh']],Rk)
   add(leg+'_thigh',thigh_plate(),leg+'_thigh_cheek_front',(0,side*5,0));add(leg+'_thigh',mirror(thigh_rear_plate(),side),leg+'_thigh_cheek_rear')
   add(leg+'_thigh',mirror(hub(3.5),side),leg+'_hip_front_rotor_spacer',(0,side*3.5,0))
   # Rear hip sleeve starts at the motor rear output and points to its rotating plate.
   add(leg+'_thigh',mirror(hub(14.4),side),leg+'_hip_rear_rotor_hub',(0,-side*49.1,0))
   add(leg+'_thigh',bearing(),leg+'_knee_passive_bearing',(0,-side*57.5,-geo['thigh']));add(leg+'_thigh',retainer(),leg+'_knee_bearing_retainer',(0,-side*61.75,-geo['thigh']))
   # Side cross-spacers lie outside both motor sweeps.
   rear=-65+8.5*((-geo['thigh']/2+20)/(-40));L=5-rear-3
   for x in [-27,27]:add(leg+'_thigh',paint(G['hole'](3.2,L,axis='y')-hole(1.65,L+5,axis='y'),'6061_spacer','frame'),leg+'_thigh_spacer_'+str(x),(x,side*(5+rear)/2,-geo['thigh']/2))
   for x in [-10,10]:
    for y,length in [(1.725,3.55),(-52.025,5.95)]:
     pad=G['hole'](3.2,length,axis='y')-hole(1.65,length+4,axis='y')
     add(leg+'_thigh',pad,leg+'_knee_mount_spacer_'+str(x)+'_'+str(y),(x,side*y,-geo['thigh']+43))
   for y in [side*8.,-side*68.]:add(leg+'_thigh',flange_heads(center=(0,y,0)),leg+'_hip_M3_heads_'+str(y))
   for y in [side*14.,-side*70.]:add(leg+'_shin',flange_heads(center=(0,y,0)),leg+'_knee_M3_heads_'+str(y))
   add(leg+'_thigh',mirror(thigh_fairing(),side),leg+'_thigh_fairing',(0,side*9,0))
   parts[leg+'_shin']=[s for s in parts[leg+'_shin'] if not any(t in s.label for t in ['shin_cheek','knee_support_fork','shin_fairing'])]
   add(leg+'_shin',shin_plate(),leg+'_shin_cheek_front',(0,side*11,0));add(leg+'_shin',mirror(shin_rear_plate(),side),leg+'_shin_cheek_rear');add(leg+'_shin',mirror(knee_fork(),side),leg+'_knee_cross_spacer')
   add(leg+'_shin',mirror(hub(9.5),side),leg+'_knee_front_rotor_hub',(0,side*9.5,0));add(leg+'_shin',mirror(hub(16.4),side),leg+'_knee_rear_rotor_hub',(0,-side*49.1,0))
   add(leg+'_shin',mirror(shin_fairing(),side),leg+'_shin_fairing',(0,side*17,0))
  add('torso',paint(box((30,22,14)),'USB_RS485_module_allowance','frame'),'rs485_adapter',(0,-38,14))
  add('torso',paint(box((24,20.1,5)),'Adafruit_CH334F_5999_envelope','frame'),'usb_hub',(20,45,15))
  newhead=[]
  for s in parts['head']:
   n=s.label
   if n.startswith('ear_output_adapter_'):
    side=int(n.rsplit('_',1)[1]);earlink='ear_left' if side>0 else 'ear_right'
    # Coupler belongs to the moving rotor, not the servo stator.
    parts[earlink].append(rigid(s,n,(-45,-side*22,-60)))
    continue
   if n.startswith('micro_ear_servo_'):
    side=int(n.rsplit('_',1)[1]);s=s.moved(bd.Location((0,-side*10,42)))
   elif n not in ['pony_head_shell','head_service_hatch','head_support_yoke']:s=s.moved(bd.Location((0,0,24)))
   if n in ['mane_crest','pink_mane_stripe']:s=s.moved(bd.Location((-20,0,0)))
   s.label=n;newhead.append(s)
  parts['head']=newhead
  # Separate bought lens/barrel and surface-seated cosmetic parts from their
  # supporting body. These reliefs are part of the exported assembly geometry.
  for link in ['torso','head']:
   by={s.label:s for s in parts[link]}
   for i,s in enumerate(parts[link]):
    n=s.label;cut=None
    if n=='passive_tail':cut=by['chassis_deck']+by['body_cowl']
    if n in ['mane_crest','pink_mane_stripe','unicorn_horn']:
     cut=by['pony_head_shell']
     for side in [-1,1]:cut=cut+by['micro_ear_servo_'+str(side)]
    if n=='camera_board':cut=by['camera_lens']
    if cut is not None:
     original_color=s.color
     s=s-cut
     # These are removable surface cosmetics; omit offcuts inside the shell.
     if n in ['mane_crest','pink_mane_stripe','unicorn_horn']:s=max(s.solids(),key=lambda a:a.volume)
     s.label=n;s.color=original_color;parts[link][i]=s
  # Interior ear graphics follow the new lobe; they are appearance only.
  for link,side in [('ear_left',1),('ear_right',-1)]:
   parts[link]=[ss for ss in parts[link] if not ss.label.startswith('ear_inner_')]
   add(link,paint(g['ellipsoid']((.25,8,16)),'ear_inner','pink'),'ear_inner_'+str(side),(.35,side*16,35))
  for i,s in enumerate(parts['torso']):
   if s.label in ['depth_board','depth_optic']:parts['torso'][i]=s.moved(bd.Location((17,0,0)))
  # Hind legs fold rearward. Build their handed geometry from the front-left
  # templates so mirrored solids are never reflected twice by OCC.
  for leg in ['rl','rr']:
   for segment in ['roll','thigh','shin']:
    rear=[]
    for src in parts['fl_'+segment]:
     n=src.label;foot=any(t in n for t in ['_hoof','_floating_sole','_TPU_','_switch'])
     stator=any(t in n for t in ['_pitch_servo','_pitch_cradle','_pitch_strap','_pitch_clamp_shims','_knee_servo','_knee_cradle','_knee_strap','_knee_clamp_shims'])
     if leg=='rl' or foot:s=bd.mirror(src,about=bd.Plane.YZ)
     elif stator:s=bd.mirror(src,about=bd.Plane.XZ)
     else:s=src.rotate(bd.Axis.Z,180)
     s.label=leg+n[2:];s.color=src.color;rear.append(s)
    parts[leg+'_'+segment]=rear
  return parts
 g['link_components']=components

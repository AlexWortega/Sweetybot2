"""v03 native CAD candidate. Purchased parts are conservative envelopes unless noted.
All coordinates mm; this module is installed only in the isolated v03 workspace.
"""
from functools import lru_cache
import numpy as np
from cadgen import build123d as bd
_G={}
def box(*a,**k):return _G['box'](*a,**k)
def hole(*a,**k):return _G['hole'](*a,**k)
def ellipsoid(*a,**k):return _G['ellipsoid'](*a,**k)
def paint(*a,**k):return _G['paint'](*a,**k)
SECTIONS=[(9,29,45,31),(12,29,46,32),(18,31,53,36),(28,32,57,42),(42,29,59,48),(58,25,55,48),(74,25,47,40),(88,23,28,28),(96,20,8,10),(98,20,1.2,1.2)]
@lru_cache(None)
def head_outer():
 return bd.Solid.make_loft([bd.Wire.make_ellipse(rx,ry,plane=bd.Plane(origin=(cx,0,z))) for z,cx,rx,ry in SECTIONS])
def eye_rotation(side):
 from simulation.model import rotation
 return rotation([0,0,1],np.deg2rad(-15 if side==1 else 195))
@lru_cache(None)
def head_inner():
 sections=[(8,29,43,29)]+[(z,cx,rx-2.2,ry-2.2) for z,cx,rx,ry in SECTIONS[1:-1]]+[(96.2,20,.7,.7)]
 return bd.Solid.make_loft([bd.Wire.make_ellipse(rx,ry,plane=bd.Plane(origin=(cx,0,z))) for z,cx,rx,ry in sections])
def flat_ellipse(rx,rz,thickness):
 wire=bd.Wire.make_ellipse(rx,rz,plane=bd.Plane(origin=(0,0,0),x_dir=(1,0,0),z_dir=(0,1,0)))
 return bd.Solid.extrude(bd.Face(wire),(0,thickness,0))
def head_shell():
 outer=head_outer();inner=head_inner()
 s=(outer-inner)-box((180,160,100),(30,0,-38))
 for side in [-1,1]:
  R=eye_rotation(side)
  s=s-_G['rigid'](hole(16.8,45,axis='y'),'eye_aperture',(53,side*42,44),R)
  s=s-hole(6.2,40,(45,side*26,82))
  s=s-ellipsoid((8.3,13.3,24.3),(45,side*28,103))
 for px in [20,60]:
  for py in [-20,20]:
   s=s+(box((8,45,8),(px,py+(15 if py>0 else -15),16)) & outer)+hole(4,8,(px,py,16))
   s=s-hole(1.65,20,(px,py,15))
 # Compact ear saddle, rigidly supported from the service-hatch bosses.
 s=s+box((36,65,2.4),(20,0,46.5))
 for py in [-20,20]:s=s+hole(3,29,(20,py,32))
 for py in [-29.5,-14.5,14.5,29.5]:
  wall=box((33,2.0,14),(20,py,54.5))
  for px in [9,30]:wall=wall-box((3,6,5),(px,py,56))
  s=s+wall
 # Camera looks through forehead; exposed lens front avoids a deep tunnel.
 s=s-hole(10,42,(83,0,67),'x')
 # Recess the complete module boards, not just their visible round screens.
 for side in [-1,1]:
  R=eye_rotation(side);ep=np.array([53.,side*42.,44.])+R@np.array([0.,-14.5,0.])
  s=s-_G['rigid'](box((40.8,4.8,38.0)),'LCD_board_clearance',ep,R)
 s=s-box((2.4,25.6,24.6),(65.6,0,64.5))
 # Short engraved smile follows the actual loft surface.
 for y in np.linspace(-8,8,49):
  z=20+3*(y/8)**2
  hits=outer.find_intersection_points(bd.Axis((140,float(y),float(z)),(-1,0,0)))
  if hits:
   x=max(p.X for p,n in hits)
   s=s-bd.Solid.make_sphere(.45).moved(bd.Location((x-.15,float(y),float(z))))
 # Horn fastener boss meets the forehead from inside.
 s=s+hole(4,10,(40,0,87))-hole(1.65,24,(40,0,91))
 # Exact mating relief for the removable camera carrier; tune print allowance on coupon.
 s=s-camera_carrier()
 for side in [-1,1]:
  s=s-_G['rigid'](hole(16.8,45,axis='y'),'final_eye_clearance',(53,side*42,44),eye_rotation(side))
 solids=list(s.solids())
 largest=max(solids,key=lambda a:a.volume)
 assert sum(a.volume for a in solids if a is not largest)<1.0, 'Disconnected structural feature'
 return paint(largest,'v03_smooth_head_shell','body')
def head_hatch():
 s=head_outer() & box((180,130,3),(37,0,10.5))
 s=s-box((100,58,10),(-34,0,10.5))
 for px in [20,60]:
  for py in [-20,20]:s=s-hole(1.65,10,(px,py,10.5))
 return paint(s,'v03_head_service_hatch','body')
def horn():
 s=bd.Cone(6.5,.7,22).moved(bd.Location((40,0,101)))
 return paint(s-hole(1.65,13,(40,0,94)),'short_horn','body')
def camera_carrier():
 s=box((12,46,2.4),(64,0,21.2))
 for side in [-1,1]:s=s+hole(2.5,45.6,(64,side*16,43.6))
 rim=box((3,30,30),(64,0,64.5))-box((8,25.6,24.6),(64,0,64.5))
 s=s+rim
 for side in [-1,1]:s=s-hole(1.65,12,(60,side*20,21.2))
 # Edge capture rather than an invented camera screw-hole pattern.
 for z in [51.5,77.5]:s=s+box((4,28,1.6),(65,0,z))
 return paint(s & head_inner(),'camera_edge_clip_carrier','frame')
def sensor_tray():
 s=box((108,48,2.4),(0,0,24))
 for px in [-50,50]:
  for py in [-20,20]:
   s=s+hole(3.5,48.8,(px,py,-.8))
   s=s-hole(1.65,60,(px,py,0))
 for px in [-.16,20.16]:
  for py in [-10.16,10.16]:
   s=s+hole(2.8,3.0,(px,py,26.7))-hole(1.1,10,(px,py,26))
 for px in [-39,-21]:s=s-box((2.2,20,8),(px,0,24))
 for py in [-18,18]:s=s-box((47,3,8),(-2,py,24),radius=1)
 return paint(s,'rigid_imu_sensor_tray','frame')
def depth_pod():
 s=box((17,35,35),(110,0,-7),radius=3)
 s=s-box((16,30,30),(107,0,-7))
 s=s-box((25,22,16),(117,0,-10))
 s=s+box((16,48,3),(100,0,-23.3))
 for py in [-20,20]:s=s-hole(1.65,15,(99,py,-23))
 # Four M2 mounting pillars for a square 25.4mm SparkFun board; pitch 20.32.
 # Board and rails share a 20 degree downward tilt.
 from simulation.model import rotation
 R=rotation([0,1,0],np.deg2rad(20));axis=R@np.array([1.,0,0])
 for py in [-10.16,10.16]:
  for z in [-10.16,10.16]:
   start=np.array([111.,0,-7])+R@np.array([1.,py,z])
   length=(117.5-start[0])/axis[0]
   plane=bd.Plane(origin=tuple(start),z_dir=tuple(axis))
   s=s+bd.Solid.make_cylinder(2.5,length,plane)
   s=s-bd.Solid.make_cylinder(1.1,length+5,plane)
 s=s-_G['rigid'](box((2.4,26,26)),'ToF_board_clearance',(111,0,-7),R)
 return paint(s,'chest_depth_pod','body')
def hoof():
 s=box((34,34,24),(0,0,-4),radius=4)-box((30,30,22),(0,0,-.5),radius=3)
 # Axial mount ends above the contact cassette rather than crossing its PCB.
 s=s+box((30,6,4),(0,0,-1))+hole(4,12,(0,0,2))
 s=s-hole(1.65,18,(0,0,3))-hole(3.1,6,(0,0,6))
 s=s-hole(2.1,10,(0,-8.5,-14))
 # Four captive guide screws and spring seats; no load passes through switch.
 for x in [-10.5,10.5]:
  for y in [-10.5,10.5]:s=s-hole(1.2,9,(x,y,-13.5))-hole(1.8,3.5,(x,y,-14.75))
 # PCB shelf sits against the rear inside wall, with a serviceable tie slot.
 s=s+box((14,3,3),(0,-13.5,-10))
 s=s-box((5,3,3),(0,-13.5,-10))
 for x in [-5.5,5.5]:s=s+box((2,12,1.0),(x,-8.5,-11.4))
 return paint(s,'contact_hoof_carrier','body')
def hoof_sole_plate():
 s=box((30,30,1.8),(0,0,-17.7),radius=.7)
 for x in [-10.5,10.5]:
  for y in [-10.5,10.5]:s=s-hole(1.2,6,(x,y,-17.7))
 s=s-hole(1.1,6,(0,-8.5,-17.7))
 return paint(s,'floating_rigid_sole_plate','frame')
def hoof_tpu_pad():
 s=box((30,30,2.4),(0,0,-19.8),radius=.8)
 for y in [-8,0,8]:s=s-box((24,1.3,.7),(0,y,-20.8),radius=.2)
 return paint(s,'replaceable_TPU_tread','hoof')
def hoof_actuator():
 s=hole(1.7,3.7,(0,-8.5,-14.95))+hole(3,1,(0,-8.5,-16.3))
 return paint(s,'compliant_TPU_actuator_tip','pink')
def eye_bezel():
 s=hole(19.1,2.4,axis='y')-hole(16.5,8,axis='y')
 # Cosmetic round seat; exact LCD PCB retention awaits the selected module revision.
 return paint(s,'round_display_bezel','body')

def ear():
 # Shorter covered root; keep original pivot and spline datum provisional.
 s=ellipsoid((8,13,24),(0,6,43))-ellipsoid((5,9,18),(5,6,45))
 stem=bd.Solid.make_loft([bd.Wire.make_ellipse(5,4,plane=bd.Plane(origin=(0,0,0))),bd.Wire.make_ellipse(5,4,plane=bd.Plane(origin=(0,6,24)))])
 s=s+stem+hole(6,12,axis='x')-hole(1.65,20,axis='x')
 return paint(s,'short_root_ear_shell','body')

def install(g):
 global _G;_G=g
 original=g['link_components']
 replacements=dict(head_shell=head_shell,head_hatch=head_hatch,horn=horn,hoof=hoof,ear=ear)
 extras=dict(camera_carrier=camera_carrier,sensor_tray=sensor_tray,depth_pod=depth_pod,hoof_sole_plate=hoof_sole_plate,hoof_tpu_pad=hoof_tpu_pad,hoof_actuator=hoof_actuator,eye_bezel=eye_bezel)
 g.update(replacements);g['FACTORIES'].update(replacements|extras)
 g['COLORS']['iris']=[.53,.24,.78,1]
 def components():
  from simulation.model import rotation
  parts=original();rigid=g['rigid'];c=g['C']();geo=c['geometry']
  def add(link,shape,label,pos=(0,0,0),rot=None,color=None):parts[link].append(rigid(shape,label,pos,rot,color))
  for link in ['ear_left','ear_right']:
   parts[link]=[s.moved(bd.Location((0,0,-8))) if s.label.startswith('ear_inner_') else s for s in parts[link]]
  parts['head']=[s for s in parts['head'] if not s.label.startswith(('eye_lens_','iris_','pupil_','highlight_','nostril_','ear_servo_'))]
  parts['torso']=[s for s in parts['torso'] if s.label!='imu_envelope']
  add('torso',sensor_tray(),'sensor_tray')
  add('torso',paint(box((25.4,25.4,1.6)),'LSM6DSV16X_PCB_envelope','mane'),'imu_board',(10,0,29))
  add('torso',paint(box((21,17.8,7)),'XIAO_RP2040_envelope','frame'),'collector',(-30,0,29))
  add('torso',depth_pod(),'chest_tof_pod')
  Rd=rotation([0,1,0],np.deg2rad(20))
  add('torso',paint(box((1.6,25.4,25.4)),'VL53L5CX_PCB_envelope','frame'),'depth_board',(111,0,-7),Rd)
  add('torso',paint(box((2,6.4,3)),'VL53L5CX_optic_envelope','eye'),'depth_optic',(116.4,0,-9),Rd)
  add('torso',paint(box((22,18.5,9.5)),'6V_3A_buck_provisional_envelope','frame'),'ear_6V_buck',(-10,29,15))
  add('torso',paint(box((13,10,5)),'3V3_500mA_buck_allowance','frame'),'depth_3V3_buck',(36,0,29))
  add('head',camera_carrier(),'camera_carrier')
  add('head',paint(box((1.6,25,24)),'Camera_Module3_Wide_PCB_envelope','frame'),'camera_board',(65.6,0,64.5))
  add('head',paint(hole(7.2,11.6,axis='x'),'camera_lens_envelope','eye'),'camera_lens',(71.8,0,67))
  for side in [-1,1]:
   add('head',paint(box((28.5,12.1,23.2)),'STS3032_envelope_not_vendor_STEP','servo'),'micro_ear_servo_'+str(side),(20,side*22,60))
   add('head',paint(hole(3.5,10.75,axis='x'),'ear_horn_provisional_datum','servo'),'ear_output_adapter_'+str(side),(39.625,side*22,60))
   Reye=eye_rotation(side);ep=np.array([53.,side*42.,44.])
   def eye(shape,label,offset=(0,0,0)):
    add('head',shape,label+'_'+str(side),ep+Reye@np.array(offset),Reye)
   eye(eye_bezel(),'eye_bezel')
   # Flat pixels on a physical circular LCD; these layers are appearance only.
   eye(paint(hole(16.2,.45,axis='y'),'display_pixels_sclera','body'),'display_sclera',(0,1.4,0))
   eye(paint(flat_ellipse(10,14.8,.12),'display_pixels_iris','iris'),'display_iris',(3*side,1.75,0))
   eye(paint(flat_ellipse(6.5,11.8,.10),'display_pixels_pupil','eye'),'display_pupil',(5*side,2.,0))
   eye(paint(flat_ellipse(3,4,.08),'display_pixels_highlight','body'),'display_highlight',(3*side,2.2,6))
   eye(paint(flat_ellipse(1.4,1.8,.08),'display_pixels_small_glint','body'),'display_glint',(7*side,2.2,-5))
   eye(paint(box((40.4,4,37.5)),'GC9A01_module_board_envelope','frame'),'eye_board',(0,-14.5,0))
   # Pigment follows the skin: thin brows, small nostrils and ink-filled smile.
   brow=[]
   for x in np.linspace(44,63,33):
    z=66+3*(1-((x-53.5)/9.5)**2)
    hits=head_outer().find_intersection_points(bd.Axis((float(x),side*100,float(z)),(0,-side,0)))
    if hits:
     y=side*max(abs(p.Y) for p,n in hits)
     brow.append(bd.Solid.make_sphere(.42).moved(bd.Location((float(x),y+side*.15,float(z)))))
   add('head',paint(bd.Compound(children=brow),'eyebrow_pigment','eye'),'eyebrow_'+str(side))
  smile=[]
  for y in np.linspace(-8,8,61):
   z=20+3*(y/8)**2;hits=head_outer().find_intersection_points(bd.Axis((140,float(y),float(z)),(-1,0,0)))
   if hits:
    x=max(p.X for p,n in hits)
    smile.append(bd.Solid.make_sphere(.23).moved(bd.Location((x+.08,float(y),float(z)))))
  add('head',paint(bd.Compound(children=smile),'ink_filled_smile','eye'),'smile_pigment')
  for y in [-8,8]:
   hits=head_outer().find_intersection_points(bd.Axis((140,y,29),(-1,0,0)))
   if hits:
    x=max(p.X for p,n in hits)
    add('head',paint(ellipsoid((.4,1.25,.85)),'small_nostril','eye'),'nostril_'+str(y),(x,y,29))
  for leg in c['legs']:
   angle=g['home'](c)[leg+'_hip_pitch']+g['home'](c)[leg+'_knee'];R=rotation([0,1,0],-angle);p=np.array([0,0,-geo['shin']])
   for name,fn in [('floating_sole',hoof_sole_plate),('TPU_pad',hoof_tpu_pad),('TPU_actuator',hoof_actuator)]:add(leg+'_shin',fn(),leg+'_'+name,p,R)
   for label,shape,local in [('switch_PCB',box((12,12,1.6)),(0,-8.5,-10.1)),('switch',box((6.2,6.2,2)),(0,-8.5,-11.9))]:
    add(leg+'_shin',paint(shape,label+'_envelope','frame'),leg+'_'+label,p+R@np.array(local),R)
  return parts
 g['link_components']=components

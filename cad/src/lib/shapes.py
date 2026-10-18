from __future__ import annotations
import math
import numpy as np
from cadgen import build123d as bd, read_step
from scripts.common import ROOT,load
from simulation.model import graph,fk,home,COLORS

C=load

def paint(s,label,color):
    s.label=label;s.color=bd.Color(*COLORS.get(color,COLORS['body'])[:3]);return s

def ellipsoid(r,pos=(0,0,0)):
    return bd.Solid.make_sphere(1).transform_geometry(bd.Matrix([[r[0],0,0,pos[0]],[0,r[1],0,pos[1]],[0,0,r[2],pos[2]],[0,0,0,1]]))

def box(size,pos=(0,0,0),radius=0):
    s=bd.Box(*size)
    if radius:s=bd.fillet(s.edges(),radius)
    return s.moved(bd.Location(pos))

def hole(r,length,pos=(0,0,0),axis='z'):
    t=bd.Cylinder(r,length)
    if axis=='y':t=t.rotate(bd.Axis.X,90)
    if axis=='x':t=t.rotate(bd.Axis.Y,90)
    return t.moved(bd.Location(pos))

def deck():
    g=C()['geometry'];x,y,z=g['body'];t=g['structural_wall']
    s=box((x,y,t),radius=1.4)
    # two side rails, battery straps and electronics mounting grid
    for px in [-75,-45,0,45,75]:
        for py in [-35,35]:s=s-hole(1.65,8,(px,py,0))
    for px in [-40,20]:
        for py in [-20,20]:s=s-box((3.2,18,8),(px,py,0))
    for px in [-95,95]:
        for py in [-45,45]:s=s-hole(1.65,8,(px,py,0))
    for px in [-90,90]:
        for py in [-37,37]:s=s-hole(1.65,8,(px,py,0))
    for px in [-50,50]:
        for py in [-20,20]:s=s-hole(1.65,8,(px,py,0))
    return paint(s,'chassis_deck','frame')

def rail():
    s=box((200,3.2,30),(0,0,0))
    for px in [-90,-45,0,45,90]:s=s-hole(1.65,8,(px,0,-10),'y')
    for px in [-90,90]:
        s=s+box((14,14,3.2),(px,-5,-13.4))
        s=s-hole(1.65,14,(px,-6,-13))
    return paint(s,'chassis_rail','frame')

def body_shell():
    g=C()['geometry'];x,y,z=g['body'];w=g['shell_wall']
    # An oval chest and rump replace the flat table-like lid.
    outer=ellipsoid(((x+4)/2,(y+4)/2,(z+4)/2))
    inner=ellipsoid(((x+4)/2-w,(y+4)/2-w,(z+4)/2-w))
    s=outer-inner
    s=s-box((x+30,y+30,100),(0,0,-74.8))
    for sx in [-1,1]:
        for sy in [-1,1]:
            s=s-box((56,44,90),(sx*g['hip_x'],sy*g['hip_y'],-34))
    # Local pockets clear the complete roll clamp envelopes at the home pose.
    for sx in [-1,1]:
        for sy in [-1,1]:
            R=np.array([[0,sx,0],[-sy,0,0],[0,0,sy*sx]],float)
            pocket=box((63.2,42.2,37.5),(12.5,-20,.55))
            s=s-rigid(pocket,'hip_service_pocket',(sx*g['hip_x'],sy*g['hip_y'],0),R)
    # Yaw actuator is embedded in the chest; its white collar emerges here.
    s=s-hole(30,160,(g['head_origin'][0],0,38))
    for px in [-35,-25,-15,-5,5,15,25]:s=s-box((3.2,24,30),(px,0,40))
    # Four low ribs reach the skin without entering the battery or motor cases.
    for px in [-50,50]:
        for side in [-1,1]:
            py=side*20
            s=s+box((5,22,4.3),(px,side*30,-22.65))+hole(4,4.3,(px,py,-22.65))
            s=s-hole(2.1,8,(px,py,-22.65))
    solids=list(s.solids());largest=max(solids,key=lambda solid:solid.volume)
    # Overlapping service pockets can leave two sub-3 mm3 fore-edge chips.
    # They are not connected printable features and are deliberately omitted.
    assert sum(a.volume for a in solids if a is not largest)<10
    return paint(largest,'rounded_body_cowl','body')

def servo_cradle():
    # Vendor output Y datum: case X[-10.112,35.112], Y[-37.8,-2.1], Z[+-12.362].
    # All walls stay outside this envelope; M3 clamps use external bosses.
    s=box((62,41,3.2),(12.5,-20,-16),radius=1.2)
    for px in [-13.0,38.0]:
        wall=box((3.2,39,31),(px,-20,0))
        wall=wall-box((10,19,19),(px,-20,1))
        s=s+wall
    for px in [-14.5,39.5]:
        for py in [-7,-33]:
            s=s+hole(3.5,31,(px,py,0))
            s=s-hole(1.65,50,(px,py,0))
    for px in [2,26]:s=s-box((18,21,12),(px,-20,-16),radius=1)
    return paint(s,'servo_cradle','frame')

def servo_strap():
    s=box((62,7,3.2),(12.5,0,0),radius=1.0)
    for px in [-14.5,39.5]:s=s-hole(1.65,10,(px,0,0))
    return paint(s,'servo_clamp_strap','frame')


def roll_yoke():
    # Two cheek plates bridge roll axis to dropped hip axis.
    g=C()['geometry'];drop=g['hip_drop'];off=g['abduction_offset']
    s=box((4,off+16,drop+18),(0,off/2,-drop/2))
    s=s-hole(1.65,12,(0,0,0),'x')
    s=s-hole(1.65,12,(0,off,-drop),'x')
    s=s-hole(5.1,2.8,(0,0,0),'x')
    for py in [8,off-6]:s=s-hole(1.6,12,(0,py,-drop*.5),'x')
    return paint(s,'roll_yoke','frame')

def link_plate(length,label):
    # Output horn at z=0 and supported knee at z=-length; holes are provisional horn datum.
    s=box((25,4,length+24),(0,0,-length/2),radius=1.8)
    for pz in [0,-length]:
        s=s-hole(1.65,12,(0,0,pz),'y')
        s=s-hole(5.1,2.8,(0,-1,pz),'y')
        for px in [-7,7]:s=s-hole(1.1,12,(px,0,pz),'y')
    # weight relief and a cable guide
    s=s-box((12,10,max(8,length-30)),(0,0,-length/2),radius=1)
    return paint(s,label,'body')

def thigh_plate():return link_plate(C()['geometry']['thigh'],'thigh_cheek')
def shin_plate():return link_plate(C()['geometry']['shin'],'shin_cheek')

def hoof():
    # Hollow flared-looking boot; the broad, level sole replaces a round ball.
    s=box((34,34,24),(0,0,-4),radius=4)
    s=s-box((30,30,22),(0,0,-.5),radius=3)
    s=s+hole(4,20,(0,0,-6))
    s=s-hole(1.65,50)
    s=s-hole(3.1,7,(0,0,6))
    return paint(s,'hollow_hoof_boot','body')

def head_shell():
    g=C()['geometry'];r=g['head_radii'];p=g['head_center'];w=g['shell_wall'];mr=g['muzzle_radii'];mp=g['muzzle_center']
    outside=ellipsoid(r,p)+ellipsoid(mr,mp)
    inside=ellipsoid([x-w for x in r],p)+ellipsoid([x-w for x in mr],mp)
    s=outside-inside
    # Open lower service hatch; displays sit behind the two side apertures.
    s=s-box((150,150,100),(37,0,-38))
    for side in [-1,1]:
        s=s-ellipsoid((23,21,26),(54,side*44,41))
        # Clearance slots for narrow ear stems; full servo cases stay inside.
        s=s-box((18,14,45),(g['ear_origin'][0],side*g['ear_origin'][1],96))
    for px in [20,60]:
        for py in [-20,20]:
            rib=box((8,40,8),(px,py+ (15 if py>0 else -15),16))
            s=s+(rib & outside)+hole(4,8,(px,py,16))
            s=s-hole(1.65,20,(px,py,15))
    # An integrated symmetric saddle retains the ear cases inside the skull.
    # Two soft shims and nylon ties complete the clamp; slots remain accessible.
    s=s+box((48,76,3.2),(26,0,21.1))
    for py in [-36.5,-7.5,7.5,36.5]:
        wall=box((39,2.4,43),(26,py,44.2))
        for px in [16,36]:wall=wall-box((3.2,10,11),(px,py,48))
        s=s+wall
    s=s+hole(4,12,(48,0,88))
    s=s-hole(1.65,30,(48,0,96))
    return paint(s,'pony_head_shell','body')

def head_hatch():
    g=C()['geometry']
    outer=ellipsoid(g['head_radii'],g['head_center'])+ellipsoid(g['muzzle_radii'],g['muzzle_center'])
    s=outer & box((170,130,3),(37,0,10.5))
    s=s-box((100,58,10),(-34,0,10.5))
    for px in [20,60]:
        for py in [-20,20]:s=s-hole(1.65,10,(px,py,10.5))
    return paint(s,'head_service_hatch','frame')


def ear():
    s=ellipsoid((8,13,24),(0,6,51))
    s=s-ellipsoid((5,9,18),(5,6,53))
    stem=bd.Solid.make_loft([bd.Wire.make_ellipse(5,4,plane=bd.Plane(origin=(0,0,0))),bd.Wire.make_ellipse(5,4,plane=bd.Plane(origin=(0,6,32)))])
    s=s+stem+hole(6,12,axis='x')
    s=s-hole(1.65,20,axis='x')
    return paint(s,'pointed_ear_shell','body')

def mane():
    # Hollow forelock and an overlapping curl; no box-like rear disk.
    a=ellipsoid((18,29,27),(6,0,87))-ellipsoid((15.8,26.8,24.8),(6,0,87))
    a=a-box((90,90,60),(6,0,40))
    curl=bd.Torus(13,5,major_angle=285).rotate(bd.Axis.X,90).moved(bd.Location((28,-8,102)))
    groove=bd.Torus(13,1.6,major_angle=285).rotate(bd.Axis.X,90).moved(bd.Location((28,-12.5,102)))
    return paint((a+curl)-groove,'curled_forelock','mane')

def mane_stripe():
    s=bd.Torus(13,1.4,major_angle=285).rotate(bd.Axis.X,90).moved(bd.Location((28,-12.5,102)))
    return paint(s,'pink_mane_stripe','pink')

def horn():
    s=bd.Cone(8,1,33).moved(bd.Location((48,0,104.5)))
    s=s-hole(1.65,24,(48,0,96))
    return paint(s,'unicorn_horn','body')

def neck_collar():
    sections=[(0,30,29),(17,28,30),(40,28,31)]
    def loft(w):
        return bd.Solid.make_loft([bd.Wire.make_ellipse(rx-w,ry-w,plane=bd.Plane(origin=(0,0,z))) for z,rx,ry in sections])
    s=loft(0)-loft(1.8)
    return paint(s,'centred_neck_collar','body')

def leg_fairing(length,label):
    outer=ellipsoid((18,4,(length+24)/2),(0,0,-length/2))
    inner=ellipsoid((16.4,3.2,(length+20.8)/2),(0,0,-length/2))
    s=(outer-inner)-box((70,30,200),(0,-15,-length/2))
    for z in [0,-length]:
        s=s+hole(4,4,(0,2,z),'y')
        s=s-hole(1.65,15,(0,2,z),'y')
    return paint(s,label,'body')

def thigh_fairing():return leg_fairing(C()['geometry']['thigh'],'thigh_fairing')
def shin_fairing():return leg_fairing(C()['geometry']['shin'],'shin_fairing')

def fit_coupon():
    s=box((80,35,4))
    for i,d in enumerate([3.1,3.2,3.3,3.4]):s=s-hole(d/2,12,(-28+i*16,-8,0))
    for i,d in enumerate([10.0,10.1,10.2]):s=s-hole(d/2,2.5,(-24+i*24,8,1.0))
    return paint(s,'fit_coupon','frame')

FACTORIES={'deck':deck,'rail':rail,'body_shell':body_shell,'servo_cradle':servo_cradle,'roll_yoke':roll_yoke,'thigh_plate':thigh_plate,'shin_plate':shin_plate,'hoof':hoof,'head_shell':head_shell,'head_hatch':head_hatch,'ear':ear,'mane':mane,'fit_coupon':fit_coupon,'horn':horn,'mane_stripe':mane_stripe,'neck_collar':neck_collar,'thigh_fairing':thigh_fairing,'shin_fairing':shin_fairing}

def matrix_location(m):
    # Orthogonal rigid transform only; preserves exact vendor STEP surfaces.
    return bd.Location(bd.Plane(origin=tuple(m[:3,3]),x_dir=tuple(m[:3,0]),z_dir=tuple(m[:3,2])))

def servo(rotation=None):
    src=read_step(ROOT/'cad/STEP/imported/feetech_sts3250.step')
    src=src.moved(bd.Location((25.5,-9.2,0)))
    if rotation is not None:
        m=np.eye(4);m[:3,:3]=rotation;src=src.moved(matrix_location(m))
    return paint(src,'STS3250_catalog_reference','servo')

def primitive(g):
    size=[v*1000 for v in g['size']];p=[v*1000 for v in g['pos']]
    if g['kind']=='ellipsoid':s=ellipsoid(size,p)
    elif g['kind']=='box':s=box(size,p)
    elif g['kind']=='sphere':s=bd.Sphere(size[0]).moved(bd.Location(p))
    else:
        end=np.array(g['end'])*1000;start=np.array(p);d=end-start;L=float(np.linalg.norm(d))
        s=bd.Cylinder(size[0],L).moved(bd.Location(tuple((start+end)/2)))
        s=s+bd.Sphere(size[0]).moved(bd.Location(tuple(start)))+bd.Sphere(size[0]).moved(bd.Location(tuple(end)))
    return paint(s,'visual_'+g['kind'],g['color'])

def link_visuals():
    return {l['name']:bd.Compound(children=[primitive(g) for g in l['geoms']],label=l['name']) for l in graph()}

def rigid(shape,name,pos=(0,0,0),rot=None,color=None):
    m=np.eye(4);m[:3,3]=pos
    if rot is not None:m[:3,:3]=rot
    out=shape.moved(matrix_location(m));out.label=name
    if color:out.color=bd.Color(*COLORS[color][:3])
    return out

def axle_adapter():
    s=hole(5,15,axis='x')-hole(1.65,22,axis='x')
    return paint(s,'roll_axle_adapter','frame')
FACTORIES['axle_adapter']=axle_adapter

def link_components():
    c=C();g=c['geometry'];parts={l['name']:[] for l in graph(c)}
    def add(link,shape,label,pos=(0,0,0),rot=None,color=None):
        parts[link].append(rigid(shape,label,pos,rot,color))
        if label.endswith('_cradle'):
            R=np.eye(3) if rot is None else rot
            for sy in [-7,-33]:
                local=np.array([0,sy,17.1])
                parts[link].append(rigid(servo_strap(),label+'_strap_'+str(sy),np.array(pos)+R@local,R))
    add('torso',deck(),'chassis_deck',(0,0,-26.4))
    for side in [-1,1]:
        r=rail() if side==1 else bd.mirror(rail(),about=bd.Plane.XZ)
        add('torso',r,f'rail_{side}',(0,side*43,-9.8))
    add('torso',body_shell(),'body_cowl')
    add('torso',tail(),'passive_tail')
    # Purchased electronic envelopes: bounded placeholders, not mounting-certified vendor models.
    add('torso',paint(box(tuple(c['power']['battery']['envelope_mm'])),'3S_LiFePO4_envelope','hoof'),'battery_envelope',(0,0,-8))
    add('torso',paint(box((65,30,9)),'Pi_Zero_2_W_envelope','frame'),'compute_envelope',(0,0,15))
    add('torso',paint(box((25,20,5)),'IMU_breakout_envelope','mane'),'imu_envelope',(0,30,15))
    add('torso',paint(box((30,23,8)),'USB_TTL_adapter_envelope','frame'),'adapter_envelope',(-62,0,-5))
    add('torso',paint(box((18,21,9)),'5V_buck_envelope','frame'),'regulator_envelope',(-62,0,16))
    base=servo();br=servo_cradle();plate=thigh_plate();shin=shin_plate();yoke=roll_yoke();ad=axle_adapter()
    for leg,sgn in c['legs'].items():
        sx=sgn['x_sign'];side=sgn['y_sign'];origin=np.array([sx*g['hip_x'],side*g['hip_y'],0.])
        Rroll=np.array([[0,sx,0],[-side,0,0],[0,0,side*sx]],float)
        add('torso',base,leg+'_roll_servo',origin,Rroll)
        # Clamp cradles are placed with each servo in its vendor case frame.
        add('torso',br,leg+'_roll_cradle',origin,Rroll)
        Rh=np.array([[0,0,side],[0,side,0],[-1,0,0]],float)
        hip=[0,side*g['abduction_offset'],-g['hip_drop']]
        add(leg+'_roll',base,leg+'_pitch_servo',hip,Rh)
        add(leg+'_roll',br,leg+'_pitch_cradle',hip,Rh)
        yy=yoke if side==1 else bd.mirror(yoke,about=bd.Plane.XZ)
        add(leg+'_roll',yy,leg+'_roll_cheek',(sx*16,0,0))
        add(leg+'_roll',ad,leg+'_roll_axle',(sx*8,0,0))
        Rk=np.array([[0,0,-side],[0,side,0],[1,0,0]],float)
        add(leg+'_thigh',base,leg+'_knee_servo',(0,0,-g['thigh']),Rk)
        for py in [side*6,-side*44]:
            add(leg+'_thigh',plate,leg+'_thigh_cheek_'+str(py),(0,py,0))
        for py in [side*6,-side*6]:
            add(leg+'_shin',shin,leg+'_shin_cheek_'+str(py),(0,py,0))
        fork=knee_fork() if side==1 else bd.mirror(knee_fork(),about=bd.Plane.XZ)
        add(leg+'_shin',fork,leg+'_knee_support_fork')
        # Three M3 spacer tubes join the cheeks; the hoof load is carried close to the front cheek.
        for pz in [-g['thigh']*.3,-g['thigh']*.7]:
            add(leg+'_thigh',paint(hole(3.2,46,axis='y')-hole(1.65,52,axis='y'),'spacer','frame'),leg+'_thigh_spacer_'+str(pz),(9,-side*19,pz))
        hpose=home(c);angle=hpose[leg+'_hip_pitch']+hpose[leg+'_knee']
        from simulation.model import rotation
        add(leg+'_shin',hoof(),leg+'_hoof',(0,0,-g['shin']),rotation([0,1,0],-angle))
        for link,shape in [(leg+'_thigh',thigh_fairing()),(leg+'_shin',shin_fairing())]:
            if side<0:shape=bd.mirror(shape,about=bd.Plane.XZ)
            add(link,shape,link+'_fairing',(0,side*9,0))
        # A smooth shin fairing is a mesh-envelope visual only; structural load is carried by cheek plates.
    yawR=np.array([[-1,0,0],[0,0,1],[0,1,0]],float)
    add('torso',base,'head_yaw_servo',g['head_origin'],yawR)
    add('torso',br,'head_yaw_cradle',g['head_origin'],yawR)
    pitch_origin=[0,18.7,g['neck_pitch_rise']]
    Rp=np.array([[0,0,1],[0,1,0],[-1,0,0]],float)
    add('neck_yaw',base,'head_pitch_servo',pitch_origin,Rp)
    add('neck_yaw',br,'head_pitch_cradle',pitch_origin,Rp)
    add('neck_yaw',neck_collar(),'white_neck_collar')
    add('head',head_shell(),'pony_head_shell')
    add('head',head_hatch(),'head_service_hatch')
    add('head',head_yoke(),'head_support_yoke')
    add('head',mane(),'mane_crest')
    add('head',mane_stripe(),'pink_mane_stripe')
    add('head',horn(),'unicorn_horn')
    for side in [-1,1]:
        # Display lenses and iris are visual representations, not printable display components.
        add('head',paint(ellipsoid((24,5,27)),'eye_lens','eye'),'eye_lens_'+str(side),(54,side*45,41))
        add('head',paint(ellipsoid((14,2,20)),'iris','iris'),'iris_'+str(side),(58,side*50,41))
        add('head',paint(ellipsoid((8,1.2,15)),'pupil','eye'),'pupil_'+str(side),(59,side*52,41))
        add('head',paint(ellipsoid((3,1,5)),'eye_highlight','body'),'highlight_'+str(side),(62,side*53.5,48))
        add('head',paint(ellipsoid((3,4,2.5)),'nostril','eye'),'nostril_'+str(side),(g['muzzle_center'][0]+g['muzzle_radii'][0]-4,side*g['muzzle_radii'][1]*13/34,g['muzzle_center'][2]))
        earorigin=[g['ear_origin'][0],side*g['ear_origin'][1],g['ear_origin'][2]]
        Re=np.array([[0,1,0],[0,0,-1],[-1,0,0]],float)
        add('head',base,'ear_servo_'+str(side),earorigin,Re)
        earlink='ear_left' if side==1 else 'ear_right'
        ear_shape=ear() if side==1 else bd.mirror(ear(),about=bd.Plane.XZ)
        add(earlink,ear_shape,'ear_shell_'+str(side))
        add(earlink,paint(ellipsoid((1.4,8,16)),'ear_inner','pink'),'ear_inner_'+str(side),(7,side*6,53))
    return parts

def assembly_shape(exploded=False,internal=False,pose=None):
    c=C();links=graph(c);q=home(c)
    if pose:q.update(pose)
    transforms=fk(links,q,c);groups=[]
    for name,children in link_components().items():
        if internal:children=[s for s in children if s.label not in ('body_cowl','pony_head_shell','mane_crest','white_neck_collar')]
        m=transforms[name].copy();m[:3,3]*=1000
        m[2,3]+=c['geometry']['stance_height']+c['geometry']['foot_radius']
        if exploded:
            if name.startswith(('fl','fr','rl','rr')):m[:3,3]+=np.array([40*(1 if name[0]=='f' else -1),60*(1 if name[1]=='l' else -1),-25])
            elif name!='torso':m[2,3]+=70
        group=bd.Compound(children=children,label=name)
        groups.append(group.moved(matrix_location(m)))
    return bd.Compound(children=groups,label='Sweetie_Mini_16DOF')


def knee_fork():
    s=box((31,4,30),(5.5,10,-5))+box((31,4,30),(5.5,-44,-5))+box((8,58,4),(17.5,-17,-18))
    s=s-hole(1.65,80,(0,-17,0),'y')
    s=s-hole(5.1,2.8,(0,-45,0),'y')
    for py in [10,-44]:s=s-hole(1.65,8,(9,py,-12),'y')
    return paint(s,'knee_support_fork','body')

def head_yoke():
    s=box((31,4,16),(5.5,25,4))+box((31,4,16),(5.5,-25,4))+box((8,54,4),(20,0,7))
    s=s-hole(1.65,80,(0,0,0),'y')
    s=s-hole(5.1,2.8,(0,-26,0),'y')
    for py in [-20,20]:s=s-hole(1.65,16,(20,py,7))
    return paint(s,'head_support_yoke','frame')

FACTORIES.update(knee_fork=knee_fork,head_yoke=head_yoke,servo_strap=servo_strap)


def tail():
    # Curved hollow pony tail, lofted on native ellipse wires.
    sections=[(16,-106,8,10),(0,-118,12,16),(-25,-132,13,18),(-50,-129,10,15),(-66,-118,4,8)]
    def loft(w):
        return bd.Solid.make_loft([bd.Wire.make_ellipse(rx-w,ry-w,plane=bd.Plane(origin=(x,0,z))) for z,x,rx,ry in sections])
    s=loft(0)-loft(1.8)
    s=s+box((3,24,16),(-104,0,6))
    for py in [-6,6]:s=s-hole(1.65,20,(-104,py,6),'x')
    return paint(s,'curved_pony_tail','mane')
FACTORIES['tail']=tail

# v03 candidate only; the accepted v02 workspace is never imported or modified.
from lib import sensors_face
sensors_face.install(globals())
from lib import actuation_v04
actuation_v04.install(globals())

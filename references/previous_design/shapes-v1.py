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
    return paint(s,'chassis_deck','frame')

def rail():
    s=box((200,3.2,30),(0,0,0))
    for px in [-90,-45,0,45,90]:s=s-hole(1.65,8,(px,0,-10),'y')
    return paint(s,'chassis_rail','frame')

def body_shell():
    g=C()['geometry'];x,y,z=g['body'];w=g['shell_wall']
    outer=box((x+4,y+4,z+4),radius=10)
    inner=box((x+4-2*w,y+4-2*w,z+4-2*w),radius=8.2)
    s=outer-inner
    s=s-box((x+20,y+20,z),(0,0,-z/2-8))
    # service seam, vents and four hip clearances
    for signx in [-1,1]:
        for signy in [-1,1]:s=s-box((48,30,38),(signx*g['hip_x'],signy*(y/2+5),-5))
    for px in [-35,-25,-15,-5,5,15,25,35]:s=s-box((4,26,20),(px,0,30))
    s=s-box((27,35,40),(85,0,28))
    for px in [-95,95]:
        for py in [-45,45]:s=s-hole(1.8,80,(px,py,0))
    return paint(s,'body_cowl','body')

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
    for px in [-5,30]:s=s-box((3.0,19,10),(px,-20,-16))
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
    r=C()['geometry']['foot_radius'];s=ellipsoid((r,r,r*.70),(0,0,r*.35))
    s=s-box((2*r+2,2*r+2,r),(0,0,-r*.80))
    s=s-hole(1.65,50)
    s=s-hole(3.1,8,(0,0,13))
    return paint(s,'hoof_pad','hoof')

def head_shell():
    g=C()['geometry'];r=g['head_radii'];p=g['head_center'];w=g['shell_wall'];mr=g['muzzle_radii'];mp=g['muzzle_center']
    outside=ellipsoid(r,p)+ellipsoid(mr,mp)
    inside=ellipsoid([x-w for x in r],p)+ellipsoid([x-w for x in mr],mp)
    for side in [-1,1]:
        outside=outside+box((44,30,50),(7,side*34,65),radius=3)
        inside=inside+box((44-2*w,30-2*w,50-2*w),(7,side*34,65),radius=1.2)
    s=outside-inside
    # Open lower service hatch; displays sit behind the two side apertures.
    s=s-box((150,150,100),(37,0,-38))
    for side in [-1,1]:
        s=s-ellipsoid((23,19,26),(53,side*39,41))
        s=s-hole(10,18,(27,side*34,77),'x')
    for px in [20,60]:
        for py in [-20,20]:
            rib=box((8,20,8),(px,py+ (8 if py>0 else -8),16))
            s=s+(rib & outside)+hole(4,8,(px,py,16))
            s=s-hole(1.65,20,(px,py,15))
    return paint(s,'pony_head_shell','body')

def head_hatch():
    g=C()['geometry']
    outer=ellipsoid(g['head_radii'],g['head_center'])+ellipsoid(g['muzzle_radii'],g['muzzle_center'])
    s=outer & box((170,130,3),(37,0,10.5))
    s=s-box((30,42,10),(0,-12,10.5))
    for px in [20,60]:
        for py in [-20,20]:s=s-hole(1.65,10,(px,py,10.5))
    return paint(s,'head_service_hatch','frame')


def ear():
    # Hollow flattened teardrop, split flat underside for support-free printing.
    s=ellipsoid((12.5,7.5,23),(0,0,21))
    s=s-ellipsoid((9.7,4.7,19),(0,0,22))
    s=s+box((20,13,4),(0,0,2),radius=1)
    s=s-hole(1.65,30,(0,0,3))
    return paint(s,'ear_shell','body')

def mane():
    # Removable crest; lightweight hollow segments remain printable independently.
    outside=ellipsoid((20,34,44),(-4,0,57))
    inside=ellipsoid((17.5,31.5,41.5),(-4,0,57))
    s=outside-inside
    s=s-box((80,100,100),(35,0,57))
    return paint(s,'mane_crest','mane')

def fit_coupon():
    s=box((80,35,4))
    for i,d in enumerate([3.1,3.2,3.3,3.4]):s=s-hole(d/2,12,(-28+i*16,-8,0))
    for i,d in enumerate([10.0,10.1,10.2]):s=s-hole(d/2,2.5,(-24+i*24,8,1.0))
    return paint(s,'fit_coupon','frame')

FACTORIES={'deck':deck,'rail':rail,'body_shell':body_shell,'servo_cradle':servo_cradle,'roll_yoke':roll_yoke,'thigh_plate':thigh_plate,'shin_plate':shin_plate,'hoof':hoof,'head_shell':head_shell,'head_hatch':head_hatch,'ear':ear,'mane':mane,'fit_coupon':fit_coupon}

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
    for side in [-1,1]:add('torso',rail(),f'rail_{side}',(0,side*49,-10))
    add('torso',body_shell(),'body_cowl')
    add('torso',tail(),'passive_tail')
    # Purchased electronic envelopes: bounded placeholders, not mounting-certified vendor models.
    add('torso',paint(box(tuple(c['power']['battery']['envelope_mm'])),'3S_LiFePO4_envelope','hoof'),'battery_envelope',(0,0,-9.5))
    add('torso',paint(box((65,30,9)),'Pi_Zero_2_W_envelope','frame'),'compute_envelope',(0,0,15))
    add('torso',paint(box((25,20,5)),'IMU_breakout_envelope','mane'),'imu_envelope',(0,30,15))
    add('torso',paint(box((30,23,8)),'USB_TTL_adapter_envelope','frame'),'adapter_envelope',(-62,0,-5))
    add('torso',paint(box((18,21,9)),'5V_buck_envelope','frame'),'regulator_envelope',(65,0,-10))
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
        add(leg+'_shin',hoof(),leg+'_hoof',(0,0,-g['shin']-g['foot_radius']*.65))
        # A smooth shin fairing is a mesh-envelope visual only; structural load is carried by cheek plates.
    yawR=np.array([[-1,0,0],[0,0,1],[0,1,0]],float)
    add('torso',base,'head_yaw_servo',g['head_origin'],yawR)
    add('torso',br,'head_yaw_cradle',g['head_origin'],yawR)
    pitch_origin=[0,0,g['neck_pitch_rise']]
    Rp=np.array([[0,0,1],[0,1,0],[-1,0,0]],float)
    add('neck_yaw',base,'head_pitch_servo',pitch_origin,Rp)
    add('neck_yaw',br,'head_pitch_cradle',pitch_origin,Rp)
    add('head',head_shell(),'pony_head_shell')
    add('head',head_hatch(),'head_service_hatch')
    add('head',head_yoke(),'head_support_yoke')
    add('head',mane(),'mane_crest')
    for side in [-1,1]:
        # Display lenses and iris are visual representations, not printable display components.
        add('head',paint(ellipsoid((24,5,27)),'eye_lens','eye'),'eye_lens_'+str(side),(53,side*39,41))
        add('head',paint(ellipsoid((14,2,20)),'iris','iris'),'iris_'+str(side),(58,side*44,41))
        add('head',paint(ellipsoid((8,1.2,15)),'pupil','eye'),'pupil_'+str(side),(59,side*46,41))
        add('head',paint(ellipsoid((3,1,5)),'eye_highlight','body'),'highlight_'+str(side),(62,side*47.5,48))
        add('head',paint(ellipsoid((3,4,2.5)),'nostril','eye'),'nostril_'+str(side),(96,side*13,14))
        earorigin=[g['ear_origin'][0],side*g['ear_origin'][1],g['ear_origin'][2]]
        Re=np.array([[0,1,0],[0,0,-1],[-1,0,0]],float)
        add('head',base,'ear_servo_'+str(side),earorigin,Re)
        add('ear_left' if side==1 else 'ear_right',ear(),'ear_shell_'+str(side))
    return parts

def assembly_shape(exploded=False,internal=False,pose=None):
    c=C();links=graph(c);q=home(c)
    if pose:q.update(pose)
    transforms=fk(links,q,c);groups=[]
    for name,children in link_components().items():
        if internal:children=[s for s in children if s.label not in ('body_cowl','pony_head_shell','mane_crest')]
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
    s=box((31,4,16),(5.5,6,4))+box((31,4,16),(5.5,-44,4))+box((8,54,4),(20,-19,7))
    s=s-hole(1.65,80,(0,-19,0),'y')
    s=s-hole(5.1,2.8,(0,-45,0),'y')
    for py in [-20,20]:s=s-hole(1.65,16,(20,py,7))
    return paint(s,'head_support_yoke','frame')

FACTORIES.update(knee_fork=knee_fork,head_yoke=head_yoke,servo_strap=servo_strap)


def tail():
    # Rounded analytic surfaces survive STEP round trips reliably.
    s=box((24,30,70),(-120,0,-5),radius=10)-box((19.6,25.6,65.6),(-120,0,-5),radius=7.8)
    s=s+box((3,24,16),(-108.5,0,10))
    for py in [-6,6]:s=s-hole(1.65,20,(-108,py,10),'x')
    return paint(s,'passive_tail','mane')
FACTORIES['tail']=tail

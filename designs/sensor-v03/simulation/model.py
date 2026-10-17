"""Shared SI link graph for CAD placements, URDF and MuJoCo. +X muzzle, +Y left, +Z up."""
from __future__ import annotations
import math
import numpy as np
from scripts.common import load, joint_names

COLORS={'body':[0.93,0.94,0.98,1], 'mane':[0.60,0.36,0.76,1], 'hoof':[0.25,0.20,0.36,1],
        'frame':[0.25,0.32,0.40,1], 'servo':[0.14,0.17,0.20,1], 'eye':[0.1,0.08,0.19,1], 'iris':[0.22,0.88,0.32,1], 'pink':[0.91,0.57,0.74,1]}

def geometry(kind, size, pos=(0,0,0), color='body', collision=True, **kwargs):
    return dict(kind=kind,size=list(size),pos=list(pos),color=color,collision=collision,**kwargs)

def graph(c=None):
    c=c or load(); g=c['geometry']; mass=c['mass']; a=c['actuator_candidates'][c['selection']['selected']]; s=a['mass']
    mm=lambda v:[x/1000 for x in v]
    links=[]
    def link(name,parent,origin,joint,m,com,size,geoms):
        # Box-envelope inertias with explicit COM; conservative mass allocations.
        x,y,z=size; inertia=[m*(y*y+z*z)/12,m*(x*x+z*z)/12,m*(x*x+y*y)/12]
        d=dict(name=name,parent=parent,origin=origin,joint=joint,mass=m,com=com,inertia=inertia,geoms=geoms)
        links.append(d);return d
    link('torso',None,[0,0,0],None,mass['torso_without_servos']+5*s,[-.014,0,0],mm(g['body']),[
      geometry('ellipsoid',mm([v/2 for v in g['body']]),color='body'),
      geometry('ellipsoid',[.017,.018,.045],[-.108,0,.015],'mane',False),
      geometry('ellipsoid',[.013,.019,.020],[-.108,0,-.018],'mane',False)])
    for leg,sgn in c['legs'].items():
        side=sgn['y_sign']; origin=[sgn['x_sign']*g['hip_x']/1000,side*g['hip_y']/1000,0]
        link(leg+'_roll','torso',origin,leg+'_hip_roll',mass['roll_link_without_servo']+s,[0,0,-(g['hip_drop']+6)/1000],[.050,.065,(g['hip_drop']+50)/1000],[
          geometry('box',[.025,.037,.045],[0,side*.006,-.043],'servo')])
        link(leg+'_thigh',leg+'_roll',[0,side*g['abduction_offset']/1000,-g['hip_drop']/1000],leg+'_hip_pitch',mass['thigh_without_servo']+s,[0,-side*.019,-g['thigh']/1000*.70],[.032,.054,(g['thigh']+24)/1000],[
          geometry('capsule',[.012,g['thigh']/1000],[0,0,0],'frame',True,end=[0,0,-g['thigh']/1000]),
          geometry('box',[.035,.025,.031],[0,0,-g['thigh']/1000],'servo')])
        link(leg+'_shin',leg+'_thigh',[0,0,-g['thigh']/1000],leg+'_knee',mass['shin_and_foot'],[0,-side*.004,-g['shin']/1000*.60],[.036,.058,(g['shin']+30)/1000],[
          geometry('capsule',[.010,g['shin']/1000],[0,0,0],'body',True,end=[0,0,-g['shin']/1000]),
          geometry('sphere',[g['foot_radius']/1000],[0,0,-g['shin']/1000],'hoof')])
    link('neck_yaw','torso',mm(g['head_origin']),'head_yaw',mass['head_yaw_without_servo']+s,[0,0,.026],[.060,.064,.060],[geometry('box',[.050,.056,.054],[0,0,.027],'body')])
    link('head','neck_yaw',[0,0,g['neck_pitch_rise']/1000],'head_pitch',mass['head_without_servos']+2*c['ear_actuator']['mass'],[.030,0,.035],[.11,.09,.10],[
      geometry('ellipsoid',mm(g['head_radii']),mm(g['head_center']),'body'),
      geometry('ellipsoid',mm(g['muzzle_radii']),mm(g['muzzle_center']),'body'),
      geometry('ellipsoid',[.025,.017,.028],[.053,.037,.041],'eye',False),
      geometry('ellipsoid',[.025,.017,.028],[.053,-.037,.041],'eye',False),
      geometry('ellipsoid',[.014,.006,.020],[.058,.051,.041],'iris',False),
      geometry('ellipsoid',[.014,.006,.020],[.058,-.051,.041],'iris',False),
      geometry('ellipsoid',[.020,.036,.041],[-.004,0,.055],'mane',False),
      geometry('capsule',[.008,.033],[.048,0,.088],'body',False,end=[.048,0,.121])])
    for name,side in [('ear_left',1),('ear_right',-1)]:
        o=g['ear_origin']; origin=mm([o[0],side*o[1],o[2]])
        link(name,'head',origin,name,mass['ear_without_servo'],[0,side*.004,.040],mm(g['ear_size']),[
          geometry('ellipsoid',[.008,.013,.024],[0,side*.006,.051],'body'),
          geometry('ellipsoid',[.008,.003,.016],[.007,0,.021],'mane',False)])
    return links

def joint_spec(name,c):
    kind='ear' if name.startswith('ear_') else name.removeprefix(name.split('_')[0]+'_') if name.split('_')[0] in c['legs'] else name
    return c['joints'][kind]

def rotation(axis,angle):
    a=np.array(axis,dtype=float);a/=np.linalg.norm(a)
    K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)

def fk(links,q=None,c=None):
    c=c or load();q=q or {}; mats={}
    for l in links:
        t=np.eye(4);t[:3,3]=l['origin']
        if l['joint']:t[:3,:3]=rotation(joint_spec(l['joint'],c)['axis'],q.get(l['joint'],0))
        mats[l['name']]=(mats[l['parent']]@t) if l['parent'] else t
    return mats

def ik(foot,side,c=None):
    c=c or load();g=c['geometry']; x,y,z=foot
    a=side*g['abduction_offset']/1000; l1=g['thigh']/1000;l2=g['shin']/1000
    h2=y*y+z*z-a*a
    if h2<=0:raise ValueError('Foot inside roll-offset cylinder')
    htotal=math.sqrt(h2);roll=math.atan2(y,-z)-math.atan2(a,htotal);h=htotal-g['hip_drop']/1000
    v=(x*x+h*h-l1*l1-l2*l2)/(2*l1*l2)
    if not -1-1e-8<=v<=1+1e-8:raise ValueError(f'Unreachable foot {foot}: cosine={v}')
    knee=math.acos(float(np.clip(v,-1,1)));hip=math.atan2(-x,h)-math.atan2(l2*math.sin(knee),l1+l2*math.cos(knee))
    return np.array([roll,hip,knee])

def home(c=None):
    c=c or load();g=c['geometry'];q={n:0.0 for n in joint_names(c)}
    for leg,s in c['legs'].items():
        qs=ik([0,s['y_sign']*g['abduction_offset']/1000,-g['stance_height']/1000],s['y_sign'],c)
        q.update(zip([leg+'_'+j for j in ('hip_roll','hip_pitch','knee')],qs))
    return q

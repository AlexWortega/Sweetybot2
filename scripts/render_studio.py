"""Render the saved CAD GLB in Blender without changing robot geometry.

Example: blender -b --factory-startup --python scripts/render_studio.py -- --preview
Full set: blender -b --factory-startup --python scripts/render_studio.py
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]


def point_at(obj, point):
    obj.rotation_euler = (Vector(point)-obj.location).to_track_quat('-Z','Y').to_euler()


def material(name, color, roughness=.3, metallic=0.):
    m=bpy.data.materials.new(name);m.use_nodes=True
    p=m.node_tree.nodes.get('Principled BSDF')
    p.inputs['Base Color'].default_value=(*color,1)
    p.inputs['Roughness'].default_value=roughness
    p.inputs['Metallic'].default_value=metallic
    return m


def area(name, location, target, power, color, size, size_y=None):
    data=bpy.data.lights.new(name,'AREA');data.energy=power;data.color=color
    data.shape='RECTANGLE';data.size=size;data.size_y=size if size_y is None else size_y
    obj=bpy.data.objects.new(name,data);bpy.context.collection.objects.link(obj)
    obj.location=location;point_at(obj,target);return obj


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--glb',type=Path,default=ROOT/'cad/GLB/assembly.glb')
    parser.add_argument('--out',type=Path,default=ROOT/'renders/studio')
    parser.add_argument('--preview',action='store_true')
    parser.add_argument('--camera-manifest',type=Path,help='Lock framing and lighting placement to an earlier studio manifest')
    parser.add_argument('--shots',nargs='+',choices=['01-midnight-hero','02-portrait','03-violet-profile'])
    parser.add_argument('--samples',type=int,default=96)
    argv=sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else []
    args=parser.parse_args(argv)
    if not args.glb.is_file():raise FileNotFoundError('Extract the complete CAD release first: '+str(args.glb))
    args.out.mkdir(parents=True,exist_ok=True)
    bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.gltf(filepath=str(args.glb))
    robot=[o for o in bpy.context.scene.objects if o.type=='MESH']
    vertices=sum(len(o.data.vertices) for o in robot)
    corners=[o.matrix_world@Vector(c) for o in robot for c in o.bound_box]
    lower=Vector([min(p[i] for p in corners) for i in range(3)])
    upper=Vector([max(p[i] for p in corners) for i in range(3)])
    reference=json.loads(args.camera_manifest.read_text()) if args.camera_manifest else None
    frame_lower=Vector(reference['original_bounds_m']['min']) if reference else lower
    frame_upper=Vector(reference['original_bounds_m']['max']) if reference else upper
    center=(frame_upper+frame_lower)/2
    print('STUDIO_BOUNDS',list(lower),list(upper),'VERTICES',vertices,flush=True)
    # Preserve imported mesh coordinates, normals, component placement and colors.
    for m in {m for o in robot for m in o.data.materials if m is not None}:
        if not m.use_nodes:continue
        p=next((n for n in m.node_tree.nodes if n.type=='BSDF_PRINCIPLED'),None)
        if not p:continue
        r,g,b,_=p.inputs['Base Color'].default_value
        value=max(r,g,b)
        if b>.95 and r<.01 and g<.01:
            # Vendor CAD face-color blue is an assembly marker, not a body finish.
            p.inputs['Base Color'].default_value=(.018,.025,.035,1)
            p.inputs['Roughness'].default_value=.32
            p.inputs['Metallic'].default_value=.12
        elif b>r*1.5 and .05<value<.25:
            # v03 optics, pupils and pigment use a deep neutral black finish.
            p.inputs['Base Color'].default_value=(.010,.012,.020,1)
            p.inputs['Roughness'].default_value=.22
            p.inputs['Coat Weight'].default_value=.3
        elif value>.8 and min(r,g,b)>.5:
            p.inputs['Roughness'].default_value=.25
            p.inputs['Coat Weight'].default_value=.24
            p.inputs['Coat Roughness'].default_value=.19
        elif g>r*1.5 and g>b*1.4:
            p.inputs['Roughness'].default_value=.20
            p.inputs['Emission Color'].default_value=(r,g,b,1)
            p.inputs['Emission Strength'].default_value=.45
        elif abs(r-g)<.12 and abs(g-b)<.12 and value<.5:
            p.inputs['Metallic'].default_value=.65
            p.inputs['Roughness'].default_value=.30
        else:
            p.inputs['Roughness'].default_value=.28
            p.inputs['Coat Weight'].default_value=.3
    scene=bpy.context.scene;scene.render.engine='CYCLES'
    scene.cycles.samples=32 if args.preview else args.samples
    scene.cycles.use_denoising=True
    scene.cycles.adaptive_threshold=.025
    prefs=bpy.context.preferences.addons['cycles'].preferences
    try:
        prefs.compute_device_type='METAL' if sys.platform=='darwin' else 'CUDA'
        prefs.get_devices()
        accelerators=[d for d in prefs.devices if d.type!='CPU']
        if accelerators:
            for d in prefs.devices:d.use=d.type!='CPU'
            scene.cycles.device='GPU'
    except (TypeError,RuntimeError):pass
    scene.render.resolution_x=960 if args.preview else 1920
    scene.render.resolution_y=720 if args.preview else 1440
    scene.render.resolution_percentage=100
    scene.render.image_settings.file_format='PNG'
    scene.render.image_settings.color_mode='RGB'
    scene.view_settings.view_transform='AgX'
    scene.view_settings.look='AgX - Medium High Contrast'
    scene.view_settings.exposure=-.45
    world=bpy.data.worlds.new('Studio ambient');world.use_nodes=True
    world.node_tree.nodes['Background'].inputs[0].default_value=(.055,.065,.095,1)
    world.node_tree.nodes['Background'].inputs[1].default_value=.22
    scene.world=world
    # A low display plinth is scenery only, below the unchanged saved CAD feet.
    ground=float(lower.z)
    floor_mat=material('Graphite cyclorama',(.012,.016,.025),.32,.18)
    bpy.ops.mesh.primitive_plane_add(size=200,location=(float(center.x),float(center.y),ground-.026))
    floor=bpy.context.object;floor.name='Studio cyclorama';floor.data.materials.append(floor_mat)
    bpy.ops.mesh.primitive_cylinder_add(vertices=128,radius=.255,depth=.026,location=(float(center.x),float(center.y),ground-.013))
    plinth=bpy.context.object;plinth.name='Display plinth';plinth.data.materials.append(material('Satin charcoal plinth',(.021,.025,.040),.26,.38))
    bevel=plinth.modifiers.new('Soft studio edge','BEVEL');bevel.width=.004;bevel.segments=4
    for p in plinth.data.polygons:p.use_smooth=True
    target=Vector((center.x,center.y,frame_lower.z+(frame_upper.z-frame_lower.z)*.5))
    camera_data=bpy.data.cameras.new('Product camera');camera=bpy.data.objects.new('Product camera',camera_data)
    bpy.context.collection.objects.link(camera);scene.camera=camera
    camera_data.lens=70;camera_data.clip_start=.01;camera_data.clip_end=250
    key=area('Large ivory softbox',(center.x+.45,center.y-.5,.95),target,26,(1.,.88,.79),.6,.9)
    rim=area('Lilac rim',(center.x-.45,center.y+.25,.62),target,42,(.50,.30,1.),.35,.65)
    fill=area('Cool fill',(center.x+.2,center.y+.55,.48),target,8,(.48,.72,1.),.45,.65)
    top=area('Top strip',(center.x-.1,center.y,.98),target,10,(1.,.92,.85),.28,.85)
    front=area('Eye catchlight',(center.x+.7,center.y-.2,.45),target,3,(1.,1.,1.),.15,.5)
    shots=[('01-midnight-hero',Vector((1.,-1.22,.54)),1.46,target.copy(),70),
           ('02-portrait',Vector((1.,-.88,.20)),.91,Vector((center.x+.073,center.y,frame_upper.z-.105)),82),
           ('03-violet-profile',Vector((.15,-1.35,.30)),1.46,target.copy(),70)]
    if args.shots:shots=[shot for shot in shots if shot[0] in args.shots]
    if args.preview:shots=shots[:1]
    manifest={'renderer':bpy.app.version_string,'source':'cad/GLB/assembly.glb','source_sha256':hashlib.sha256(args.glb.read_bytes()).hexdigest(),'robot_mesh_objects':len(robot),'robot_vertices':vertices,'original_bounds_m':{'min':list(lower),'max':list(upper)},'robot_geometry_unchanged':True,'changes':'camera, stage, lighting and surface finishes; vendor CAD blue markers styled graphite, v03 pupils/optics styled deep black; native robot geometry preserved','shots':[]}
    for name,direction,distance,look,lens in shots:
        camera.location=look+direction.normalized()*distance;camera_data.lens=lens
        if reference:
            saved=next(row for row in reference['shots'] if Path(row['file']).stem==name)
            camera.location=Vector(saved['camera_location_m']);look=Vector(saved['camera_target_m']);lens=saved['lens_mm'];camera_data.lens=lens
        point_at(camera,look)
        scene.render.filepath=str(args.out/(('preview' if args.preview else name)+'.png'))
        print('STUDIO_RENDER',name,scene.render.filepath,flush=True)
        bpy.ops.render.render(write_still=True)
        manifest['shots'].append({'file':Path(scene.render.filepath).name,'camera_location_m':list(camera.location),'camera_target_m':list(look),'lens_mm':lens,'resolution':[scene.render.resolution_x,scene.render.resolution_y],'samples':scene.cycles.samples})
    (args.out/('preview-manifest.json' if args.preview else 'manifest.json')).write_text(json.dumps(manifest,indent=2)+'\n')


if __name__=='__main__':main()

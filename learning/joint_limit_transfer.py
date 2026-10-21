"""Opt-in actor warm start for a mechanically revalidated joint-limit change.

Normal resume and teacher loading remain strict. No optimizer or critic is
transferred. All physical inputs except roll limits and their mirroring must
be identical to the verified donor source.
"""
import ast
import copy
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

from learning.readiness import PHYSICAL_FILES, ROOT, require_training_ready


MIRROR = """if kind=='hip_roll' and spec.get('mirror_limits_by_side') and c['legs'][name.split('_')[0]]['y_sign']<0:
    spec['limits']=[-spec['limits'][1],-spec['limits'][0]]
"""


def _model_ast(path):
    tree=ast.parse(path.read_text())
    allowed=ast.dump(ast.parse(MIRROR).body[0])
    for node in tree.body:
        if isinstance(node,ast.FunctionDef) and node.name=='joint_spec':
            node.body=[x for x in node.body if ast.dump(x)!=allowed]
    return ast.dump(tree)


def _config_without_limits(c):
    c=copy.deepcopy(c)
    c['joints']['hip_roll'].pop('limits')
    c['joints']['hip_roll'].pop('mirror_limits_by_side',None)
    return c


def _xml_without_roll_ranges(path,c):
    tree=ET.parse(path)
    names={leg+'_hip_roll':side for leg,side in c['legs'].items()}
    found=set()
    for node in tree.iter('joint'):
        name=node.get('name')
        if name in names:
            limits=c['joints']['hip_roll']['limits']
            if c['joints']['hip_roll'].get('mirror_limits_by_side') and names[name]['y_sign']<0:
                limits=[-limits[1],-limits[0]]
            if not np.allclose(list(map(float,node.attrib.pop('range').split())),limits,atol=1e-6,rtol=0):
                raise ValueError('XML roll limits disagree with configuration')
            found.add(name)
    if found!=set(names):raise ValueError('Missing roll joints')
    return ET.tostring(tree.getroot())


def validate_limit_change(source,target=ROOT):
    source,target=Path(source),Path(target)
    old=yaml.safe_load((source/'config/robot.yaml').read_text())
    new=yaml.safe_load((target/'config/robot.yaml').read_text())
    if _config_without_limits(old)!=_config_without_limits(new):
        raise ValueError('Joint-limit transfer changed other robot configuration')
    if _model_ast(source/'simulation/model.py')!=_model_ast(target/'simulation/model.py'):
        raise ValueError('Joint-limit transfer changed kinematics beyond limit mirroring')
    if _xml_without_roll_ranges(source/'simulation/generated/sweetie_mini.xml',old)!=_xml_without_roll_ranges(target/'simulation/generated/sweetie_mini.xml',new):
        raise ValueError('Joint-limit transfer changed other MuJoCo physics')
    allowed={'config/robot.yaml','simulation/model.py','simulation/generated/sweetie_mini.xml'}
    for name in set(PHYSICAL_FILES)-allowed:
        if (source/name).read_bytes()!=(target/name).read_bytes():
            raise ValueError('Joint-limit transfer changed physical input: '+name)
    import json
    manifest=json.loads((target/'reports/collision_meshes.json').read_text())
    for row in manifest['parts']:
        for name in row['hulls']:
            if (source/name).read_bytes()!=(target/name).read_bytes():
                raise ValueError('Joint-limit transfer changed collision geometry: '+name)
    if target.resolve()!=ROOT.resolve():raise ValueError('Transfer target must be the active validated source')
    require_training_ready()
    gate=target/'reports/mechanical_gate.json'
    return dict(mode='explicit_audited_hip_roll_limit_transfer',
                previous_canonical_limits=old['joints']['hip_roll']['limits'],
                next_canonical_limits=new['joints']['hip_roll']['limits'],
                mechanical_gate_sha256=hashlib.sha256(gate.read_bytes()).hexdigest(),
                other_physics_identical=True)

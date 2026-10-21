"""Mirrored limits must agree at simulation and command boundaries."""
import json
import math
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import yaml

from scripts.common import ROOT,load
from simulation.model import home,joint_spec
from simulation.control_contract import JointCommand,validate


def test_wide_roll_limits_match_mjcf_urdf_and_command_gate():
    c=load();model=mujoco.MjModel.from_xml_path(str(ROOT/'simulation/generated/sweetie_mini.xml'))
    urdf=ET.parse(ROOT/'simulation/generated/sweetie_mini.urdf')
    for leg,side in c['legs'].items():
        name=leg+'_hip_roll';expected=np.deg2rad([-6,45] if side['y_sign']>0 else [-45,6])
        np.testing.assert_allclose(joint_spec(name,c)['limits'],expected)
        np.testing.assert_allclose(model.joint(name).range,expected,atol=1e-6)
        limit=urdf.find(f"joint[@name='{name}']/limit")
        np.testing.assert_allclose([float(limit.get(k)) for k in ('lower','upper')],expected)
        for value in expected:validate(JointCommand(1,1.,{**home(c),name:float(value)}),c)
        for value in (expected[0]-.01,expected[1]+.01):
            with pytest.raises(ValueError):validate(JointCommand(1,1.,{**home(c),name:float(value)}),c)


@pytest.fixture
def narrow_source(tmp_path):
    from learning.readiness import PHYSICAL_FILES
    import os
    source=tmp_path/'old'
    manifest=json.loads((ROOT/'reports/collision_meshes.json').read_text())
    files=set(PHYSICAL_FILES)|{f for row in manifest['parts'] for f in row['hulls']}
    for name in files:
        dest=source/name;dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,dest)
    path=source/'config/robot.yaml';c=yaml.safe_load(path.read_text())
    c['joints']['hip_roll'].pop('mirror_limits_by_side');c['joints']['hip_roll']['limits']=[-.15,.15]
    path.write_text(yaml.safe_dump(c))
    path=source/'simulation/model.py'
    path.write_text(path.read_text().replace("    if kind=='hip_roll' and spec.get('mirror_limits_by_side') and c['legs'][name.split('_')[0]]['y_sign']<0:\n        spec['limits']=[-spec['limits'][1],-spec['limits'][0]]\n",''))
    path=source/'simulation/generated/sweetie_mini.xml';tree=ET.parse(path)
    for node in tree.iter('joint'):
        if node.get('name','').endswith('hip_roll'):node.set('range','-.15 .15')
    tree.write(path,encoding='unicode')
    return source


def test_explicit_limit_transfer_accepts_only_limit_change(narrow_source,monkeypatch):
    import learning.joint_limit_transfer as transfer
    called=[];monkeypatch.setattr(transfer,'require_training_ready',lambda:called.append(True))
    report=transfer.validate_limit_change(narrow_source)
    assert report['other_physics_identical'] and called==[True]
    assert report['previous_canonical_limits']==[-.15,.15]
    path=narrow_source/'config/robot.yaml';c=yaml.safe_load(path.read_text());c['geometry']['hip_y']+=1
    path.write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError,match='other robot configuration'):transfer.validate_limit_change(narrow_source)


def test_limit_transfer_rejects_motor_and_kinematics_changes(narrow_source,monkeypatch):
    import learning.joint_limit_transfer as transfer
    monkeypatch.setattr(transfer,'require_training_ready',lambda:None)
    path=narrow_source/'learning/native.cpp';original=path.read_bytes();path.write_bytes(original+b'\n// changed\n')
    with pytest.raises(ValueError,match='physical input'):transfer.validate_limit_change(narrow_source)
    path.write_bytes(original)
    path=narrow_source/'simulation/model.py';path.write_text(path.read_text().replace("side*g['hip_y']/1000","side*g['hip_y']/999"))
    with pytest.raises(ValueError,match='kinematics'):transfer.validate_limit_change(narrow_source)

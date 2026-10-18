"""Bad mechanics must stop training before allocating GPU or making checkpoints."""
import pytest
from learning.readiness import dynamics_signature,require_training_ready

def test_bad_or_unvalidated_mechanics_block_training(monkeypatch):
 import learning.readiness as readiness
 def failed():raise ValueError("mechanical failure")
 monkeypatch.setattr(readiness,"require_mechanical_gate",failed)
 with pytest.raises(ValueError):require_training_ready()

def test_robot_train_does_not_touch_cuda_when_mechanics_failed(monkeypatch,tmp_path):
 import learning.readiness as readiness
 def failed():raise ValueError('mechanical failure')
 monkeypatch.setattr(readiness,'require_mechanical_gate',failed)
 from learning.ppo import train,TrainConfig
 import torch
 class E:observation_contract='v05-sensor-suite-1'
 def forbidden(*a,**k):raise AssertionError('CUDA touched before mechanical gate')
 monkeypatch.setattr(torch.cuda,'get_device_properties',forbidden)
 with pytest.raises(ValueError):train(E(),TrainConfig(out=str(tmp_path/'must-not-exist'),device='cuda:0'))
 assert not (tmp_path/'must-not-exist').exists()

def test_same_observation_size_does_not_allow_old_dynamics():
 from learning.ppo import validate_environment_contract
 class E:observation_contract='v05-sensor-suite-1'
 with pytest.raises(ValueError,match='dynamics'):
  validate_environment_contract({'environment_metadata':{'observation_contract':E.observation_contract,'dynamics_signature':'old'}},E())
 validate_environment_contract({'environment_metadata':{'observation_contract':E.observation_contract,'dynamics_signature':dynamics_signature()}},E())

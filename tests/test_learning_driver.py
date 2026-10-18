"""Budget isolation and task-metric promotion tests without GPU allocation."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from learning.driver import promote, regresses, run_child, score_development, select_gpu


def test_stationary_survivor_does_not_pass_locomotion_gate():
    metric = {"self_contact_fraction":0.,"survival_fraction": 1., "linear_command_rmse": .1,
              "yaw_command_rmse": .1, "rms_joint_torque_nm": [.4]*16, "continuous_torque_limits_nm": [1.5691]*16,
              "per_command": {name: {"measured_speed_mps": 0., "command_speed_mps": .2,
                                     "survival_fraction": 1.}
                              for name in ("stand", "forward", "fast_forward", "backward", "sideways", "turn")}}
    score, qualified, _ = score_development("locomotion", [metric, metric])
    assert score > 0
    assert not qualified
    metric["per_command"]["forward"]["measured_speed_mps"] = .15
    assert score_development("locomotion", [metric])[1]


def test_recovery_gate_requires_each_orientation_and_atomic_promotion(tmp_path):
    metric = {"self_contact_fraction":0.,"recovery_success_rate": .9, "rms_joint_torque_nm": [.5]*16, "continuous_torque_limits_nm": [1.5691]*16,
              "per_orientation": {pose: {"success_rate": .9} for pose in ("folded_crouch", "left", "right", "supine")}}
    assert score_development("recovery", [metric])[1]
    metric["per_orientation"]["supine"]["success_rate"] = 0.
    assert not score_development("recovery", [metric])[1]
    checkpoint = tmp_path / "candidate.pt"
    checkpoint.write_bytes(b"test")
    link = tmp_path / "walk.pt"
    promote(checkpoint, link)
    assert link.resolve() == checkpoint.resolve()


def test_budget_timeout_stops_only_own_child(tmp_path):
    unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(10)"])
    try:
        result = run_child([sys.executable, "-c", "import time; time.sleep(10)"],
                           tmp_path / "child.log", time.time()+3, .1, dict(os.environ))
        assert result["status"] == "timed_out"
        assert result["wall_seconds"] < 2
        assert unrelated.poll() is None
        exhausted = run_child([sys.executable, "-c", "raise RuntimeError('must not run')"],
                              tmp_path / "never.log", time.time()-1, 10, dict(os.environ))
        assert exhausted["status"] == "budget_exhausted"
        assert not (tmp_path / "never.log").exists()
    finally:
        unrelated.terminate()
        unrelated.wait(timeout=3)


def test_resume_rejects_metric_regression_and_quality_gate_loss():
    champion = {"score": 1.0, "quality_gate_passed": True}
    assert regresses(.89, True, champion)
    assert regresses(1.2, False, champion)  # higher aggregate cannot erase safety gate
    assert not regresses(.95, True, champion)
    assert not regresses(1.2, True, champion)
    assert not regresses(-100., False, None)


def test_torque_screen_is_required_for_quality_but_does_not_change_rank():
    metric = {"self_contact_fraction":0.,"recovery_success_rate": 1.,
              "per_orientation": {pose: {"success_rate": 1.} for pose in ("folded_crouch", "left", "right", "supine")}}
    score_missing, qualified, aggregate = score_development("recovery", [metric])
    assert not qualified
    assert not aggregate["torque_gate_passed"]
    assert aggregate["max_observed_rms_joint_torque_nm"] is None
    metric["continuous_torque_limits_nm"] = [1.5691]*16
    metric["rms_joint_torque_nm"] = [.4]*15 + [1.5691]
    score_good, qualified, aggregate = score_development("recovery", [metric])
    assert qualified
    assert aggregate["torque_gate_passed"]
    metric["rms_joint_torque_nm"][-1] = 1.57
    score_bad, qualified, aggregate = score_development("recovery", [metric])
    assert not qualified
    assert not aggregate["torque_gate_passed"]
    assert score_missing == score_good == score_bad


def test_gpu_capacity_prefers_designated_then_alternate_then_cpu():
    inventory = [
        {"index": "1", "memory_free_mib": 23., "utilization_percent": 0.},
        {"index": "2", "memory_free_mib": 4000., "utilization_percent": 20.},
        {"index": "3", "memory_free_mib": 6363., "utilization_percent": 0.},
    ]
    chosen = select_gpu(inventory, "3")
    assert chosen["physical_gpu"] == "3"
    assert chosen["reason"] == "preferred_gpu_has_capacity"
    alternate = select_gpu(inventory, "1")
    assert alternate["physical_gpu"] == "3"
    assert alternate["reason"] == "alternate_gpu_has_capacity"
    inventory[2]["utilization_percent"] = 20.
    assert select_gpu(inventory, "1")["physical_gpu"] == "3"  # tie: more free memory
    for gpu in inventory:
        gpu["memory_free_mib"] = 3071.
    assert select_gpu(inventory, "1")["device"] == "cpu"
    assert select_gpu([], "3")["device"] == "cpu"


def test_walk_quality_requires_yaw_tracking_and_every_command_survival():
    metric = {"self_contact_fraction":0.,"survival_fraction": 1., "linear_command_rmse": .1,
              "yaw_command_rmse": .6, "rms_joint_torque_nm": [.4]*16, "continuous_torque_limits_nm": [1.5691]*16,
              "per_command": {name: {"measured_speed_mps": .15, "command_speed_mps": .2,
                                     "survival_fraction": 1.}
                              for name in ("stand", "forward", "fast_forward", "backward", "sideways", "turn")}}
    assert not score_development("locomotion", [metric])[1]
    metric["yaw_command_rmse"] = .5
    assert score_development("locomotion", [metric])[1]
    metric["per_command"]["sideways"]["survival_fraction"] = .59
    assert not score_development("locomotion", [metric])[1]
    del metric["per_command"]["sideways"]["survival_fraction"]
    assert not score_development("locomotion", [metric])[1]
    del metric["per_command"]["sideways"]
    assert not score_development("locomotion", [metric])[1]

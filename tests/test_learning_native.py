"""Physical invariants for the optional native training backend."""
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
import pytest

native = pytest.importorskip("learning._sweetie_native")
ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture()
def physics_xml(tmp_path):
    root = ET.parse(ROOT / "simulation/generated/sweetie_mini.xml").getroot()
    for mesh in list(root.find("asset")):
        root.find("asset").remove(mesh)
    for body in root.iter("body"):
        for geom in list(body.findall("geom")):
            if geom.get("type") == "mesh":
                body.remove(geom)
    path = tmp_path / "physics.xml"
    path.write_text(ET.tostring(root, encoding="unicode"))
    return path


def reset(batch, qpos=None, qvel=None, params=None):
    state = batch.state()
    qpos = state["qpos"] if qpos is None else qpos
    qvel = state["qvel"] if qvel is None else qvel
    params = np.tile([1, .9, 8.7, 1, 1, .0174533, 1, 1], (batch.num_envs, 1)) if params is None else params
    batch.reset(np.arange(batch.num_envs), qpos, qvel, params)
    return qpos[:, 7:].copy()


def test_native_determinism_delay_and_reset(physics_xml):
    batch = native.Batch(str(physics_xml), 2, 2)
    initial = batch.state()
    target = reset(batch)
    sequence = np.random.default_rng(42).normal(0, .02, (50, 2, 16))
    sequence[:, 1] = sequence[:, 0]
    for action in sequence:
        batch.step(target + action)
    final = batch.state()
    np.testing.assert_array_equal(final["qpos"][0], final["qpos"][1])
    reset(batch, initial["qpos"], initial["qvel"])
    for action in sequence:
        batch.step(target + action)
    np.testing.assert_array_equal(batch.state()["qpos"], final["qpos"])
    assert np.all(final["torque_rms"] <= 4.9033 * 8.7 / 12 * .55 + 1e-12)
    assert np.all(final["torque_peak"] + 1e-12 >= final["torque_rms"])
    assert np.all(final["torque_peak"] + 1e-12 >= np.abs(final["torque"]))


def test_native_neutral_stand_and_no_root_support(physics_xml):
    batch = native.Batch(str(physics_xml), 1, 1)
    target = reset(batch)
    for _ in range(1000):
        batch.step(target)
    state = batch.state()
    assert .18 < state["qpos"][0, 2] < .21
    assert np.all(state["contacts"])
    assert not state["bad_contacts"].any()
    assert np.isfinite(state["qpos"]).all()
    qpos = state["qpos"].copy()
    qpos[:, 2] = 1.0
    target = reset(batch, qpos, np.zeros((1, batch.nv)))
    for _ in range(10):
        batch.step(target)
    state = batch.state()
    assert state["qpos"][0, 2] < .82  # Free fall, not supported by hidden root forces.
    assert state["qvel"][0, 2] < -1.8


def test_native_matches_independent_python_servo_including_braking(physics_xml):
    batch = native.Batch(str(physics_xml), 1, 1)
    qpos = batch.state()["qpos"]
    qvel = np.zeros((1, batch.nv))
    qvel[0, -1] = 20.0  # Overspeed must still receive braking torque.
    params = np.array([[1.2, .55, 8.7, .9, .95, .01, 0, 1.1]])
    target = reset(batch, qpos, qvel, params)
    target[0, -1] = .2
    model = mujoco.MjModel.from_xml_path(str(physics_xml))
    data = mujoco.MjData(model)
    model.body_mass[1:] *= params[0, 0]
    model.body_inertia[1:] *= params[0, 0]
    model.geom_friction[:, 0] = params[0, 1]
    mujoco.mj_resetData(model, data)
    mujoco.mj_setConst(model, data)
    data.qpos[:] = qpos[0]
    data.qvel[:] = qvel[0]
    mujoco.mj_forward(model, data)
    kp = np.array([10.] * 12 + [2, 4, .5, .5]) * params[0, 7]
    kd = np.array([.12] * 12 + [.06, .08, .01, .01]) * params[0, 7]
    stall = 4.9033 * params[0, 2] / 12 * params[0, 3]
    omega = 7.854 * params[0, 2] / 12 * params[0, 4]
    cap = .55 * stall
    first_torque = None
    for _ in range(10):
        error = target[0] - data.qpos[7:]
        error = np.sign(error) * np.maximum(0, np.abs(error) - params[0, 5])
        velocity = data.qvel[6:]
        low = np.clip(-stall * (1 + velocity / omega), -cap, cap)
        high = np.clip(stall * (1 - velocity / omega), -cap, cap)
        data.ctrl[:] = np.clip(kp * error - kd * velocity, low, high)
        if first_torque is None:
            first_torque = data.ctrl.copy()
        mujoco.mj_step(model, data)
    assert first_torque[-1] == pytest.approx(-cap)
    batch.step(target)
    state = batch.state()
    assert state["torque_peak"][0, -1] == pytest.approx(cap)
    np.testing.assert_allclose(state["qpos"][0], data.qpos, atol=1e-12, rtol=1e-12)
    np.testing.assert_allclose(state["qvel"][0], data.qvel, atol=1e-12, rtol=1e-12)


def test_native_rejects_invalid_resets_and_targets(physics_xml):
    batch = native.Batch(str(physics_xml), 1, 1)
    target = reset(batch)
    with pytest.raises(ValueError):
        batch.step(np.full((1, 16), np.nan))
    with pytest.raises(ValueError):
        batch.reset(np.array([0, 0]), np.tile(batch.state()["qpos"], (2, 1)), np.zeros((2, batch.nv)), np.tile([1, .9, 8.7, 1, 1, 0, 0, 1], (2, 1)))
    batch.step(target)
    assert np.isfinite(batch.state()["qpos"]).all()

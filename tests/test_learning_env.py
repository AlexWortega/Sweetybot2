"""Training model physics, reset validity and actor observation boundaries."""
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
import pytest
from scripts.common import ROOT
from simulation.model import graph

pytest.importorskip("learning._sweetie_native")
from learning.env import SweetieVecEnv, make_model


@pytest.mark.parametrize("terrain", ["flat", "rough", "slope", "steps"])
def test_learning_model_preserves_inertials_and_collisions(terrain):
    path, _ = make_model(terrain, 1., 2000001)
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nq == 23 and model.nv == 22 and model.nu == 16
    for link in graph():
        body = model.body(link["name"])
        assert body.mass[0] == pytest.approx(link["mass"], abs=1e-12)
        np.testing.assert_allclose(np.sort(body.inertia),np.linalg.eigvalsh(link["inertia_matrix"]), atol=1e-12)
        np.testing.assert_allclose(body.ipos, link["com"], atol=1e-12)
    source = ET.parse(ROOT / "simulation/generated/sweetie_mini.xml").getroot()
    trained = ET.parse(path).getroot()
    for geom in source.iter("geom"):
        if not geom.get("name") or geom.get("type") == "mesh" or geom.get("name") == "floor":
            continue
        name = geom.get("name")

        exported = trained.find(f".//geom[@name='{name}']")
        assert exported is not None
        for attribute in ("type", "size", "pos", "fromto"):
            assert geom.get(attribute) == exported.get(attribute)
        if geom.get("contype", "1") != "0":
            assert exported.get("contype", "1") != "0"
    # The real horn/brackets, retained as CAD-derived convex collision meshes.
    horn_ids=[i for i in range(model.ngeom) if 'unicorn_horn' in (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) or '')]
    assert horn_ids and all(model.geom_type[i]==mujoco.mjtGeom.mjGEOM_MESH for i in horn_ids)
    assert all(model.geom_contype[i]==4 and model.geom_conaffinity[i]==5 for i in horn_ids)
    assert model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
    assert trained.find('asset/mesh') is not None
    for mesh in source.findall('asset/mesh'):
        if not mesh.get('name').endswith('_visual'):
            assert trained.find(f"asset/mesh[@name='{mesh.get('name')}']") is not None



@pytest.mark.parametrize("terrain", ["flat", "rough", "slope", "steps"])
def test_learning_all_reset_poses_clear_terrain_and_neutral_stands(terrain):
    env = SweetieVecEnv(num_envs=1, threads=1, randomize=False,
                        auto_reset=False, terrain=terrain, difficulty=1., seed=2000001)
    model = mujoco.MjModel.from_xml_path(str(env.model_path))
    data = mujoco.MjData(model)
    for pose in ("stand", "folded_crouch", "left", "right", "supine"):
        env.reset_pose(pose)
        data.qpos[:] = env.state["qpos"][0]
        mujoco.mj_forward(model, data)
        for contact in data.contact:
            one_world = (model.geom_bodyid[contact.geom1] == 0) != (model.geom_bodyid[contact.geom2] == 0)
            if one_world:
                assert contact.dist >= -.0000011, (terrain, pose, contact.dist)
    env.reset_pose("stand")
    env.set_command(0, 0, 0)
    for _ in range(150):
        observation, reward, done, _ = env.step(np.zeros((1, 12)))
    assert observation.shape == (1, 591)
    assert np.isfinite(observation).all() and np.isfinite(reward).all()
    assert not done[0]
    assert env.rot[0, 2, 2] > .9
    assert env.height[0] > .16


def test_actor_imu_frames_and_root_translation_velocity_exclusion():
    env = SweetieVecEnv(num_envs=1, threads=1, randomize=False, auto_reset=False)
    qpos = env.state["qpos"].copy()
    qvel = env.state["qvel"].copy()
    axis = np.array([.3, -.4, .7])
    axis /= np.linalg.norm(axis)
    quaternion = np.empty(4)
    mujoco.mju_axisAngle2Quat(quaternion, axis, 1.1)
    qpos[0, 3:7] = quaternion
    qpos[0, 2] = .6
    qvel[0, 3:6] = [.4, -.2, .7]
    env.native.reset(np.array([0]), qpos, qvel, env.params)
    env._read_state()
    env.sensors.reset([0],quaternion[None]);env.sensors.gyro[0]=qvel[0,3:6]
    frame = env._frame().copy()
    model = mujoco.MjModel.from_xml_path(str(env.model_path))
    data = mujoco.MjData(model)
    data.qpos[:] = qpos[0]
    data.qvel[:] = qvel[0]
    mujoco.mj_forward(model, data)
    velocity = np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE,
                            model.site("torso_imu").id, velocity, 1)
    rotation = data.xmat[model.body("torso").id].reshape(3, 3)
    np.testing.assert_allclose(env.local_angular[0], velocity[:3], atol=1e-12)
    np.testing.assert_allclose(frame[0, :3], rotation.T @ [0, 0, -1], atol=1e-7)
    np.testing.assert_allclose(frame[0, 3:6], velocity[:3] * .25, atol=1e-7)
    qpos[0, :3] = [1.1, -.7, 1.4]
    qvel[0, :3] = [3., -2., 1.]
    env.native.reset(np.array([0]), qpos, qvel, env.params)
    env._read_state()
    np.testing.assert_array_equal(env._frame(), frame)


def test_resampled_command_is_visible_before_next_action():
    env = SweetieVecEnv(num_envs=1, threads=1, randomize=False, auto_reset=True)
    env.steps[:] = 199
    env.commands[:] = 0
    def replace_command(ids):
        env.commands[ids] = [.3, .05, .4]
    env._sample_commands = replace_command
    observation, _, done, _ = env.step(np.zeros((1, 12)))
    assert not done[0]
    np.testing.assert_allclose(observation.reshape(1, 3, 197)[0, -1, 50:53], [.3, .05, .4])


@pytest.mark.parametrize("terrain", ["flat", "rough", "slope", "steps"])
def test_randomized_fallen_initializations_clear_actual_terrain(terrain):
    env = SweetieVecEnv(task="recovery", num_envs=16, threads=1,
                        randomize=True, auto_reset=True, terrain=terrain,
                        difficulty=1., seed=2000007)
    for pose in ("folded_crouch", "left", "right", "supine"):
        env.reset_pose(pose)
        qpos = env.state["qpos"]
        assert np.isfinite(qpos).all()
        np.testing.assert_allclose(np.linalg.norm(qpos[:, 3:7], axis=1), 1, atol=1e-12)
        assert np.all(qpos[:, 7:] >= env.low - 1e-12)
        assert np.all(qpos[:, 7:] <= env.high + 1e-12)
        if pose != "folded_crouch":
            assert np.max(np.std(qpos[:, 3:7], axis=0)) > .1
        for row in qpos:
            env._reset_data.qpos[:] = row
            mujoco.mj_forward(env._reset_model, env._reset_data)
            for contact in env._reset_data.contact:
                bodies = env._reset_model.geom_bodyid
                if (bodies[contact.geom1] == 0) != (bodies[contact.geom2] == 0):
                    assert contact.dist >= -.0000011


@pytest.mark.parametrize("randomize,auto_reset,progress", [(False, True, 1.), (True, False, 1.), (True, True, .2)])
def test_automatic_disturbances_disabled_outside_randomized_training(randomize, auto_reset, progress):
    env = SweetieVecEnv(num_envs=2, threads=1, randomize=randomize, auto_reset=auto_reset)
    env.set_progress(progress)
    env.steps[:] = 250
    env.next_push_step[:] = 200
    before = env.native.state()["qvel"].copy()
    env._training_disturbances()
    np.testing.assert_array_equal(env.native.state()["qvel"], before)
    assert env.total_disturbances == 0 and not env.events


def test_training_kicks_are_bounded_logged_and_have_no_vertical_support():
    env = SweetieVecEnv(num_envs=16, threads=1, randomize=True, auto_reset=True)
    env.set_progress(1.)
    env.steps[:] = 250
    env.next_push_step[:8] = 200
    env.next_push_step[8:] = 1000
    before = env.native.state()["qvel"].copy()
    env._training_disturbances()
    delta = env.native.state()["qvel"] - before
    np.testing.assert_array_equal(delta[8:], 0)
    np.testing.assert_array_equal(delta[:, [2, 5]], 0)
    np.testing.assert_array_equal(delta[:, 6:], 0)
    assert np.max(np.abs(delta[:8, :2])) <= .15
    assert np.max(np.abs(delta[:8, 3:5])) <= .4
    assert np.any(np.abs(delta[:8]) > 0)
    assert np.all((env.next_push_step[:8] >= 450) & (env.next_push_step[:8] <= 750))
    assert env.total_disturbances == 8 and env.events[-1]["count"] == 8
    assert "delta_velocity" not in env.events[-1]


@pytest.mark.parametrize("terrain", ["flat", "rough", "slope", "steps"])
def test_expanded_arena_interpolation_edges_and_fast_policy_travel(terrain):
    env = SweetieVecEnv(num_envs=1, threads=1, randomize=False,
                        auto_reset=False, terrain=terrain, difficulty=1., seed=2000003)
    extent = env.terrain["extent_m"]
    assert extent == 10
    if terrain == "rough":
        heights = np.asarray(env.terrain["height_samples"])
        assert heights.shape == (321, 321)
        np.testing.assert_allclose(env.ground_height([-extent, extent], [-extent, extent]),
                                   heights[[0, -1], [0, -1]] * env.terrain["height_range_m"])
    elif terrain == "steps":
        assert env.terrain["n_steps"] == 200
        np.testing.assert_allclose(env.ground_height([-extent, extent], [0, 0]), [0, .005])
    qpos = env.state["qpos"].copy()
    qpos[0, 0] = 6.6  # Twelve seconds of successful 0.55m/s travel must fit.
    qpos[0, 2] = env.nominal_height + env.ground_height(qpos[0, 0], qpos[0, 1])
    env._clear_reset_penetration(qpos)
    env.native.reset(np.array([0]), qpos, np.zeros((1, 22)), env.params)
    env._read_state()
    _, _, done, _ = env.step(np.zeros((1, 12)))
    assert not done[0]

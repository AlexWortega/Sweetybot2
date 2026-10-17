"""Replay position-target policies in native MuJoCo with keyboard/gamepad input."""
from __future__ import annotations

import argparse
import copy
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np

from learning.motion import MotionController, RunButtonLatch


class RecoverySupervisor:
    """Use separate enter/exit thresholds and dwell times to avoid mode chatter."""

    def __init__(self, available: bool):
        self.available = available
        self.mode = "walk"
        self.fallen_for = 0.0
        self.upright_for = 0.0

    def update(self, height: float, tilt: float, dt: float) -> str:
        finite = math.isfinite(height) and math.isfinite(tilt)
        fallen = finite and (height < 0.10 or tilt > 0.90)
        upright = finite and height > 0.15 and tilt < 0.45
        self.fallen_for = self.fallen_for + dt if fallen else 0.0
        self.upright_for = self.upright_for + dt if upright else 0.0
        if self.available and self.mode == "walk" and self.fallen_for >= 0.25:
            self.mode = "recover"
            self.upright_for = 0.0
        elif self.mode == "recover" and self.upright_for >= 0.50:
            self.mode = "walk"
            self.fallen_for = 0.0
        return self.mode

    def reset(self):
        self.mode = "walk"
        self.fallen_for = self.upright_for = 0.0


class Gamepad:
    """SDL controller mapping, with configurable raw-joystick fallback."""

    def __init__(self, args):
        try:
            import pygame
        except ImportError as exc:
            raise RuntimeError("Install pygame==2.6.1 for --joystick") from exc
        self.pg = pygame
        pygame.init()
        pygame.joystick.init()
        self.args = args
        self.device = None
        self.controller = False
        self.last_poll = 0.0
        self.last_scan = -math.inf
        self.enabled = False
        self.run_toggle = False
        self.run_button = RunButtonLatch()
        try:
            from pygame._sdl2 import controller
            controller.init()
            self.controllers = controller
        except ImportError:
            self.controllers = None

    def poll(self):
        pg, now = self.pg, time.monotonic()
        self.enabled = False
        self.run_toggle = False
        try:
            pg.event.pump()
            removed = [pg.JOYDEVICEREMOVED]
            if hasattr(pg, "CONTROLLERDEVICEREMOVED"):
                removed.append(pg.CONTROLLERDEVICEREMOVED)
            if pg.event.get(removed):
                self.device = None
                self.last_poll = 0.0
                self.run_button.reset()
                return np.zeros(3)
            if self.device is None and now - self.last_scan >= 1.0:
                self.last_scan = now
                if pg.joystick.get_count():
                    if self.controllers and self.controllers.is_controller(0):
                        self.device = self.controllers.Controller(0)
                        self.controller = True
                    else:
                        self.device = pg.joystick.Joystick(0)
                        self.device.init()
                        self.controller = False
                    print("Gamepad connected; hold left shoulder to command motion.", flush=True)
            if self.device is None:
                return np.zeros(3)
            if self.controller:
                if not self.device.attached():
                    raise RuntimeError("controller disconnected")
                held = self.device.get_button(pg.CONTROLLER_BUTTON_LEFTSHOULDER)
                run_pressed = self.device.get_button(pg.CONTROLLER_BUTTON_RIGHTSHOULDER)
                axes = [-self.device.get_axis(pg.CONTROLLER_AXIS_LEFTY),
                        -self.device.get_axis(pg.CONTROLLER_AXIS_LEFTX),
                        -self.device.get_axis(pg.CONTROLLER_AXIS_RIGHTX)]
                axes = np.asarray(axes, dtype=float) / 32768.0
            else:
                if hasattr(self.device, "get_attached") and not self.device.get_attached():
                    raise RuntimeError("joystick disconnected")
                held = self.device.get_button(self.args.deadman_button)
                run_pressed = self.device.get_button(self.args.run_button)
                axes = np.asarray([-self.device.get_axis(1), -self.device.get_axis(0),
                                   -self.device.get_axis(self.args.yaw_axis)], dtype=float)
            self.last_poll = now
            if not np.isfinite(axes).all():
                self.run_button.reset()
                return np.zeros(3)
            self.run_toggle = self.run_button.update(run_pressed)
            self.enabled = bool(held)
            if not held:
                return np.zeros(3)
            deadzone = self.args.deadzone
            axes = np.sign(axes) * np.maximum(np.abs(axes) - deadzone, 0) / (1 - deadzone)
            return np.clip(axes, -1, 1)
        except (RuntimeError, pg.error, IndexError):
            self.device = None
            self.last_poll = 0.0
            self.enabled = False
            self.run_toggle = False
            self.run_button.reset()
            return np.zeros(3)

    def close(self):
        self.pg.quit()


def _first(value):
    array = np.asarray(value)
    return array[0] if array.ndim > 1 else array


def pose_metrics(env):
    state = env.state
    qpos = _first(state["qpos"])
    w, x, y, z = qpos[3:7]
    tilt = math.acos(float(np.clip(1 - 2 * (x*x + y*y), -1, 1)))
    height = float(qpos[2])
    if "height_above_ground" in state:
        height = float(np.asarray(state["height_above_ground"]).ravel()[0])
    elif hasattr(env, "ground_height"):
        height -= float(env.ground_height(float(qpos[0]), float(qpos[1])))
    return height, tilt


def render_model(env, cad=False):
    """Attach saved v02 CAD visuals to a separate rendering model only.

    The environment keeps its original lightweight native physics model. Joint
    state is copied into this model for display; its dynamics are never stepped.
    Terrain and collision geometry come from the environment, not the CAD scene.
    """
    import mujoco
    if not cad:
        return mujoco.MjModel.from_xml_path(str(env.model_path))
    from scripts.common import ROOT
    source_path = ROOT / "simulation/generated/sweetie_mini.xml"
    if not source_path.is_file():
        raise FileNotFoundError(f"--cad requires the original v02 MJCF: {source_path}")
    source = ET.parse(source_path).getroot()
    target = ET.parse(env.model_path).getroot()
    source_asset = source.find("asset")
    meshes = source_asset.findall("mesh") if source_asset is not None else []
    if not meshes:
        raise ValueError("--cad requires saved CAD mesh assets in the original v02 MJCF")
    assets = target.find("asset")
    if assets is None:
        assets = ET.SubElement(target, "asset")
    # from_xml_string has no source directory. Preserve terrain heightfields
    # and any other existing file assets by resolving them before compilation.
    for asset in assets:
        if asset.get("file"):
            path = Path(asset.get("file"))
            if not path.is_absolute():
                path = Path(env.model_path).parent / path
            asset.set("file", str(path.resolve()))
    compiler = source.find("compiler")
    mesh_directory = source_path.parent
    if compiler is not None and compiler.get("meshdir"):
        mesh_directory /= compiler.get("meshdir")
    mesh_names = set()
    for original in meshes:
        asset = copy.deepcopy(original)
        filename = asset.get("file")
        if not filename:
            raise ValueError(f"--cad mesh {asset.get('name')} has no saved file")
        path = Path(filename)
        if not path.is_absolute():
            path = mesh_directory / path
        path = path.resolve()
        if not path.is_file():
            raise FileNotFoundError(f"--cad requires saved mesh: {path}")
        asset.set("file", str(path))
        mesh_names.add(asset.get("name"))
        assets.append(asset)
    bodies = {body.get("name"): body for body in target.iter("body")}
    attached = 0
    for source_body in source.iter("body"):
        cad_geoms = [geom for geom in source_body.findall("geom")
                     if geom.get("name", "").endswith("_cad")]
        if not cad_geoms:
            continue
        body = bodies.get(source_body.get("name"))
        if body is None:
            raise ValueError(f"--cad body {source_body.get('name')} is absent from the physics model")
        for geom in body.findall("geom"):
            geom.set("group", "3")
            geom.set("rgba", "0 0 0 0")
        for original in cad_geoms:
            if original.get("mesh") not in mesh_names:
                raise ValueError(f"--cad geometry references missing mesh {original.get('mesh')}")
            geom = copy.deepcopy(original)
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
            geom.set("group", "1")
            body.append(geom)
            attached += 1
    if not attached:
        raise ValueError("--cad found no *_cad body geometries in the original v02 MJCF")
    return mujoco.MjModel.from_xml_string(ET.tostring(target, encoding="unicode"))


def validate_policy_metadata(label, metadata, observed_dim):
    if metadata.get("obs_dim", observed_dim) != observed_dim:
        raise ValueError(f"{label} checkpoint observation dimension differs from environment")
    if metadata.get("action_dim", 12) != 12:
        raise ValueError(f"{label} checkpoint must control twelve leg joints")
    expected_task = {"walking": "locomotion", "running": "running", "recovery": "recovery"}[label]
    trained_task = metadata.get("config", {}).get("task")
    if (label == "running" or trained_task is not None) and trained_task != expected_task:
        raise ValueError(f"{label} checkpoint was trained for {trained_task!r}, expected {expected_task!r}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--walk", required=True, type=Path)
    parser.add_argument("--recover", type=Path)
    parser.add_argument("--run", type=Path, help="Optional trained running specialist checkpoint")
    parser.add_argument("--mode", choices=("walk", "run"), default="walk")
    parser.add_argument("--joystick", action="store_true")
    parser.add_argument("--terrain", choices=("flat", "slope", "rough", "steps"), default="flat")
    parser.add_argument("--difficulty", type=float, default=0.25)
    parser.add_argument("--seconds", type=float, default=120)
    parser.add_argument("--seed", type=int, default=1_000_090)
    parser.add_argument("--pose", choices=("stand", "left", "right", "prone", "supine"), default="stand")
    parser.add_argument("--command", nargs=3, type=float, default=(0., 0., 0.), metavar=("VX", "VY", "YAW"))
    parser.add_argument("--record", type=Path)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--cad", action="store_true", help="Render saved v02 CAD meshes; physics remains lightweight")
    parser.add_argument("--max-vx", type=float, default=0.25)
    parser.add_argument("--run-max-vx", type=float, default=0.60)
    parser.add_argument("--max-vy", type=float, default=0.20)
    parser.add_argument("--max-yaw", type=float, default=1.0)
    parser.add_argument("--deadzone", type=float, default=0.12)
    parser.add_argument("--deadman-button", type=int, default=4)
    parser.add_argument("--run-button", type=int, default=5)
    parser.add_argument("--yaw-axis", type=int, default=3)
    args = parser.parse_args(argv)
    if args.seconds <= 0 or not 0 <= args.deadzone < 1:
        parser.error("--seconds must be positive; --deadzone must be in [0, 1)")
    if not np.isfinite([args.seconds, args.difficulty, args.max_vx, args.run_max_vx, args.max_vy, args.max_yaw]).all():
        parser.error("Timing, difficulty and command limits must be finite")
    if min(args.max_vx, args.run_max_vx, args.max_vy, args.max_yaw) < 0:
        parser.error("Command limits must be nonnegative")
    if any(path is not None and not path.is_file() for path in (args.walk, args.recover, args.run)):
        parser.error("Policy checkpoint does not exist")

    import mujoco
    import torch
    from learning.env import SweetieVecEnv
    from learning.ppo import load_policy

    torch.set_num_threads(1)
    walk, walk_meta = load_policy(args.walk, device="cpu")
    recovery, recovery_meta = load_policy(args.recover, device="cpu") if args.recover else (None, {})
    running, run_meta = load_policy(args.run, device="cpu") if args.run else (None, {})
    policies = {"walk": walk, "recover": recovery, "run": running, "walk-fast-fallback": walk}
    supervisor = RecoverySupervisor(recovery is not None)
    env = SweetieVecEnv(task="locomotion", num_envs=1, seed=args.seed, terrain=args.terrain,
                       difficulty=args.difficulty, randomize=False, threads=1)
    env.auto_reset = False
    env.reset()
    env.reset_pose(args.pose)
    observed_dim = int(np.asarray(env.observe()).shape[-1])
    for label, metadata, policy in (("walking", walk_meta, walk), ("running", run_meta, running),
                                    ("recovery", recovery_meta, recovery)):
        if policy is not None:
            validate_policy_metadata(label, metadata, observed_dim)
    if args.pose != "stand" and recovery is not None:
        supervisor.mode = "recover"
    model = render_model(env, cad=args.cad)
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.distance, camera.azimuth, camera.elevation = 0.8, 135, -20
    command = np.asarray(args.command, dtype=float)
    if not np.isfinite(command).all():
        parser.error("--command must contain finite numbers")
    motion = MotionController(args.mode, args.max_vx, args.run_max_vx, args.max_vy, args.max_yaw)
    motion.set_command(command)
    request = {"reset": False, "fall": False, "stop": False, "halt": False}

    def key_callback(key):
        if key == 256:
            request["stop"] = True
        elif key == ord("R"):
            request["reset"] = True
        elif key == ord("F"):
            request["fall"] = True
        elif key == 32:
            request["halt"] = True
        elif key == ord("B"):
            motion.toggle_mode()
        elif key in (ord("W"), ord("S")):
            motion.axes[0] = 1. if key == ord("W") else -1.
        elif key in (ord("A"), ord("D")):
            motion.axes[1] = 1. if key == ord("A") else -1.
        elif key in (ord("Q"), ord("E")):
            motion.axes[2] = 1. if key == ord("Q") else -1.

    viewer = None
    renderer = None
    writer = None
    gamepad = None
    try:
        if args.joystick:
            gamepad = Gamepad(args)
        if not args.headless:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(model, data, key_callback=key_callback)
            viewer.cam.distance = camera.distance
            viewer.cam.azimuth = camera.azimuth
            viewer.cam.elevation = camera.elevation
        if args.record:
            import imageio.v2 as imageio
            args.record.parent.mkdir(parents=True, exist_ok=True)
            renderer = mujoco.Renderer(model, height=720, width=960)
            writer = imageio.get_writer(str(args.record), fps=25, codec="libx264", quality=7)
        print("W/S forward/back, A/D sideways, Q/E turn; B walk/run; Space stop; F fall; R reset; Escape quit.", flush=True)
        print("Keyboard commands latch until Space. Joystick: LB/L1 deadman, RB/R1 toggles walk/run; release/disconnect commands zero.", flush=True)
        if recovery is None:
            print("No recovery checkpoint supplied: automatic recovery switching is disabled.", flush=True)
        if running is None:
            print("WARNING: no --run specialist: run mode uses an UNVERIFIED faster-target walking policy fallback.", flush=True)
        print(f"Walking checkpoint metadata: {walk_meta}; running: {run_meta}; recovery: {recovery_meta}", flush=True)
        period, last_mode, wall_start = 0.02, None, time.monotonic()
        next_frame = 0.0
        for tick in range(math.ceil(args.seconds / period)):
            if request["stop"] or viewer is not None and not viewer.is_running():
                break
            if request["reset"]:
                env.reset_pose("stand")
                supervisor.reset()
                motion.stop()
                request["halt"] = True
                request["reset"] = False
                print(f"Explicit reset at simulated t={tick*period:.2f}s", flush=True)
            if request["fall"]:
                request["halt"] = True
                if hasattr(env, "fall"):
                    env.fall()
                    print(f"Explicit fall disturbance at simulated t={tick*period:.2f}s", flush=True)
                else:
                    print("This environment does not provide an explicit fall disturbance.", flush=True)
                request["fall"] = False
            enabled = True
            if gamepad:
                motion.set_axes(gamepad.poll())
                if gamepad.run_toggle:
                    motion.toggle_mode()
                enabled = gamepad.enabled and time.monotonic() - gamepad.last_poll <= 0.25
            if request["halt"]:
                motion.stop()
                enabled = False
                request["halt"] = False
            height, tilt = pose_metrics(env)
            mode = supervisor.update(height, tilt, period)
            command = motion.step(period, enabled=enabled, recovering=mode == "recover")
            effective = motion.effective_policy(recovering=mode == "recover", run_available=running is not None)
            status = (motion.mode, effective)
            if status != last_mode:
                print(f"t={tick*period:.2f}s armed={motion.mode} effective_policy={effective} "
                      f"height={height:.3f}m tilt={math.degrees(tilt):.1f}deg", flush=True)
                last_mode = status
            # All motion policies use home offsets; recovery uses incremental targets.
            env.task = {"run": "running", "recover": "recovery"}.get(effective, "locomotion")
            env.set_command(*command)
            observation = torch.as_tensor(env.observe(), dtype=torch.float32)
            with torch.inference_mode():
                output = policies[effective].act(observation, deterministic=True)
            action = output[0] if isinstance(output, tuple) else output
            if not torch.isfinite(action).all():
                raise RuntimeError("Nonfinite policy output; stopping simulation")
            _, _, done, info = env.step(action.cpu().numpy())
            state = env.state
            data.qpos[:] = _first(state["qpos"])
            data.qvel[:] = _first(state["qvel"])
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.qpos).all():
                raise RuntimeError("Nonfinite simulator state")
            camera.lookat[:] = data.qpos[:3]
            if viewer is not None:
                viewer.cam.lookat[:] = data.qpos[:3]
                viewer.sync()
            if writer is not None and tick*period + 1e-8 >= next_frame:
                renderer.update_scene(data, camera)
                writer.append_data(renderer.render())
                next_frame += 1 / 25
            if viewer is not None or gamepad is not None:
                time.sleep(max(0., wall_start + (tick+1)*period - time.monotonic()))
        return 0
    finally:
        if writer is not None:
            writer.close()
        if renderer is not None:
            renderer.close()
        if viewer is not None:
            viewer.close()
        if gamepad is not None:
            gamepad.close()
        if hasattr(env, "close"):
            env.close()


if __name__ == "__main__":
    raise SystemExit(main())

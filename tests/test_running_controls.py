"""Running inputs remain distinct from evidence that a checkpoint can run."""
import numpy as np
import pytest

from learning.motion import MotionController, RunButtonLatch
from learning.play import RecoverySupervisor, validate_policy_metadata


def advance(control, seconds=2.):
    for _ in range(round(seconds / .02)):
        control.step(.02)


def test_latched_keyboard_forward_rescales_when_mode_changes():
    control = MotionController()
    control.set_axes([1, 0, 0])
    advance(control)
    assert control.command[0] == pytest.approx(.25)
    control.toggle_mode()
    command = control.step(.02)
    assert command[0] == pytest.approx(.262)
    advance(control)
    assert control.command[0] == pytest.approx(.60)
    assert control.effective_policy(run_available=True) == "run"
    control.toggle_mode()
    assert control.step(.02)[0] == pytest.approx(.588)
    advance(control)
    assert control.command[0] == pytest.approx(.25)


def test_diagonal_acceleration_is_bounded_and_release_is_immediate():
    control = MotionController(mode="run")
    control.set_axes([1, 1, 1])
    previous = control.command.copy()
    for _ in range(30):
        command = control.step(.02)
        assert np.linalg.norm((command - previous)[:2]) <= .6 * .02 + 1e-12
        previous = command
    np.testing.assert_array_equal(control.step(.02, enabled=False), np.zeros(3))
    # Returning from deadman release also ramps from zero.
    assert np.linalg.norm(control.step(.02)[:2]) <= .012 + 1e-12
    control.stop()  # Space/reset must clear both output and latched input.
    np.testing.assert_array_equal(control.step(.02), np.zeros(3))


def test_recovery_overrides_running_then_resumes_from_zero():
    control = MotionController(mode="run")
    control.set_axes([1, 0, 0])
    advance(control)
    np.testing.assert_array_equal(control.step(.02, recovering=True), np.zeros(3))
    assert control.effective_policy(recovering=True, run_available=True) == "recover"
    assert control.mode == "run"
    assert control.step(.02)[0] == pytest.approx(.012)
    assert control.effective_policy(run_available=True) == "walk"
    advance(control)
    assert control.effective_policy(run_available=True) == "run"


def test_specialist_only_used_for_forward_and_fallback_is_explicit():
    control = MotionController(mode="run")
    for axes in ([-1, 0, 0], [0, 1, 0], [0, 0, 1]):
        control.stop()
        control.set_axes(axes)
        advance(control)
        assert control.effective_policy(run_available=True) == "walk"
        assert abs(control.command[0]) <= .25
    control.stop()
    control.set_axes([1, 0, 0])
    advance(control)
    assert control.effective_policy(run_available=False) == "walk-fast-fallback"


def test_rb_edges_do_not_repeat_or_replay_after_disconnect():
    button = RunButtonLatch()
    assert not button.update(True)  # Reconnecting while RB held is not a press.
    assert not button.update(False)
    assert button.update(True)
    assert not button.update(True)
    button.reset()  # Removal, invalid input, and device errors reset the latch.
    assert not button.update(True)
    assert not button.update(False)
    assert button.update(True)


def test_running_checkpoint_metadata_cannot_silently_use_recovery_semantics():
    metadata = {"obs_dim": 165, "action_dim": 12, "config": {"task": "running"}}
    validate_policy_metadata("running", metadata, 165)
    for task in (None, "locomotion", "recovery"):
        with pytest.raises(ValueError, match="expected 'running'"):
            validate_policy_metadata("running", {**metadata, "config": {"task": task}}, 165)
    with pytest.raises(ValueError, match="observation dimension"):
        validate_policy_metadata("running", {**metadata, "obs_dim": 164}, 165)
    with pytest.raises(ValueError, match="twelve"):
        validate_policy_metadata("running", {**metadata, "action_dim": 16}, 165)


def test_recovery_supervisor_hysteresis_still_applies():
    supervisor = RecoverySupervisor(available=True)
    for _ in range(13):
        mode = supervisor.update(.08, 1.2, .02)
    assert mode == "recover"
    for _ in range(24):
        assert supervisor.update(.2, 0., .02) == "recover"
    assert supervisor.update(.2, 0., .02) == "walk"


def test_gamepad_poll_deadman_and_disconnect_clear_commands_and_edges():
    from types import SimpleNamespace
    from learning.play import Gamepad

    buttons = {4: True, 5: False}
    removed = []
    attached = [True]
    pg = SimpleNamespace(
        JOYDEVICEREMOVED=10, CONTROLLERDEVICEREMOVED=11,
        CONTROLLER_BUTTON_LEFTSHOULDER=4, CONTROLLER_BUTTON_RIGHTSHOULDER=5,
        CONTROLLER_AXIS_LEFTY=1, CONTROLLER_AXIS_LEFTX=0, CONTROLLER_AXIS_RIGHTX=3,
        error=OSError,
        event=SimpleNamespace(pump=lambda: None, get=lambda _: list(removed)))
    device = SimpleNamespace(attached=lambda: attached[0],
                             get_button=lambda button: buttons[button],
                             get_axis=lambda axis: -32768 if axis == 1 else 0)
    gamepad = Gamepad.__new__(Gamepad)
    gamepad.pg, gamepad.device, gamepad.controller = pg, device, True
    gamepad.args = SimpleNamespace(deadzone=.12)
    gamepad.last_poll, gamepad.last_scan = 0., 0.
    gamepad.run_button = RunButtonLatch()
    np.testing.assert_array_equal(gamepad.poll(), [1., 0., 0.])
    assert gamepad.enabled and not gamepad.run_toggle
    buttons[5] = True
    gamepad.poll()
    assert gamepad.run_toggle
    gamepad.poll()
    assert not gamepad.run_toggle
    buttons[4] = False
    np.testing.assert_array_equal(gamepad.poll(), np.zeros(3))
    assert not gamepad.enabled
    removed.append(10)
    np.testing.assert_array_equal(gamepad.poll(), np.zeros(3))
    assert gamepad.last_poll == 0 and not gamepad.run_toggle and not gamepad.enabled
    removed.clear()
    gamepad.device = device  # Reconnect with RB still held.
    gamepad.poll()
    assert not gamepad.run_toggle
    attached[0] = False
    gamepad.poll()
    assert gamepad.device is None and not gamepad.enabled

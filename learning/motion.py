"""Input mode selection and bounded command changes, independent of physics."""
from __future__ import annotations

import numpy as np


class RunButtonLatch:
    """A fresh connection must release RB before its next press can toggle mode."""

    def __init__(self):
        self.previous = True

    def reset(self):
        self.previous = True

    def update(self, pressed: bool) -> bool:
        rising = bool(pressed) and not self.previous
        self.previous = bool(pressed)
        return rising


class MotionController:
    def __init__(self, mode="walk", walk_max_vx=.25, run_max_vx=.60,
                 max_vy=.20, max_yaw=1., acceleration=.6):
        if mode not in ("walk", "run"):
            raise ValueError("Mode must be walk or run")
        self.mode = mode
        self.walk_max_vx, self.run_max_vx = walk_max_vx, run_max_vx
        self.max_vy, self.max_yaw = max_vy, max_yaw
        self.acceleration = acceleration
        self.axes = np.zeros(3)
        self.command = np.zeros(3)

    @property
    def limits(self):
        # Running specializes forward motion; reverse retains the walking limit.
        return np.array([self.run_max_vx if self.mode == "run" and self.axes[0] > 0
                         else self.walk_max_vx, self.max_vy, self.max_yaw])

    def toggle_mode(self):
        self.mode = "run" if self.mode == "walk" else "walk"

    def set_axes(self, axes):
        axes = np.asarray(axes, dtype=float)
        if axes.shape != (3,) or not np.isfinite(axes).all():
            self.stop()
            return
        self.axes[:] = np.clip(axes, -1., 1.)

    def set_command(self, command):
        """Convert a CLI command to normalized input once, before mode changes."""
        self.set_axes(command)
        limits = self.limits
        self.set_axes(np.divide(command, limits, out=np.zeros(3), where=limits > 0))

    def stop(self):
        self.axes[:] = 0.
        self.command[:] = 0.

    def step(self, dt, *, enabled=True, recovering=False):
        if not enabled or recovering:
            self.command[:] = 0.
            return self.command.copy()
        target = self.axes * self.limits
        # Bound planar acceleration by its vector magnitude, even on diagonals.
        delta = target - self.command
        norm = np.linalg.norm(delta[:2])
        if norm > self.acceleration * dt:
            delta[:2] *= self.acceleration * dt / norm
        delta[2] = np.clip(delta[2], -self.acceleration * dt, self.acceleration * dt)
        self.command += delta
        return self.command.copy()

    def effective_policy(self, *, recovering=False, run_available=False):
        if recovering:
            return "recover"
        if self.mode == "run" and self.command[0] > .05:
            return "run" if run_available else "walk-fast-fallback"
        return "walk"

"""Persistent easy-to-hard training, inspired by pollen-robotics/microduck_rl.

Schedules use accumulated policy steps per environment, never chunk wall time.
Resets place an episode's initial state only; they do not assist active motion.
"""
import numpy as np

RECIPE = 'v05-microduck-rewards-2'


def progress(env_steps, horizon=50_000):
    if not np.isfinite(env_steps) or env_steps < 0 or not np.isfinite(horizon) or horizon <= 0:
        raise ValueError('Finite nonnegative steps and positive horizon required')
    return float(np.clip(env_steps / horizon, 0., 1.))


def recovery_stage(value):
    value = float(np.clip(value, 0., 1.))
    if value < .20:
        return dict(name='stand_and_crouch', probabilities=[.35,.55,.05,.05,0.], roll_scale=.20, full_fall_probability=0.)
    if value < .50:
        return dict(name='partial_rolls', probabilities=[.20,.35,.20,.20,.05], roll_scale=.45, full_fall_probability=.10)
    if value < .85:
        return dict(name='side_recovery', probabilities=[.10,.20,.25,.25,.20], roll_scale=.70, full_fall_probability=.35)
    return dict(name='full_falls', probabilities=[.10,.10,.20,.20,.40], roll_scale=1., full_fall_probability=.75)


def polish_weight(value):
    """Introduce motion damping after initial skill discovery, retaining safety."""
    return float(.05 + .95*np.clip((value-.50)/.35, 0., 1.))


def recovery_terrain(task_round, champion):
    """Consolidate flat get-up before spending experience on harder surfaces."""
    if not champion or not champion.get('quality_gate_passed'):
        return 'flat', 0.
    cycle=('flat','mixed','slope','mixed','steps','mixed')
    terrain=cycle[task_round % len(cycle)]
    return terrain, 0. if terrain=='flat' else min(1., .12+.04*max(0,task_round-1))


def recovery_progress_cap(champion):
    """Keep elapsed experience, but unlock hard starts after measured skills."""
    rates=(champion or {}).get('metrics',{}).get('per_orientation_success_rate',{})
    if rates.get('folded_crouch',0.)<.25:return .19
    if min(rates.get('left',0.),rates.get('right',0.))<.25:return .49
    if rates.get('supine',0.)<.25:return .84
    return 1.

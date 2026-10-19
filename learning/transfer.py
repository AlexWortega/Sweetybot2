"""Explicit weights-only transfer across audited training-recipe revisions.

Playback/resume stay strict. Donors must match geometry, native motor physics,
sensor code, encoder layout, dimensions and their original source signature.
The caller deliberately opts into changed rewards/resets by naming that source.
"""
from pathlib import Path
import hashlib
import torch
from learning.readiness import ROOT, DYNAMICS_FILES, LEGACY_DYNAMICS_FILES, dynamics_signature, physical_signature, source_signature


def validate_initializer(saved, env, source_root=None):
    metadata=saved.get('environment_metadata',{})
    expected=getattr(env,'observation_contract',None)
    if not expected:
        if source_root:raise ValueError('Recipe transfer requires the robot sensor contract')
        return None
    if metadata.get('observation_contract')!=expected:
        raise ValueError('Initializer observation contract differs')
    if saved.get('obs_dim')!=env.obs_dim or saved.get('action_dim')!=env.action_dim:
        raise ValueError('Initializer dimensions differ')
    target=dynamics_signature()
    if metadata.get('dynamics_signature')==target:return None
    if not source_root:raise ValueError('Changed recipe initializer requires explicit --initializer-source')
    source=Path(source_root).resolve()
    files=DYNAMICS_FILES if (source/'learning/curriculum.py').is_file() else LEGACY_DYNAMICS_FILES
    old=source_signature(source,files)
    if old!=metadata.get('dynamics_signature'):
        raise ValueError('Initializer does not match the supplied source snapshot')
    for name,digest in saved.get('source_hashes',{}).items():
        if hashlib.sha256((source/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Initializer source was modified: '+name)
    physical=physical_signature(source)
    if physical!=physical_signature():raise ValueError('Initializer robot physics/sensors differ')
    return dict(mode='explicit_training_recipe_transfer',source_snapshot=str(source),
                source_dynamics_signature=old,target_dynamics_signature=target,
                physical_sensor_signature=physical,optimizer_restored=False,
                source_hashes_verified=True)


@torch.no_grad()
def recovery_teacher_targets(teacher, obs):
    """Convert home-relative walking targets to incremental recovery actions.

The gate uses actor-visible IMU/contact health only. A walking teacher has no
authority over side/inverted recovery, where it has never learned a solution.
"""
    if obs.shape[-1]!=591:raise ValueError('Teacher requires the v05 3x197 frame')
    frames=obs.reshape(-1,3,197)
    teacher_obs=frames.clone()
    teacher_obs[:,:,50:53]=0.
    walk=teacher.act(teacher_obs.reshape(-1,591))
    current=frames[:,-1,6:18]
    action=((.65*walk-current)/.35).clamp(-1,1)
    latest=frames[:,-1]
    gate=(-latest[:,2]>.85)&(latest[:,66]>.5)&(latest[:,67]<.1)
    valid_support=(latest[:,58:62]>.5)&(latest[:,62:66]>.5)
    gate &= valid_support.sum(1)>=2
    return action,gate

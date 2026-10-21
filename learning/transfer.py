"""Explicit weights-only transfer across audited training-recipe revisions.

Playback/resume stay strict. Donors must match geometry, native motor physics,
sensor code, encoder layout, dimensions and their original source signature.
The caller deliberately opts into changed rewards/resets by naming that source.
"""
from pathlib import Path
import hashlib
import torch
from learning.readiness import ROOT, DYNAMICS_FILES, DYNAMICS_V2_FILES, LEGACY_DYNAMICS_FILES, dynamics_signature, physical_signature, source_signature


def _verified_source(saved,source_root,joint_limit_transfer=False):
    source=Path(source_root).resolve()
    files=(DYNAMICS_FILES if (source/'hardware/policy_observation.py').is_file() else
           DYNAMICS_V2_FILES if (source/'learning/curriculum.py').is_file() else LEGACY_DYNAMICS_FILES)
    old=source_signature(source,files)
    if old!=saved.get('environment_metadata',{}).get('dynamics_signature'):
        raise ValueError('Initializer does not match the supplied source snapshot')
    for name,digest in saved.get('source_hashes',{}).items():
        if hashlib.sha256((source/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Initializer source was modified: '+name)
    physical=physical_signature(source)
    limits=None
    if physical!=physical_signature():
        if not joint_limit_transfer:raise ValueError('Initializer robot physics/sensors differ')
        from learning.joint_limit_transfer import validate_limit_change
        limits=validate_limit_change(source)
    return dict(source_snapshot=str(source),source_dynamics_signature=old,
                target_dynamics_signature=dynamics_signature(),physical_sensor_signature=physical,
                target_physical_sensor_signature=physical_signature(),joint_limit_transfer=limits,
                optimizer_restored=False,source_hashes_verified=True)


def validate_initializer(saved, env, source_root=None, *, joint_limit_transfer=False):
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
    return dict(mode='explicit_training_recipe_transfer',**_verified_source(saved,source_root,joint_limit_transfer))


def validate_teacher(saved,env,source_root=None):
    if saved.get('obs_dim')==env.obs_dim:
        return validate_initializer(saved,env,source_root)
    from hardware.policy_observation import CONTRACT
    if (getattr(env,'observation_contract',None)!=CONTRACT or saved.get('obs_dim')!=591
            or saved.get('action_dim')!=12
            or saved.get('environment_metadata',{}).get('observation_contract')!='v05-sensor-suite-1'):
        raise ValueError('Unsupported teacher observation contract')
    if not source_root:raise ValueError('Legacy teacher requires an explicit verified source snapshot')
    return dict(mode='frozen_legacy_teacher_only',**_verified_source(saved,source_root))


@torch.no_grad()
def recovery_teacher_targets(teacher, obs, legacy_obs=None):
    """Convert home-relative walking targets to incremental recovery actions.

The gate uses actor-visible IMU/contact health only. A walking teacher has no
authority over side/inverted recovery, where it has never learned a solution.
"""
    if obs.shape[-1] not in (63,591):raise ValueError('Unsupported recovery actor observation')
    latest=obs.reshape(-1,3,197)[:,-1] if obs.shape[-1]==591 else obs
    teacher_dim=getattr(teacher,'obs_dim',obs.shape[-1])
    if teacher_dim==591:
        teacher_input=obs if obs.shape[-1]==591 else legacy_obs
        if teacher_input is None:raise ValueError('Legacy teacher needs training-only raw sensor history')
        teacher_obs=teacher_input.reshape(-1,3,197).clone();teacher_obs[:,:,50:53]=0.
        walk=teacher.act(teacher_obs.reshape(-1,591))
    elif teacher_dim==63:
        teacher_obs=(latest if latest.shape[-1]==63 else torch.cat((latest[:,:53],latest[:,58:68]),1)).clone()
        teacher_obs[:,50:53]=0.
        walk=teacher.act(teacher_obs)
    else:raise ValueError('Unsupported walking teacher architecture')
    current=latest[:,6:18]
    action=((.65*walk-current)/.35).clamp(-1,1)
    if obs.shape[-1]==591:
        gate=(-latest[:,2]>.85)&(latest[:,66]>.5)&(latest[:,67]<.1)
        valid_support=(latest[:,58:62]>.5)&(latest[:,62:66]>.5)
    else:
        gate=(-latest[:,2]>.85)&(latest[:,61]>.5)&(latest[:,62]<.1)
        valid_support=(latest[:,53:57]>.5)&(latest[:,57:61]>.5)
    gate &= valid_support.sum(1)>=2
    return action,gate

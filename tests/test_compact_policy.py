"""Compact actor isolation, privileged timeout bootstrap and training continuity."""
import json
from pathlib import Path

import numpy as np
import pytest
import torch

from hardware.policy_observation import from_sensor_frame
from learning.driver import apply_selection
from learning.ppo import Policy,TrainConfig,adapt_learning_rate,load_policy,train
from tests.test_ppo_learning import FakeEnv


def test_depth_clock_and_acceleration_cannot_enter_compact_actor():
    raw=np.arange(2*197,dtype=float).reshape(2,197)
    before=from_sensor_frame(raw)
    raw[:,53:58]=np.nan;raw[:,68:]=np.nan
    np.testing.assert_array_equal(before,from_sensor_frame(raw))
    assert before.shape==(2,63)
    raw[0,0]=np.nan
    with pytest.raises(ValueError):from_sensor_frame(raw)


def test_compact_actor_cannot_see_privileged_velocity_and_preserves_physics():
    from learning.env import SweetieVecEnv
    kwargs=dict(num_envs=2,threads=1,randomize=False,auto_reset=False,seed=55)
    legacy=SweetieVecEnv(**kwargs);compact=SweetieVecEnv(**kwargs,observation_mode='compact')
    try:
        assert compact.observe().shape==(2,63) and compact.observe_critic().shape==(2,81)
        actor=compact.observe();critic=compact.observe_critic()
        compact.local_velocity+=2.
        np.testing.assert_array_equal(actor,compact.observe())
        assert not np.array_equal(critic,compact.observe_critic())
        compact._read_state()
        legacy.set_command(0,0,0);compact.set_command(0,0,0)
        for _ in range(12):
            actions=np.full((2,12),.01)
            legacy.step(actions);compact.step(actions)
            for name in ('qpos','qvel','torque_rms','torque_peak'):
                np.testing.assert_allclose(legacy.state[name],compact.state[name],atol=1e-12)
        obs=compact.observe();value=compact.observe_critic()
        model=Policy(63,12,critic_obs_dim=81,hidden_dims=(512,256,128))
        model.update_norm(obs,value)
        action=model.act(obs).clone()
        model.critic_normalizer.mean+=100
        torch.testing.assert_close(action,model.act(obs),rtol=0,atol=0)
        with pytest.raises(ValueError,match='critic'):model.value(obs)
    finally:legacy.close();compact.close()


def test_asymmetric_timeout_uses_terminal_critic_before_reset(tmp_path,monkeypatch):
    class AsymmetricEnv(FakeEnv):
        critic_obs_dim=5
        def observe_critic(self):return np.zeros((4,5),np.float32)
        def step(self,action):
            obs,reward,done,info=super().step(action)
            info['terminal_critic_observation']=np.full((4,5),77.,np.float32)
            return obs,reward,done,info
    seen=[];original=Policy.value
    def recording(self,obs,critic_obs=None):
        if critic_obs is not None:seen.append(float(torch.as_tensor(critic_obs).mean()))
        return original(self,obs,critic_obs)
    monkeypatch.setattr(Policy,'value',recording)
    config=TrainConfig(out=str(tmp_path),device='cpu',seconds=30,max_steps=32,
                       rollout_steps=4,epochs=1,minibatch_size=16)
    result=train(AsymmetricEnv(),config)
    assert result['updates']==2 and 77. in seen
    policy,metadata=load_policy(tmp_path/'latest.pt')
    assert policy.obs_dim==9 and policy.critic_obs_dim==5
    assert metadata['architecture']['hidden_dims']==(512,256,128)
    before=policy.act(np.zeros((4,9),np.float32))
    config.resume=str(tmp_path/'latest.pt');config.max_steps=48
    assert train(AsymmetricEnv(),config)['updates']==3
    newer,_=load_policy(tmp_path/'latest.pt')
    assert not torch.equal(before,newer.act(np.zeros((4,9),np.float32)))


def test_poor_candidate_does_not_rewind_training_or_replace_qualified_champion():
    old=dict(score=2.,quality_gate_passed=True,checkpoint='best.pt')
    state=dict(champions={'running':old.copy()},latest_training={'running':'previous.pt'})
    candidate=dict(score=.1,quality_gate_passed=False,checkpoint='fresh.pt')
    assert not apply_selection(state,'running',candidate)
    assert state['latest_training']['running']=='fresh.pt'
    assert state['champions']['running']['checkpoint']=='best.pt'
    assert candidate['discarded_for_resume'] is False and candidate['regressed_against_champion']
    state['champions']['running']['quality_gate_passed']=False
    qualified=dict(score=.05,quality_gate_passed=True,checkpoint='qualified.pt')
    assert apply_selection(state,'running',qualified)
    assert state['champions']['running']['checkpoint']=='qualified.pt'


def test_fixed_commands_include_combinations_and_do_not_expand_with_time():
    from learning.env import SweetieVecEnv
    env=SweetieVecEnv.__new__(SweetieVecEnv)
    env.observation_mode='compact';env.manual_command=False;env.task='locomotion'
    env.steps=np.zeros(10000,int);env.commands=np.zeros((10000,3));env.command_targets=env.commands.copy()
    env.progress=0.;env.rng=np.random.default_rng(4)
    env._sample_commands(np.arange(10000));first=env.commands.copy()
    env.progress=1.;env.rng=np.random.default_rng(4);env._sample_commands(np.arange(10000))
    np.testing.assert_array_equal(first,env.commands)
    assert .18<np.all(first==0,1).mean()<.22
    assert .23<(np.count_nonzero(first,axis=1)==3).mean()<.27
    assert first[:,0].max()<=.30 and first[:,0].min()>=-.18


def test_resampled_training_commands_have_bounded_acceleration(monkeypatch):
    from learning.env import SweetieVecEnv
    env=SweetieVecEnv(num_envs=1,threads=1,randomize=False,observation_mode='compact')
    try:
        env.commands[:]=0;env.command_targets[:]=0;env.steps[:]=199
        def resample(ids):env.command_targets[ids]=1.
        monkeypatch.setattr(env,'_sample_commands',resample)
        obs,_,_,_=env.step(np.zeros((1,12)))
        assert np.linalg.norm(env.commands[0,:2])==pytest.approx(.6*.02)
        assert env.commands[0,2]==pytest.approx(.6*.02)
        np.testing.assert_allclose(obs[:,50:53],env.commands)
    finally:env.close()


def test_compact_recovery_teacher_gate_and_legacy_label_adapter():
    from learning.transfer import recovery_teacher_targets
    class Teacher:
        obs_dim=591
        def act(self,obs):
            assert not obs.reshape(-1,3,197)[:,:,50:53].any()
            return torch.full((len(obs),12),.2)
    raw=torch.zeros((2,3,197));raw[:,-1,2]=-1
    raw[:,-1,66]=1;raw[:,-1,58:60]=1;raw[:,-1,62:64]=1;raw[:,-1,6:18]=.06
    compact=torch.as_tensor(from_sensor_frame(raw[:,-1].numpy()))
    compact[1,61]=0
    actions,gate=recovery_teacher_targets(Teacher(),compact,legacy_obs=raw.reshape(2,591))
    assert gate.tolist()==[True,False]
    torch.testing.assert_close(actions,torch.full((2,12),.2))
    with pytest.raises(ValueError,match='raw sensor'):recovery_teacher_targets(Teacher(),compact)


def test_kl_schedule_lowers_step_size_without_restoring_parameters():
    model=Policy(9);optimizer=torch.optim.Adam(model.parameters(),lr=3e-4)
    weights=model.actor[0].weight.clone()
    assert adapt_learning_rate(optimizer,.1,.01)==pytest.approx(2e-4)
    assert adapt_learning_rate(optimizer,.001,.01)==pytest.approx(3e-4)
    torch.testing.assert_close(weights,model.actor[0].weight,rtol=0,atol=0)
    with pytest.raises(FloatingPointError):adapt_learning_rate(optimizer,float('nan'),.01)


def test_distillation_exports_compact_actor_with_fresh_ppo_optimizer(tmp_path):
    from learning.distill import distill
    class CompactEnv(FakeEnv):
        obs_dim=63;critic_obs_dim=81;observation_mode='compact';task='locomotion'
        def reset(self):self.index=0;return np.ones((4,63),np.float32)
        def step(self,action):
            self.index+=1
            return np.full((4,63),1+self.index*.01,np.float32),np.zeros(4),np.zeros(4,bool),{}
        def observe_critic(self):return np.ones((4,81),np.float32)
    teacher=Policy(63)
    with torch.no_grad():
        for parameter in teacher.actor.parameters():parameter.zero_()
        teacher.actor[-1].bias.fill_(float(np.arctanh(.2)))
    path=tmp_path/'teacher.pt';path.write_bytes(b'fixture')
    result=distill(CompactEnv(),teacher,{'obs_dim':63,'config':{'task':'locomotion'}},path,None,tmp_path/'student',
                    max_steps=32,rollout_steps=4,seconds=30,epochs=5)
    assert result['metrics']['training_mse_after']<result['metrics']['training_mse_before']
    saved=torch.load(tmp_path/'student/latest.pt',weights_only=False)
    assert saved['steps']==saved['updates']==0 and saved['optimizer']['state']=={}
    policy,_=load_policy(tmp_path/'student/latest.pt')
    assert policy.obs_dim==63 and policy.critic_obs_dim==81

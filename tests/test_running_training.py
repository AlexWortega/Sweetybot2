"""Running reward validity, unchanged physics, and honest checkpoint transfer."""
import hashlib

import numpy as np
import pytest

from learning.env import SweetieVecEnv, running_flight_bonus


@pytest.mark.parametrize("invalid", ["still", "slow", "rest_command", "tilted", "low", "high",
                                      "jump", "foot_contact", "body_contact", "mistracking"])
def test_running_flight_reward_rejects_jumping_still_or_invalid_flight(invalid):
    velocity=np.array([[.5,0.,0.]])
    command=velocity.copy()
    up=np.ones(1);height=np.array([.20]);contacts=np.zeros((1,4),bool);bad=np.zeros(1,int)
    assert running_flight_bonus(velocity,command,up,height,contacts,bad)[0]==pytest.approx(.2)
    if invalid=="still":velocity[0,0]=0
    elif invalid=="slow":velocity[0,0]=.349
    elif invalid=="rest_command":command[:]=0
    elif invalid=="tilted":up[:]=.8
    elif invalid=="low":height[:]=.15
    elif invalid=="high":height[:]=.28
    elif invalid=="jump":velocity[0,2]=.6
    elif invalid=="foot_contact":contacts[0,0]=True
    elif invalid=="body_contact":bad[:]=1
    elif invalid=="mistracking":velocity[0,1]=.3
    assert running_flight_bonus(velocity,command,up,height,contacts,bad)[0]==0


def test_running_curriculum_episode_bounds_and_identical_physical_response():
    pytest.importorskip("learning._sweetie_native")
    walk=SweetieVecEnv(task="locomotion",num_envs=8,threads=1,randomize=False,auto_reset=False,seed=99)
    run=SweetieVecEnv(task="running",num_envs=8,threads=1,randomize=False,auto_reset=False,seed=99)
    np.testing.assert_array_equal(walk._reset_model.body_mass,run._reset_model.body_mass)
    np.testing.assert_array_equal(walk._reset_model.body_inertia,run._reset_model.body_inertia)
    np.testing.assert_array_equal(walk._reset_model.body_ipos,run._reset_model.body_ipos)
    assert run.obs_dim==walk.obs_dim==591 and run.action_dim==walk.action_dim==12
    assert run.max_episode_steps*run.dt<=12
    assert 1.2+.65*run.max_episode_steps*run.dt<run.terrain['extent_m']-.5
    for progress in (0.,.5,1.):
        run.set_progress(progress)
        run._sample_commands(np.arange(run.num_envs))
        moving=run.commands[:,0]>0
        assert np.all(run.commands[moving,0]>=.30)
        assert np.all(run.commands[moving,0]<=.30+.35*progress)
        assert np.max(np.abs(run.commands[:,1]))<=.08
        assert np.max(np.abs(run.commands[:,2]))<=.6
    run.set_command(9,-9,9)
    np.testing.assert_allclose(run.commands,np.tile([.65,-.08,.6],(8,1)))
    run.set_command(-9,0,0)
    assert np.all(run.commands[:,0]==0)
    # Commands and clocks affect observations/rewards; the same joint actions
    # must retain identical masses, target slew, PD gains and servo saturation.
    actions=np.random.default_rng(7).uniform(-.15,.15,(8,12))
    old_walk_phase=walk.phase.copy();old_run_phase=run.phase.copy()
    walk.set_command(0,0,0);run.set_command(0,0,0)
    for _ in range(10):
        walk.step(actions)
        obs,reward,done,info=run.step(actions)
        for name in ('qpos','qvel','torque','torque_rms','torque_peak'):
            np.testing.assert_allclose(walk.state[name],run.state[name],atol=1e-12)
        np.testing.assert_array_equal(walk.targets,run.targets)
        assert np.isfinite(obs).all() and np.isfinite(reward).all()
        assert np.all(info['reward_terms']['forward_flight_event']==0)
    assert np.all(run.phase-old_run_phase>walk.phase-old_walk_phase)


class TransferEnv:
    num_envs,obs_dim,action_dim=4,165,12
    def reset(self):
        self.index=0
        return np.full((self.num_envs,self.obs_dim),2.,np.float32)
    def step(self,action):
        self.index+=1
        return np.full((self.num_envs,self.obs_dim),2+self.index*.02,np.float32), -np.square(action).mean(1),np.zeros(self.num_envs,bool),{}
    def set_progress(self,progress):pass


@pytest.fixture
def donor(tmp_path):
    torch=pytest.importorskip("torch")
    from learning.ppo import TrainConfig,train
    out=tmp_path/'donor'
    train(TransferEnv(),TrainConfig(task='locomotion',out=str(out),device='cpu',seed=18,
          seconds=30,max_steps=32,rollout_steps=4,epochs=1,minibatch_size=16))
    path=out/'latest.pt'
    return torch,path,torch.load(path,weights_only=False)


def test_running_initializer_copies_weights_normalizer_but_starts_fresh(donor,tmp_path):
    torch,path,saved=donor
    from learning.ppo import TrainConfig,train,load_policy
    out=tmp_path/'running'
    result=train(TransferEnv(),TrainConfig(task='running',out=str(out),device='cpu',seed=71,
                 seconds=30,max_steps=0,initialize_from=str(path)))
    transferred=torch.load(out/'latest.pt',weights_only=False)
    for key,value in saved['policy'].items():
        assert torch.equal(value,transferred['policy'][key])
    assert transferred['optimizer']['state']=={}
    assert result['steps']==transferred['steps']==0
    assert result['updates']==transferred['updates']==0
    assert result['resume_environment']=='fresh_episodes'
    assert not torch.equal(saved['torch_rng'],transferred['torch_rng'])
    _,metadata=load_policy(out/'latest.pt')
    provenance=metadata['initialization']
    assert provenance['donor_sha256']==hashlib.sha256(path.read_bytes()).hexdigest()
    assert provenance['donor_config']==saved['config']
    assert provenance['donor_steps']==32
    assert metadata['config']['task']=='running'
    assert saved['optimizer']['state']


def test_resume_restores_same_task_but_rejects_cross_task_and_mixed_modes(donor,tmp_path):
    torch,path,saved=donor
    from learning.ppo import TrainConfig,train
    out=tmp_path/'resumed'
    result=train(TransferEnv(),TrainConfig(task='locomotion',out=str(out),device='cpu',seed=71,
                 seconds=30,max_steps=32,resume=str(path)))
    resumed=torch.load(out/'latest.pt',weights_only=False)
    assert result['steps']==32 and result['updates']==2
    assert torch.equal(resumed['torch_rng'],saved['torch_rng'])
    for index,entry in saved['optimizer']['state'].items():
        for key,value in entry.items():
            assert torch.equal(resumed['optimizer']['state'][index][key],value)
    with pytest.raises(ValueError,match='Cannot resume another task'):
        train(TransferEnv(),TrainConfig(task='running',out=str(out),device='cpu',resume=str(path)))
    with pytest.raises(ValueError,match='mutually exclusive'):
        train(TransferEnv(),TrainConfig(task='running',out=str(out),device='cpu',resume=str(path),initialize_from=str(path)))
    with pytest.raises(ValueError,match='Cross-task initialization'):
        train(TransferEnv(),TrainConfig(task='recovery',out=str(out),device='cpu',initialize_from=str(path)))

"""Curriculum, reward-hacking, teacher routing and budget-handoff regressions."""
import numpy as np
import pytest
from learning.curriculum import progress, recovery_stage, polish_weight, recovery_terrain
from learning.driver import campaign_window
from learning.rewards import recovery_terms, recovery_potential, locomotion_terms


def test_handoff_does_not_buy_a_new_budget():
    start=1792350507.774651
    original=campaign_window(36,start)
    assert campaign_window(36,start+15*3600,start_epoch=start)==original
    assert original[1]==1792480107.774651
    for epoch in (float('nan'),float('inf'),start+1):
        with pytest.raises(ValueError):campaign_window(36,start,start_epoch=epoch)
    with pytest.raises(ValueError):campaign_window(36,start,prior_seconds=1,start_epoch=start)


def test_recovery_curriculum_retains_easy_starts_and_introduces_hard_falls_late():
    early=recovery_stage(0);late=recovery_stage(1)
    assert early['probabilities'][4]>=.10 and early['full_fall_probability']>=.20
    assert sum(early['probabilities'][:2])==pytest.approx(.6)
    assert late['probabilities'][4]>.3 and late['roll_scale']==1
    for value in np.linspace(0,1,21):
        plan=recovery_stage(value)
        assert sum(plan['probabilities'])==pytest.approx(1)
        assert min(plan['probabilities'])>-.0001
        assert sum(plan['probabilities'][:2])>=.19
    assert progress(25_000)==.5 and progress(100_000)==1
    assert polish_weight(.1)<polish_weight(.9)
    assert recovery_terrain(4,None)==('flat',0.)
    assert recovery_terrain(4,{'quality_gate_passed':False})==('flat',0.)
    assert recovery_terrain(4,{'quality_gate_passed':True})[0]=='steps'


def test_progress_survives_ppo_checkpoint_resume(tmp_path):
    torch=pytest.importorskip('torch')
    from learning.ppo import TrainConfig,train
    from tests.test_ppo_learning import FakeEnv
    class RecordingEnv(FakeEnv):
        def __init__(self):super().__init__();self.progresses=[]
        def set_progress(self,value):self.progresses.append(value)
    first=RecordingEnv();out=tmp_path/'train'
    config=TrainConfig(out=str(out),device='cpu',seconds=30,max_steps=32,rollout_steps=4,epochs=1,minibatch_size=16,curriculum_horizon_steps=100)
    train(first,config)
    saved=torch.load(out/'latest.pt',weights_only=False)
    assert saved['curriculum_env_steps']==8
    config.resume=str(out/'latest.pt');config.max_steps=48
    second=RecordingEnv();train(second,config)
    assert second.progresses[0]==pytest.approx(.08)
    assert torch.load(out/'latest.pt',weights_only=False)['curriculum_env_steps']==12


def test_walking_teacher_is_masked_when_fallen_and_converts_action_coordinates():
    torch=pytest.importorskip('torch')
    from learning.transfer import recovery_teacher_targets
    class Teacher:
        def act(self,obs):
            assert (obs.reshape(-1,3,197)[:,:,50:53]==0).all()
            return torch.full((len(obs),12),.2)
    obs=torch.zeros((5,3,197));obs[:,-1,2]=-1
    obs[:,-1,66]=1;obs[:,-1,58:60]=1;obs[:,-1,62:64]=1
    obs[:,-1,6:18]=.06
    obs[1,-1,2]=1 # inverted
    obs[2,-1,66]=0 # invalid IMU
    obs[3,-1,62:64]=0 # invalid contacts
    obs[4,-1,67]=.2 # stale IMU
    action,gate=recovery_teacher_targets(Teacher(),obs.reshape(5,591))
    assert gate.tolist()==[True,False,False,False,False]
    torch.testing.assert_close(action,torch.full((5,12),.2)) # (.65*.2-.06)/.35


def test_crouched_or_inverted_stationary_states_cannot_farm_progress():
    for up,height in ((-1.,.08),(0.,.15),(.9,.15)):
        u=np.array([up]);h=np.array([height]);pose=np.ones(1)
        phi=recovery_potential(u,h,.296,pose)
        terms,_=recovery_terms(u,h,.296,pose,phi,np.zeros(1),np.zeros(1,bool),np.ones((1,12)),np.zeros((1,12)),np.zeros((1,16)),np.ones((1,16)),np.ones(1),progress=0.)
        assert sum(terms.values())[0]<0


def test_late_motion_polish_preserves_overload_penalty():
    args=(np.ones(1),np.array([.296]),.296,np.zeros(1),np.zeros(1),np.zeros(1),np.zeros(1,bool),np.ones((1,12)),np.zeros((1,12)),np.full((1,16),2.),np.ones((1,16)),np.zeros(1))
    early,_=recovery_terms(*args,progress=0.,omega=np.ones((1,3)))
    late,_=recovery_terms(*args,progress=1.,omega=np.ones((1,3)))
    assert early['overload'][0]==late['overload'][0]<0
    assert late['action_rate'][0]<early['action_rate'][0]<=0
    assert late['arrival_damping'][0]<early['arrival_damping'][0]<=0


def test_command_buckets_include_idle_turns_backward_and_sideways_without_physics():
    from learning.env import SweetieVecEnv
    env=SweetieVecEnv.__new__(SweetieVecEnv)
    env.manual_command=False;env.task='locomotion';env.progress=1
    env.rng=np.random.default_rng(4);env.commands=np.zeros((10000,3))
    env._sample_commands(np.arange(10000));cmd=env.commands
    assert np.allclose((cmd==0).all(1).mean(),.2,atol=.02)
    for mask in ((cmd[:,0]<0),(cmd[:,1]!=0),(cmd[:,2]!=0)):
        assert .13<mask.mean()<.17
    assert np.all(cmd[cmd[:,2]!=0,:2]==0)


def test_new_progress_reward_rejects_still_wrong_direction_and_body_support():
    from tests.test_v04_sensors_rewards import reward_args
    a=reward_args();a['velocity'][0,0]=.1
    good=locomotion_terms(**a)['command_progress'][0]
    assert good>0
    for speed in (0.,-.1):
        a['velocity'][0,0]=speed
        assert locomotion_terms(**a)['command_progress'][0]==0
    a['velocity'][0,0]=.1;a['bad_contacts'][:]=1
    assert locomotion_terms(**a)['command_progress'][0]==0


def test_recipe_transfer_requires_exact_parent_and_unchanged_robot(tmp_path):
    import json,shutil,hashlib
    from types import SimpleNamespace
    from learning.readiness import ROOT,DYNAMICS_FILES,source_signature
    from learning.transfer import validate_initializer
    source=tmp_path/'parent'
    manifest=json.loads((ROOT/'reports/collision_meshes.json').read_text())
    files=set(DYNAMICS_FILES)|{name for row in manifest['parts'] for name in row['hulls']}
    for name in files:
        dest=source/name;dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,dest)
    recipe=source/'learning/rewards.py'
    recipe.write_text(recipe.read_text()+'\n# distinct parent recipe\n')
    env=SimpleNamespace(observation_contract='v05-sensor-suite-1',obs_dim=591,action_dim=12)
    saved={'obs_dim':591,'action_dim':12,'environment_metadata':{
        'observation_contract':env.observation_contract,'dynamics_signature':source_signature(source,DYNAMICS_FILES)},
        'source_hashes':{'learning/rewards.py':hashlib.sha256(recipe.read_bytes()).hexdigest()}}
    with pytest.raises(ValueError,match='explicit'):validate_initializer(saved,env)
    provenance=validate_initializer(saved,env,source)
    assert provenance['optimizer_restored'] is False and provenance['source_hashes_verified']
    robot=source/'simulation/generated/sweetie_mini.xml'
    robot.write_text(robot.read_text()+'\n<!-- changed mechanics -->\n')
    saved['environment_metadata']['dynamics_signature']=source_signature(source,DYNAMICS_FILES)
    with pytest.raises(ValueError,match='physics/sensors differ'):validate_initializer(saved,env,source)
    saved['environment_metadata']['observation_contract']='old'
    with pytest.raises(ValueError,match='observation contract'):validate_initializer(saved,env,source)


def test_new_recovery_starts_clear_without_mid_episode_assistance():
    pytest.importorskip('learning._sweetie_native')
    from learning.env import SweetieVecEnv
    env=SweetieVecEnv(task='recovery',num_envs=4,threads=1,randomize=True,auto_reset=True,seed=602)
    for value in (0.,.3,.6,1.):
        env.set_progress(value)
        for _ in range(3):
            obs=env.reset()
            assert np.isfinite(obs).all()
            assert not env.state['self_contacts'].any()
        before=env.steps.copy()
        env.step(np.zeros((4,12)))
        np.testing.assert_array_equal(env.steps,before+1)

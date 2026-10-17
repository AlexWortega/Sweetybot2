"""PPO math, observation contract and checkpoint smoke verification."""
import numpy as np
import pytest

torch = pytest.importorskip("torch")
from learning.ppo import Policy, RunningNorm, TrainConfig, compute_gae, load_policy, train


def test_gae_stops_at_autoreset_and_bootstraps_timeouts():
    # Timeout reward includes gamma*V(terminal), whereas the next row belongs to
    # an unrelated reset episode and must not leak through the done mask.
    rewards = torch.tensor([[5.5], [2.0]])
    dones = torch.tensor([[1.0], [0.0]])
    values = torch.tensor([[0.5], [0.6]])
    advantage, returns = compute_gae(rewards, dones, values, torch.tensor([0.7]), .9, .95)
    assert torch.allclose(advantage[:, 0], torch.tensor([5.0, 2.03]))
    assert torch.allclose(returns[:, 0], torch.tensor([5.5, 2.63]))


def test_squashed_policy_logprobs_and_normalizer_are_consistent():
    torch.set_num_threads(2)
    policy = Policy(9, 12)
    samples = torch.randn(32, 9) * 2 + 3
    policy.update_norm(samples)
    action, logprob, value, raw = policy.sample(samples)
    recalculated, entropy, value2 = policy.evaluate_actions(samples, action, raw)
    assert torch.allclose(logprob, recalculated)
    assert torch.allclose(value, value2)
    assert torch.isfinite(entropy).all()
    assert action.abs().max() <= 1
    assert torch.allclose(policy.normalizer.mean, samples.mean(0), atol=2e-5)
    huge = torch.full((32, 12), 30.0)
    logprob2, _, _ = policy.evaluate_actions(samples, huge.tanh(), huge)
    assert torch.isfinite(logprob2).all()


class FakeEnv:
    num_envs, obs_dim, action_dim = 4, 9, 12

    def __init__(self):
        self.index = 0

    def reset(self):
        self.index = 0
        return np.ones((4, 9), np.float32)

    def step(self, action):
        self.index += 1
        obs = np.full((4, 9), 1 + self.index * .01, np.float32)
        done = np.full(4, self.index % 3 == 0)
        truncated = done.copy()
        return obs, -np.square(action).mean(1), done, {
            "truncated": truncated, "terminal_observation": obs.copy(),
            "episode": {"test_return": np.array([1.0])} if done.any() else {},
        }

    def set_progress(self, fraction):
        assert 0 <= fraction <= 1


def test_training_checkpoint_resume_and_deterministic_inference(tmp_path):
    config = TrainConfig(out=str(tmp_path), nenvs=4, device="cpu", seconds=30,
                         max_steps=32, rollout_steps=4, epochs=1, minibatch_size=16)
    result = train(FakeEnv(), config)
    assert result["steps"] == 32
    assert result["updates"] == 2
    loaded, metadata = load_policy(tmp_path / "latest.pt")
    assert metadata["config"]["task"] == "locomotion"
    assert metadata["obs_dim"] == 9
    obs = np.ones((4, 9), np.float32)
    assert torch.equal(loaded.act(obs), loaded.act(obs))
    before = loaded.act(obs).clone()
    config.resume = str(tmp_path / "latest.pt")
    config.max_steps = 48
    result2 = train(FakeEnv(), config)
    assert result2["steps"] == 48
    assert result2["updates"] == 3
    loaded2, _ = load_policy(tmp_path / "latest.pt")
    assert not torch.equal(before, loaded2.act(obs))
    assert torch.isfinite(loaded2.act(obs)).all()

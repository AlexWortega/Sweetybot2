"""Small, bounded PPO trainer for the native SweetieMini MuJoCo environment.

The actor consumes only the environment's proprioceptive observation contract.
Checkpoints include the observation normalizer: use ``load_policy`` for inference.
A resume restores learning/RNG state but starts fresh simulation episodes unless
an environment implements the optional state_dict/load_state_dict protocol.
"""
from __future__ import annotations

import argparse
import hashlib
import io
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import random
import time

MODULE_STARTED = time.monotonic()  # Includes torch import/setup when used as CLI.

from typing import Any

import numpy as np
import torch
from torch import nn
from torch.distributions import Normal


class RunningNorm(nn.Module):
    def __init__(self, dimension: int):
        super().__init__()
        self.register_buffer("mean", torch.zeros(dimension))
        self.register_buffer("var", torch.ones(dimension))
        self.register_buffer("count", torch.tensor(1e-4, dtype=torch.float64))

    @torch.no_grad()
    def update(self, values: torch.Tensor) -> None:
        values = values.reshape(-1, self.mean.numel()).float()
        if not len(values) or not torch.isfinite(values).all():
            raise ValueError("Observation normalizer received empty/nonfinite data")
        n = values.shape[0]
        mean, var = values.mean(0), values.var(0, unbiased=False)
        total = self.count + n
        delta = mean - self.mean
        new_mean = self.mean + delta * (n / total).to(self.mean.dtype)
        m2 = self.var * self.count + var * n + delta.square() * self.count * n / total
        self.mean.copy_(new_mean)
        self.var.copy_((m2 / total).to(self.var.dtype).clamp_min(1e-8))
        self.count.copy_(total)

    def forward(self, values: torch.Tensor) -> torch.Tensor:
        return ((values - self.mean) / torch.sqrt(self.var + 1e-8)).clamp(-10, 10)


def _mlp(input_dim: int, output_dim: int, output_gain: float) -> nn.Sequential:
    layers: list[nn.Module] = []
    for width in (256, 128, 128):
        linear = nn.Linear(input_dim, width)
        nn.init.orthogonal_(linear.weight, math.sqrt(2))
        nn.init.zeros_(linear.bias)
        layers.extend((linear, nn.ELU()))
        input_dim = width
    linear = nn.Linear(input_dim, output_dim)
    nn.init.orthogonal_(linear.weight, output_gain)
    nn.init.zeros_(linear.bias)
    layers.append(linear)
    return nn.Sequential(*layers)


class Policy(nn.Module):
    def __init__(self, obs_dim: int, action_dim: int = 12):
        super().__init__()
        self.obs_dim, self.action_dim = int(obs_dim), int(action_dim)
        self.normalizer = RunningNorm(self.obs_dim)
        self.actor = _mlp(self.obs_dim, self.action_dim, 0.01)
        self.critic = _mlp(self.obs_dim, 1, 1.0)
        self.log_std = nn.Parameter(torch.full((self.action_dim,), -0.7))

    @property
    def device(self) -> torch.device:
        return self.log_std.device

    def tensor(self, obs: np.ndarray | torch.Tensor) -> torch.Tensor:
        return torch.as_tensor(obs, dtype=torch.float32, device=self.device)

    def normalize(self, obs: np.ndarray | torch.Tensor) -> torch.Tensor:
        return self.normalizer(self.tensor(obs))

    def update_norm(self, obs: np.ndarray | torch.Tensor) -> None:
        self.normalizer.update(self.tensor(obs))

    def distribution(self, obs: np.ndarray | torch.Tensor) -> Normal:
        mean = self.actor(self.normalize(obs))
        return Normal(mean, self.log_std.clamp(-5, 1).exp().expand_as(mean))

    def value(self, obs: np.ndarray | torch.Tensor) -> torch.Tensor:
        return self.critic(self.normalize(obs)).squeeze(-1)

    @staticmethod
    def _log_prob(distribution: Normal, raw: torch.Tensor) -> torch.Tensor:
        # Stable log(1-tanh(x)^2), including saturated actions.
        correction = 2.0 * (math.log(2.0) - raw - nn.functional.softplus(-2.0 * raw))
        return (distribution.log_prob(raw) - correction).sum(-1)

    def sample(self, obs: np.ndarray | torch.Tensor):
        distribution = self.distribution(obs)
        raw = distribution.sample()
        action = raw.tanh()
        return action, self._log_prob(distribution, raw), self.value(obs), raw

    @torch.no_grad()
    def act(self, obs: np.ndarray | torch.Tensor, deterministic: bool = True) -> torch.Tensor:
        distribution = self.distribution(obs)
        return (distribution.mean if deterministic else distribution.sample()).tanh()

    def evaluate_actions(self, obs: torch.Tensor, actions: torch.Tensor,
                         raw_actions: torch.Tensor | None = None):
        distribution = self.distribution(obs)
        raw = torch.atanh(actions.clamp(-1 + 1e-6, 1 - 1e-6)) if raw_actions is None else raw_actions
        # Base Normal entropy is a stable exploration regularizer. Report its
        # identity explicitly; it is not transformed action differential entropy.
        return self._log_prob(distribution, raw), distribution.entropy().sum(-1), self.value(obs)


def compute_gae(rewards: torch.Tensor, dones: torch.Tensor, values: torch.Tensor,
                next_value: torch.Tensor, gamma: float, gae_lambda: float):
    """Dones includes timeouts; timeout value must already augment its reward."""
    advantages = torch.zeros_like(rewards)
    last = torch.zeros_like(next_value)
    for index in reversed(range(rewards.shape[0])):
        following = next_value if index == rewards.shape[0] - 1 else values[index + 1]
        continuation = 1.0 - dones[index]
        delta = rewards[index] + gamma * following * continuation - values[index]
        last = delta + gamma * gae_lambda * continuation * last
        advantages[index] = last
    return advantages, advantages + values


@dataclass
class TrainConfig:
    task: str = "locomotion"
    out: str = "learning/runs/locomotion"
    nenvs: int = 128
    threads: int = 16
    seconds: float = 1200
    max_steps: int | None = None
    seed: int = 1
    device: str = "cuda:0"
    terrain: str = "flat"
    difficulty: float = 0.0
    randomize: bool = True
    resume: str | None = None
    initialize_from: str | None = None
    rollout_steps: int = 64
    epochs: int = 4
    minibatch_size: int = 2048
    learning_rate: float = 3e-4
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip: float = 0.2
    entropy: float = 0.005
    value_coefficient: float = 0.5
    save_interval: float = 60
    gpu_memory_gib: float = 2.0


def _safe_load(path: str | Path, device: str | torch.device):
    # Only load project-produced checkpoints: pickle is not an untrusted format.
    return torch.load(path, map_location=device, weights_only=False)


def validate_environment_contract(saved,env):
    expected=getattr(env,'observation_contract',None)
    if expected and saved.get('environment_metadata',{}).get('observation_contract')!=expected:
        raise ValueError('Checkpoint sensor observation contract differs; use a matching robot version')
    if expected:
        from learning.readiness import dynamics_signature
        if saved.get('environment_metadata',{}).get('dynamics_signature')!=dynamics_signature():raise ValueError('Checkpoint dynamics differ from current robot; explicit retraining required')


def load_policy(checkpoint: str | Path, device: str = "cpu") -> tuple[Policy, dict[str, Any]]:
    saved = _safe_load(checkpoint, device)
    model = Policy(saved["obs_dim"], saved["action_dim"]).to(device)
    model.load_state_dict(saved["policy"])
    model.eval()
    metadata = {key: saved[key] for key in ("config", "steps", "updates", "wall_seconds", "obs_dim", "action_dim", "format_version", "environment_metadata", "source_hashes", "initialization") if key in saved}
    return model, metadata


def _checkpoint(path: Path, model: Policy, optimizer, config: TrainConfig,
                steps: int, updates: int, wall: float, env=None, initialization=None) -> None:
    saved = {
        "format_version": 1,
        "policy": model.state_dict(), "optimizer": optimizer.state_dict(),
        "config": asdict(config), "obs_dim": model.obs_dim,
        "action_dim": model.action_dim, "steps": steps,
        "updates": updates, "wall_seconds": wall,
        "python_rng": random.getstate(), "numpy_rng": np.random.get_state(),
        "torch_rng": torch.get_rng_state(),
        "cuda_rng": torch.cuda.get_rng_state(model.device) if model.device.type == "cuda" else None,
        "environment_resume": "fresh_episodes",
        "initialization": initialization,
    }
    project = Path(__file__).resolve().parent.parent
    tracked = ("config/robot.yaml", "simulation/generated/sweetie_mini.xml", "learning/env.py", "learning/native.cpp", "learning/ppo.py", "learning/sensors.py", "learning/rewards.py", "hardware/observation.py")
    saved["source_hashes"] = {name: hashlib.sha256((project/name).read_bytes()).hexdigest()
                              for name in tracked if (project/name).exists()}
    if env is not None:
        model_path = getattr(env, "model_path", None)
        saved["environment_metadata"] = {
            "task": getattr(env, "task", config.task),
            "observation_dimension": model.obs_dim,
            "observation_contract": getattr(env,"observation_contract",None),
            "reward_version": getattr(env,"reward_version",None),
            "dynamics_signature": __import__("learning.readiness",fromlist=["dynamics_signature"]).dynamics_signature() if getattr(env,"observation_contract",None) else None,
            "action_dimension": model.action_dim,
            "model_path": str(model_path) if model_path is not None else None,
            "model_sha256": hashlib.sha256(Path(model_path).read_bytes()).hexdigest() if model_path is not None else None,
            "terrain": {key: value for key, value in getattr(env, "terrain", {}).items() if key != "height_samples"},
        }
    if env is not None and callable(getattr(env, "state_dict", None)):
        saved["environment"] = env.state_dict()
        saved["environment_resume"] = "state_dict"
    temporary = path.with_suffix(path.suffix + ".tmp")
    torch.save(saved, temporary)
    os.replace(temporary, path)


def _json_value(value):
    if isinstance(value, (np.ndarray, torch.Tensor)):
        value = value.detach().cpu().numpy() if isinstance(value, torch.Tensor) else value
        return float(np.mean(value)) if value.size else None
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): _json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(v) for v in value]
    return value


def train(env, config: TrainConfig, process_started: float | None = None) -> dict[str, Any]:
    # --seconds governs useful learning time; the driver separately charges all
    # import/initialization/evaluation wall time against the fixed global cap.
    if getattr(env,"observation_contract",None):
        from learning.readiness import require_training_ready
        require_training_ready()
    process_started = time.monotonic() if process_started is None else process_started
    if config.seconds <= 0 or config.rollout_steps < 1 or config.epochs < 1:
        raise ValueError("Positive seconds, rollout_steps and epochs required")
    if config.resume and config.initialize_from:
        raise ValueError("--resume and --initialize-from are mutually exclusive")
    torch.set_num_threads(2)
    device = torch.device(config.device)
    if device.type == "cuda":
        # Restrict this process without allocating a giant reservation.
        index = device.index or 0
        total = torch.cuda.get_device_properties(index).total_memory
        torch.cuda.set_per_process_memory_fraction(min(1.0, config.gpu_memory_gib * 2**30 / total), index)
    random.seed(config.seed)
    np.random.seed(config.seed)
    torch.manual_seed(config.seed)
    model = Policy(env.obs_dim, env.action_dim).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=config.learning_rate, eps=1e-5)
    steps = updates = 0
    prior_wall = 0.0
    restored_env = False
    initialization = None
    if config.initialize_from:
        donor_path = Path(config.initialize_from)
        # Hash the exact bytes loaded, even if a training writer atomically
        # replaces the donor's latest.pt while this process initializes.
        donor_bytes = donor_path.read_bytes()
        saved = torch.load(io.BytesIO(donor_bytes), map_location=device, weights_only=False)
        if saved["obs_dim"] != env.obs_dim or saved["action_dim"] != env.action_dim:
            raise ValueError("Initializer observation/action dimensions differ")
        validate_environment_contract(saved,env)
        donor_task = saved.get("config", {}).get("task")
        if donor_task != config.task and (donor_task, config.task) != ("locomotion", "running"):
            raise ValueError("Cross-task initialization only supports locomotion to running")
        model.load_state_dict(saved["policy"])
        # Normalizer is part of policy state. Optimizer, RNG, counters and
        # episodes deliberately remain those of this newly seeded experiment.
        initialization = {
            "mode": "weights_and_observation_normalizer_only",
            "donor_path": str(donor_path.resolve()),
            "donor_sha256": hashlib.sha256(donor_bytes).hexdigest(),
            "donor_config": saved.get("config", {}),
            "donor_steps": saved.get("steps"),
            "donor_updates": saved.get("updates"),
        }
    if config.resume:
        saved = _safe_load(config.resume, device)
        if saved["obs_dim"] != env.obs_dim or saved["action_dim"] != env.action_dim:
            raise ValueError("Resume observation/action dimensions differ")
        validate_environment_contract(saved,env)
        old = saved.get("config", {})
        if old.get("task", config.task) != config.task:
            raise ValueError("Cannot resume another task; use --initialize-from for locomotion to running")
        model.load_state_dict(saved["policy"])
        optimizer.load_state_dict(saved["optimizer"])
        steps, updates = saved["steps"], saved["updates"]
        prior_wall = saved.get("wall_seconds", 0.0)
        initialization = saved.get("initialization")
        random.setstate(saved["python_rng"])
        np.random.set_state(saved["numpy_rng"])
        torch.set_rng_state(saved["torch_rng"].cpu())
        if device.type == "cuda" and saved.get("cuda_rng") is not None:
            torch.cuda.set_rng_state(saved["cuda_rng"].cpu(), device)
        if "environment" in saved and callable(getattr(env, "load_state_dict", None)):
            env.load_state_dict(saved["environment"])
            restored_env = True
    out = Path(config.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "config.json").write_text(json.dumps(asdict(config), indent=2) + "\n")
    obs = env.observation() if restored_env and callable(getattr(env, "observation", None)) else env.reset()
    obs = np.asarray(obs, dtype=np.float32)
    if obs.shape != (env.num_envs, env.obs_dim):
        raise ValueError(f"Unexpected reset observation shape {obs.shape}")
    initial_steps = steps
    last_save = time.monotonic()
    status = "budget_exhausted"
    last_metrics: dict[str, Any] = {}
    start = time.monotonic()
    initialization_seconds = start-process_started
    try:
        with (out / "metrics.jsonl").open("a", buffering=1) as logfile:
            while time.monotonic() - start < config.seconds:
                if config.max_steps is not None and steps >= config.max_steps:
                    status = "max_steps_reached"
                    break
                fraction = min(1.0, (time.monotonic() - start) / config.seconds)
                if callable(getattr(env, "set_progress", None)):
                    env.set_progress(fraction)
                # Frozen normalizer throughout collection and all associated updates.
                model.update_norm(obs)
                observations, actions, raw_actions = [], [], []
                logprobs, values, rewards, dones = [], [], [], []
                episode_metrics: dict[str, list[float]] = {}
                for _ in range(config.rollout_steps):
                    if time.monotonic() - start >= config.seconds:
                        break
                    if config.max_steps is not None and steps >= config.max_steps:
                        break
                    observation = model.tensor(obs)
                    with torch.no_grad():
                        action, logprob, value, raw = model.sample(observation)
                    next_obs, reward, done, info = env.step(action.cpu().numpy())
                    reward = torch.as_tensor(reward, dtype=torch.float32, device=device).clone()
                    done_tensor = torch.as_tensor(done, dtype=torch.float32, device=device)
                    truncated = np.asarray(info.get("truncated", np.zeros(env.num_envs, dtype=bool)), dtype=bool)
                    if truncated.any():
                        if "terminal_observation" not in info:
                            raise ValueError("Timeout requires terminal_observation for correct bootstrap")
                        with torch.no_grad():
                            terminal_value = model.value(info["terminal_observation"])
                        reward += config.gamma * terminal_value * model.tensor(truncated)
                    if not torch.isfinite(reward).all() or not np.isfinite(next_obs).all():
                        raise FloatingPointError("Nonfinite environment transition")
                    observations.append(observation)
                    actions.append(action)
                    raw_actions.append(raw)
                    logprobs.append(logprob)
                    values.append(value)
                    rewards.append(reward)
                    dones.append(done_tensor)
                    for key, val in info.get("episode", {}).items():
                        array = np.asarray(val, dtype=float).reshape(-1)
                        finite = array[np.isfinite(array)]
                        if len(finite):
                            episode_metrics.setdefault(key, []).extend(finite.tolist())
                    obs = np.asarray(next_obs, dtype=np.float32)
                    steps += env.num_envs
                if not rewards:
                    break
                with torch.no_grad():
                    advantages, returns = compute_gae(torch.stack(rewards), torch.stack(dones), torch.stack(values), model.value(obs), config.gamma, config.gae_lambda)
                flat_obs = torch.cat(observations)
                flat_actions = torch.cat(actions)
                flat_raw = torch.cat(raw_actions)
                flat_logprobs = torch.cat(logprobs)
                flat_values = torch.cat(values)
                flat_returns = returns.flatten()
                flat_advantages = advantages.flatten()
                flat_advantages = (flat_advantages - flat_advantages.mean()) / (flat_advantages.std(unbiased=False) + 1e-8)
                losses, kls, clips = [], [], []
                stop_update = False
                for _ in range(config.epochs):
                    permutation = torch.randperm(len(flat_obs), device=device)
                    for begin in range(0, len(flat_obs), config.minibatch_size):
                        if time.monotonic() - start >= config.seconds:
                            stop_update = True
                            break
                        ids = permutation[begin:begin + config.minibatch_size]
                        new_logprob, entropy, new_value = model.evaluate_actions(flat_obs[ids], flat_actions[ids], flat_raw[ids])
                        logratio = new_logprob - flat_logprobs[ids]
                        ratio = logratio.exp()
                        surrogate = torch.minimum(ratio * flat_advantages[ids], ratio.clamp(1-config.clip, 1+config.clip) * flat_advantages[ids])
                        clipped_value = flat_values[ids] + (new_value-flat_values[ids]).clamp(-config.clip, config.clip)
                        value_loss = 0.5 * torch.maximum((new_value-flat_returns[ids]).square(), (clipped_value-flat_returns[ids]).square()).mean()
                        loss = -surrogate.mean() + config.value_coefficient * value_loss - config.entropy * entropy.mean()
                        if not torch.isfinite(loss):
                            raise FloatingPointError("Nonfinite PPO objective")
                        optimizer.zero_grad(set_to_none=True)
                        loss.backward()
                        grad_norm = nn.utils.clip_grad_norm_(model.parameters(), 1.0)
                        if not torch.isfinite(grad_norm):
                            raise FloatingPointError("Nonfinite PPO gradients")
                        optimizer.step()
                        losses.append(loss.detach())
                        kls.append(((ratio-1)-logratio).mean().detach())
                        clips.append(((ratio-1).abs() > config.clip).float().mean().detach())
                    if stop_update:
                        break
                updates += 1
                elapsed = time.monotonic() - start
                last_metrics = {
                    "steps": steps, "updates": updates, "wall_seconds": elapsed,
                    "initialization_seconds": initialization_seconds,
                    "process_wall_seconds": time.monotonic()-process_started,
                    "total_wall_seconds": prior_wall+time.monotonic()-process_started,
                    "transitions_per_second": (steps-initial_steps)/max(elapsed, 1e-6),
                    "reward_mean": float(torch.stack(rewards).mean().cpu()),
                    "loss": float(torch.stack(losses).mean().cpu()) if losses else None,
                    "approx_kl": float(torch.stack(kls).mean().cpu()) if kls else None,
                    "clip_fraction": float(torch.stack(clips).mean().cpu()) if clips else None,
                    "normal_entropy_mean": float(model.distribution(obs).entropy().sum(-1).mean().detach().cpu()),
                    "episode": {key: {"mean": float(np.mean(value)), "count": len(value)} for key, value in episode_metrics.items()},
                }
                logfile.write(json.dumps(last_metrics, allow_nan=False) + "\n")
                print(json.dumps(last_metrics, allow_nan=False), flush=True)
                if time.monotonic()-last_save >= config.save_interval:
                    _checkpoint(out / "latest.pt", model, optimizer, config, steps, updates, prior_wall+time.monotonic()-process_started, env, initialization)
                    last_save = time.monotonic()
    except KeyboardInterrupt:
        status = "interrupted"
    except BaseException:
        _checkpoint(out / "failure.pt", model, optimizer, config, steps, updates, prior_wall+time.monotonic()-process_started, env, initialization)
        raise
    finally:
        elapsed = time.monotonic() - start
        _checkpoint(out / "latest.pt", model, optimizer, config, steps, updates, prior_wall+time.monotonic()-process_started, env, initialization)
    result = {"status": status, "steps": steps, "updates": updates, "wall_seconds": elapsed,
              "initialization_seconds": initialization_seconds,
              "process_wall_seconds": time.monotonic()-process_started,
              "total_wall_seconds": prior_wall+time.monotonic()-process_started,
              "latest_checkpoint": str((out / "latest.pt").resolve()),
              "resume_environment": "restored" if restored_env else "fresh_episodes", "metrics": last_metrics}
    (out / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=("locomotion", "running", "recovery"), default="locomotion")
    parser.add_argument("--out", required=True)
    parser.add_argument("--nenvs", type=int, default=128)
    parser.add_argument("--threads", type=int, default=16)
    parser.add_argument("--seconds", type=float, default=1200)
    parser.add_argument("--max-steps", type=int)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--terrain", choices=("flat", "mixed", "rough", "slope", "steps"), default="flat")
    parser.add_argument("--difficulty", type=float, default=0.0)
    parser.add_argument("--no-randomize", action="store_false", dest="randomize")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--resume")
    source.add_argument("--initialize-from")
    parser.add_argument("--rollout-steps", type=int, default=64)
    parser.add_argument("--epochs", type=int, default=4)
    parser.add_argument("--minibatch-size", type=int, default=2048)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--entropy", type=float, default=0.005)
    parser.add_argument("--save-interval", type=float, default=60)
    parser.add_argument("--gpu-memory-gib", type=float, default=2.0)
    config = TrainConfig(**vars(parser.parse_args()))
    from learning.readiness import require_training_ready
    require_training_ready()
    # Import lazily: CPU inference tools need only this module and torch/numpy.
    from learning.env import SweetieVecEnv
    env = SweetieVecEnv(task=config.task, nenvs=config.nenvs, threads=config.threads,
                        seed=config.seed, terrain=config.terrain,
                        difficulty=config.difficulty, randomize=config.randomize)
    try:
        print(json.dumps(train(env, config, process_started=MODULE_STARTED), indent=2), flush=True)
    finally:
        if callable(getattr(env, "close", None)):
            env.close()


if __name__ == "__main__":
    main()

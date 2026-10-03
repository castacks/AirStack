"""Checkpoint loaders for deployable policies."""

from __future__ import annotations

from io import BytesIO
from pathlib import Path
from typing import Any, Tuple
from zipfile import ZipFile

import numpy as np


def load_torchscript(checkpoint_path: str) -> Any:
    """Load a TorchScript actor, accepting either the file or its directory.

    Searches ``exported_actor.pt2`` under the directory when a folder is given.
    """
    import torch

    path = Path(checkpoint_path)
    if not path.is_file():
        for candidate in (path / 'checkpoints' / 'exported_actor.pt2',
                          path / 'exported_actor.pt2'):
            if candidate.exists():
                path = candidate
                break
        else:
            raise FileNotFoundError(
                f'no exported_actor.pt2 under {checkpoint_path}')
    module = torch.jit.load(str(path))
    module.eval()
    return module


def load_sb3_ppo(
    model_path: str,
    obs_dim: int,
    action_low,
    action_high,
) -> Tuple[Any, bool]:
    """Load a Stable-Baselines3 PPO archive, exposing ``predict``.

    Falls back to a weights-only policy rebuild when the archive was pickled
    under a NumPy the running interpreter cannot unpickle; the returned flag is
    True when that fallback was used.
    """
    from stable_baselines3 import PPO

    try:
        return PPO.load(model_path, device='cpu'), False
    except ModuleNotFoundError as exc:
        if not exc.name or not exc.name.startswith('numpy._core'):
            raise

    import torch
    from gymnasium.spaces import Box
    from stable_baselines3.common.policies import ActorCriticPolicy

    observation_space = Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)
    action_space = Box(
        low=np.asarray(action_low, dtype=np.float32),
        high=np.asarray(action_high, dtype=np.float32),
        dtype=np.float32,
    )
    policy = ActorCriticPolicy(observation_space, action_space, lr_schedule=lambda _: 0.0)
    with ZipFile(model_path) as archive:
        state_dict = torch.load(
            BytesIO(archive.read('policy.pth')),
            map_location='cpu',
            weights_only=True,
        )
    policy.load_state_dict(state_dict)
    policy.eval()
    return policy, True

"""Run SAFE's own velocity baselines through AirStack's trajectory interface.

The policy code is imported from a SAFE source checkout; it is never copied
into AirStack. Its output is in m/s, while AirstackStackEnv accepts a velocity
normalized by its configured max speed. This preserves the policy's requested
velocity until AirStack's controller clamps it.
"""

from __future__ import annotations

import numpy as np

from safe_core.core.policy import Policy


def _swap_xy(value):
    arr = np.asarray(value).copy()
    if arr.size:
        arr[..., [0, 1]] = arr[..., [1, 0]]
    return arr


def _to_safe_benchmark_frame(obs):
    """SAFE's aerial benchmark position convention swaps Isaac X/Y."""
    converted = dict(obs)
    for key in ('agent', 'target', 'agent_vel', 'static_obstacles',
                'static_cylinders'):
        if key in converted and converted[key] is not None:
            converted[key] = _swap_xy(converted[key])
    if obs.get('dynamic_obstacles') is not None:
        dynamic = np.asarray(obs['dynamic_obstacles']).copy()
        if dynamic.size:
            dynamic[..., [0, 1]] = dynamic[..., [1, 0]]
            if dynamic.shape[-1] >= 6:
                dynamic[..., [3, 4]] = dynamic[..., [4, 3]]
        converted['dynamic_obstacles'] = dynamic
    if obs.get('static_obstacle_aabbs') is not None:
        aabb = np.asarray(obs['static_obstacle_aabbs']).copy()
        if aabb.size:
            aabb[..., [0, 1, 2, 3]] = aabb[..., [1, 0, 3, 2]]
        converted['static_obstacle_aabbs'] = aabb
    if obs.get('env_bounds') is not None:
        bounds = np.asarray(obs['env_bounds']).copy()
        bounds[[0, 1, 3, 4]] = bounds[[1, 0, 4, 3]]
        converted['env_bounds'] = bounds
    return converted


class _SafeVelocity(Policy):
    sensors = ['full_state']
    source_name = ''

    def __init__(self):
        try:
            import sys
            from pathlib import Path
            import safe_core
            source_root = Path(safe_core.__file__).resolve().parent.parent
            if str(source_root) not in sys.path:
                sys.path.insert(0, str(source_root))
            from integrations.aerial_nav.agents import baselines
        except ImportError as exc:
            raise RuntimeError(
                'SAFE aerial policy source is unavailable. Install SAFE from its '
                'source checkout or add the checkout to PYTHONPATH; the safe_core '
                'wheel alone does not include integrations/aerial_nav.'
            ) from exc
        self._source = getattr(baselines, self.source_name)()
        self._source.reset()

    def reset(self):
        self._source.reset()

    def act(self, obs):
        max_speed = float(obs['max_speed'])
        if max_speed <= 0:
            raise ValueError('max_speed must be positive')
        safe_obs = _to_safe_benchmark_frame(obs)
        if hasattr(self, "_env"):
            self._source._env = self._env
        velocity_benchmark = np.asarray(self._source.act(safe_obs), dtype=np.float32)
        return _swap_xy(velocity_benchmark) / max_speed


class SafeRandom(_SafeVelocity):
    source_name = 'Random'


class SafeAggressive(_SafeVelocity):
    source_name = 'Aggressive'


class SafeConservative(_SafeVelocity):
    source_name = 'Conservative'


class SafeAdaptiveApf(_SafeVelocity):
    source_name = 'AdaptiveApf'


class SafeAdaptive(_SafeVelocity):
    """SAFE main-study Adaptive baseline with the same source policy logic."""

    source_name = "Adaptive"
    sensors = ["full_state", "lidar", "imu", "rgb", "depth", "tof_depth"]

"""SAFE learned policies executed from the SAFE source checkout in AirStack.

Observation frame adaptation is explicit; models and checkpoints remain in SAFE.
The ROS bridge must supply a real depth image and an FRD/NED IMU sample.
"""
from __future__ import annotations

import numpy as np
from safe_core.core.policy import Policy

from .safe_source import _to_safe_benchmark_frame, _swap_xy


class _SafeLearned(Policy):
    source_module = ''
    source_class = ''
    sensors = ['odometry', 'goal', 'depth', 'imu']
    action_type = 'attitude_thrust'

    def __init__(self):
        try:
            import importlib
            module = importlib.import_module(
                f'integrations.aerial_nav.agents.learned.{self.source_module}')
        except ImportError as exc:
            raise RuntimeError('SAFE aerial policy source/dependencies are unavailable') from exc
        self._source = getattr(module, self.source_class)()
        self.pov_env = dict(getattr(self._source, 'pov_env', {}))

    def reset(self):
        self._source.reset()

    def act(self, obs):
        if hasattr(self._source, '_env'):
            self._source._env = getattr(self, '_env', None)
        if obs.get('depth') is None:
            raise RuntimeError(f'{self.source_class} requires a real AirStack depth frame')
        if obs.get('imu') is None:
            raise RuntimeError(f'{self.source_class} requires an AirStack IMU sample')
        safe_obs = _to_safe_benchmark_frame(obs)
        safe_obs['velocity'] = _swap_xy(obs.get('velocity', obs['agent_vel']))
        expected = self.pov_env.get('SAFE_POV_RES')
        if expected and np.asarray(obs['depth']).shape != (int(expected), int(expected)):
            raise RuntimeError(
                f'{self.source_class} requires {expected}x{expected} depth with its '
                f'training camera intrinsics; received {np.asarray(obs["depth"]).shape}. '
                'Configure AirStack camera geometry before running this policy.')
        return self._source.act(safe_obs)


class SafeDiffPhysDrone(_SafeLearned):
    source_module = 'agent_diffphysdrone'
    source_class = 'DiffPhysDrone'


class SafeDepthNav(_SafeLearned):
    source_module = 'agent_depthnav'
    source_class = 'DepthNav'


class SafeDiffAero(_SafeLearned):
    source_module = 'agent_diffaero'
    source_class = 'DiffAero'
    required_dt_s = 1.0 / 30.0


class SafeAgileAutonomy(_SafeLearned):
    source_module = 'agent_agile_autonomy'
    source_class = 'AgileAutonomy'


class SafeVitfly(_SafeLearned):
    source_module = 'agent_vitfly'
    source_class = 'Vitfly'
    action_type = 'velocity'

    def act(self, obs):
        # The source Vitfly returns a SAFE-frame velocity in m/s.
        velocity = np.asarray(super().act(obs), dtype=np.float32)
        return _swap_xy(velocity) / float(obs['max_speed'])

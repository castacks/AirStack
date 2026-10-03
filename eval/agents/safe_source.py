"""Run SAFE's own velocity baselines through AirStack's trajectory interface.

The policy code is imported from a SAFE source checkout; it is never copied
into AirStack. Its output is in m/s, while AirstackStackEnv accepts a velocity
normalized by its configured max speed. This preserves the policy's requested
velocity until AirStack's controller clamps it.
"""

from __future__ import annotations

import numpy as np

from safe_core.core.policy import Policy


class _SafeVelocity(Policy):
    sensors = ['full_state']
    source_name = ''

    def __init__(self):
        try:
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
        velocity_m_s = np.asarray(self._source.act(obs), dtype=np.float32)
        return velocity_m_s / max_speed


class SafeRandom(_SafeVelocity):
    source_name = 'Random'


class SafeAggressive(_SafeVelocity):
    source_name = 'Aggressive'


class SafeConservative(_SafeVelocity):
    source_name = 'Conservative'

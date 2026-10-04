"""SAFE's published ROS planner policies, driven by AirStack observations.

The planner and its sidecar remain in the SAFE checkout. AirStack supplies
its own sensor data in SAFE's expected observation encoding and executes the
returned setpoint through its own stack.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
import safe_core

# Editable SAFE installs put safe_core on sys.path, but their repository-level
# integrations/ package is not itself installed. Resolve it from that checkout.
_source_root = Path(safe_core.__file__).resolve().parent.parent
if not (_source_root / 'integrations' / 'aerial_nav').is_dir():
    raise RuntimeError('SAFE aerial integration source checkout is required for planners')
if str(_source_root) not in sys.path:
    sys.path.insert(0, str(_source_root))

from .safe_source import _to_safe_benchmark_frame, _swap_xy
from integrations.aerial_nav.agents.planners import agent_fast_planner as _fast_planner
from integrations.aerial_nav.agents.planners import agent_ego_planner as _ego_planner
from integrations.aerial_nav.agents.planners import agent_raptor as _raptor
from integrations.aerial_nav.agents.planners import agent_faster as _faster
from integrations.aerial_nav.agents.planners import agent_super as _super
from integrations.aerial_nav.agents.planners import agent_mighty as _mighty
from integrations.aerial_nav.agents.planners import agent_dynus as _dynus
from integrations.aerial_nav.agents.planners import agent_panther as _panther


class _AirStackPlanner:
    """Frame bridge shared by all SAFE sidecar policies.

    SAFE's benchmark frame swaps Isaac X/Y, while AirStack speaks ENU. SAFE's
    source planner consumes the benchmark frame and emits a compass-heading
    setpoint; AirStack gets back ENU vectors and ENU yaw.
    """

    def act(self, obs):
        # Keep pointcloud in ENU: SAFE's sidecar client explicitly expects it
        # already in ENU even though agent and target use benchmark axes.
        result = super().act(_to_safe_benchmark_frame(obs))
        if not isinstance(result, dict):
            return result  # source policy's no-command hover fallback
        action = dict(result)
        for key in ('position', 'velocity', 'acceleration'):
            action[key] = _swap_xy(result[key])
        action['heading'] = math.pi / 2 - float(result['heading'])
        if result.get('heading_rate') is not None:
            action['heading_rate'] = -float(result['heading_rate'])
        return action


class SafeFastPlanner(_AirStackPlanner, _fast_planner.FastPlanner):
    pass


class SafeEgoPlanner(_AirStackPlanner, _ego_planner.EgoPlanner):
    pass


class SafeRaptor(_AirStackPlanner, _raptor.Raptor):
    pass


class SafeFaster(_AirStackPlanner, _faster.Faster):
    pass


class SafeSuper(_AirStackPlanner, _super.Super):
    pass


class SafeMighty(_AirStackPlanner, _mighty.Mighty):
    pass


class SafeDynus(_AirStackPlanner, _dynus.Dynus):
    pass


class SafePanther(_AirStackPlanner, _panther.Panther):
    pass


PLANNER_NAMES = {
    'safefastplanner': 'fastplanner',
    'safeegoplanner': 'egoplanner',
    'saferaptor': 'raptor',
    'safefaster': 'faster',
    'safesuper': 'super',
    'safemighty': 'mighty',
    'safedynus': 'dynus',
    'safepanther': 'panther',
}

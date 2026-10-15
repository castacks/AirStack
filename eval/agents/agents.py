"""Agents for the AirStack × S.A.F.E. integration.

Two kinds:

Stack agents — the planner-under-test IS the AirStack autonomy stack. The
    Policy is a passthrough (act() returns zeros; the env ignores it): the
    drone is flown by the stack itself, fed a real global plan. Which planner
    runs is a bring-up profile, not a bridge.

Velocity baselines — vendored from the S.A.F.E. reference integration
    (integrations/navigation/aerial/policies/baselines.py). Their normalized velocity
    actions are executed as trajectory_override segments on the real
    trajectory controller, so baselines and planners share the same
    interface layer and the comparison is apples-to-apples.

The runner looks up each agent's bring-up profile in PROFILES by lowercase
class name (one process per agent — safe_core's multi-agent suite spawns
child processes, so a single bring-up config per process is enough).
"""

from __future__ import annotations

import numpy as np

from safe_core.core.policy import Policy

from .baselines import Random, Aggressive, Conservative  # noqa: F401  (auto-discovered)
from .safe_source import (SafeRandom, SafeAggressive, SafeConservative,
                          SafeAdaptiveApf, SafeAdaptive)  # noqa: F401
from .learned_source import (SafeDiffPhysDrone, SafeDepthNav, SafeDiffAero,
                             SafeVitfly, SafeAgileAutonomy)  # noqa: F401
from .planner_source import (SafeFastPlanner, SafeEgoPlanner, SafeRaptor,
                             SafeFaster, SafeSuper, SafeMighty, SafeDynus,
                             SafePanther, SafeSando)  # noqa: F401


class DROAN(Policy):
    """DROAN through the full stack: ZED → perception → droan_gl →
    trajectory controller → PX4. Launched by the default AirStack bring-up
    (role full), so no extra commands are needed."""

    sensors = ["full_state"]

    def act(self, obs: dict) -> np.ndarray:
        return np.zeros(3, dtype=np.float32)

    def reset(self) -> None:
        pass


# Bring-up profile per lowercase agent class name. The env consumes:
#   control_mode  "stack" (env publishes global_plan, ignores actions) or
#                 "velocity" (actions → trajectory_override)
#   agent_env     extra env vars for `airstack up`
#   post_up_cmds  commands run in the robot container after readiness
PROFILES: dict[str, dict] = {
    "droan": {
        "name": "droan",
        "control_mode": "stack",
        "agent_env": {"AUTONOMY_ROLE": "full"},
        "post_up_cmds": [],
    },
    # Velocity baselines fly through the trajectory controller; the local
    # planner's own output is unused (it has no global plan to follow).
    "random":       {"name": "random",       "control_mode": "velocity"},
    "aggressive":   {"name": "aggressive",   "control_mode": "velocity"},
    "conservative": {"name": "conservative", "control_mode": "velocity"},
    "saferandom":       {"name": "saferandom",       "control_mode": "velocity", "max_speed": 10.0},
    "safeaggressive":   {"name": "safeaggressive",   "control_mode": "velocity", "max_speed": 10.0},
    "safeconservative": {"name": "safeconservative", "control_mode": "velocity", "max_speed": 10.0},
    "safeadaptiveapf":  {"name": "safeadaptiveapf",  "control_mode": "velocity", "max_speed": 10.0},
    "safeadaptive":     {"name": "safeadaptive", "control_mode": "velocity", "max_speed": 10.0,
                         "agent_env": {"ENABLE_LIDAR": "true"}},
    **{name: ({"name": name, "control_mode": "setpoint",
               "agent_env": {"ENABLE_LIDAR": "true"}}
              if name in ("safesuper", "safemighty", "safedynus",
                          "safeegoplanner", "safesando")
              else {"name": name, "control_mode": "setpoint"})
       for name in ("safefastplanner", "safeegoplanner", "saferaptor",
                    "safefaster", "safesuper", "safemighty", "safedynus",
                    "safepanther", "safesando")},
    **{name: {"name": name, "control_mode": "attitude_thrust"} for name in (
        "safediffphysdrone", "safedepthnav", "safediffaero",
        "safeagileautonomy")},
    "safevitfly": {"name": "safevitfly", "control_mode": "velocity", "max_speed": 10.0},
}


def get_profile(agent_name: str | None) -> dict:
    if not agent_name:
        return PROFILES["droan"]
    profile = PROFILES.get(agent_name.lower())
    if profile is None:
        raise SystemExit(
            f"No bring-up profile for agent '{agent_name}'. "
            f"Known: {', '.join(sorted(PROFILES))}")
    profile = {**profile, "agent_env": dict(profile.get("agent_env") or {})}
    if agent_name.lower().startswith("safe"):
        profile["agent_env"]["SAFE_EVAL_STATE_HZ"] = "60"
    # Read each trained camera contract from SAFE's policy class, so changing a
    # SAFE checkpoint/camera definition does not leave an AirStack copy stale.
    learned = {
        "safediffphysdrone": SafeDiffPhysDrone,
        "safedepthnav": SafeDepthNav,
        "safediffaero": SafeDiffAero,
        "safevitfly": SafeVitfly,
        "safeagileautonomy": SafeAgileAutonomy,
    }
    wrapper = learned.get(agent_name.lower())
    if wrapper is not None:
        import importlib
        from pathlib import Path
        import sys
        import safe_core
        root = Path(safe_core.__file__).resolve().parent.parent
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        module = importlib.import_module(
            f"integrations.navigation.aerial.policies.learned.{wrapper.source_module}")
        source = getattr(module, wrapper.source_class)
        profile["agent_env"].update(getattr(source, "pov_env", {}))
        camera_hz = profile["agent_env"].get("SAFE_POV_HZ")
        if camera_hz is not None:
            profile["agent_env"]["PX4_RENDERING_HZ"] = str(camera_hz)
    return profile

"""Agents for the AirStack × S.A.F.E. integration.

Two kinds:

Stack agents — the planner-under-test IS the AirStack autonomy stack. The
    Policy is a passthrough (act() returns zeros; the env ignores it): the
    drone is flown by the stack itself, fed a real global plan. Which planner
    runs is a bring-up profile, not a bridge.

SAFE policies — SAFE's own oracles, learned policies and planners under their
    SAFE names, wrapped so AirStack's stack executes their actions. Their
    profiles carry "source": "safe".

The runner looks up each agent's bring-up profile in PROFILES by lowercase
class name (one process per agent — safe_core's multi-agent suite spawns
child processes, so a single bring-up config per process is enough).
"""

from __future__ import annotations

import numpy as np

from safe_core.core.policy import Policy

from .baselines import AirstackRandom, AirstackAggressive, AirstackConservative  # noqa: F401  (auto-discovered)
from .safe_source import (Random, Aggressive, Conservative,
                          AdaptiveApf, Adaptive)  # noqa: F401
from .learned_source import (DiffPhysDrone, DepthNav, DiffAero,
                             Vitfly, AgileAutonomy)  # noqa: F401
from .planner_source import (FastPlanner, EgoPlanner, Raptor,
                             Faster, Super, Mighty, Dynus,
                             Panther, Sando)  # noqa: F401


class DROAN(Policy):
    """DROAN through the full stack: ZED → perception → droan_gl →
    trajectory controller → PX4. Launched by the default AirStack bring-up
    (role full), so no extra commands are needed."""

    sensors = ["full_state"]

    def act(self, obs: dict) -> np.ndarray:
        return np.zeros(3, dtype=np.float32)

    def reset(self) -> None:
        pass


def _safe(name: str, mode: str, **extra) -> dict:
    """The bring-up profile of a SAFE policy flown by AirStack's stack."""
    return {"name": name, "control_mode": mode, "source": "safe", **extra}


# Bring-up profile per lowercase agent class name. The env consumes:
#   control_mode  "stack" (env publishes global_plan, ignores actions),
#                 "velocity" (actions → trajectory_override), "setpoint" or "attitude_thrust"
#   source        "safe" for SAFE's policies, which step Isaac one tick per act() and need the
#                 60 Hz state stream
#   agent_env     extra env vars for `airstack up`
#   post_up_cmds  commands run in the robot container after readiness
LIDAR = {"agent_env": {"ENABLE_LIDAR": "true"}}
PROFILES: dict[str, dict] = {
    "droan": {
        "name": "droan",
        "control_mode": "stack",
        "agent_env": {"AUTONOMY_ROLE": "full"},
        "post_up_cmds": [],
        "restart_planner_each_episode": True,
    },
    # AirStack's own velocity baselines fly through the trajectory controller; the local
    # planner's own output is unused (it has no global plan to follow).
    "airstackrandom":       {"name": "airstackrandom",       "control_mode": "velocity"},
    "airstackaggressive":   {"name": "airstackaggressive",   "control_mode": "velocity"},
    "airstackconservative": {"name": "airstackconservative", "control_mode": "velocity"},
    "random":       _safe("random", "velocity", max_speed=10.0),
    "aggressive":   _safe("aggressive", "velocity", max_speed=10.0),
    "conservative": _safe("conservative", "velocity", max_speed=10.0),
    "adaptiveapf":  _safe("adaptiveapf", "velocity", max_speed=10.0),
    "adaptive":     _safe("adaptive", "velocity", max_speed=10.0, **LIDAR),
    **{name: _safe(name, "setpoint", **(LIDAR if name in ("super", "mighty", "dynus", "egoplanner", "sando") else {}))
       for name in ("fastplanner", "egoplanner", "raptor", "faster", "super", "mighty", "dynus", "panther", "sando")},
    **{name: _safe(name, "attitude_thrust") for name in ("diffphysdrone", "depthnav", "diffaero", "agileautonomy")},
    "vitfly": _safe("vitfly", "velocity", max_speed=10.0),
}


def is_safe_policy(agent_name: str | None) -> bool:
    """Whether the named agent is one of SAFE's policies rather than AirStack's own."""
    return bool(agent_name) and PROFILES.get(agent_name.lower(), {}).get("source") == "safe"


def get_profile(agent_name: str | None) -> dict:
    if not agent_name:
        return PROFILES["droan"]
    profile = PROFILES.get(agent_name.lower())
    if profile is None:
        raise SystemExit(
            f"No bring-up profile for agent '{agent_name}'. "
            f"Known: {', '.join(sorted(PROFILES))}")
    profile = {**profile, "agent_env": dict(profile.get("agent_env") or {})}
    if profile.get("source") == "safe":
        profile["agent_env"]["SAFE_EVAL_STATE_HZ"] = "60"
    # Read each trained camera contract from SAFE's policy class, so changing a
    # SAFE checkpoint/camera definition does not leave an AirStack copy stale.
    learned = {
        "diffphysdrone": DiffPhysDrone,
        "depthnav": DepthNav,
        "diffaero": DiffAero,
        "vitfly": Vitfly,
        "agileautonomy": AgileAutonomy,
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
            f"safe_navigation.aerial.policies.learned.{wrapper.source_module}")
        source = getattr(module, wrapper.source_class)
        profile["agent_env"].update(getattr(source, "pov_env", {}))
        camera_hz = profile["agent_env"].get("SAFE_POV_HZ")
        if camera_hz is not None:
            profile["agent_env"]["PX4_RENDERING_HZ"] = str(camera_hz)
    return profile

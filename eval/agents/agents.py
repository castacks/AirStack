"""Agents for the AirStack × S.A.F.E. integration.

Two kinds:

Stack agents — the planner-under-test IS the AirStack autonomy stack. The
    Policy is a passthrough (act() returns zeros; the env ignores it): the
    drone is flown by the stack itself, fed a real global plan. Which planner
    runs is a bring-up profile, not a bridge.

Velocity baselines — vendored from the S.A.F.E. reference integration
    (integrations/aerial_nav/agents/baselines.py). Their normalized velocity
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
    # TODO(super): a SuperPlanner profile needs a bring-up variant that
    # launches super_planner INSTEAD of droan_gl (two local planners fight
    # over the trajectory controller). local_bringup exposes no swap arg yet.
}


def get_profile(agent_name: str | None) -> dict:
    if not agent_name:
        return PROFILES["droan"]
    profile = PROFILES.get(agent_name.lower())
    if profile is None:
        raise SystemExit(
            f"No bring-up profile for agent '{agent_name}'. "
            f"Known: {', '.join(sorted(PROFILES))}")
    return profile

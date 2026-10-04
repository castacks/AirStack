"""AirStack × S.A.F.E. benchmark entry point.

Runs the REAL AirStack stack (Isaac Sim via AirStack's own bring-up + the
full autonomy stack in the robot container) as the system under test. Plain
python — no Isaac interpreter needed on the host, only `pip install
safe-benchmark` and the docker CLI.

Every run knob (agents, domains, grid, dt, keep_up, ...) is a key in an
experiment yaml; the CLI carries only actions and session facts:

    python eval/run.py --config eval/experiments/<name>.yaml -y
    python eval/run.py --list

Output: runs/airstack/<run_id>/ under the working directory (the job pool
in safe_core resolves the run root relative to cwd).
"""

from __future__ import annotations

import os
import sys

# Allow running as `python eval/run.py` from the repo root
if __name__ == "__main__" and (__package__ is None or __package__ == ""):
    _here = os.path.dirname(os.path.abspath(__file__))
    _repo_root = os.path.dirname(_here)
    if _repo_root not in sys.path:
        sys.path.insert(0, _repo_root)
    __package__ = "eval"
    import importlib
    importlib.import_module(__package__)

import argparse

from safe_core.core.runner import BaseRunner


class AirstackRunner(BaseRunner):

    def get_integration(self) -> str:
        return "airstack"

    def get_agents_dir(self) -> str:
        return os.path.join(os.path.dirname(__file__), "agents")

    def get_available_agents(self) -> list[str]:
        from .agents.agents import PROFILES
        return sorted(PROFILES)

    def add_args(self, parser: argparse.ArgumentParser) -> None:
        # Yaml-only run knobs: registered so the yaml auto-map finds the dest,
        # hidden from --help and rejected as flags by enforce_cli_surface().
        parser.add_argument("--epsilon-unsafe", type=float, default=0.3,
                            dest="epsilon_unsafe", help=argparse.SUPPRESS)
        parser.add_argument("--goal-tolerance", type=float, default=1.0,
                            dest="goal_tolerance", help=argparse.SUPPRESS)
        parser.add_argument("--takeoff-velocity", type=float, default=1.0,
                            dest="takeoff_velocity", help=argparse.SUPPRESS)
        parser.add_argument("--keep-up", action="store_true", dest="keep_up",
                            help=argparse.SUPPRESS)

    def available_domains(self, args) -> list[str]:
        from .sim.environment_config import get_domain_registry
        return sorted(get_domain_registry())

    def validate(self, args: argparse.Namespace) -> int:
        import integrations.aerial_nav.config.domains_isaac  # noqa: F401
        from integrations.aerial_nav.config.domains import scale_procedural_domains
        scale_procedural_domains(getattr(args, 'goal_distance', None), sim_backend='isaac')
        from .sim.environment_config import get_domain_registry
        available = list(get_domain_registry())
        if not args.domains:
            args.domains = available
            args.num_env_domains = len(available)
        else:
            unknown = [d for d in args.domains if d not in available]
            if unknown:
                raise SystemExit(f"Unknown domain(s): {unknown}. Available: {available}")
            args.num_env_domains = len(args.domains)
        selected = ([args.agent] if getattr(args, "agent", None) else
                    list(getattr(args, "agents", None) or []))
        if any(name.lower().startswith("safe") for name in selected):
            from safe_isaac.sim_config import PHYSICS_DT
            if args.env_dt is not None and abs(args.env_dt - PHYSICS_DT) > 1e-9:
                raise SystemExit(
                    f"[validate] SAFE Isaac policies call act() once per Isaac physics "
                    f"tick ({PHYSICS_DT:g} s); env_dt={args.env_dt:g} s would change "
                    "their control and sensor cadence. Set env_dt to "
                    f"{PHYSICS_DT:g} or omit it.")
            args.env_dt = PHYSICS_DT
        return int(args.max_episode_time / args.env_dt)

    def get_runs_root(self):
        return os.path.join(os.path.dirname(__file__), "runs")

    def get_env_config_module(self):
        from .sim import environment_config
        return environment_config

    def run_agent_hook(self, args: argparse.Namespace) -> None:
        """Start a selected SAFE planner's real ROS sidecar on the host.

        Its hook owns the planner container and publishes a verified TCP record.
        This runs before AirStack's Isaac/ROS bring-up; no planner is silently
        replaced with a kinematic baseline.
        """
        selected = (args.agent or
                    (args.agents[0] if args.agents and len(args.agents) == 1 else None))
        if not selected:
            return
        from .agents.planner_source import PLANNER_NAMES
        planner = PLANNER_NAMES.get(selected.lower()) or (
            "vitfly" if selected.lower() == "safevitfly" else None)
        if planner is None:
            return
        import subprocess
        from pathlib import Path
        from integrations.aerial_nav.agents.planners import sidecar_registry
        from safe_core.core.records import rulebook
        from safe_core.utils.plumbing import (
            MAP_ARENA_EXTENT_FALLBACK_M, MAP_MARGIN_M,
            MAP_Z_ABOVE_M, MAP_Z_BELOW_M,
        )
        from .sim.environment_config import get_domain_registry

        registry = get_domain_registry()
        domains = [registry[name] for name in (args.domains or list(registry))]
        goal = float(getattr(args, 'goal_distance', None) or 0.0)
        xy = (goal if goal > 0 else
              max((max(c.size_x, c.size_y) for c in domains),
                  default=MAP_ARENA_EXTENT_FALLBACK_M)) + 2 * MAP_MARGIN_M
        terrain = any(bool(c.usd_path) for c in domains)
        env = dict(os.environ)
        record_dir = Path(self.get_runs_root()) / '.sidecars'
        record_dir.mkdir(parents=True, exist_ok=True)
        env['SAFE_SIDECAR_RECORD_DIR'] = str(record_dir)
        env.setdefault('SAFE_MAP_SIZE_XY', f'{xy:.1f}')
        env.setdefault('SAFE_MAP_Z_MIN',
                       f'{-MAP_Z_BELOW_M if terrain else 0.0:.1f}')
        env.setdefault('SAFE_MAP_Z_MAX',
                       f'{rulebook().altitude_floor_m + MAP_Z_ABOVE_M:.1f}')
        hook = Path(sidecar_registry.HOOK_DIR) / f'{planner}.sh'
        if not hook.is_file():
            raise RuntimeError(f'SAFE planner hook is missing: {hook}')
        print(f'[airstack] Starting SAFE {planner} planner: {hook}', flush=True)
        subprocess.run(['bash', str(hook)], check=True, env=env)
        # The child runner and source policy must resolve the same sidecar map.
        for key in ('SAFE_MAP_SIZE_XY', 'SAFE_MAP_Z_MIN', 'SAFE_MAP_Z_MAX',
                    'SAFE_SIDECAR_RECORD_DIR'):
            os.environ[key] = env[key]
        # Hooks may publish their record just before the TCP listener binds.
        import time
        deadline = time.monotonic() + 120.0
        while True:
            fault = sidecar_registry.fault(planner)
            if fault is None:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(f'{planner} sidecar failed verification: {fault}')
            time.sleep(2.0)

    def build_adapter(self, args: argparse.Namespace):
        from .adapter import Adapter
        return Adapter()

    def build_env_ctx(self, args: argparse.Namespace, ctx: dict, max_steps: int,
                      sensor_union: list) -> dict:
        from .agents.agents import get_profile
        agent_name = args.agent or (args.agents[0] if args.agents else None)
        return {
            **ctx,
            "sensors":          sensor_union,
            "domains":          args.domains,
            "agent_profile":    get_profile(agent_name),
            "max_episode_time": args.max_episode_time,
            "env_dt":           args.env_dt,
            "max_steps":        max_steps,
            "epsilon_unsafe":   args.epsilon_unsafe,
            "goal_tolerance":   args.goal_tolerance,
            "takeoff_velocity": args.takeoff_velocity,
            "headless":         args.headless,
            "keep_up":          args.keep_up,
            "humans":           bool(getattr(args, "humans", False)),
            "goal_distance":    getattr(args, "goal_distance", None),
            "static_only":      bool(getattr(args, "static_only", False)),
        }


if __name__ == "__main__":
    AirstackRunner().main()

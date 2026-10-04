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
        return int(args.max_episode_time / args.env_dt)

    def get_runs_root(self):
        return os.path.join(os.path.dirname(__file__), "runs")

    def get_env_config_module(self):
        from .sim import environment_config
        return environment_config

    def run_agent_hook(self, args: argparse.Namespace) -> None:
        # No sidecars: the stack IS the environment; bring-up happens inside
        # AirstackStackEnv.reset() via `airstack up`.
        pass

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

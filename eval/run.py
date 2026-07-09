"""AirStack × S.A.F.E. benchmark entry point.

Usage — smoke test (one episode, DROAN planner, warehouse domain):

    pip install -e /path/to/benchmark

    ~/isaacsim/python.sh eval/run.py \\
        --agent airstackdroan \\
        --domains warehouse \\
        --num_scenarios 1 --num_perturbations 1 --max_episodes 1 \\
        --headless

Full comparison (DROAN vs Super Planner, two domains, full SAFE stats):

    ~/isaacsim/python.sh eval/run.py \\
        --agents airstackdroan airstacksuperplanner \\
        --domains warehouse hospital \\
        --num_scenarios 5 --num_perturbations 5 --max_episodes 10 \\
        --headless

Pure-Python baselines (no Isaac Sim needed):
    python eval/run.py --agent conservative --domains warehouse --simple -q

Output: runs/airstack/<run_id>/
    <agent>/   — per-agent RE/RP/CVaR plots, text summary, episodes.csv
    comparison.png / comparison_summary.txt (when multiple agents)
"""

from __future__ import annotations

import os
import sys

# Allow running as `~/isaacsim/python.sh eval/run.py` from the repo root
if __name__ == "__main__" and (__package__ is None or __package__ == ""):
    _here = os.path.dirname(os.path.abspath(__file__))
    _repo_root = os.path.dirname(_here)
    if _repo_root not in sys.path:
        sys.path.insert(0, _repo_root)
    __package__ = "eval"
    import importlib
    importlib.import_module(__package__)

import argparse

import safe_core.utils.config as config
from safe_core.core.runner import BaseRunner


class AirstackRunner(BaseRunner):

    def get_integration(self) -> str:
        return "airstack"

    def get_agents_dir(self) -> str:
        return os.path.join(os.path.dirname(__file__), "agents")

    def get_available_agents(self) -> list[str]:
        return [
            "random",
            "aggressive",
            "conservative",
            "adaptive",
            "adaptiveapf",
            "droan",
            "super",
        ]

    def add_args(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--direct-velocity",
            dest="direct_velocity",
            action="store_true",
            default=False,
            help="Bypass PX4 SITL and drive the drone directly via Isaac dynamic control.",
        )
        parser.add_argument(
            "--epsilon-unsafe",
            type=float,
            default=0.3,
            dest="epsilon_unsafe",
            help="Risk-proximate shell width δ (metres). Default: 0.3.",
        )

    def validate(self, args: argparse.Namespace) -> int:
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

    def setup(self, args: argparse.Namespace) -> tuple:
        from . import launch
        ctx = launch.setup(args)
        return ctx, launch

    def get_runs_root(self):
        return os.path.join(os.path.dirname(__file__), "runs")

    def get_env_config_module(self):
        from .sim import environment_config
        return environment_config

    def run_agent_hook(self, args: argparse.Namespace) -> None:
        import subprocess
        agent_name = args.agent or (args.agents[0] if args.agents else None)
        if not agent_name:
            return
        hook = os.path.join(os.path.dirname(__file__), "agent_hooks", f"{agent_name.lower()}.sh")
        if not os.path.isfile(hook):
            return
        print(f"[airstack] Starting agent hook in background: {hook}", flush=True)
        subprocess.Popen(["bash", hook])

    def build_adapter(self, args: argparse.Namespace):
        from .adapter import Adapter
        return Adapter()

    def build_env_ctx(self, args: argparse.Namespace, ctx: dict, max_steps: int,
                       sensor_union: list) -> dict:
        return {
            **ctx,
            "sensors":          sensor_union,
            "sim_backend":      "airstack_isaac",
            "max_episode_time": args.max_episode_time,
            "env_dt":           args.env_dt,
            "max_steps":        max_steps,
            "epsilon_unsafe":   getattr(args, "epsilon_unsafe", 0.3),
        }


if __name__ == "__main__":
    AirstackRunner().main()

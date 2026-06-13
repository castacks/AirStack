"""Isaac Sim bootstrap for AirStack × S.A.F.E.

Creates SimulationApp before any omni.* / isaacsim.* imports, then loads
the Pegasus Simulator extension that AirStack uses.  Returns a context dict
consumed by Adapter.build_env() and teardown().

AirStack's SimulatorManager (simulation/isaac-sim/…) handles PX4 SITL,
vehicle spawning, and world stepping — this module just boots the app and
loads the right extensions.

Usage (from eval/run.py):
    from . import launch
    ctx = launch.setup(args)
    # ... run benchmark ...
    launch.teardown(ctx, env, args)
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from typing import Any, Dict, Optional

import safe_core.utils.config as config

# Path overrides — can be set via environment variables
_ISAAC_ROOT = os.environ.get("ISAAC_PATH", os.path.expanduser("~/isaacsim"))
_AIRSTACK_ROOT = os.environ.get(
    "AIRSTACK_ROOT",
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
)
_PEGASUS_EXTS = os.environ.get(
    "PEGASUS_EXTENSIONS_PATH",
    os.path.join(_AIRSTACK_ROOT, "simulation", "isaac-sim", "extensions", "PegasusSimulator", "extensions"),
)


def _kill_stale_px4() -> None:
    if os.environ.get("SAFE_SKIP_PX4_PKILL", "").lower() in ("1", "true", "yes"):
        return
    for pat in ["px4_sitl", "ros_bridge_worker"]:
        subprocess.run(
            ["pkill", "-f", pat],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


def setup(args: Any) -> Dict[str, Any]:
    """Boot Isaac Sim, load Pegasus + AirStack extensions, warm up one drone.

    Returns context dict with keys:
        simulation_app — SimulationApp handle
        sim            — primary SimulatorManager
        sims           — [SimulatorManager] list (length 1 for single-drone runs)
        env_cfg        — AirstackDomainCfg for the first requested domain
        headless       — bool
        sim_backend    — "airstack_isaac"
    """
    if config.DEBUG:
        print("[DEBUG] airstack.launch.setup: enter", flush=True)

    _kill_stale_px4()

    from isaacsim import SimulationApp

    headless = bool(getattr(args, "headless", False))
    simulation_app = SimulationApp({"headless": headless, "anti_aliasing": 0})

    if config.DEBUG:
        print("[DEBUG] airstack.launch.setup: SimulationApp created", flush=True)

    import omni.kit.app

    # Add Pegasus extension path and load required extensions
    sys.path.insert(0, os.path.join(_PEGASUS_EXTS, "pegasus.simulator"))

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.add_path(_PEGASUS_EXTS)

    for ext in [
        "isaacsim.ros2.bridge",
        "omni.graph.core",
        "omni.graph.action",
        "omni.graph.action_nodes",
        "omni.graph.ui",
        "omni.graph.scriptnode",
        "omni.graph.ui_nodes",
        "pegasus.simulator",
    ]:
        if not ext_manager.is_extension_enabled(ext):
            ext_manager.set_extension_enabled(ext, True)

    # Resolve initial domain config
    from .sim.environment_config import get_domain_registry

    domain_name = (getattr(args, "domains", None) or [None])[0]
    domains = get_domain_registry()
    env_cfg = domains.get(domain_name) if domain_name else None
    if domain_name and env_cfg is None:
        print(
            f"[airstack.launch] Unknown domain '{domain_name}', using default config.",
            flush=True,
        )

    try:
        from safe_core.utils.isaacsim.simulator_manager import SimulatorManager
        if config.DEBUG:
            print("[DEBUG] airstack.launch: SimulatorManager imported", flush=True)
    except ImportError as exc:
        raise RuntimeError(
            f"Cannot import SimulatorManager — install the S.A.F.E. benchmark:\n"
            "  pip install -e /path/to/benchmark"
        ) from exc

    direct_velocity = bool(getattr(args, "direct_velocity", False))
    if config.DEBUG:
        print(
            f"[DEBUG] airstack.launch: domain={domain_name!r} direct_velocity={direct_velocity}",
            flush=True,
        )

    sim = SimulatorManager(
        simulation_app,
        size_x=env_cfg.size_x if env_cfg else 20.0,
        size_y=env_cfg.size_y if env_cfg else 20.0,
        headless=headless,
        env_cfg=env_cfg,
        vehicle_id=0,
        direct_velocity=direct_velocity,
    )
    sim.warmup()

    if config.DEBUG:
        print("[DEBUG] airstack.launch.setup: warmup done", flush=True)

    return {
        "simulation_app": simulation_app,
        "sim":            sim,
        "sims":           [sim],
        "env_cfg":        env_cfg,
        "headless":       headless,
        "sim_backend":    "airstack_isaac",
        "direct_velocity": direct_velocity,
    }


def teardown(ctx: Optional[Dict[str, Any]], env: Any, args: Any) -> None:
    if ctx is None:
        return
    app = ctx.get("simulation_app")
    if app is not None:
        try:
            app.close()
        except Exception:
            pass

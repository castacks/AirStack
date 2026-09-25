"""Scene configuration for search_mission_scene.py (stdlib + optional PyYAML).

Resolves, without touching Isaac:
  * the scenario bundle (scenario.json, ground_truth.json, belief.png),
  * the drone spawn list — from the fleet file when ``FLEET_CONFIG_FILE`` is set
    (``airstack up --fleet``), otherwise from the scenario's team homes,
  * which sensor subgraphs each robot gets (vehicle manifest sensor TYPES:
    ``gimbal_cam`` -> native gimbal, ``stereo_cam`` -> ZED, ``lidar*`` -> RTX lidar),
  * the gimbal hardware parameters (``airstack.sim_gimbal`` in the scenario).
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any

ROBOT_CONTAINER_ROOT = "/root/AirStack"
DEFAULT_SPAWN_Z = 0.07

__all__ = ["repo_root", "scenario_dir", "load_bundle", "remap_path", "vehicle_sensor_types",
           "drone_configs_from_fleet", "drone_configs_from_scenario", "gimbal_params", "spawn_mismatches"]


def repo_root(launch_scripts_dir: str | Path) -> Path:
    """``<repo>/simulation/isaac-sim/launch_scripts`` -> ``<repo>``."""
    return Path(launch_scripts_dir).resolve().parents[2]


def remap_path(path: str, repo: Path) -> str:
    """Map a robot-container path (/root/AirStack/...) onto this checkout."""
    if path.startswith(ROBOT_CONTAINER_ROOT + "/"):
        return str(repo) + path[len(ROBOT_CONTAINER_ROOT):]
    return path


def scenario_dir(repo: Path, env: dict | None = None) -> Path:
    env = os.environ if env is None else env
    raw = env.get("MTL_SCENARIO_DIR", "").strip()
    return Path(remap_path(raw, repo)) if raw else repo / "stacks" / "mtl_search" / "config"


def load_bundle(directory: Path) -> tuple[dict, dict, Path | None]:
    sc_path = directory / "scenario.json"
    gt_path = directory / "ground_truth.json"
    if not sc_path.is_file() or not gt_path.is_file():
        raise FileNotFoundError(
            f"MTL scenario bundle missing in {directory} (need scenario.json + ground_truth.json) - "
            "run `python3 scripts/mtl_generate_scenario.py` or set MTL_SCENARIO_DIR")
    sc = json.loads(sc_path.read_text(encoding="utf-8"))
    gt = json.loads(gt_path.read_text(encoding="utf-8"))
    png = directory / "belief.png"
    return sc, gt, (png if png.is_file() else None)


def _load_yaml(path: Path) -> dict:
    import yaml  # PyYAML ships in the Isaac image (fleet_spawn.py relies on it too)
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def vehicle_sensor_types(repo: Path, vehicle: str) -> list[str]:
    manifest = repo / "config" / "vehicles" / vehicle / "vehicle.yaml"
    if not vehicle or not manifest.is_file():
        return []
    sensors = _load_yaml(manifest).get("sensors") or []
    return [str(s.get("type", "")) for s in sensors if isinstance(s, dict)]


def _flags(types: list[str], default_gimbal: bool = True) -> dict[str, bool]:
    if not types:  # no manifest: the MTL scene's default payload
        return {"gimbal": default_gimbal, "camera": False, "lidar": False}
    return {"gimbal": "gimbal_cam" in types, "camera": "stereo_cam" in types,
            "lidar": any("lidar" in t for t in types)}


def drone_configs_from_fleet(fleet_path: Path, repo: Path) -> list[dict[str, Any]]:
    """Fleet file -> PegasusApp drone configs (robot N -> domain N, file order)."""
    fleet = _load_yaml(fleet_path)
    robots = fleet.get("robots") or {}
    if not robots:
        raise ValueError(f"{fleet_path}: fleet has no robots")
    defaults = fleet.get("defaults") or {}
    out = []
    for i, (name, entry) in enumerate(robots.items(), start=1):
        entry = entry or {}
        x, y, z = entry.get("spawn", [0.0, 0.0, DEFAULT_SPAWN_Z])
        vehicle = entry.get("vehicle", defaults.get("vehicle", ""))
        out.append({"domain_id": i, "robot_name": name, "x_m": float(x), "y_m": float(y), "z_m": float(z),
                    "vehicle": vehicle, **_flags(vehicle_sensor_types(repo, vehicle))})
    return out


def drone_configs_from_scenario(scenario: dict) -> list[dict[str, Any]]:
    """No fleet: spawn every scenario team agent at its home (NED -> ENU)."""
    out = []
    for i, a in enumerate(scenario["team"]["agents"], start=1):
        n, e = a["home_ned"]
        out.append({"domain_id": i, "robot_name": a["name"], "x_m": float(e), "y_m": float(n),
                    "z_m": DEFAULT_SPAWN_Z, "vehicle": "", "gimbal": True, "camera": False, "lidar": False})
    return out


def spawn_mismatches(configs: list[dict], scenario: dict, tol_m: float = 0.5) -> list[str]:
    homes = {a["name"]: (a["home_ned"][1], a["home_ned"][0]) for a in scenario["team"]["agents"]}
    problems = []
    for c in configs:
        h = homes.get(c["robot_name"])
        if h is None:
            problems.append(f"{c['robot_name']} is not a scenario agent - its planner will refuse to plan")
        elif math.hypot(c["x_m"] - h[0], c["y_m"] - h[1]) > tol_m:
            problems.append(f"{c['robot_name']} spawns at ({c['x_m']:g}, {c['y_m']:g}) but the scenario home is "
                            f"({h[0]:g}, {h[1]:g}) - regenerate with scripts/mtl_generate_scenario.py")
    return problems


def gimbal_params(scenario: dict) -> dict[str, Any]:
    g = dict(scenario.get("airstack", {}).get("sim_gimbal", {}))
    roll = g.get("roll_limit_deg", [-80.0, 80.0])
    pitch = g.get("pitch_limit_deg", [-20.0, 110.0])
    return {
        "fov_deg": float(scenario["sensor"]["fov_deg"]),
        "width": int(g.get("width", 640)),
        "height": int(g.get("height", 480)),
        "publish_hz": float(g.get("publish_hz", 15.0)),
        "mount_offset_m": [float(v) for v in g.get("mount_offset_m", [0.10, 0.0, -0.08])],
        "slew_rate_rad_s": math.radians(float(g.get("slew_rate_deg_s", 120.0))),
        "roll_limit_rad": (math.radians(float(roll[0])), math.radians(float(roll[1]))),
        "pitch_limit_rad": (math.radians(float(pitch[0])), math.radians(float(pitch[1]))),
        "initial_pitch_rad": math.radians(float(g.get("initial_pitch_deg", 60.0))),
        "max_range_m": max(200.0, 4.0 * float(scenario["aircraft"]["altitude_m"])),
    }

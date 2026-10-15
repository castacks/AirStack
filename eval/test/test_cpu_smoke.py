"""CPU smoke of the droan_empty experiment against the current SAFE checkout: every eval
module imports, the yaml loads and validates, the adapter and the droan policy build, and
one episode flown by a stand-in stack goes through SAFE's real rollout, scoring, npz,
episodes.csv and episode-video path. No Docker, Isaac or ROS is touched."""

from __future__ import annotations

import base64
import glob
import importlib
import os
import pkgutil
import py_compile
import sys
import zlib
from pathlib import Path

import numpy as np
import pytest

import eval as eval_pkg

AIRSTACK = Path(__file__).resolve().parents[2]
EXPERIMENT = AIRSTACK / "eval" / "experiments" / "droan_empty.yaml"
# Modules that only run inside a container: the rclpy bridge and the Kit launcher.
CONTAINER_ONLY = {"eval.stack.container_bridge"}
LAUNCHER = AIRSTACK / "simulation" / "isaac-sim" / "launch_scripts" / "safe_eval_launch_script.py"


def test_every_eval_module_imports():
    names = [m.name for m in pkgutil.walk_packages(eval_pkg.__path__, "eval.")
             if ".test" not in m.name and ".runs" not in m.name]
    assert "eval.run" in names and "eval.adapter" in names
    for name in names:
        if name in CONTAINER_ONLY:
            py_compile.compile(importlib.util.find_spec(name).origin, doraise=True)
        else:
            importlib.import_module(name)
    py_compile.compile(str(LAUNCHER), doraise=True)


def _load_args(monkeypatch, *extra):
    from safe_core.core.runner import validate_run_config
    from eval.run import AirstackRunner

    runner = AirstackRunner()
    monkeypatch.setattr(sys, "argv", ["eval/run.py", "--config", str(EXPERIMENT),
                                      "-y", "--headless", *extra])
    args = runner._parse_args()
    max_steps = runner.validate(args)
    validate_run_config(args, required_yaml_keys=runner.required_yaml_keys)
    return runner, args, max_steps


def test_droan_empty_loads_and_validates(monkeypatch):
    from safe_core.utils.benchmark_io import REQUIRED_YAML_KEYS, check_experiment_yaml
    from eval.run import AirstackRunner

    assert check_experiment_yaml(
        str(EXPERIMENT), REQUIRED_YAML_KEYS + AirstackRunner.required_yaml_keys) == []
    _, args, max_steps = _load_args(monkeypatch)
    assert args.robot == "iris" and args.agents == ["droan"] and args.domains == ["empty"]
    assert max_steps == int(args.max_episode_time / args.env_dt)
    # debug: default turns on SAFE's live episode capture, the smoke flight's video.
    from safe_core.utils.debug_config import DebugConfig
    dbg = DebugConfig.from_args(args)
    assert dbg.capture_video and dbg.video_save == "all"


def _build(monkeypatch):
    from safe_core.core.robot_spec import set_active_robot
    from safe_core.utils.benchmark_utils import (build_agents_from_classes,
                                                 discover_policies, validate_agents_built)

    runner, args, max_steps = _load_args(monkeypatch)
    runner._configure_runtime(args)
    set_active_robot(args.robot)
    classes = list(discover_policies(runner.get_agents_dir()).values())
    agents = build_agents_from_classes(agent_name=None, agent_names=args.agents,
                                       policy_classes=classes)
    validate_agents_built(agents)
    adapter = runner.build_adapter(args)
    sensors = list(dict.fromkeys(s for _, a in agents for s in a.sensors))
    env = adapter.build_env(runner.build_env_ctx(args, {}, max_steps, sensors))
    return runner, args, agents, adapter, env


def test_adapter_and_droan_agent_build(monkeypatch):
    runner, args, agents, adapter, env = _build(monkeypatch)
    assert [name for name, _ in agents] == ["droan"]
    assert type(agents[0][1]).__name__ == "DROAN"
    assert env.env_cfg.source_domain == "empty"
    # goal_distance 20 scales SAFE's procedural arena to 1.5 x 20 m.
    assert env.size_x == env.size_y == 30.0
    params = env.get_env_params()
    assert {"RE_max", "epsilon_unsafe", "max_steps", "dt"} <= set(params)
    adapter.build_taxonomy()


# ── stand-in stack: the Isaac ScenarioManager and robot bridge, on CPU ─────────

def _safe_empty_cfg():
    import integrations.navigation.common.config.domains_isaac  # noqa: F401
    from integrations.navigation.common.config.domains import get_domain_registry
    return get_domain_registry("isaac")["empty"]


class _FakeScenario:
    """Answers the ScenarioManager commands the env sends, with SAFE's real sampler and a
    drone that flies straight at the goal at 3 m/s of sim time."""

    def __init__(self):
        self.t = 0.0
        self.pos = np.zeros(3)
        self.target = None
        self.capture_built = 0
        self.alive = True

    def _state(self):
        return {"t": self.t, "drone_pos": self.pos.tolist(), "drone_vel": [0.0, 0.0, 0.0],
                "dynamic": [], "dynamic_vel": [], "collision": False, "collision_with": ""}

    def command(self, cmd, args=None, timeout=30.0):
        args = args or {}
        if cmd == "get_info":
            cfg = _safe_empty_cfg()
            return {"domain_info": {  # SAFE benchmark frame, as the launcher reports it
                "size_x": cfg.size_x, "size_y": cfg.size_y, "size_z": cfg.size_z,
                "origin_x": cfg.origin_x, "origin_y": cfg.origin_y, "origin_z": cfg.origin_z,
                "static_mesh_positions": [], "static_mesh_aabbs": []}}
        if cmd == "generate_safe_scenario":
            from eval.sim.safe_source_scene import sample_scenario
            cfg = _safe_empty_cfg()
            layout, _, difficulty = sample_scenario(
                cfg, seed=args["seed"], goal_distance=args["goal_distance"],
                perturbation_seed=args["perturbation_seed"], humans=False,
                agent_radius=0.31, obstacle_radius=0.3,
                scenario_index=args["scenario_index"])

            def isaac(v):
                a = np.asarray(v, dtype=float).reshape(-1, 3).copy()
                a[:, [0, 1]] = a[:, [1, 0]]
                return a.tolist()
            start, target = isaac([layout["agent_location"]])[0], isaac([layout["target_location"]])[0]
            self.target = np.asarray(target)
            return {"start": start, "target": target,
                    "static_cylinder_locations": isaac(layout["static_cylinder_locations"]),
                    "static_obstacle_locations": isaac(layout["static_obstacle_locations"]),
                    "dynamic_obstacle_locations": [], "dynamic_obstacle_velocities": [],
                    "dynamic_obstacle_waypoints": [],
                    "ground_z": float(cfg.origin_z), "scenario_difficulty": difficulty}
        if cmd == "reset_vehicle":
            self.pos = np.asarray(args["pos"], dtype=float)
            return {}
        if cmd == "capture_frames":
            # Fresh render products come back empty for a few renders, as in Isaac.
            self.capture_built += 1
            if self.capture_built < 4:
                return {} if self.capture_built == 1 else {"overview": None, "tracking": None}
            rgb = np.full((64, 64, 3), 90, dtype=np.uint8)
            enc = {"shape": [64, 64, 3],
                   "rgb_zlib_b64": base64.b64encode(zlib.compress(rgb.tobytes())).decode()}
            return {"overview": enc, "tracking": enc}
        return {}

    def wait_state(self, timeout=60.0):
        return self._state()

    latest_state = wait_state

    def wait_sim_time(self, target_t, timeout=60.0):
        dt = target_t - self.t
        self.t = target_t
        if self.target is not None:
            step = self.target[:2] - self.pos[:2]
            dist = float(np.linalg.norm(step))
            if dist > 1e-9:
                self.pos[:2] += step / dist * min(dist, 3.0 * dt)
        return self._state()

    def close(self):
        self.alive = False


class _FakeBridge:
    alive = True

    def __getattr__(self, name):
        return lambda *a, **k: True

    def latest_odom_pos(self):
        return None

    def latest_sensor_obs(self):
        return {}


class _FakeStack:
    """Stands in for eval.stack.compose.StackHandle: no airstack up, no Docker."""

    def __init__(self, *a, **k):
        self.scenario = _FakeScenario()
        self.bridge = _FakeBridge()

    def ensure_up(self, *a, **k):
        pass

    def takeoff(self, altitude_m, velocity_m_s=1.0):
        self.scenario.pos[2] = altitude_m

    def __getattr__(self, name):
        return lambda *a, **k: True


def run_entry_point_as_pool_worker(argv):
    """Run AirstackRunner().main() exactly as SAFE's job pool runs its worker child,
    with the stack replaced by _FakeStack. Called in a subprocess by the test below."""
    import eval.envs.AirstackStackEnv as env_module
    from eval.run import AirstackRunner
    env_module.StackHandle = _FakeStack
    env_module.time.sleep = lambda s: None
    sys.argv = argv
    AirstackRunner().main()


def test_droan_empty_entry_point_writes_episode_and_video(tmp_path):
    """`eval/run.py --config droan_empty.yaml` as the pool's worker child, with a stand-in
    stack: SAFE's real rollout, simple-mode scoring, npz record, episodes.csv, report
    and the composite episode video, in the layout the owner's run writes."""
    import subprocess
    from safe_core.utils.benchmark_io import load_episode_record

    run_dir = tmp_path / "runs" / "airstack" / "droan_empty_0000"
    run_dir.mkdir(parents=True)
    argv = ["eval/run.py", "--config", str(EXPERIMENT), "-y", "--headless",
            "--resume", str(run_dir), "--skip-confirm", "--benchmark-seed", "42",
            "--agent", "droan"]
    env = {**os.environ, "SAFE_WORKER": "1",
           "PYTHONPATH": os.pathsep.join([str(Path(__file__).parent), str(AIRSTACK),
                                          os.environ.get("PYTHONPATH", "")])}
    code = ("import sys, test_cpu_smoke; "
            f"test_cpu_smoke.run_entry_point_as_pool_worker({argv!r})")
    done = subprocess.run([sys.executable, "-c", code], cwd=tmp_path, env=env,
                          capture_output=True, text=True, timeout=600)
    assert done.returncode == 0, done.stdout[-4000:] + done.stderr[-4000:]

    npzs = glob.glob(str(run_dir / "raw" / "empty" / "scenario_1" / "perturbation_1"
                         / "trajectories" / "ep1.npz"))
    assert len(npzs) == 1, done.stdout[-3000:]
    record = load_episode_record(npzs[0])
    assert record["nominal_outcome"] == "task_completed"
    assert (run_dir / "meta" / "episodes.csv").is_file()
    assert (run_dir / "report.html").is_file()
    videos = glob.glob(str(run_dir / "raw" / "empty" / "scenario_1" / "perturbation_1"
                           / "ep1_*" / "*.mp4"))
    assert [os.path.basename(v) for v in videos] == ["episode_composite.mp4"], videos

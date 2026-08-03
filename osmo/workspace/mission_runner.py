#!/usr/bin/env python3
"""mission_runner.py — batch mission executor for AirStack-on-OSMO pods.

Reads a declarative mission spec (YAML, see osmo/missions/README.md) and runs
N full iterations of:

    airstack down → airstack up → wait for PX4 ready → start mcap recording
    → execute steps (ros2 action goals / topic pubs / service calls / raw
    commands / waits) → stop recording → collect bags + container logs →
    airstack down

Artifacts land under one results root per mission run:

    <results_root>/<mission-name>/<UTC stamp>/
    ├── summary.json                 # per-iteration pass/fail + durations + method
    └── iter_001__<env>__<method>/   # e.g. iter_001__retro_r1__frontier
                                     # (iter_NNN prefix kept; env+method appended)
        ├── bags/robot_1/*.mcap      # Foxglove-ready (open the .mcap directly)
        ├── logs/<container>.log     # docker logs snapshot, per container
        ├── ready.json               # per-robot PX4 readiness timings
        └── steps.json               # per-step command, output tail, status

The results root prefers /osmo/output (OSMO uploads that directory to the
workflow's `outputs:` destinations when the task exits) and falls back to
<airstack>/osmo/results. Either way a symlink is left at
<airstack>/osmo/results so `airstack osmo:fetch` always finds it.

Designed to run on the OSMO workspace pod (invoked by entrypoint.sh when
OSMO_MISSION_FILE is set), but has no OSMO dependency: it only needs docker,
python3 + PyYAML, and a checkout with airstack.sh — so it can be tested on
any dev machine that can run `airstack up`.

Command patterns (ros2 exec into the robot container with a per-robot
ROS_DOMAIN_ID, two-gate PX4 readiness, action-result parsing) mirror
tests/conftest.py and tests/system/test_takeoff_hover_land.py.
"""

import argparse
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path

import yaml

ROS_DISTRO_SETUP = "/opt/ros/jazzy/setup.bash"
DEFAULT_ROBOT_SETUP_BASH = "/root/AirStack/robot/ros_ws/install/setup.bash"
GCS_SETUP_BASH = "/root/AirStack/gcs/ros_ws/install/setup.bash"
ROBOT_CONTAINER_PATTERN = "robot.*desktop"

# Default topics for `record.scope: gcs` — one recorder in the GCS container
# on domain 0, where each robot's domain_bridge (autonomy_bringup
# onboard_all/config/domain_bridge.yaml) forwards its state topics. All
# robots land in ONE mcap, and replaying it in Foxglove reproduces the live
# GCS view (gcs_visualizer markers are already in the global 'map' frame).
# {robot} expands to robot_1..robot_N and the result is unioned.
DEFAULT_GCS_RECORD_TOPICS = [
    "/tf",
    "/tf_static",
    "/{robot}/odometry_conversion/odometry",
    "/{robot}/interface/mavros/global_position/global",
    "/{robot}/trajectory_controller/trajectory_vis",
    "/{robot}/global_plan",
    "/gcs/robot_markers",
    "/gcs/map_origin/location",
    "/gcs/map_origin/ground_msl",
    "/gcs/{robot}/location",
]

# Default topics for `record.scope: robot` — one recorder (and one mcap) per
# robot on its own DDS domain. Use this scope for topics that are NOT
# bridged to the GCS (raw sensors, high-rate local topics).
DEFAULT_ROBOT_RECORD_TOPICS = [
    "/tf",
    "/tf_static",
    "/{robot}/odometry_conversion/odometry",
    "/{robot}/interface/mavros/global_position/global",
    "/{robot}/global_plan",
]

# In-container staging dir for bag recordings (docker cp'd out before the
# stack goes down).
BAG_STAGING_DIR = "/tmp/osmo_bags"

# Tasks the GCS action_relay bridges (gcs/ros_ws/src/action_relay). Goals for
# these can be routed `via: gcs` — published as String JSON on
# /<robot>/tasks/<task>/goal (GCS domain 0), exactly the path Foxglove uses.
GCS_RELAY_TASKS = {"takeoff", "land", "navigate", "fixed_trajectory",
                   "semantic_search", "exploration"}

MISSION_DEFAULTS = {
    "iterations": 1,
    # How many times to (re)run a single iteration that doesn't pass before
    # giving up and moving on. 1 = no redo (the historical behavior). >1 = if an
    # iteration ends failed/errored (a step up to and including the gating step
    # failed — e.g. takeoff never armed, or semantic_search never got feedback),
    # tear the stack down and run that same iteration again, up to this many
    # total attempts. A `stopped` (manual) or `abort_mission` outcome is never
    # retried. Failed attempts' artifacts are preserved under
    # iter_NNN_failed_attempt_K so the canonical iter_NNN holds the last attempt.
    "iteration_attempts": 1,
    # With an `environments:` list: round_robin → iteration i uses
    # environments[(i-1) % n], so `iterations` is the TOTAL run count.
    # grouped → each environment runs `iterations` times consecutively
    # (retro×N, then fireacademy×N, …), so the total is iterations * n.
    "environment_order": "round_robin",  # round_robin | grouped
    "on_step_failure": "abort_iteration",  # continue | abort_iteration | abort_mission
    "ready": {"timeout_s": 600, "poll_interval_s": 5},
    # scope "gcs": one recorder on GCS domain 0 → one mcap with every robot's
    # bridged topics (default). scope "robot": one recorder + one mcap per
    # robot on its own domain (for unbridged/high-rate topics). scope "both":
    # the GCS recorder AND the per-robot recorders together.
    "record": {"enabled": True, "scope": "gcs"},
    # How the stack is brought up. Either (or both):
    #   services: [isaac-sim, robot-desktop, gcs]  → ./airstack.sh up <services>
    #     (compose auto-enables a named service's profile)
    #   profiles: [desktop, isaac-sim]             → exported as COMPOSE_PROFILES
    # With neither, plain `airstack up` with COMPOSE_PROFILES=desktop,isaac-sim.
    "stack": {"services": [], "profiles": []},
    # Default route for `action` steps: "gcs" sends goals through the GCS
    # action_relay (same path as Foxglove / the GCS panels — exercises the
    # full GCS→robot chain incl. the relay's airborne preconditions);
    # "robot" sends ros2 action goals directly on the robot's DDS domain.
    # Override per step with `via:`.
    "command_route": "gcs",
    "up_timeout_s": 3600,    # first `up` on a fresh pod pulls the full image set
    "down_timeout_s": 300,
    "robot_setup_bash": DEFAULT_ROBOT_SETUP_BASH,
}


class MissionStop(Exception):
    """Raised in the main thread on SIGINT/SIGTERM (e.g. `airstack
    osmo:stop`): aborts the current step, finalizes recordings, collects
    artifacts, and brings the stack down."""


STOP_EVENT = threading.Event()


def _stop_handler(signum, frame):
    if STOP_EVENT.is_set():
        return  # already stopping — don't interrupt artifact collection
    STOP_EVENT.set()
    raise MissionStop(f"stop requested (signal {signum})")


def log(msg):
    print(f"[mission {datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)


def utc_stamp():
    return datetime.now(timezone.utc).strftime("%Y-%m-%d_%H-%M-%S")


# ── subprocess / docker plumbing ───────────────────────────────────────────

def sh(cmd_list, timeout, env=None, cwd=None):
    """Run a command, capturing output. Never raises on non-zero exit; a
    TimeoutExpired is converted into a synthetic failed result so step
    failures stay data, not exceptions."""
    try:
        return subprocess.run(cmd_list, capture_output=True, text=True,
                              timeout=timeout, env=env, cwd=cwd)
    except subprocess.TimeoutExpired as e:
        return subprocess.CompletedProcess(
            cmd_list, returncode=124,
            stdout=(e.stdout or b"").decode() if isinstance(e.stdout, bytes) else (e.stdout or ""),
            stderr=f"timed out after {timeout}s",
        )


def docker_exec(container, cmd, timeout):
    return sh(["docker", "exec", container, "bash", "-c", cmd], timeout=timeout)


def ros2_env_prefix(setup_bash, domain_id):
    # Workspace setup is conditional: the GCS container has a different (or
    # no) workspace at the robot path, and plain std_msgs pub/echo only needs
    # the distro setup anyway.
    return (f"source {ROS_DISTRO_SETUP} && "
            f"{{ [ -f {setup_bash} ] && source {setup_bash}; }}; "
            f"export ROS_DOMAIN_ID={domain_id}")


def ros2_exec(container, ros2_cmd, domain_id, setup_bash, timeout):
    return docker_exec(container, f"{ros2_env_prefix(setup_bash, domain_id)} && {ros2_cmd}",
                       timeout=timeout)


def list_containers(name_pattern=None, all_states=False):
    cmd = ["docker", "ps", "--format", "{{.Names}}"]
    if all_states:
        cmd.insert(2, "-a")
    if name_pattern:
        cmd += ["--filter", f"name={name_pattern}"]
    result = sh(cmd, timeout=15)
    return [n for n in result.stdout.strip().splitlines() if n]


def robot_containers():
    """Running robot containers sorted by replica index (replica n ↔ robot_n)."""
    def index(name):
        tail = name.rsplit("-", 1)[-1]
        return int(tail) if tail.isdigit() else 0
    return sorted(list_containers(ROBOT_CONTAINER_PATTERN), key=index)


def gcs_container():
    names = list_containers("gcs")
    return names[0] if names else None


def stack_containers():
    """Running containers belonging to the AirStack compose stack (robot, gcs,
    and any sim variant — isaac-sim / isaac-sim-livestream / ms-airsim)."""
    pats = ("airstack", "isaac-sim", "ms-airsim")
    return [n for n in list_containers() if any(p in n for p in pats)]


# ── mission spec ───────────────────────────────────────────────────────────

def load_mission(path):
    with open(path, encoding="utf-8") as f:
        spec = yaml.safe_load(f)
    if not isinstance(spec, dict):
        raise ValueError(f"{path}: mission spec must be a YAML mapping")
    if not spec.get("steps"):
        raise ValueError(f"{path}: mission spec has no 'steps'")
    merged = dict(MISSION_DEFAULTS)
    for key, default in MISSION_DEFAULTS.items():
        if isinstance(default, dict):
            merged[key] = {**default, **(spec.get(key) or {})}
    merged.update({k: v for k, v in spec.items() if not isinstance(MISSION_DEFAULTS.get(k), dict)})
    merged.setdefault("name", Path(path).stem)
    merged.setdefault("env", {})
    if merged["on_step_failure"] not in ("continue", "abort_iteration", "abort_mission"):
        raise ValueError(f"on_step_failure must be continue|abort_iteration|abort_mission, "
                         f"got {merged['on_step_failure']!r}")
    if merged["environment_order"] not in ("round_robin", "grouped"):
        raise ValueError(f"environment_order must be round_robin|grouped, "
                         f"got {merged['environment_order']!r}")
    try:
        merged["iteration_attempts"] = int(merged["iteration_attempts"])
    except (TypeError, ValueError):
        raise ValueError(f"iteration_attempts must be a positive integer, "
                         f"got {merged['iteration_attempts']!r}")
    if merged["iteration_attempts"] < 1:
        raise ValueError(f"iteration_attempts must be >= 1, "
                         f"got {merged['iteration_attempts']}")
    if merged["command_route"] not in ("gcs", "robot"):
        raise ValueError(f"command_route must be gcs|robot, got {merged['command_route']!r}")
    if merged["record"].get("scope", "gcs") not in ("gcs", "robot", "both"):
        raise ValueError(f"record.scope must be gcs|robot|both, "
                         f"got {merged['record'].get('scope')!r}")
    envs = merged.get("environments") or []
    if not isinstance(envs, list):
        raise ValueError("environments must be a list of environment entries")
    for e in envs:
        if not isinstance(e, dict) or "name" not in e:
            raise ValueError(f"each environments entry must be a mapping with a "
                             f"'name' key, got {e!r}")
    merged["environments"] = envs
    for step in merged["steps"]:
        action = step.get("action") if isinstance(step, dict) else None
        if action:
            via = action.get("via", merged["command_route"])
            if via not in ("gcs", "robot"):
                raise ValueError(f"action via must be gcs|robot, got {via!r}")
            if via == "gcs" and action.get("task") not in GCS_RELAY_TASKS:
                raise ValueError(
                    f"task '{action.get('task')}' is not bridged by the GCS action_relay "
                    f"({', '.join(sorted(GCS_RELAY_TASKS))}); add `via: robot` to send it "
                    f"directly on the robot's domain")
            if action.get("pass_on_feedback") and via != "gcs":
                raise ValueError(
                    "pass_on_feedback requires via: gcs (feedback liveness is only "
                    "tracked on the GCS action_relay path)")
    return merged


def task_action_type(task):
    """tasks/semantic_search → task_msgs/action/SemanticSearchTask"""
    camel = "".join(part.capitalize() for part in task.split("_"))
    return f"task_msgs/action/{camel}Task"


def expand(text, n):
    """Substitute per-robot placeholders: {n} → robot index, {robot} → robot_<n>."""
    return text.replace("{robot}", f"robot_{n}").replace("{n}", str(n))


def has_placeholder(*strings):
    """True if any string contains a per-robot placeholder ({robot} or {n})."""
    return any(s and ("{robot}" in s or "{n}" in s) for s in strings)


def step_robots(step_spec, num_robots):
    robots = step_spec.get("robots", "all")
    if robots == "all":
        return list(range(1, num_robots + 1))
    return [int(r) for r in robots]


def uses_gcs_route(mission):
    """True if any action step routes through the GCS action_relay."""
    for step in mission["steps"]:
        action = step.get("action") if isinstance(step, dict) else None
        if action and action.get("via", mission["command_route"]) == "gcs":
            return True
    return False


# ── per-iteration environment cycling ──────────────────────────────────────
# An optional top-level `environments:` list lets one mission run several
# scenes. environment_order=round_robin: iteration i uses environments[(i-1) %
# len] (`iterations` = total). environment_order=grouped: each env runs
# `iterations` times in a row (total = iterations * len). Within an entry,
# UPPERCASE keys (ENV_URL, SPAWN_POLY, …) are exported as env vars before
# `airstack up` (so they reach the isaac-sim container); any key can be
# referenced from a step via a {{env.KEY}} placeholder (e.g. the search_area
# polygon, which must track the scene).

_ENV_PLACEHOLDER = re.compile(r"\{\{\s*env\.([A-Za-z0-9_]+)\s*\}\}")


def env_exports(env_entry):
    """UPPERCASE keys of an environment entry → env vars for `airstack up`.
    Structured values are JSON-encoded so SPAWN_POLY can be written either as a
    YAML list or as a JSON string in the mission spec."""
    return {k: (v if isinstance(v, str) else json.dumps(v))
            for k, v in env_entry.items() if k.isupper()}


def substitute_env(obj, env_entry):
    """Replace {{env.KEY}} placeholders in a (copy of a) step spec with values
    from the current environment entry. A string that is *exactly* one
    placeholder yields the raw value (so {{env.search_area}} can resolve to a
    dict); a placeholder embedded in a larger string is substituted textually."""
    if isinstance(obj, str):
        whole = _ENV_PLACEHOLDER.fullmatch(obj.strip())
        if whole:
            return _lookup_env(env_entry, whole.group(1))

        def repl(m):
            val = _lookup_env(env_entry, m.group(1))
            return val if isinstance(val, str) else json.dumps(val)
        return _ENV_PLACEHOLDER.sub(repl, obj)
    if isinstance(obj, dict):
        return {k: substitute_env(v, env_entry) for k, v in obj.items()}
    if isinstance(obj, list):
        return [substitute_env(v, env_entry) for v in obj]
    return obj


def _lookup_env(env_entry, key):
    if key not in env_entry:
        raise ValueError(f"{{{{env.{key}}}}} referenced in a step but not found in "
                         f"environment '{env_entry.get('name')}' "
                         f"(available keys: {sorted(env_entry)})")
    return env_entry[key]


# ── iteration naming (env + method in the dir name) ─────────────────────────
# env-name suffix -> normalized method label. Compare missions encode the method
# in the env name (retro_r1_frontier / retro_r1_raven); single-method missions
# leave the env name as the scene and the method is inferred from the mission.
_METHOD_SUFFIXES = (
    ("_frontier", "frontier"),
    ("_raven", "multi-raven"),
    ("_multiraven", "multi-raven"),
    ("_multi-raven", "multi-raven"),
    ("_lvlm", "lvlm"),
    ("_vlfm", "vlfm"),
)


def _slug(s):
    """Filesystem-safe lowercase token."""
    s = re.sub(r"[^A-Za-z0-9._-]+", "-", str(s).strip().lower())
    return s.strip("-")


def resolve_method(mission, env_entry):
    """Normalized method label (frontier / multi-raven / lvlm / …). Explicit
    `method:` on the environment entry wins, then a mission-level `method:`, then
    inference from the env-name suffix (compare missions), then the mission name."""
    if env_entry and env_entry.get("method"):
        return _slug(env_entry["method"])
    if mission.get("method"):
        return _slug(mission["method"])
    name = (env_entry or {}).get("name", "").lower()
    for suf, meth in _METHOD_SUFFIXES:
        if name.endswith(suf):
            return meth
    mn = mission.get("name", "").lower()
    if "lvlm" in mn:
        return "lvlm"
    if "vlfm" in mn:
        return "vlfm"
    if "raven" in mn:
        return "multi-raven"
    if "frontier" in mn:
        return "frontier"
    return "na"


def iter_dir_name(i, env_entry, method):
    """iter_NNN[__<env>][__<method>]. The iter_NNN prefix is kept so `iter_*`
    globs and numeric ordering still work; a trailing method suffix already in
    the env name is dropped to avoid e.g. retro_r1_frontier__frontier."""
    parts = [f"iter_{i:03d}"]
    env_slug = _slug(env_entry["name"]) if env_entry and env_entry.get("name") else ""
    if env_slug:
        for suf, meth in _METHOD_SUFFIXES:
            if env_slug.endswith(suf) and meth == method:
                env_slug = env_slug[: -len(suf)]
                break
        if env_slug:
            parts.append(env_slug)
    if method and method != "na":
        parts.append(method)
    return "__".join(parts)


# ── stack lifecycle ────────────────────────────────────────────────────────

class Stack:
    def __init__(self, airstack_root, mission):
        self.root = airstack_root
        self.mission = mission
        self.env = os.environ.copy()
        self.env.update({k: str(v) for k, v in mission["env"].items()})
        self.env.setdefault("AUTOLAUNCH", "true")
        self.env.setdefault("NUM_ROBOTS", "1")
        # Bring-up selection (see MISSION_DEFAULTS["stack"]): an explicit
        # service list is passed to `airstack up <services>`; explicit
        # profiles become COMPOSE_PROFILES. With neither, default profiles.
        self.services = [str(s) for s in mission["stack"].get("services") or []]
        profiles = [str(p) for p in mission["stack"].get("profiles") or []]
        if profiles:
            self.env["COMPOSE_PROFILES"] = ",".join(profiles)
        elif not self.services:
            self.env.setdefault("COMPOSE_PROFILES", "desktop,isaac-sim")
        self.num_robots = int(self.env["NUM_ROBOTS"])
        self.setup_bash = mission["robot_setup_bash"]

    def apply_env(self, overrides):
        """Merge per-iteration env-var overrides (from an `environments:` entry)
        into the bring-up environment so the next `airstack up` sees them."""
        for k, v in overrides.items():
            self.env[k] = v if isinstance(v, str) else str(v)

    def _airstack(self, verb, timeout, extra_args=()):
        log(f"airstack {verb} {' '.join(extra_args)} "
            f"(NUM_ROBOTS={self.env['NUM_ROBOTS']}, "
            f"COMPOSE_PROFILES={self.env.get('COMPOSE_PROFILES', '<from .env>')})")
        return sh([str(Path(self.root) / "airstack.sh"), verb, *extra_args],
                  timeout=timeout, env=self.env, cwd=self.root)

    def up(self):
        result = self._airstack("up", self.mission["up_timeout_s"],
                                extra_args=self.services)
        if result.returncode != 0:
            raise RuntimeError(f"airstack up failed (exit {result.returncode}):\n"
                               + "\n".join(result.stdout.splitlines()[-20:])
                               + "\n" + "\n".join(result.stderr.splitlines()[-20:]))
        deadline = time.time() + 120
        while time.time() < deadline:
            containers = robot_containers()
            if len(containers) >= self.num_robots:
                return containers
            time.sleep(3)
        raise RuntimeError(f"expected {self.num_robots} robot containers, "
                           f"found {robot_containers()} after airstack up")

    def down(self):
        result = self._airstack("down", self.mission["down_timeout_s"])
        if result.returncode != 0:
            log(f"WARN: airstack down exited {result.returncode}")

    def ensure_down(self):
        """`airstack status`; if any stack containers exist, `airstack down`
        and BLOCK until they're actually gone before returning.

        This is the pre-`up` guard: a fresh `airstack up` must not race a
        previous bring-up's containers/network (the baked entrypoint leaves a
        stack up on a stale image; the prior iteration leaves one too).
        Starting `up` while those still exist causes duplicate-network and
        container-name conflicts."""
        self._airstack("status", 60)
        existing = stack_containers()
        if not existing:
            return
        log(f"existing stack containers found {existing} — bringing them down first")
        self.down()
        deadline = time.time() + self.mission["down_timeout_s"]
        while time.time() < deadline:
            remaining = stack_containers()
            if not remaining:
                log("stack fully down; safe to bring up")
                return
            log(f"waiting for teardown: {remaining}")
            time.sleep(2)
        log(f"WARN: containers still present after down: {stack_containers()} "
            f"— proceeding, `up` may conflict")

    def wait_ready(self, container):
        """Two sequential gates per robot (same as the system tests):
        1. mavros/state reports connected=True (MAVROS ↔ PX4 heartbeat);
        2. local_position/odom publishes (PX4 EKF converged — the actual
           precondition for arming; `connected` alone fires ~25s too early).
        Returns {robot_n: seconds_to_ready}; raises on timeout."""
        cfg = self.mission["ready"]
        started = time.time()
        deadline = started + cfg["timeout_s"]
        connected, ready_at = set(), {}
        pending = list(range(1, self.num_robots + 1))

        while pending and time.time() < deadline:
            for n in list(pending):
                if n not in connected:
                    r = ros2_exec(container,
                                  f"timeout 5 ros2 topic echo --once --csv "
                                  f"--field connected /robot_{n}/interface/mavros/state",
                                  domain_id=n, setup_bash=self.setup_bash, timeout=15)
                    if any(line.strip() == "True" for line in r.stdout.splitlines()):
                        connected.add(n)
                    else:
                        continue
                r = ros2_exec(container,
                              f"timeout 5 ros2 topic echo --once "
                              f"/robot_{n}/interface/mavros/local_position/odom",
                              domain_id=n, setup_bash=self.setup_bash, timeout=15)
                if r.returncode == 0 and "pose:" in r.stdout:
                    ready_at[n] = round(time.time() - started, 2)
                    pending.remove(n)
            if pending:
                log(f"waiting for PX4: connected={sorted(connected)} pending={pending} "
                    f"elapsed={time.time() - started:.0f}s")
                time.sleep(cfg["poll_interval_s"])

        if pending:
            raise RuntimeError(f"robots {pending} not ready within {cfg['timeout_s']}s "
                               f"(connected so far: {sorted(connected)})")
        log(f"PX4 ready: {ready_at}")

        # Gate 3 (only when actions route via the GCS): the per-robot
        # action_relay nodes must be up on the GCS before goals are sent —
        # the relay's goal subscription is volatile, so a goal published
        # before the relay exists is silently lost.
        if uses_gcs_route(self.mission):
            while time.time() < deadline:
                gcs = gcs_container()
                if gcs:
                    r = ros2_exec(gcs, "timeout 10 ros2 node list",
                                  domain_id=0, setup_bash=GCS_SETUP_BASH, timeout=20)
                    missing = [n for n in range(1, self.num_robots + 1)
                               if f"action_relay_robot_{n}" not in r.stdout]
                    if not missing:
                        ready_at["gcs_relay"] = round(time.time() - started, 2)
                        log(f"GCS action_relay ready ({ready_at['gcs_relay']}s)")
                        break
                    log(f"waiting for GCS action_relay: missing robots {missing}")
                else:
                    log("waiting for gcs container")
                time.sleep(cfg["poll_interval_s"])
            if "gcs_relay" not in ready_at:
                raise RuntimeError(
                    f"GCS action_relay not ready within {cfg['timeout_s']}s — "
                    f"is the gcs service in stack.services/profiles and "
                    f"AUTOLAUNCH=true? (or set command_route: robot)")
        return ready_at


# ── bag recording ──────────────────────────────────────────────────────────

class Recorder:
    """`ros2 bag record -s mcap` per the mission's record.scope.

    scope "gcs" (default): a single recorder in the GCS container on domain
    0, where every robot's domain_bridge forwards its state topics — one
    mcap holds all robots, and Foxglove replay matches the live GCS view.

    scope "robot": one recorder per robot inside robot container 1, each on
    that robot's DDS domain — for topics that aren't bridged to the GCS.

    scope "both": the GCS recorder and the per-robot recorders together —
    one mcap with the fleet view from domain 0 plus one raw-data mcap per
    robot domain.

    Each recorder is started detached with its PID dropped to a file, and
    stopped with SIGTERM so rosbag2 finalizes the mcap cleanly. (Not SIGINT:
    jobs backgrounded from a non-interactive shell have SIGINT set to
    SIG_IGN, so it would never be delivered; rosbag2 handles SIGTERM the
    same way.) Bags are docker cp'd to the host before the stack goes down."""

    def __init__(self, robot_container, mission, num_robots, setup_bash):
        self.robot_container = robot_container
        self.cfg = mission["record"]
        self.scope = self.cfg.get("scope", "gcs")
        self.num_robots = num_robots
        self.setup_bash = setup_bash
        # (container, domain_id, tag) per active recorder.
        self.active = []

    def _topic_selection(self, robots, scope):
        """Build the `ros2 bag record` topic args; `robots` is the list of
        robot indices whose {robot}/{n} placeholders to expand (unioned,
        order-preserving, deduplicated). `scope` picks the default topic set
        for the recorder being started ("gcs" or "robot")."""
        if self.cfg.get("all"):
            sel = "-a"
            # record.exclude: regex of topics to drop from `-a`.
            exclude = self.cfg.get("exclude")
            if exclude:
                sel += f" --exclude-regex {shlex.quote(exclude)}"
            return sel
        default = (DEFAULT_GCS_RECORD_TOPICS if scope == "gcs"
                   else DEFAULT_ROBOT_RECORD_TOPICS)
        topics = []
        for t in self.cfg.get("topics", default):
            if "{robot}" in t or "{n}" in t:
                topics.extend(expand(t, n) for n in robots)
            else:
                topics.append(t)
        seen = set()
        unique = [t for t in topics if not (t in seen or seen.add(t))]
        return " ".join(shlex.quote(t) for t in unique)

    def _start_one(self, container, domain_id, tag, selection, setup_bash):
        out_dir = f"{BAG_STAGING_DIR}/{tag}"
        log_file = f"{BAG_STAGING_DIR}/record_{tag}.log"
        pid_file = f"{BAG_STAGING_DIR}/record_{tag}.pid"
        # Create the staging dir in the FOREGROUND, THEN background only the
        # recorder. Folding `mkdir` into the same `... &` chain backgrounds the
        # mkdir too, so the foreground pidfile write races ahead of the dir
        # existing ("record_<tag>.pid: No such file or directory"). nohup +
        # pidfile lets the recorder outlive this docker exec; topics that
        # don't exist yet are fine (rosbag2 waits for them).
        inner = (
            f"mkdir -p {BAG_STAGING_DIR} && rm -rf {out_dir}\n"
            f"{ros2_env_prefix(setup_bash, domain_id)}\n"
            f"nohup ros2 bag record -s mcap -o {out_dir} {selection} "
            f"> {log_file} 2>&1 &\n"
            f"echo $! > {pid_file}\n"
            # Confirm the recorder is actually alive — a bad topic name or a
            # missing mcap plugin would make it exit immediately, and a silent
            # recording failure is the worst outcome for a mission.
            f"sleep 2\n"
            f"if kill -0 \"$(cat {pid_file})\" 2>/dev/null; then echo RECORDER_ALIVE; "
            f"else echo RECORDER_DEAD; tail -n 5 {log_file} 2>/dev/null; fi"
        )
        r = docker_exec(container, inner, timeout=30)
        if "RECORDER_ALIVE" in r.stdout:
            self.active.append((container, tag))
            n_topics = ("all topics" if selection.startswith("-a")
                        else f"{len(selection.split())} topics")
            log(f"recording [{tag}] in {container} (domain {domain_id}) "
                f"→ {out_dir} ({n_topics})")
            return True
        log(f"WARN: recorder [{tag}] failed to start / exited immediately: "
            f"{tail(r.stdout + r.stderr, 6)}")
        return False

    def start(self):
        if not self.cfg.get("enabled", True):
            log("recording disabled by mission spec")
            return
        robots = list(range(1, self.num_robots + 1))
        failed = []
        if self.scope in ("gcs", "both"):
            gcs = gcs_container()
            if gcs:
                if not self._start_one(gcs, 0, "gcs",
                                       self._topic_selection(robots, "gcs"),
                                       GCS_SETUP_BASH):
                    failed.append("gcs")
            else:
                log("WARN: record.scope includes 'gcs' but no gcs container is "
                    "running — GCS recording skipped (bring up the gcs service "
                    "or use scope: robot)")
                failed.append("gcs (no container)")
                if self.scope == "gcs" and not self.cfg.get("required"):
                    return
        if self.scope in ("robot", "both"):
            for n in robots:
                if not self._start_one(self.robot_container, n, f"robot_{n}",
                                       self._topic_selection([n], "robot"),
                                       self.setup_bash):
                    failed.append(f"robot_{n}")
        # record.required: a mission whose deliverable is the bags shouldn't
        # silently fly unrecorded.
        if failed and self.cfg.get("required"):
            raise RuntimeError(
                f"record.required is set and recorder(s) failed to start: "
                f"{failed} — aborting iteration")

    def stop(self):
        for container, tag in self.active:
            docker_exec(container, f"""
                pid=$(cat {BAG_STAGING_DIR}/record_{tag}.pid 2>/dev/null) || exit 0
                kill -TERM "$pid" 2>/dev/null || exit 0
                for i in $(seq 1 20); do
                    kill -0 "$pid" 2>/dev/null || exit 0
                    sleep 1
                done
                kill -9 "$pid" 2>/dev/null || true
            """, timeout=40)
        if self.active:
            log(f"recorders stopped: {[tag for _, tag in self.active]}")

    def collect(self, dest_dir):
        if not self.active:
            return
        dest_dir.mkdir(parents=True, exist_ok=True)
        for container in {c for c, _ in self.active}:
            r = sh(["docker", "cp", f"{container}:{BAG_STAGING_DIR}/.", str(dest_dir)],
                   timeout=1800)
            if r.returncode != 0:
                log(f"WARN: docker cp of bags from {container} failed: "
                    f"{r.stderr.strip()[:200]}")
        self.active = []
        mcaps = list(dest_dir.rglob("*.mcap"))
        log(f"collected {len(mcaps)} mcap file(s) → {dest_dir}")


# ── step execution ─────────────────────────────────────────────────────────

def action_ok(stdout):
    """ros2 action send_goal --feedback prints the result as YAML;
    AirStack task results carry `success: true` on completion."""
    return "success: true" in stdout


def tail(text, lines=15):
    return "\n".join((text or "").strip().splitlines()[-lines:])


def _run_one(stack, robot_container, target, cmd, timeout, expect_success):
    """Run one resolved `run` command. `target` is already robot-expanded:
      pod          → on the pod itself (cwd = AirStack root)
      gcs          → in the gcs container on domain 0
      robot_<n>    → in robot container 1 on robot N's DDS domain
      <other>      → literal container name (domain 0)
    """
    if target == "pod":
        r = sh(["bash", "-c", cmd], timeout=timeout, cwd=stack.root)
    elif target == "gcs":
        g = gcs_container()
        if g:
            r = ros2_exec(g, cmd, domain_id=0, setup_bash=GCS_SETUP_BASH,
                          timeout=timeout + 15)
        else:
            r = subprocess.CompletedProcess(cmd, returncode=1, stdout="",
                                            stderr="no gcs container running")
    elif target.startswith("robot_") and target[6:].isdigit():
        n = int(target[6:])
        r = ros2_exec(robot_container, cmd, domain_id=n,
                      setup_bash=stack.setup_bash, timeout=timeout + 15)
    else:
        r = docker_exec(target, cmd, timeout=timeout + 15)
    ok = (r.returncode == 0) if expect_success else True
    return {"target": target, "exit": r.returncode, "ok": ok,
            "output_tail": tail(r.stdout + r.stderr)}


def _run_one_retry(stack, robot_container, target, cmd, timeout, expect_success,
                   attempts, retry_delay_s):
    """_run_one with up to `attempts` tries (retry_delay_s between failures)."""
    res = {"target": target, "exit": 1, "ok": False,
           "output_tail": "not attempted — mission stop requested", "attempts": 0}
    for attempt in range(1, max(1, attempts) + 1):
        if STOP_EVENT.is_set():
            break
        res = _run_one(stack, robot_container, target, cmd, timeout, expect_success)
        res["attempts"] = attempt
        if res["ok"]:
            break
        if attempt < attempts and not STOP_EVENT.is_set():
            log(f"run [{target}] failed (attempt {attempt}/{attempts}); "
                f"retrying in {retry_delay_s}s")
            time.sleep(retry_delay_s)
    return res


def run_step(stack, container, step_spec, step_index):
    """Execute one step; returns a result dict with ok: bool."""
    record = {"index": step_index, "spec": step_spec,
              "started_at": datetime.now(timezone.utc).isoformat()}
    t0 = time.time()

    if "wait" in step_spec:
        seconds = float(step_spec["wait"])
        log(f"step {step_index}: wait {seconds}s")
        time.sleep(seconds)
        record.update(type="wait", ok=True)

    elif "action" in step_spec:
        spec = step_spec["action"]
        task = spec["task"]
        via = spec.get("via", stack.mission["command_route"])
        goal = spec.get("goal", {})
        # Normalize the goal to JSON: dicts directly; strings may be ros2-style
        # YAML ("{target_altitude_m: 10}") — YAML-parse then re-dump. JSON is
        # what the GCS relay requires, and it's a YAML subset so the direct
        # send_goal path accepts it too.
        goal_obj = yaml.safe_load(goal) if isinstance(goal, str) else goal
        goal_json = json.dumps(goal_obj or {})
        timeout = float(spec.get("timeout_s", 120))
        # Per-robot retries for transient rejections (no GPS fix yet, PX4
        # position not converged, goal lost on the relay's volatile sub).
        max_attempts = int(spec.get("attempts", 3))
        retry_delay_s = float(spec.get("retry_delay_s", 10))
        # via: gcs only — no relay_feedback AND no result within this window
        # after publishing ⇒ the goal is presumed lost; fail fast and retry.
        feedback_timeout_s = int(spec.get("feedback_timeout_s", 15))
        # cancel_on_timeout (via:gcs only): on `timeout_s`, publish a CancelGoal
        # to the relay's .../cancel topic and wait up to cancel_grace_s for the
        # cancelled result, so an on-cancel finalize (e.g. semantic_search's
        # metrics) can run and return.
        cancel_on_timeout = bool(spec.get("cancel_on_timeout", False))
        cancel_grace_s = int(spec.get("cancel_grace_s", 120))
        # pass_on_feedback (via:gcs only): treat the step as passed as soon as the
        # action is confirmed to have actually started running — i.e. any
        # relay_feedback was seen (or a result came back), regardless of whether
        # the result is success or failure. The ONLY failure left is "goal lost /
        # never ran" (no feedback and no result within the retries), which is what
        # then fails the step (→ iteration redo when iteration_attempts>1). Use
        # this when you only care that the task ran, not its outcome.
        pass_on_feedback = bool(spec.get("pass_on_feedback", False))
        robots = step_robots(spec, stack.num_robots)
        log(f"step {step_index}: action {task} {goal_json} via {via} → robots {robots}")

        if via == "gcs":
            gcs = gcs_container()
            if not gcs:
                record.update(type="action", task=task, via=via, ok=False,
                              error="no gcs container running — `via: gcs` needs the "
                                    "gcs service up (or use `via: robot`)")
                record["duration_s"] = round(time.time() - t0, 2)
                log(f"step {step_index}: FAILED (no gcs container)")
                return record

            # The result echo must outlive `timeout` to catch the post-cancel
            # result.
            echo_timeout = int(timeout + cancel_grace_s) if cancel_on_timeout \
                else int(timeout)

            def send(n):
                # Foxglove's path: String JSON on /<robot>/tasks/<task>/goal,
                # result on .../relay_result. relay_result is latched, so a
                # fresh subscriber gets the PREVIOUS goal's result — count
                # messages before publishing and only accept a NEW one.
                # relay_feedback is the liveness signal: nothing there (and
                # no result) within feedback_timeout_s ⇒ goal lost, bail out
                # so the retry loop re-sends.
                base = f"/robot_{n}/tasks/{task}"
                result_file = f"/tmp/relay_result_{task}_{n}.out"
                fb_file = f"/tmp/relay_feedback_{task}_{n}.out"
                msg_yaml = json.dumps({"data": expand(goal_json, n)})
                cancel_yaml = json.dumps({"data": "osmo: timeout cancel"})
                script = (
                    f"rm -f {result_file} {fb_file}\n"
                    f"touch {result_file} {fb_file}\n"
                    f"( timeout {echo_timeout} ros2 topic echo --field data "
                    f"{base}/relay_result >> {result_file} 2>&1 ) &\n"
                    f"sub=$!\n"
                    f"( timeout {echo_timeout} ros2 topic echo --field data "
                    f"{base}/relay_feedback >> {fb_file} 2>&1 ) &\n"
                    f"fb_sub=$!\n"
                    f"sleep 3\n"
                    f"pre=$(grep -c '^---' {result_file})\n"
                    # -w 1: wait for the relay's (VOLATILE) goal sub to match
                    # before publishing; --keep-alive 3: outlive the publish so
                    # the RELIABLE handshake flushes (0.1s default drops goals).
                    f"ros2 topic pub --once -w 1 --keep-alive 3 "
                    f"{base}/goal std_msgs/msg/String "
                    f"{shlex.quote(msg_yaml)} > /dev/null\n"
                    f"sent=$(date +%s)\n"
                    f"fb_seen=0\n"
                    f"cancelled=0\n"
                    f"deadline=$(( sent + {int(timeout)} ))\n"
                    f"while :; do\n"
                    f"  cur=$(grep -c '^---' {result_file})\n"
                    f"  if [ \"$cur\" -gt \"$pre\" ]; then break; fi\n"
                    f"  now=$(date +%s)\n"
                    f"  if [ \"$fb_seen\" -eq 0 ] && "
                    f"[ \"$(grep -c '^---' {fb_file})\" -gt 0 ]; then fb_seen=1; fi\n"
                    f"  if [ \"$cancelled\" -eq 0 ] && [ \"$fb_seen\" -eq 0 ] && "
                    f"[ $(( now - sent )) -ge {feedback_timeout_s} ]; then\n"
                    f"    kill $sub $fb_sub 2>/dev/null\n"
                    f"    echo 'no relay_feedback within {feedback_timeout_s}s "
                    f"of goal publish — goal presumed lost'\n"
                    f"    exit 0\n"
                    f"  fi\n"
                    f"  if [ $now -ge $deadline ]; then\n"
                    # Deadline: cancel + keep waiting if cancel_on_timeout, else stop.
                    f"    if [ {1 if cancel_on_timeout else 0} -eq 1 ] && "
                    f"[ \"$cancelled\" -eq 0 ]; then\n"
                    f"      echo 'osmo: {int(timeout)}s timeout — sending cancel'\n"
                    f"      ros2 topic pub --once -w 1 --keep-alive 3 "
                    f"{base}/cancel std_msgs/msg/String "
                    f"{shlex.quote(cancel_yaml)} > /dev/null\n"
                    f"      cancelled=1\n"
                    f"      deadline=$(( now + {cancel_grace_s} ))\n"
                    f"    else\n"
                    f"      break\n"
                    f"    fi\n"
                    f"  fi\n"
                    f"  kill -0 $sub 2>/dev/null || break\n"
                    f"  sleep 1\n"
                    f"done\n"
                    f"kill $sub $fb_sub 2>/dev/null\n"
                    # Surface whether the action ever produced feedback (it ran),
                    # so the runner can pass on liveness when pass_on_feedback is
                    # set. The early no-feedback bail above exits before here, so
                    # this only prints when feedback was genuinely observed.
                    f"if [ \"$fb_seen\" -eq 1 ]; then echo OSMO_FEEDBACK_SEEN; fi\n"
                    f"cur=$(grep -c '^---' {result_file})\n"
                    f"if [ \"$cur\" -gt \"$pre\" ]; then "
                    f"grep -v '^---' {result_file} | tail -n 1; "
                    f"else echo 'no relay_result within {echo_timeout}s'; fi"
                )
                r = ros2_exec(gcs, script, domain_id=0, setup_bash=GCS_SETUP_BASH,
                              timeout=int(echo_timeout + 30))
                # For cancel_on_timeout, a returned (success=false) cancel result
                # is the intended outcome — treat it as ok so the step doesn't
                # retry the search or trip on_step_failure.
                got_result = '"success"' in r.stdout
                feedback_seen = 'OSMO_FEEDBACK_SEEN' in r.stdout
                ok = ('"success": true' in r.stdout
                      or (cancel_on_timeout and got_result))
                # pass_on_feedback: ran-at-all (feedback seen, or any result
                # came back) is enough — outcome doesn't matter.
                if pass_on_feedback:
                    ok = ok or feedback_seen or got_result
                return {"exit": r.returncode, "ok": ok,
                        "feedback_seen": feedback_seen,
                        "output_tail": tail(r.stdout + r.stderr)}
        else:
            action_type = spec.get("type", task_action_type(task))

            def send(n):
                cmd = (f"ros2 action send_goal --feedback /robot_{n}/tasks/{task} "
                       f"{action_type} {shlex.quote(expand(goal_json, n))}")
                r = ros2_exec(container, cmd, domain_id=n, setup_bash=stack.setup_bash,
                              timeout=int(timeout + 15))
                return {"exit": r.returncode, "ok": action_ok(r.stdout),
                        "output_tail": tail(r.stdout + r.stderr)}

        def send_with_retry(n):
            res = {"ok": False, "exit": -1,
                   "output_tail": "not attempted — mission stop requested"}
            attempt = 0
            for attempt in range(1, max_attempts + 1):
                if STOP_EVENT.is_set():
                    break
                log(f"step {step_index}: sending {task} to robot_{n} "
                    f"(attempt {attempt}/{max_attempts})")
                res = send(n)
                if res["ok"]:
                    log(f"step {step_index}: {task} robot_{n} succeeded"
                        + (f" on attempt {attempt}" if attempt > 1 else ""))
                    break
                log(f"step {step_index}: {task} robot_{n} attempt {attempt} "
                    f"FAILED: {tail(res.get('output_tail', ''), 1)}")
                if attempt < max_attempts and not STOP_EVENT.is_set():
                    time.sleep(retry_delay_s)
            res["attempts"] = attempt
            return n, res

        # No context manager: on MissionStop the `with` form would block in
        # shutdown(wait=True) until the in-flight docker execs hit their
        # timeouts. The workers die when `airstack down` kills the containers.
        pool = ThreadPoolExecutor(max_workers=len(robots))
        try:
            results = dict(pool.map(send_with_retry, robots))
        finally:
            pool.shutdown(wait=False, cancel_futures=True)
        record.update(type="action", task=task, via=via, per_robot=results,
                      ok=all(v["ok"] for v in results.values()))

    elif "run" in step_spec:
        spec = step_spec["run"]
        cmd = spec["cmd"]
        timeout = float(spec.get("timeout_s", 60))
        expect = spec.get("expect_success", True)
        attempts = int(spec.get("attempts", 1))
        retry_delay_s = float(spec.get("retry_delay_s", 10))
        # `container` may reference {n}/{robot} to fan out over robots. If it's
        # omitted, default to robot_{n} when the command is per-robot (has a
        # placeholder) and robot_1 otherwise. A step fans out over `robots`
        # (default all) iff its command or container references {n}/{robot}.
        target = spec.get("container") or ("robot_{n}" if has_placeholder(cmd) else "robot_1")
        if has_placeholder(cmd, target):
            robots = step_robots(spec, stack.num_robots)
            log(f"step {step_index}: run [{target}] {cmd} → robots {robots}")
            results = {}
            for n in robots:
                results[n] = _run_one_retry(stack, container, expand(target, n),
                                            expand(cmd, n), timeout, expect,
                                            attempts, retry_delay_s)
            record.update(type="run", per_robot=results,
                          ok=all(v["ok"] for v in results.values()))
        else:
            log(f"step {step_index}: run [{target}] {cmd}")
            res = _run_one_retry(stack, container, target, cmd, timeout, expect,
                                 attempts, retry_delay_s)
            record.update(type="run", **res)

    elif "topic_pub" in step_spec:
        spec = step_spec["topic_pub"]
        msg = spec.get("msg", {})
        msg_str = msg if isinstance(msg, str) else json.dumps(msg)
        robots = step_robots(spec, stack.num_robots)
        log(f"step {step_index}: topic_pub {spec['topic']} → robots {robots}")
        results = {}
        for n in robots:
            cmd = (f"ros2 topic pub --once {expand(spec['topic'], n)} "
                   f"{spec['type']} {shlex.quote(expand(msg_str, n))}")
            r = ros2_exec(container, cmd, domain_id=n, setup_bash=stack.setup_bash,
                          timeout=float(spec.get("timeout_s", 30)) + 15)
            results[n] = {"exit": r.returncode, "ok": r.returncode == 0,
                          "output_tail": tail(r.stdout + r.stderr)}
        record.update(type="topic_pub", per_robot=results,
                      ok=all(v["ok"] for v in results.values()))

    elif "service_call" in step_spec:
        spec = step_spec["service_call"]
        req = spec.get("request", {})
        req_str = req if isinstance(req, str) else json.dumps(req)
        robots = step_robots(spec, stack.num_robots)
        log(f"step {step_index}: service_call {spec['service']} → robots {robots}")
        results = {}
        for n in robots:
            cmd = (f"ros2 service call {expand(spec['service'], n)} "
                   f"{spec['type']} {shlex.quote(expand(req_str, n))}")
            r = ros2_exec(container, cmd, domain_id=n, setup_bash=stack.setup_bash,
                          timeout=float(spec.get("timeout_s", 30)) + 15)
            results[n] = {"exit": r.returncode, "ok": r.returncode == 0,
                          "output_tail": tail(r.stdout + r.stderr)}
        record.update(type="service_call", per_robot=results,
                      ok=all(v["ok"] for v in results.values()))

    else:
        record.update(type="unknown", ok=False,
                      error=f"unrecognized step keys: {sorted(step_spec)}")

    record["duration_s"] = round(time.time() - t0, 2)
    log(f"step {step_index}: {'OK' if record['ok'] else 'FAILED'} "
        f"({record['duration_s']}s)")
    return record


# ── artifacts ──────────────────────────────────────────────────────────────

# Navigation / control-stack node-name globs. These log via rcl to ~/.ros/log
# (not /tmp), so they're collected separately — they're what reveals why
# droan_gl's plan isn't becoming motion (e.g. a robot stuck at a frontier).
# lvlm_baseline / semantic_search_task are here too so the planner's rcl log is
# captured as a SECOND path independent of the /tmp tee (TEE_LOG_GLOBS) — if one
# collection path misses, the other still shows why the drone wasn't commanded.
CONTROL_STACK_GLOBS = ("droan_gl*", "droan_local_planner*", "trajectory_controller*",
                       "fixed_trajectory*", "takeoff_landing*", "mavros*",
                       "lvlm_baseline*", "semantic_search_task*", "raven_nav*")


# /tmp tee logs written by semantic_search_task's _spawn (rayfronts/raven/lvlm)
# and the GCS relay/gossip/ddsrouter. lvlm_* is the FPV+LVLM baseline node's
# stdout (model load, planned actions, NavigateTask sends, errors).
TEE_LOG_GLOBS = ("/tmp/rayfronts_*.log", "/tmp/raven_*.log", "/tmp/lvlm_*.log",
                 "/tmp/gossip_*.log", "/tmp/ddsrouter_*.log", "/tmp/relay_*.log")


def _copy_tmp_tee_logs(dest_dir):
    """Copy the raw /tmp tee logs from each container into <dest_dir>/<container>/.
    Idempotent (docker cp overwrites), so it is safe to call repeatedly — the
    snapshot loop calls it periodically so a hard pod cancel mid-iteration still
    leaves the most recent rayfronts/raven/lvlm stdout on disk."""
    targets = list(robot_containers())
    gcs = gcs_container()
    if gcs:
        targets.append(gcs)
    total = 0
    for name in targets:
        r = docker_exec(name, "ls " + " ".join(TEE_LOG_GLOBS) + " 2>/dev/null",
                        timeout=15)
        files = [f for f in r.stdout.split() if f]
        if not files:
            continue
        out = dest_dir / name
        out.mkdir(parents=True, exist_ok=True)
        for f in files:
            sh(["docker", "cp", f"{name}:{f}", str(out)], timeout=120)
        total += len(files)
    return total


def snapshot_task_logs(dest_dir):
    """Copy the raw rayfronts/raven/lvlm/gossip/ddsrouter/relay tees (written to
    /tmp inside each container) into <dest_dir>/<container>/, plus the navigation /
    control-stack node logs that rcl writes under ~/.ros/log rather than /tmp.
    Includes the GCS container so the GCS-side gossip ddsrouter log is captured."""
    targets = list(robot_containers())
    gcs = gcs_container()
    if gcs:
        targets.append(gcs)
    _copy_tmp_tee_logs(dest_dir)
    name_pred = " -o ".join(f"-name '{g}'" for g in CONTROL_STACK_GLOBS)
    tar = "/tmp/control_stack_logs.tar.gz"
    for name in targets:
        r = docker_exec(name, "ls " + " ".join(TEE_LOG_GLOBS) + " 2>/dev/null",
                        timeout=15)
        files = [f for f in r.stdout.split() if f]
        out = dest_dir / name
        made = docker_exec(
            name,
            f"f=$(find /root/.ros/log -type f \\( {name_pred} \\) 2>/dev/null); "
            f"[ -n \"$f\" ] && tar czf {tar} $f 2>/dev/null && echo COLLECTED",
            timeout=30)
        if "COLLECTED" in (made.stdout or ""):
            out.mkdir(parents=True, exist_ok=True)
            sh(["docker", "cp", f"{name}:{tar}",
                str(out / "control_stack_logs.tar.gz")], timeout=120)
            files = files + ["control_stack_logs.tar.gz"]
        if files:
            log(f"collected {len(files)} node log(s) from {name}")


def snapshot_raven_results(dest_dir):
    """Copy raven metrics (per-robot dumps + compiled_results / metrics /
    groundtruth_comparison) from the shared /root/.cache/raven_results into
    <dest_dir>/ so osmo:fetch pulls them alongside the bags."""
    names = robot_containers()
    if not names:
        return
    src = "/root/.cache/raven_results"
    name = names[0]   # shared ./cache bind mount: any robot sees all files
    r = docker_exec(name, f"ls {src} 2>/dev/null", timeout=15)
    if not r.stdout.strip():
        return
    dest_dir.mkdir(parents=True, exist_ok=True)
    cp = sh(["docker", "cp", f"{name}:{src}/.", str(dest_dir)], timeout=120)
    if cp.returncode != 0:
        log(f"WARN: docker cp of raven_results from {name} failed: "
            f"{cp.stderr.strip()[:200]}")
        return
    log(f"collected {len(list(dest_dir.glob('*.json')))} raven_results file(s) "
        f"→ {dest_dir}")


def snapshot_container_logs(dest_dir):
    dest_dir.mkdir(parents=True, exist_ok=True)
    pats = ("airstack", "isaac-sim", "ms-airsim")
    names = [n for n in list_containers(all_states=True)
             if any(p in n for p in pats)]
    for name in names:
        r = sh(["docker", "logs", name], timeout=120)
        (dest_dir / f"{name}.log").write_text(
            (r.stdout or "") + (("\n--- stderr ---\n" + r.stderr) if r.stderr else ""),
            encoding="utf-8")


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, default=str) + "\n", encoding="utf-8")


def resolve_results_root(airstack_root):
    """Prefer /osmo/output (auto-uploaded by OSMO `outputs:` when the task
    exits); keep <airstack>/osmo/results working in both cases so
    `airstack osmo:fetch` has one stable path to rsync."""
    fallback = Path(airstack_root) / "osmo" / "results"
    override = os.environ.get("OSMO_RESULTS_ROOT")
    if override:
        root = Path(override)
        root.mkdir(parents=True, exist_ok=True)
        return root
    osmo_output = Path(os.environ.get("OSMO_OUTPUT_DIR", "/osmo/output"))
    if osmo_output.is_dir() and os.access(osmo_output, os.W_OK):
        root = osmo_output / "airstack-mission-results"
        root.mkdir(parents=True, exist_ok=True)
        if not fallback.exists():
            fallback.parent.mkdir(parents=True, exist_ok=True)
            fallback.symlink_to(root)
        return root
    fallback.mkdir(parents=True, exist_ok=True)
    return fallback


# ── main loop ──────────────────────────────────────────────────────────────

# Per-container health snapshot. Sections are emitted on tagged lines so the
# multi-line ps output stays parseable. cgroup paths are v2; absent on v1 (the
# fields just come back empty).
_SNAP_CMD = r"""
echo "T|$(date '+%F %T')"
echo "DDS|procs=$(pgrep -c ddsrouter 2>/dev/null) estab=$(ss -Htn 2>/dev/null | grep -c ESTAB) ddstcp=$(ss -Htnp 2>/dev/null | grep -c ddsrouter) cpu=$(ps -C ddsrouter -o %cpu= 2>/dev/null | tr '\n' ',')"
echo "MEM|cur=$(cat /sys/fs/cgroup/memory.current 2>/dev/null) max=$(cat /sys/fs/cgroup/memory.max 2>/dev/null) oom_kill=$(awk '/^oom_kill /{print $2}' /sys/fs/cgroup/memory.events 2>/dev/null)"
echo "PS|"
ps -eo rss=,args= --sort=-rss 2>/dev/null | head -12
"""

# Processes whose early death we want to catch. Matched as substrings of the
# full command line (the python nodes all run as `python3 ...`). lvlm_baseline_node
# is the FPV+LVLM baseline — tracking it shows whether InternVL3 actually loaded
# (a loaded model is GB-scale RSS; a missing/crashed node shows 0).
_WATCH_PROCS = ("rayfronts", "raven_nav_node", "lvlm_baseline_node",
                "ddsrouter", "gossip")


def _parse_snapshot(out):
    ts = dds = mem = ""
    oom = None
    ps_rows = []
    in_ps = False
    for ln in out.splitlines():
        if ln.startswith("T|"):
            ts = ln[2:].strip()
        elif ln.startswith("DDS|"):
            dds = ln[4:].strip()
        elif ln.startswith("MEM|"):
            mem = ln[4:].strip()
            m = re.search(r"oom_kill=(\d+)", mem)
            oom = int(m.group(1)) if m else None
        elif ln.startswith("PS|"):
            in_ps = True
        elif in_ps:
            p = ln.split(None, 1)
            if len(p) == 2 and p[0].isdigit():
                ps_rows.append((int(p[0]), p[1].strip()))
    return ts, dds, mem, oom, ps_rows


def _format_resources(mem, ps_rows):
    def mb(v):
        return f"{int(v) // 1048576}MB" if v.isdigit() else (v or "?")
    m = re.search(r"cur=(\S+) max=(\S+)", mem)
    memstr = f"mem={mb(m.group(1))}/{mb(m.group(2))}" if m else mem
    oom = re.search(r"oom_kill=\S+", mem)
    key = {k: 0 for k in _WATCH_PROCS}
    for rss, args in ps_rows:
        for k in _WATCH_PROCS:
            if k in args:
                key[k] += rss
    keystr = " ".join(
        f"{k.replace('_nav_node', '')}={v // 1024}MB" for k, v in key.items())
    top = []
    for rss, args in ps_rows[:5]:
        label = next((k for k in _WATCH_PROCS if k in args),
                     args.split()[0].rsplit("/", 1)[-1][:16])
        top.append(f"{label}:{rss // 1024}MB")
    return f"{memstr} {oom.group(0) if oom else ''} | {keystr} | top: {' '.join(top)}", key


def _capture_oom_dmesg(iter_dir, container, ts, why):
    """Append the kernel OOM-killer trail (host ring buffer, readable from the
    privileged container) when a watched process dies or oom_kill increments."""
    r = docker_exec(
        container,
        "dmesg -T 2>/dev/null | grep -iE 'killed process|out of memory|oom-kill|"
        "invoked oom|memory cgroup' | tail -40", timeout=15)
    txt = (r.stdout or "").strip()
    block = (f"### {container} @ {ts} — {why}\n"
             f"{txt or '(dmesg unavailable in container — run host: dmesg -T | grep -i oom)'}\n")
    with open(iter_dir / "oom_dmesg.log", "a", encoding="utf-8") as f:
        f.write(block)


def transport_snapshot_loop(iter_dir, stop_event, interval_s=5):
    """Periodic per-container health snapshot for debugging early process death:
      - transport.log : ddsrouter procs + domain-99 TCP (gossip dropout)
      - resources.log : cgroup memory, oom_kill counter, watched + heaviest procs
      - oom_dmesg.log : kernel OOM trail, captured when a watched process
        vanishes or the cgroup oom_kill counter increments
    """
    iter_dir.mkdir(parents=True, exist_ok=True)
    tlog, rlog = iter_dir / "transport.log", iter_dir / "resources.log"
    prev_oom, prev_key = {}, {}
    # Periodically copy the /tmp tee logs into iter_dir/logs so a hard pod cancel
    # (SIGKILL — run_iteration's finally never runs) still leaves recent
    # rayfronts/raven/lvlm stdout. ~every 30s regardless of the health interval.
    tee_every = max(1, round(30 / interval_s))
    tick = 0
    while not stop_event.is_set():
        targets = list(robot_containers())
        gcs = gcs_container()
        if gcs:
            targets.append(gcs)
        tlines, rlines = [], []
        for name in targets:
            r = docker_exec(name, _SNAP_CMD, timeout=15)
            ts, dds, mem, oom, ps_rows = _parse_snapshot(r.stdout or "")
            if not ts:
                continue
            tlines.append(f"{name} [{ts}] {dds}")
            res, key = _format_resources(mem, ps_rows)
            rlines.append(f"{name} [{ts}] {res}")
            if oom is not None:
                if prev_oom.get(name, oom) < oom:
                    _capture_oom_dmesg(iter_dir, name, ts, f"cgroup oom_kill -> {oom}")
                prev_oom[name] = oom
            if ps_rows:   # only trust presence when we got a real ps sample
                for k in _WATCH_PROCS:
                    if prev_key.get((name, k), 0) > 0 and key[k] == 0:
                        _capture_oom_dmesg(iter_dir, name, ts, f"{k} vanished")
                    prev_key[(name, k)] = key[k]
        if tlines:
            with open(tlog, "a", encoding="utf-8") as f:
                f.write("\n".join(tlines) + "\n")
        if rlines:
            with open(rlog, "a", encoding="utf-8") as f:
                f.write("\n".join(rlines) + "\n")
        if tick % tee_every == 0:
            try:
                _copy_tmp_tee_logs(iter_dir / "logs")
            except Exception as e:
                log(f"WARN: periodic tee-log copy failed: {e}")
        tick += 1
        stop_event.wait(interval_s)


def run_iteration(stack, mission, iter_dir):
    """One full up → ready → record → steps → collect → down cycle.
    Returns the iteration summary dict; never raises (failures are data)."""
    summary = {"status": "passed", "steps_ok": 0, "steps_failed": 0}
    recorder = None
    container = None
    snap_stop = None
    t0 = time.time()
    try:
        # `airstack status`; if a stack is already up (baked entrypoint on a
        # stale image, or the previous iteration), down it and wait for the
        # containers to fully disappear before bringing up — otherwise the
        # `up` races leftover containers/network.
        stack.ensure_down()
        containers = stack.up()
        container = containers[0]
        summary["up_duration_s"] = round(time.time() - t0, 2)

        snap_stop = threading.Event()
        threading.Thread(target=transport_snapshot_loop,
                         args=(iter_dir, snap_stop), daemon=True).start()

        ready_at = stack.wait_ready(container)
        write_json(iter_dir / "ready.json", ready_at)

        # Clear stale raven metrics from the shared ./cache mount so this
        # iteration's compile doesn't pick up a prior iteration's files.
        docker_exec(container, "rm -rf /root/.cache/raven_results", timeout=15)

        recorder = Recorder(container, mission, stack.num_robots, stack.setup_bash)
        recorder.start()

        steps = []
        for i, step_spec in enumerate(mission["steps"], start=1):
            record = run_step(stack, container, step_spec, i)
            steps.append(record)
            if record["ok"]:
                summary["steps_ok"] += 1
                continue
            summary["steps_failed"] += 1
            # An `optional: true` step records its failure but never fails the
            # iteration (so it doesn't trip on_step_failure or an iteration redo)
            # — e.g. a post-search land after the gating step already ran.
            if isinstance(step_spec, dict) and step_spec.get("optional"):
                log(f"step {i}: failed but marked optional — continuing")
                summary.setdefault("optional_failures", []).append(i)
                continue
            summary["status"] = "failed"
            policy = mission["on_step_failure"]
            if policy == "continue":
                continue
            if policy == "abort_mission":
                summary["abort_mission"] = True
            break
        write_json(iter_dir / "steps.json", steps)

    except MissionStop as e:
        summary["status"] = "stopped"
        summary["abort_mission"] = True
        log(f"STOP: {e} — finalizing recordings and collecting artifacts")

    except Exception as e:
        summary["status"] = "error"
        summary["error"] = str(e)
        log(f"ERROR: iteration aborted: {e}")

    finally:
        if snap_stop is not None:
            snap_stop.set()
        # Artifact collection happens even on failure — a failed flight's
        # bag is usually the most interesting one.
        if recorder is not None:
            recorder.stop()
            recorder.collect(iter_dir / "bags")
        if container is not None or list_containers("airstack", all_states=True):
            snapshot_container_logs(iter_dir / "logs")
            snapshot_task_logs(iter_dir / "logs")
            # Metrics live in the robot ./cache mount, not the bag staging dir —
            # collect them before down() so osmo:fetch includes them.
            snapshot_raven_results(iter_dir / "raven_results")
        stack.down()
        summary["duration_s"] = round(time.time() - t0, 2)
        write_json(iter_dir / "iteration.json", summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("mission_file", help="Path to mission spec YAML")
    parser.add_argument("--airstack-root",
                        default=os.environ.get("AIRSTACK_ROOT", "/root/AirStack"))
    parser.add_argument("--dry-run", action="store_true",
                        help="Validate the spec and print the plan without touching docker")
    args = parser.parse_args()

    mission = load_mission(args.mission_file)
    stack = Stack(args.airstack_root, mission)

    # Graceful stop: SIGINT/SIGTERM (Ctrl-C locally, `airstack osmo:stop` on
    # a pod) aborts the current step but still finalizes the mcaps, collects
    # bags + logs, and brings the stack down.
    signal.signal(signal.SIGINT, _stop_handler)
    signal.signal(signal.SIGTERM, _stop_handler)

    log(f"mission '{mission['name']}': {mission['iterations']} iteration(s), "
        f"{len(mission['steps'])} step(s), {stack.num_robots} robot(s)"
        + (f", up to {mission['iteration_attempts']} attempt(s)/iteration"
           if mission["iteration_attempts"] > 1 else ""))
    if args.dry_run:
        print(yaml.safe_dump(mission, sort_keys=False))
        return 0

    results_root = resolve_results_root(args.airstack_root)
    run_dir = results_root / mission["name"] / utc_stamp()
    run_dir.mkdir(parents=True)
    log(f"results → {run_dir}")

    environments = mission.get("environments") or []
    n_iter = mission["iterations"]
    # Build the per-iteration environment schedule. round_robin: cycle envs,
    # `iterations` is the total. grouped: run each env `iterations` times in a
    # row, so the total is iterations * len(environments).
    if not environments:
        schedule = [None] * n_iter
    elif mission["environment_order"] == "grouped":
        schedule = [env for env in environments for _ in range(n_iter)]
    else:
        schedule = [environments[i % len(environments)] for i in range(n_iter)]
    total = len(schedule)
    if environments:
        log(f"{mission['environment_order']} over {len(environments)} "
            f"environment(s) → {total} iteration(s): "
            f"{[e['name'] for e in schedule]}")

    iterations = []
    max_attempts = mission["iteration_attempts"]
    for i, env_entry in enumerate(schedule, start=1):
        log(f"━━━ iteration {i}/{total} ━━━")
        method = resolve_method(mission, env_entry)
        iter_dir = run_dir / iter_dir_name(i, env_entry, method)
        # Apply this iteration's environment (scene URL + spawn area as env vars
        # for `airstack up`; search_area et al. templated into steps).
        if env_entry is not None:
            log(f"environment: {env_entry['name']} "
                f"(ENV_URL={env_entry.get('ENV_URL')})")
            stack.apply_env(env_exports(env_entry))
            iter_mission = {**mission, "steps": substitute_env(mission["steps"], env_entry)}
        else:
            iter_mission = mission

        # Iteration redo: re-run the SAME iteration up to iteration_attempts
        # times until it passes. A passed run, a manual stop, or an explicit
        # abort_mission is never retried; a failed/errored run is (the stack is
        # already down by run_iteration's finally, so each attempt is a clean
        # down→up→fly→down cycle — and with a fixed SPAWN_SEED the spawn layout
        # is reproduced exactly).
        attempt = 0
        while True:
            attempt += 1
            if attempt > 1:
                log(f"iteration {i}: attempt {attempt}/{max_attempts}")
            summary = run_iteration(stack, iter_mission, iter_dir)
            done = (summary["status"] in ("passed", "stopped")
                    or summary.get("abort_mission")
                    or attempt >= max_attempts)
            if done:
                break
            # Failed with retries left: stash this attempt's artifacts so the
            # canonical iter_NNN ends up holding the final attempt.
            failed_dir = run_dir / f"{iter_dir.name}_failed_attempt_{attempt}"
            try:
                if iter_dir.exists():
                    iter_dir.rename(failed_dir)
                log(f"iteration {i} {summary['status']} on attempt {attempt}"
                    f"/{max_attempts}; artifacts → {failed_dir.name}; redoing")
            except OSError as e:
                log(f"WARN: could not stash failed attempt dir ({e}); "
                    f"redoing into {iter_dir.name}")

        summary["iteration"] = i
        summary["attempts"] = attempt
        summary["method"] = method
        if env_entry is not None:
            summary["environment"] = env_entry["name"]
        iterations.append(summary)
        write_json(run_dir / "summary.json",
                   {"mission": mission["name"], "mission_file": args.mission_file,
                    "iterations": iterations})
        if summary.get("abort_mission"):
            log("stopping remaining iterations"
                + (" (manual stop)" if summary["status"] == "stopped" else
                   " (on_step_failure=abort_mission)"))
            break

    passed = sum(1 for s in iterations if s["status"] == "passed")
    log(f"mission complete: {passed}/{len(iterations)} iteration(s) passed; "
        f"results in {run_dir}")
    return 0 if passed == len(iterations) else 1


if __name__ == "__main__":
    sys.exit(main())

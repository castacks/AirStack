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
    ├── summary.json                 # per-iteration pass/fail + durations
    └── iter_001/
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
import shlex
import subprocess
import sys
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
    "on_step_failure": "abort_iteration",  # continue | abort_iteration | abort_mission
    "ready": {"timeout_s": 600, "poll_interval_s": 5},
    # scope "gcs": one recorder on GCS domain 0 → one mcap with every robot's
    # bridged topics (default). scope "robot": one recorder + one mcap per
    # robot on its own domain (for unbridged/high-rate topics).
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
    if merged["command_route"] not in ("gcs", "robot"):
        raise ValueError(f"command_route must be gcs|robot, got {merged['command_route']!r}")
    if merged["record"].get("scope", "gcs") not in ("gcs", "robot"):
        raise ValueError(f"record.scope must be gcs|robot, "
                         f"got {merged['record'].get('scope')!r}")
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

    def _topic_selection(self, robots):
        """Build the `ros2 bag record` topic args; `robots` is the list of
        robot indices whose {robot}/{n} placeholders to expand (unioned,
        order-preserving, deduplicated)."""
        if self.cfg.get("all"):
            return "-a"
        default = (DEFAULT_GCS_RECORD_TOPICS if self.scope == "gcs"
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
            n_topics = "all topics" if selection == "-a" else f"{len(selection.split())} topics"
            log(f"recording [{tag}] in {container} (domain {domain_id}) "
                f"→ {out_dir} ({n_topics})")
        else:
            log(f"WARN: recorder [{tag}] failed to start / exited immediately: "
                f"{tail(r.stdout + r.stderr, 6)}")

    def start(self):
        if not self.cfg.get("enabled", True):
            log("recording disabled by mission spec")
            return
        robots = list(range(1, self.num_robots + 1))
        if self.scope == "gcs":
            gcs = gcs_container()
            if not gcs:
                log("WARN: record.scope is 'gcs' but no gcs container is running — "
                    "recording skipped (bring up the gcs service or use scope: robot)")
                return
            self._start_one(gcs, 0, "gcs", self._topic_selection(robots),
                            GCS_SETUP_BASH)
        else:
            for n in robots:
                self._start_one(self.robot_container, n, f"robot_{n}",
                                self._topic_selection([n]), self.setup_bash)

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

            def send(n):
                # Same path as Foxglove: publish String JSON on
                # /<robot>/tasks/<task>/goal (GCS domain 0); the per-robot
                # action_relay forwards it as a typed action goal on domain N
                # and reports {"success": ..., "message": ...} on
                # .../relay_result. Subscribe to the result *before*
                # publishing the goal so a fast result can't be missed.
                base = f"/robot_{n}/tasks/{task}"
                result_file = f"/tmp/relay_result_{task}_{n}.out"
                msg_yaml = json.dumps({"data": expand(goal_json, n)})
                script = (
                    f"rm -f {result_file}\n"
                    f"( timeout {int(timeout)} ros2 topic echo --once --field data "
                    f"{base}/relay_result > {result_file} 2>&1 ) &\n"
                    f"sub=$!\n"
                    f"sleep 3\n"
                    f"ros2 topic pub --once {base}/goal std_msgs/msg/String "
                    f"{shlex.quote(msg_yaml)} > /dev/null\n"
                    f"wait $sub\n"
                    f"cat {result_file}"
                )
                r = ros2_exec(gcs, script, domain_id=0, setup_bash=GCS_SETUP_BASH,
                              timeout=int(timeout + 30))
                ok = '"success": true' in r.stdout
                return n, {"exit": r.returncode, "ok": ok,
                           "output_tail": tail(r.stdout + r.stderr)}
        else:
            action_type = spec.get("type", task_action_type(task))

            def send(n):
                cmd = (f"ros2 action send_goal --feedback /robot_{n}/tasks/{task} "
                       f"{action_type} {shlex.quote(expand(goal_json, n))}")
                r = ros2_exec(container, cmd, domain_id=n, setup_bash=stack.setup_bash,
                              timeout=int(timeout + 15))
                return n, {"exit": r.returncode, "ok": action_ok(r.stdout),
                           "output_tail": tail(r.stdout + r.stderr)}

        with ThreadPoolExecutor(max_workers=len(robots)) as pool:
            results = dict(pool.map(send, robots))
        record.update(type="action", task=task, via=via, per_robot=results,
                      ok=all(v["ok"] for v in results.values()))

    elif "run" in step_spec:
        spec = step_spec["run"]
        cmd = spec["cmd"]
        timeout = float(spec.get("timeout_s", 60))
        expect = spec.get("expect_success", True)
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
                results[n] = _run_one(stack, container, expand(target, n),
                                      expand(cmd, n), timeout, expect)
            record.update(type="run", per_robot=results,
                          ok=all(v["ok"] for v in results.values()))
        else:
            log(f"step {step_index}: run [{target}] {cmd}")
            res = _run_one(stack, container, target, cmd, timeout, expect)
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

def snapshot_container_logs(dest_dir):
    dest_dir.mkdir(parents=True, exist_ok=True)
    for name in list_containers(name_pattern="airstack", all_states=True):
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

def run_iteration(stack, mission, iter_dir):
    """One full up → ready → record → steps → collect → down cycle.
    Returns the iteration summary dict; never raises (failures are data)."""
    summary = {"status": "passed", "steps_ok": 0, "steps_failed": 0}
    recorder = None
    container = None
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

        ready_at = stack.wait_ready(container)
        write_json(iter_dir / "ready.json", ready_at)

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
            summary["status"] = "failed"
            policy = mission["on_step_failure"]
            if policy == "continue":
                continue
            if policy == "abort_mission":
                summary["abort_mission"] = True
            break
        write_json(iter_dir / "steps.json", steps)

    except Exception as e:
        summary["status"] = "error"
        summary["error"] = str(e)
        log(f"ERROR: iteration aborted: {e}")

    finally:
        # Artifact collection happens even on failure — a failed flight's
        # bag is usually the most interesting one.
        if recorder is not None:
            recorder.stop()
            recorder.collect(iter_dir / "bags")
        if container is not None or list_containers("airstack", all_states=True):
            snapshot_container_logs(iter_dir / "logs")
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

    log(f"mission '{mission['name']}': {mission['iterations']} iteration(s), "
        f"{len(mission['steps'])} step(s), {stack.num_robots} robot(s)")
    if args.dry_run:
        print(yaml.safe_dump(mission, sort_keys=False))
        return 0

    results_root = resolve_results_root(args.airstack_root)
    run_dir = results_root / mission["name"] / utc_stamp()
    run_dir.mkdir(parents=True)
    log(f"results → {run_dir}")

    iterations = []
    for i in range(1, mission["iterations"] + 1):
        log(f"━━━ iteration {i}/{mission['iterations']} ━━━")
        iter_dir = run_dir / f"iter_{i:03d}"
        summary = run_iteration(stack, mission, iter_dir)
        summary["iteration"] = i
        iterations.append(summary)
        write_json(run_dir / "summary.json",
                   {"mission": mission["name"], "mission_file": args.mission_file,
                    "iterations": iterations})
        if summary.get("abort_mission"):
            log("on_step_failure=abort_mission — stopping remaining iterations")
            break

    passed = sum(1 for s in iterations if s["status"] == "passed")
    log(f"mission complete: {passed}/{len(iterations)} iteration(s) passed; "
        f"results in {run_dir}")
    return 0 if passed == len(iterations) else 1


if __name__ == "__main__":
    sys.exit(main())

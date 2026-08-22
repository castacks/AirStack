# Copyright (c) 2026 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Check implementations for ``airstack doctor`` (RFC #379 §4, RFC #380 §2).

Doctor is **entirely observational**: it never generates, edits, or infers
anything — it observes the checkout (compose time) or the running graph
(``--live``) and reports. It hardens into a hard error (exit 1 in default
mode) in exactly **two enumerated places** and nowhere else:

1. **Dep conflicts** that would compose a broken image
   (``compose_module_layers.py --check-conflicts`` — RFC #379 §6);
2. **Safety-placement violations** — control-setpoint or trajectory-group
   names in any stack's ``bridge.yaml``
   (``gen_dds_router.py --check`` — RFC #380 §2).

Everything else reports and steps aside. Growing that list requires the
RFC #379 §8 process.

Compose-time checks reuse the existing single-purpose tools
(``validate_module.py``, ``module_overlay.py --check``,
``compose_module_layers.py --check-conflicts``, ``gen_dds_router.py --check``)
rather than reimplementing them; ``--live`` reuses ``tests/wiring_snapshot.py``
(the same capture/normalize/diff machinery as the CI wiring-snapshot test).
"""
import importlib.util
import json
import os
import re
import socket
import subprocess
import sys
import time
from pathlib import Path

# NOTE: the yaml import doubles as doctor's python3+PyYAML availability check
# (the shell CLI's twin lives in .airstack/modules/_lib.sh:_require_python_yaml
# — cross-language dedupe deferred; keep the two behaviors in sync).
import yaml

# Doctor always runs its own repo's tools, even when --project-root points at
# another checkout (a sandbox under test has no tools/ of its own).
TOOLS_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = TOOLS_DIR.parent

# Container-side ROS environment (mirrors .airstack/modules/ready.sh).
ROS_DISTRO_SETUP = "/opt/ros/jazzy/setup.bash"
ROBOT_WS_SETUP = "/root/AirStack/robot/ros_ws/install/setup.bash"

_TOPICS_PER_EXEC = 40
_INFO_TIMEOUT_S = 20

OK, WARN, FAIL = "ok", "warn", "fail"


class CheckResult:
    """One named check: status (ok/warn/fail), hard-gate flag, and messages."""

    def __init__(self, name, hard=False):
        self.name = name
        self.hard = hard
        self.status = OK
        self.messages = []

    def note(self, message):
        self.messages.append(message)

    def warn(self, message):
        self.messages.append(message)
        if self.status == OK:
            self.status = WARN

    def fail(self, message):
        self.messages.append(message)
        self.status = FAIL

    @property
    def gates(self):
        """True when this result must fail the doctor run (exit 1)."""
        return self.hard and self.status == FAIL


def _load_tool(name):
    """Import a tools/*.py module by path (mirrors the meta-test idiom)."""
    spec = importlib.util.spec_from_file_location(
        f"airstack_doctor_{name}", TOOLS_DIR / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _run_tool(script, args, timeout=120):
    """Run a tools/*.py script in a subprocess; return (returncode, output)."""
    proc = subprocess.run(
        [sys.executable, str(TOOLS_DIR / script), *args],
        capture_output=True, text=True, timeout=timeout,
    )
    output = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode, output.strip()


def _stack_dirs(root, stack=None):
    stacks_dir = Path(root) / "stacks"
    if not stacks_dir.is_dir():
        return []
    dirs = sorted(
        d for d in stacks_dir.iterdir()
        if d.is_dir() and not d.name.startswith(".")
    )
    if stack:
        dirs = [d for d in dirs if d.name == stack]
    return dirs


# ── compose-time checks ──────────────────────────────────────────────────────

def check_module_manifests(root):
    """Every synced / in-tree module manifest validates (observe-only)."""
    result = CheckResult("module-manifests")
    validator = _load_tool("validate_module")
    module_dirs = []
    for base in (Path(root) / "modules",
                 Path(root) / "robot" / "ros_ws" / "src" / "modules"):
        if not base.is_dir():
            continue
        for child in sorted(base.iterdir()):
            # in-tree overlay symlinks point back at modules/<name> checkouts,
            # already covered by the first base (same rule as module doctor)
            if base.name == "modules" and base.parent.name == "src" and child.is_symlink():
                continue
            if child.is_dir() and (child / "module.yaml").is_file():
                module_dirs.append(child)
    if not module_dirs:
        result.note("no modules synced — nothing to validate")
        return result
    for module_dir in module_dirs:
        verdict, warnings = validator.validate_module(module_dir)
        rel = os.path.relpath(module_dir, root)
        for warning in warnings:
            result.warn(f"{rel}: {warning}")
        if verdict["valid"]:
            result.note(f"{rel}: manifest OK")
        else:
            for error in verdict["errors"]:
                result.warn(f"{rel}: {error['path']}: {error['message']}")
            result.warn(f"{rel}: manifest INVALID (sync is what fails on this)")
    return result


def check_overlay(root):
    """Overlay integrity: symlinks + generated compose fresh (observe-only)."""
    result = CheckResult("module-overlay")
    code, output = _run_tool(
        "module_overlay.py", ["--check", "--project-root", str(root)])
    if code == 0:
        result.note("overlay symlinks and generated compose are consistent")
    else:
        result.warn("overlay is broken or stale — run 'airstack module sync'")
        if output:
            result.warn(output)
    return result


def check_layer_conflicts(root):
    """HARD GATE #1 (RFC #379 §6): dep conflicts across module layers."""
    result = CheckResult("module-dep-conflicts", hard=True)
    code, output = _run_tool(
        "compose_module_layers.py",
        ["--check-conflicts", "--project-root", str(root)])
    if code == 0:
        result.note("no apt/pip pin conflicts across modules")
    else:
        result.fail("module dependency conflict — composing would build a "
                    "broken image (doctor hard gate #1, RFC #379 §6)")
        if output:
            result.fail(output)
    return result


_COMMENT_RE = re.compile(r"<!--.*?-->", re.DOTALL)


def check_stack_layout(root, stack=None):
    """Stack folder anatomy (RFC #379 §3, observe-only): the four committed
    files, plus the split-stack rule — two or more entry points require a
    bridge.yaml (RFC #380 §2). Mirrors tests/meta/test_stack_layout_contract.py."""
    result = CheckResult("stack-layout")
    stack_dirs = _stack_dirs(root, stack)
    if not stack_dirs:
        result.warn(f"no stack folders under {Path(root) / 'stacks'}"
                    + (f" matching {stack!r}" if stack else ""))
        return result
    for stack_dir in stack_dirs:
        name = stack_dir.name
        repos = stack_dir / "modules.repos"
        if not repos.is_file():
            result.warn(f"{name}: missing modules.repos")
        else:
            try:
                data = yaml.safe_load(repos.read_text(encoding="utf-8"))
            except yaml.YAMLError as exc:
                data = None
                result.warn(f"{name}: modules.repos is invalid YAML: {exc}")
            if isinstance(data, dict):
                compat = data.get("airstack_compat")
                if not (isinstance(compat, str) and compat.strip()):
                    result.warn(f"{name}: modules.repos needs a top-level "
                                "airstack_compat semver-range string")
                if "repositories" not in data:
                    result.warn(f"{name}: modules.repos needs a repositories: key")

        entries = sorted((stack_dir / "launch").glob("*.launch.xml")) \
            if (stack_dir / "launch").is_dir() else []
        if not entries:
            result.warn(f"{name}: launch/ has no *.launch.xml entry point")
        for entry in entries:
            text = _COMMENT_RE.sub("", entry.read_text(encoding="utf-8",
                                                       errors="replace"))
            if "robot.launch.xml" in text:
                result.warn(f"{name}/launch/{entry.name}: includes the "
                            "dispatcher robot.launch.xml (infinite recursion)")

        compose = stack_dir / "docker-compose.yaml"
        if not compose.is_file():
            result.warn(f"{name}: missing docker-compose.yaml")
        else:
            try:
                if yaml.safe_load(compose.read_text(encoding="utf-8")) is None:
                    result.warn(f"{name}: docker-compose.yaml is empty")
            except yaml.YAMLError as exc:
                result.warn(f"{name}: docker-compose.yaml invalid YAML: {exc}")

        readme = stack_dir / "README.md"
        if not readme.is_file():
            result.warn(f"{name}: missing README.md")
        elif len(readme.read_text(encoding="utf-8").strip()) < 200:
            result.warn(f"{name}: README.md is trivial")

        # split-stack rule: >= 2 entry points require an explicit bridge.yaml
        if len(entries) >= 2 and not (stack_dir / "bridge.yaml").is_file():
            result.warn(
                f"{name}: {len(entries)} launch entry points but no "
                "bridge.yaml — a split stack must declare its machine "
                "boundary explicitly (RFC #380 §2)")

        wiring = stack_dir / "wiring.md"
        if wiring.is_file():
            ws = _wiring_snapshot()
            try:
                ws.extract_graph_from_md(wiring.read_text(encoding="utf-8"))
            except ValueError as exc:
                result.warn(f"{name}: wiring.md has no parseable trailer "
                            f"({exc}) — regenerate, never hand-edit")
        else:
            result.note(f"{name}: wiring.md not committed yet (bootstrap)")
    if result.status == OK:
        result.note(f"{len(stack_dirs)} stack folder(s) pass the anatomy check")
    return result


def check_bridge_gates(root, stack=None):
    """HARD GATE #2 (RFC #380 §2): bridge.yaml schema + placement gate, via
    gen_dds_router --check, for every stack that carries a bridge.yaml."""
    result = CheckResult("bridge-placement-gate", hard=True)
    gen = _load_tool("gen_dds_router")
    checked = 0
    for stack_dir in _stack_dirs(root, stack):
        bridge = stack_dir / "bridge.yaml"
        if not bridge.is_file():
            continue
        checked += 1
        try:
            data = gen.load_bridge(bridge)
        except yaml.YAMLError as exc:
            result.fail(f"{stack_dir.name}/bridge.yaml: invalid YAML: {exc}")
            continue
        errors = gen.validate_bridge(data)
        if errors:
            for error in errors:
                result.fail(f"{stack_dir.name}/bridge.yaml: {error['path']}: "
                            f"{error['message']}")
        else:
            result.note(f"{stack_dir.name}/bridge.yaml: valid; no "
                        "control-setpoint/trajectory-group names cross "
                        "the boundary")
    if checked == 0:
        result.note("no stack carries a bridge.yaml — gate vacuously satisfied")
    return result


def run_compose_time_checks(root, stack=None):
    """The default `airstack doctor` check battery, in order."""
    return [
        check_module_manifests(root),
        check_overlay(root),
        check_layer_conflicts(root),
        check_stack_layout(root, stack=stack),
        check_bridge_gates(root, stack=stack),
    ]


# ── live capture (docker exec per robot; reuses tests/wiring_snapshot.py) ────

def _wiring_snapshot():
    """Import tests/wiring_snapshot.py (stdlib-only, runnable anywhere)."""
    spec = importlib.util.spec_from_file_location(
        "airstack_doctor_wiring_snapshot",
        REPO_ROOT / "tests" / "wiring_snapshot.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _docker(args, timeout=60, check=True):
    proc = subprocess.run(["docker", *args], capture_output=True, text=True,
                          timeout=timeout)
    if check and proc.returncode != 0:
        raise RuntimeError(
            f"docker {' '.join(args[:3])}... failed: {proc.stderr.strip()}")
    return proc.stdout


def discover_robot_containers():
    """Running robot containers, sorted (mirrors ready.sh): compose replicas
    (airstack-robot-desktop-N) and fleet-generated services
    (airstack-robot_N-1); ground-host tenants (gcs-robot_N) are not robots."""
    out = _docker(["ps", "--format", "{{.Names}}"])
    return sorted(
        n for n in out.splitlines()
        if ("-robot-" in n or "-robot_" in n) and "gcs-" not in n
    )


def container_identity(container):
    """(robot_name, domain_id) via the container's login-shell resolution —
    the same .bashrc path `airstack status` and `airstack ready` use."""
    out = _docker([
        "exec", container, "bash", "--login", "-c",
        'printf "AIRSTACK_VARS:%s:%s\\n" "$ROBOT_NAME" "$ROS_DOMAIN_ID"',
    ], timeout=30)
    lines = [l for l in out.splitlines() if l.startswith("AIRSTACK_VARS:")]
    if not lines:
        raise RuntimeError(f"{container}: could not resolve ROBOT_NAME/ROS_DOMAIN_ID")
    _, robot_name, domain = lines[-1].split(":", 2)
    return robot_name, domain


def _ros2_exec(container, domain, command, timeout=40):
    script = (
        f"source {ROS_DISTRO_SETUP} >/dev/null 2>&1\n"
        f"[ -f {ROBOT_WS_SETUP} ] && source {ROBOT_WS_SETUP} >/dev/null 2>&1\n"
        f"export ROS_DOMAIN_ID={domain}\n"
        f"timeout {timeout - 10} {command}"
    )
    return _docker(["exec", container, "bash", "-c", script],
                   timeout=timeout, check=False)


def _batched_topic_info(container, domain, topics, ws):
    """`ros2 topic info --verbose` per topic, chunk-batched per docker exec —
    the tests/system/test_wiring_snapshot.py idiom (backgrounded probes into
    /tmp files, catted back with ===FILE=== sentinels)."""
    outputs = {}
    for start in range(0, len(topics), _TOPICS_PER_EXEC):
        chunk = topics[start:start + _TOPICS_PER_EXEC]
        temp_files = {}
        probes = []
        for i, topic in enumerate(chunk, start=start):
            fname = f"/tmp/doctor_wiring_{i}.out"
            temp_files[topic] = fname
            probes.append(
                f"(ROS_DOMAIN_ID={domain} timeout {_INFO_TIMEOUT_S} "
                f"ros2 topic info --verbose {topic} > {fname} 2>&1) &")
        lines = [
            f"source {ROS_DISTRO_SETUP} >/dev/null 2>&1",
            f"[ -f {ROBOT_WS_SETUP} ] && source {ROBOT_WS_SETUP} >/dev/null 2>&1",
        ]
        lines += probes + ["wait"]
        for fname in temp_files.values():
            lines.append(f"echo '===FILE {fname}==='")
            lines.append(f"cat {fname} 2>/dev/null || true")
        out = _docker(["exec", container, "bash", "-c", "\n".join(lines)],
                      timeout=_INFO_TIMEOUT_S + 90, check=False)
        for piece in out.split("===FILE ")[1:]:
            header, _, content = piece.partition("===")
            fname = header.strip()
            topic = next((t for t, f in temp_files.items() if f == fname), None)
            if topic is not None:
                outputs[topic] = content
    return outputs


def container_stack_dir(container):
    """The container's AIRSTACK_STACK_DIR ('' when legacy role dispatch)."""
    try:
        out = _docker(["exec", container, "printenv", "AIRSTACK_STACK_DIR"])
        return out.strip()
    except Exception:
        return ""


def capture_live_graph(log=print, stack=None):
    """Snapshot running robots' graphs and merge into one normalized graph —
    the same capture tests/system/test_wiring_snapshot.py performs, driven by
    plain `docker exec` so it runs on any host with the stack up.

    In a heterogeneous fleet, robots run different stacks; when ``stack`` is
    given, only containers whose AIRSTACK_STACK_DIR names that stack are
    captured — one wiring.md describes one stack, not the fleet union."""
    ws = _wiring_snapshot()
    containers = discover_robot_containers()
    if not containers:
        raise RuntimeError("no running robot containers (docker ps shows no "
                           "robot names) — bring the stack up first")
    if stack:
        matched = [c for c in containers
                   if container_stack_dir(c).rstrip("/").endswith(f"/{stack}")]
        skipped = [c for c in containers if c not in matched]
        if skipped:
            log(f"[doctor] skipping {len(skipped)} robot container(s) on other "
                f"stacks: {', '.join(skipped)}")
        if not matched:
            raise RuntimeError(
                f"no running robot container has AIRSTACK_STACK_DIR ending in "
                f"/{stack} — is that stack actually up?")
        containers = matched
    nodes, topics, edges = set(), {}, []
    for container in containers:
        robot_name, domain = container_identity(container)
        node_out = _ros2_exec(container, domain, "ros2 node list 2>/dev/null")
        robot_nodes = ws.parse_node_list(node_out)
        topic_out = _ros2_exec(container, domain, "ros2 topic list 2>/dev/null")
        robot_topics = sorted({
            line.strip() for line in topic_out.splitlines()
            if line.strip().startswith("/")
        })
        log(f"[doctor] {container} ({robot_name}, domain {domain}): "
            f"{len(robot_nodes)} nodes, {len(robot_topics)} topics")
        nodes.update(robot_nodes)
        infos = _batched_topic_info(container, domain, robot_topics, ws)
        for topic in robot_topics:
            text = infos.get(topic)
            if not text:
                continue
            entry, topic_edges = ws.parse_topic_info_verbose(topic, text)
            topics.setdefault(topic, entry)
            edges.extend(topic_edges)
    graph = {"version": 1, "nodes": sorted(nodes), "topics": topics,
             "edges": edges}
    return ws.normalize_graph(graph)


# ── safety-floor visibility (RFC #379 §4: report loudly, never gate) ─────────

# Trajectory-group command inputs: ANY module may publish these — that is the
# selling point (a trajectory_override publisher inherits the whole safety
# apparatus). They are listed as the command-authority map, never flagged.
_COMMAND_INPUT_BASENAMES = {"trajectory_override", "trajectory_segment_to_add"}
# Trajectory-group outputs: only the trajectory controller may publish these.
_CONTROLLER_OUTPUT_BASENAMES = {"tracking_point", "look_ahead"}
# Control setpoints: the interface's command inputs (today's concrete spelling
# of the conventions-spec `control_setpoint` interchange).
_SETPOINT_TOPIC_RE = re.compile(r"/interface/cmd_[A-Za-z0-9_]+$")
# Blessed publishers of control setpoints (node-name suffixes).
_BLESSED_SETPOINT_NODES = ("/control/pid_controller", "/interface/odom_modifier")


def check_safety_floor(graph):
    """Flag publishers of control-setpoint / controller-output topics that are
    not the blessed controller chain. Visibility, not enforcement: WARN only
    (exit 1 only under --strict)."""
    result = CheckResult("safety-floor")
    authority_map = []
    for edge in graph.get("edges") or []:
        if edge.get("dir") != "pub":
            continue
        topic = edge.get("topic") or ""
        node = edge.get("node") or ""
        basename = topic.rsplit("/", 1)[-1]
        if basename in _COMMAND_INPUT_BASENAMES:
            authority_map.append(f"{node} -> {topic}")
        elif basename in _CONTROLLER_OUTPUT_BASENAMES:
            if "/trajectory_controller/" not in node + "/":
                result.warn(
                    f"UNBLESSED controller-output publisher: {node} publishes "
                    f"{topic} but is not the trajectory controller — "
                    "something is impersonating the blessed controller "
                    "(RFC #379 §4 safety floor)")
        elif _SETPOINT_TOPIC_RE.search(topic) or basename == "control_setpoint":
            if not node.endswith(_BLESSED_SETPOINT_NODES):
                result.warn(
                    f"UNBLESSED control-setpoint publisher: {node} publishes "
                    f"{topic} — command authority is bypassing the blessed "
                    "controller chain (RFC #379 §4 safety floor)")
    if authority_map:
        result.note("command-authority map (trajectory-group publishers — "
                    "informational):")
        for line in sorted(authority_map):
            result.note(f"  {line}")
    if result.status == OK:
        result.note("all control-setpoint/controller-output publishers are "
                    "the blessed controller chain")
    return result


# ── live / snapshot drivers ──────────────────────────────────────────────────

def infer_stack(stack):
    """--stack wins; else infer from AIRSTACK_STACK_DIR (how `airstack up`
    selects a stack)."""
    if stack:
        return stack
    stack_dir = os.environ.get("AIRSTACK_STACK_DIR", "").rstrip("/")
    if stack_dir:
        return os.path.basename(stack_dir)
    return None


def run_live(root, stack, strict=False, log=print):
    """`doctor --live`: capture the running graph, diff against the stack's
    committed wiring.md, and run the safety-floor scan. Returns exit code."""
    ws = _wiring_snapshot()
    stack = infer_stack(stack)
    if not stack:
        log("[doctor] --live needs a stack: pass --stack <name> or set "
            "AIRSTACK_STACK_DIR (airstack up ...stack <name>)")
        return 1

    graph = capture_live_graph(log=log, stack=stack)

    exit_code = 0
    safety = check_safety_floor(graph)
    for message in safety.messages:
        log(f"[doctor] {message}")
    if safety.status != OK and strict:
        log("[doctor] --strict: safety-floor warnings are fatal")
        exit_code = 1

    wiring_path = Path(root) / "stacks" / stack / "wiring.md"
    if not wiring_path.is_file():
        log(f"[doctor] no committed wiring.md at {wiring_path} — nothing to "
            "diff against. Bootstrap it from a validated run "
            "(airstack test -m wiring --stack {0}) or, on hardware, "
            "'airstack doctor --snapshot --stack {0}'.".format(stack))
        return 1
    expected = ws.extract_graph_from_md(wiring_path.read_text(encoding="utf-8"))
    verdict = ws.diff_graphs(expected, graph)
    if verdict["identical"]:
        log(f"[doctor] graph matches wiring.md ({wiring_path})")
        return exit_code
    log(f"[doctor] DRIFT: the running graph differs from {wiring_path}:")
    log(json.dumps(verdict, indent=2))
    log("[doctor] if the change is intentional, regenerate wiring.md from a "
        "wiring-snapshot run (or --snapshot on hardware) and commit it.")
    return 1


def run_snapshot(root, stack, log=print):
    """`doctor --snapshot`: the identical capture, committed as the stack's
    wiring.md with hardware provenance (RFC #379 §4.4: a stack that cannot run
    in CI still gets an *observed* wiring.md, refreshed by hand; the CI drift
    check reports it as unverified-in-CI rather than silently passing)."""
    ws = _wiring_snapshot()
    stack = infer_stack(stack)
    if not stack:
        log("[doctor] --snapshot needs a stack: pass --stack <name> or set "
            "AIRSTACK_STACK_DIR")
        return 1
    stack_dir = Path(root) / "stacks" / stack
    if not stack_dir.is_dir():
        log(f"[doctor] no stack folder at {stack_dir}")
        return 1

    graph = capture_live_graph(log=log, stack=stack)
    safety = check_safety_floor(graph)
    for message in safety.messages:
        log(f"[doctor] {message}")

    hostname = socket.gethostname()
    date = time.strftime("%Y-%m-%d %H:%M:%S")
    sha = _git_short_sha(root)
    meta = {
        "stack": stack,
        "generated-by": "airstack doctor --snapshot",
        "date": date,
        "source-sha": sha,
        "provenance": f"observed on {hostname}, {date}, {sha} — "
                      "unverified-in-CI",
    }
    wiring_path = stack_dir / "wiring.md"
    wiring_path.write_text(ws.render_wiring_md(graph, meta), encoding="utf-8")
    log(f"[doctor] wrote {wiring_path} ({len(graph['nodes'])} nodes, "
        f"{len(graph['edges'])} edges) — review and commit it; the CI drift "
        "check will report this stack as unverified-in-CI")
    return 0


def _git_short_sha(root):
    try:
        proc = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                              cwd=str(root), capture_output=True, text=True,
                              timeout=10)
        if proc.stdout.strip():
            return proc.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        pass
    return "unknown"

"""Observed wiring snapshot + golden drift check (RFC #379 §4.4).

Runs after ``system.test_liveliness`` (see ``_MODULE_ORDER``). Once the
sentinel nodes are up, snapshots the *running* ROS graph per robot —
``ros2 node list`` + ``ros2 topic list`` + one batched ``docker exec`` of
``ros2 topic info --verbose`` probes per robot — merges it into one graph
(node/edge names keep their robot namespaces), and renders it to
``<run_dir>/wiring/observed_full_default.md`` via ``tests/wiring_snapshot.py``.

Golden logic: if ``tests/goldens/wiring/full_default.<sim>.<N>robot.md`` is
missing, the test logs a bootstrap instruction and PASSES (goldens are
committed from a validated local run — see ``tests/goldens/wiring/README.md``);
if present, ``diff_graphs`` must report identical or the test fails with the
JSON drift verdict.

Stack mode (RFC #379 §3/§4): with ``--stack <name>`` the fixture launches the
stack's entry launch file and the golden becomes ``stacks/<name>/wiring.md``
(one observed-wiring document per stack, committed in the stack folder). The
observed snapshot is written as ``observed_<name>.md`` and the same
bootstrap-pass + INSTRUCTION behavior applies when the stack has no wiring.md
yet. Legacy runs (no ``--stack``) keep the tests/goldens/wiring path
unchanged.
"""
import json
import os
import subprocess
import time

import pytest

from conftest import (
    AIRSTACK_ROOT,
    ROS_DISTRO_SETUP,
    current_test_id,
    docker_exec,
    get_metrics,
    get_robot_containers,
    logger,
    repo_path,
    ros2_exec,
)
from harness.session import run_dir
from system.test_liveliness import _check_sentinel_nodes, _poll_until

import wiring_snapshot as ws

pytestmark = pytest.mark.wiring

# Topics per batched `docker exec` (each probe backgrounded into its own /tmp
# file — the parallel_sample_hz idiom).
_TOPICS_PER_EXEC = 40
# Wall seconds each backgrounded `ros2 topic info --verbose` gets.
_INFO_TIMEOUT_S = 20
# Node-set settle poll: snapshot only once two consecutive samples agree.
_SETTLE_TIMEOUT_S = 120
_SETTLE_INTERVAL_S = 10


def _node_list(container, domain_id, setup_bash):
    result = ros2_exec(
        container,
        "ros2 node list 2>/dev/null",
        domain_id=domain_id,
        setup_bash=setup_bash,
        timeout=30,
    )
    return ws.parse_node_list(result.stdout)


def _topic_list(container, domain_id, setup_bash):
    result = ros2_exec(
        container,
        "ros2 topic list 2>/dev/null",
        domain_id=domain_id,
        setup_bash=setup_bash,
        timeout=30,
    )
    return sorted({
        line.strip() for line in result.stdout.splitlines()
        if line.strip().startswith("/")
    })


def _batched_topic_info(container, topics, domain_id, setup_bash):
    """``ros2 topic info --verbose`` for every topic, batched per docker exec.

    One ``docker exec`` per chunk of ``_TOPICS_PER_EXEC`` topics: each probe is
    backgrounded into its own /tmp file, then the files are catted back with
    ``===FILE ...===`` sentinels (same idiom as ``parallel_sample_hz``).
    Returns ``{topic: verbose_output_text}``.
    """
    outputs = {}
    for start in range(0, len(topics), _TOPICS_PER_EXEC):
        chunk = topics[start:start + _TOPICS_PER_EXEC]
        temp_files = {}
        probes = []
        for i, topic in enumerate(chunk, start=start):
            fname = f"/tmp/wiring_{i}.out"
            temp_files[topic] = fname
            probes.append(
                f"(ROS_DOMAIN_ID={domain_id} timeout {_INFO_TIMEOUT_S} "
                f"ros2 topic info --verbose {topic} > {fname} 2>&1) &"
            )
        # Newlines, not `&& ... &`: bash precedence makes `A && B && C & D &`
        # only apply the && chain to C (see parallel_sample_hz).
        lines = [f"source {ROS_DISTRO_SETUP}", f"source {setup_bash}"]
        lines += probes + ["wait"]
        for fname in temp_files.values():
            lines.append(f"echo '===FILE {fname}==='")
            lines.append(f"cat {fname} 2>/dev/null || true")
        result = docker_exec(container, "\n".join(lines),
                             timeout=_INFO_TIMEOUT_S + 60)
        for piece in result.stdout.split("===FILE ")[1:]:
            header, _, content = piece.partition("===")
            fname = header.strip()
            topic = next((t for t, f in temp_files.items() if f == fname), None)
            if topic is not None:
                outputs[topic] = content
    return outputs


def _wait_for_settled_node_sets(env):
    """Poll per-robot node sets until two consecutive samples agree.

    Sentinel presence proves the core stack is up, but late modules can still
    be joining the graph; snapshotting mid-bring-up would produce flaky drift.
    Proceeds (with a warning) if the graph is still changing after the budget.
    """
    cfg = env["cfg"]
    containers = get_robot_containers(env["robot_pattern"])
    previous = None
    deadline = time.time() + _SETTLE_TIMEOUT_S
    while time.time() < deadline:
        sample = tuple(
            tuple(_node_list(containers[n - 1], n, cfg["robot_setup_bash"]))
            for n in range(1, env["num_robots"] + 1)
        )
        if sample == previous:
            logger.info("Node graph settled (%d nodes total)",
                        sum(len(s) for s in sample))
            return
        previous = sample
        time.sleep(_SETTLE_INTERVAL_S)
    logger.warning("Node graph still changing after %ds; snapshotting anyway",
                   _SETTLE_TIMEOUT_S)


def _capture_merged_graph(env):
    """Snapshot every robot's ROS graph and merge into one normalized graph.

    Robot container index n-1 hosts ``robot_n`` on ROS_DOMAIN_ID n; node and
    edge names carry their robot namespaces as-is, so the merge is a plain
    union (first-seen wins for a topic observed on multiple domains, e.g.
    ``/clock``).
    """
    cfg = env["cfg"]
    containers = get_robot_containers(env["robot_pattern"])
    assert len(containers) >= env["num_robots"], (
        f"only {len(containers)}/{env['num_robots']} robot containers visible"
    )
    nodes = set()
    topics = {}
    edges = []
    for n in range(1, env["num_robots"] + 1):
        container = containers[n - 1]
        setup_bash = cfg["robot_setup_bash"]
        robot_nodes = _node_list(container, n, setup_bash)
        robot_topics = _topic_list(container, n, setup_bash)
        logger.info("robot_%d graph: %d nodes, %d topics (%s, domain %d)",
                    n, len(robot_nodes), len(robot_topics), container, n)
        nodes.update(robot_nodes)
        infos = _batched_topic_info(container, robot_topics, n, setup_bash)
        for topic in robot_topics:
            text = infos.get(topic)
            if not text:
                logger.warning("no `topic info --verbose` output for %s "
                               "(robot_%d)", topic, n)
                continue
            entry, topic_edges = ws.parse_topic_info_verbose(topic, text)
            topics.setdefault(topic, entry)
            edges.extend(topic_edges)
    graph = {"version": 1, "nodes": sorted(nodes), "topics": topics,
             "edges": edges}
    return ws.normalize_graph(graph)


def _git_short_sha():
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=AIRSTACK_ROOT, capture_output=True, text=True, timeout=10,
        )
        if result.stdout.strip():
            return result.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        pass
    # The tests container has no git binary; read .git/HEAD directly (repo is
    # mounted read-only, which is fine for reads).
    try:
        git_dir = os.path.join(AIRSTACK_ROOT, ".git")
        head = open(os.path.join(git_dir, "HEAD")).read().strip()
        if head.startswith("ref:"):
            ref = head.split(None, 1)[1]
            ref_path = os.path.join(git_dir, ref)
            if os.path.exists(ref_path):
                return open(ref_path).read().strip()[:12]
            packed = os.path.join(git_dir, "packed-refs")
            if os.path.exists(packed):
                for line in open(packed):
                    if line.strip().endswith(" " + ref):
                        return line.split()[0][:12]
        elif head:
            return head[:12]
    except OSError:
        pass
    return "unknown"


@pytest.mark.timeout(1800)
class TestWiringSnapshot:

    @pytest.mark.dependency(name="wiring_nodes")
    def test_sentinel_nodes_present(self, airstack_env):
        """Wait up to 300s for the expected sentinel nodes per robot."""
        last_msg = [""]

        def ready():
            ok, msg = _check_sentinel_nodes(airstack_env)
            last_msg[0] = msg
            return ok

        _poll_until(
            ready,
            timeout=300,
            interval=5,
            fail_msg=lambda: f"sentinel nodes not ready after 300s: {last_msg[0]}",
        )

    @pytest.mark.dependency(depends=["wiring_nodes"])
    def test_wiring_matches_golden(self, airstack_env):
        """Snapshot the running graph, render wiring.md, diff against golden."""
        sim = airstack_env["sim"]
        num_robots = airstack_env["num_robots"]
        m = get_metrics()
        tid = current_test_id()

        # Legacy (no --stack) keeps the full_default naming and the
        # tests/goldens/wiring golden; a selected stack owns its golden at
        # stacks/<name>/wiring.md.
        stack = airstack_env.get("stack")
        stack_name = stack or "full_default"

        _wait_for_settled_node_sets(airstack_env)
        t0 = time.time()
        graph = _capture_merged_graph(airstack_env)
        m.record(tid, "wiring_capture_duration_s",
                 round(time.time() - t0, 2), unit="s")

        meta = {
            "stack": stack_name,
            "generated-by": "tests/system/test_wiring_snapshot.py",
            "date": time.strftime("%Y-%m-%d %H:%M:%S"),
            "sim": sim,
            "num_robots": num_robots,
            "source-sha": _git_short_sha(),
        }
        out_dir = run_dir() / "wiring"
        out_dir.mkdir(parents=True, exist_ok=True)
        observed_path = out_dir / f"observed_{stack_name}.md"
        observed_path.write_text(ws.render_wiring_md(graph, meta))
        logger.info("Wrote observed wiring snapshot to %s", observed_path)

        # Counts are drift telemetry: a silently vanishing node/edge should
        # read as a regression, hence higher_is_better.
        m.record(tid, "wiring_node_count", len(graph["nodes"]),
                 unit="count", direction="higher_is_better")
        m.record(tid, "wiring_edge_count", len(graph["edges"]),
                 unit="count", direction="higher_is_better")
        m.record(tid, "wiring_topic_count", len(graph["topics"]),
                 unit="count", direction="higher_is_better")

        if stack:
            golden_path = repo_path("stacks", stack, "wiring.md")
        else:
            golden_path = repo_path(
                "tests", "goldens", "wiring",
                f"full_default.{sim}.{num_robots}robot.md",
            )
        if not golden_path.exists():
            logger.info(
                "INSTRUCTION: no golden at %s — bootstrap by validating the "
                "observed snapshot (%s) and copying it to that path, then "
                "commit it (see tests/goldens/wiring/README.md).",
                golden_path, observed_path,
            )
            return

        expected = ws.extract_graph_from_md(golden_path.read_text())
        verdict = ws.diff_graphs(expected, graph)
        if not verdict["identical"]:
            pytest.fail(
                f"observed wiring drifted from golden {golden_path.name} — "
                "if the change is intentional, regenerate the golden from "
                f"{observed_path} and commit it:\n"
                + json.dumps(verdict, indent=2)
            )

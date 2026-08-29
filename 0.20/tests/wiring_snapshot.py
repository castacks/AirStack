#!/usr/bin/env python3
"""Standalone observed-wiring snapshot renderer and drift differ.

Turns a snapshot of a *running* ROS 2 graph (``ros2 node list`` plus
``ros2 topic info --verbose`` per topic) into a committed ``wiring.md`` — a
mermaid dataflow diagram with a machine-readable JSON trailer — and diffs two
such snapshots for drift (RFC #379 §4.4: the wiring picture is observed, never
generated from configuration, so it cannot lie or rot). Used by
``tests/system/test_wiring_snapshot.py``, but deliberately dependency-free
(stdlib only) and runnable outside the AirStack harness: the input is plain
``ros2`` CLI output, so the same tool can snapshot any ROS 2 system::

    python3 wiring_snapshot.py render --graph-json graph.json --out wiring.md \
        --meta sim=isaacsim --meta num_robots=1
    python3 wiring_snapshot.py diff --expected wiring.md --observed observed.md

Exit code 0 iff the two graphs are identical after normalization; a JSON
verdict is printed to stdout either way.

Graph data model (``version`` 1)::

    {"version": 1,
     "nodes": ["/robot_1/...", ...],
     "topics": {"/robot_1/odometry": {"type": "...", "qos": {...}}, ...},
     "edges": [{"node": ..., "topic": ..., "dir": "pub"|"sub",
                "type": ..., "qos_profile": {...}}, ...]}

QoS is normalized to ``{reliability, durability, history, depth}`` strings;
GIDs and type hashes are dropped (non-deterministic across bring-ups).
"""

import argparse
import json
import os
import re
import sys

# Topics that exist on every ROS 2 system and carry no wiring information.
# /tf and /tf_static are deliberately NOT excluded — frame plumbing is wiring.
DEFAULT_EXCLUDES = ("/parameter_events", "/rosout")

# Node-name (final segment) prefixes that are per-process/per-pid artifacts of
# the launch system or CLI tooling, not stack wiring. transform_listener_impl_*
# and launch_ros_* carry hex/pid suffixes; _ros2cli* nodes are the probes
# themselves (ros2 topic info/echo spin one up); _CREATED_BY_BARE_DDS_APP_ is
# the placeholder name rmw reports for non-rclcpp DDS participants (sim
# bridges, uXRCE agents) — real endpoints, but not stack nodes.
_EXCLUDED_NODE_PREFIXES = (
    "launch_ros_",
    "transform_listener_impl",
    "_ros2cli",
    "_CREATED_BY_BARE_DDS_APP_",
)

# Substrings marking sim render-pipeline internals whose visibility on the
# robot domain is timing-dependent (they join the graph once Isaac's SDG
# pipeline spins up — sometimes before the snapshot, sometimes after). They
# are prim-path-derived names, not stack wiring; RFC #380 §1 later normalizes
# sim sensor endpoints to vehicle-manifest sensor ids properly.
_EXCLUDED_NODE_SUBSTRINGS = (
    "_Render_PostProcess_SDGPipeline",
    "_PX4MultirotorGraph_",
)

_QOS_UNKNOWN = "UNKNOWN"

_HISTORY_RE = re.compile(r"History\s*\(Depth\):\s*([A-Za-z_]+)(?:\s*\((\d+)\))?")


def _is_excluded_node(name):
    """True for launch/CLI helper and sim render-pipeline nodes."""
    segment = name.rsplit("/", 1)[-1]
    if segment.startswith(_EXCLUDED_NODE_PREFIXES):
        return True
    return any(s in name for s in _EXCLUDED_NODE_SUBSTRINGS)


def parse_node_list(text):
    """Parse ``ros2 node list`` output into a sorted list of node names."""
    return sorted({
        line.strip() for line in (text or "").splitlines()
        if line.strip().startswith("/")
    })


def _empty_qos():
    return {"reliability": _QOS_UNKNOWN, "durability": _QOS_UNKNOWN,
            "history": _QOS_UNKNOWN, "depth": _QOS_UNKNOWN}


def _full_node_name(namespace, name):
    ns = (namespace or "/").rstrip("/")
    return f"{ns}/{name}"


def parse_topic_info_verbose(topic_name, text):
    """Parse ``ros2 topic info --verbose`` output for one topic.

    The output (ROS 2 Jazzy) is a sequence of endpoint blocks, each opened by
    a ``Node name:`` line and carrying ``Node namespace:``, ``Topic type:``,
    ``Endpoint type: PUBLISHER|SUBSCRIPTION``, a ``GID:`` and an indented
    ``QoS profile:`` block. Unknown lines are ignored (defensive parsing);
    GIDs and type hashes are dropped as non-deterministic.

    Returns ``(topic_entry, edges)`` where ``topic_entry`` is the ``topics``
    dict value ``{"type": ..., "qos": {...}}`` (QoS taken from the first
    publisher, else the first endpoint) and ``edges`` is a list of edge dicts.
    """
    topic_type = None
    endpoints = []
    current = None
    for raw in (text or "").splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("Type:") and topic_type is None:
            topic_type = line.split(":", 1)[1].strip()
        elif line.startswith("Node name:"):
            current = {"name": line.split(":", 1)[1].strip(), "namespace": "/",
                       "type": None, "endpoint": None, "qos": _empty_qos()}
            endpoints.append(current)
        elif current is None:
            continue
        elif line.startswith("Node namespace:"):
            current["namespace"] = line.split(":", 1)[1].strip() or "/"
        elif line.startswith("Topic type:"):
            current["type"] = line.split(":", 1)[1].strip()
        elif line.startswith("Endpoint type:"):
            current["endpoint"] = line.split(":", 1)[1].strip().upper()
        elif line.startswith("Reliability:"):
            current["qos"]["reliability"] = line.split(":", 1)[1].strip()
        elif line.startswith("Durability:"):
            current["qos"]["durability"] = line.split(":", 1)[1].strip()
        elif line.startswith("History"):
            m = _HISTORY_RE.match(line)
            if m:
                current["qos"]["history"] = m.group(1)
                current["qos"]["depth"] = m.group(2) or _QOS_UNKNOWN

    dir_map = {"PUBLISHER": "pub", "SUBSCRIPTION": "sub"}
    edges = []
    first_pub_qos = None
    first_qos = None
    for ep in endpoints:
        direction = dir_map.get(ep["endpoint"])
        if direction is None:
            continue
        if first_qos is None:
            first_qos = ep["qos"]
        if direction == "pub" and first_pub_qos is None:
            first_pub_qos = ep["qos"]
        edges.append({
            "node": _full_node_name(ep["namespace"], ep["name"]),
            "topic": topic_name,
            "dir": direction,
            "type": ep["type"] or topic_type,
            "qos_profile": dict(ep["qos"]),
        })
    topic_entry = {
        "type": topic_type,
        "qos": dict(first_pub_qos or first_qos or _empty_qos()),
    }
    return topic_entry, edges


def _edge_sort_key(edge):
    return (edge.get("topic") or "", edge.get("dir") or "", edge.get("node") or "")


def normalize_graph(graph, exclude_topics=DEFAULT_EXCLUDES):
    """Return a normalized copy of ``graph``: infra topics and per-pid helper
    nodes removed, edges deduped, everything deterministically sorted.

    Idempotent — ``diff_graphs`` normalizes both sides, so re-normalizing an
    already-normalized graph must be a no-op.
    """
    excluded = set(exclude_topics)
    topics = {
        name: {"type": entry.get("type"), "qos": dict(entry.get("qos") or {})}
        for name, entry in (graph.get("topics") or {}).items()
        if name not in excluded
    }
    seen = set()
    edges = []
    for edge in sorted(graph.get("edges") or [], key=_edge_sort_key):
        node = edge.get("node") or ""
        topic = edge.get("topic") or ""
        if topic in excluded or _is_excluded_node(node):
            continue
        key = (node, topic, edge.get("dir"))
        if key in seen:
            continue
        seen.add(key)
        edges.append({
            "node": node,
            "topic": topic,
            "dir": edge.get("dir"),
            "type": edge.get("type"),
            "qos_profile": dict(edge.get("qos_profile") or {}),
        })
    nodes = {n for n in (graph.get("nodes") or []) if not _is_excluded_node(n)}
    nodes |= {e["node"] for e in edges}
    return {
        "version": graph.get("version", 1),
        "nodes": sorted(nodes),
        "topics": topics,
        "edges": edges,
    }


# ── mermaid rendering ──────────────────────────────────────────────────────

def _esc(label):
    """Escape mermaid-special characters inside a quoted label."""
    return str(label).replace('"', "#quot;")


def _short_type(type_name):
    """``nav_msgs/msg/Odometry`` → ``Odometry``."""
    return (type_name or "?").rsplit("/", 1)[-1]


def _default_group(node):
    """Subgraph for a node: the namespace segment after the robot namespace
    (``/robot_1/perception/foo`` → ``perception``), the sole namespace segment
    for shallow nodes (``/robot_1/foo`` → ``robot_1``), else ``root``."""
    segments = [s for s in node.split("/") if s]
    if len(segments) >= 3:
        return segments[1]
    if len(segments) == 2:
        return segments[0]
    return "root"


def render_mermaid(graph, group_fn=None):
    """Render the graph as deterministic mermaid ``graph LR`` text.

    Nodes are grouped into subgraphs by ``group_fn`` (default: namespace
    segment after the robot namespace). Topics appear as edge labels
    (``topic<br/>ShortType``) with one edge per pub×sub pair; a topic with
    publishers but no subscribers (or vice versa) gets a dangling stadium
    annotation node so the loose end is visible.
    """
    group_fn = group_fn or _default_group
    nodes = sorted(set(graph.get("nodes") or [])
                   | {e["node"] for e in (graph.get("edges") or [])})
    ids = {n: f"n{i}" for i, n in enumerate(nodes)}
    groups = {}
    for n in nodes:
        groups.setdefault(group_fn(n), []).append(n)

    lines = ["graph LR"]
    for gi, gname in enumerate(sorted(groups)):
        lines.append(f'  subgraph g{gi}["{_esc(gname)}"]')
        for n in groups[gname]:
            lines.append(f'    {ids[n]}["{_esc(n)}"]')
        lines.append("  end")

    by_topic = {}
    for e in graph.get("edges") or []:
        info = by_topic.setdefault(e["topic"], {"pub": set(), "sub": set(),
                                                "type": e.get("type")})
        info[e["dir"]].add(e["node"])

    dangling = 0
    for topic in sorted(by_topic):
        info = by_topic[topic]
        label = _esc(f"{topic}<br/>{_short_type(info['type'])}")
        pubs = sorted(info["pub"])
        subs = sorted(info["sub"])
        if pubs and subs:
            for pub in pubs:
                for sub in subs:
                    lines.append(f'  {ids[pub]} -->|"{label}"| {ids[sub]}')
        elif pubs:
            annotation = f'd{dangling}(["{_esc(topic)} (no subscribers)"])'
            dangling += 1
            for pub in pubs:
                lines.append(f'  {ids[pub]} -->|"{label}"| {annotation}')
                annotation = f"d{dangling - 1}"  # define once, reference after
        elif subs:
            annotation = f'd{dangling}(["{_esc(topic)} (no publishers)"])'
            dangling += 1
            for sub in subs:
                lines.append(f'  {annotation} -->|"{label}"| {ids[sub]}')
                annotation = f"d{dangling - 1}"
    return "\n".join(lines) + "\n"


# ── wiring.md rendering / parsing ──────────────────────────────────────────

_TRAILER_OPEN = "<!-- wiring-graph-v1"
_TRAILER_CLOSE = "wiring-graph-v1 -->"
_TRAILER_RE = re.compile(
    re.escape(_TRAILER_OPEN) + r"\n(.*?)\n" + re.escape(_TRAILER_CLOSE),
    re.DOTALL,
)

# Provenance keys rendered first, in this order; extra meta keys follow sorted.
_META_ORDER = ("generated-by", "date", "sim", "num_robots", "source-sha")


def render_wiring_md(graph, meta):
    """Render the full ``wiring.md`` text: title, provenance from ``meta``,
    the mermaid fence, and a machine-readable canonical-JSON trailer that
    ``extract_graph_from_md`` round-trips exactly."""
    lines = [f"# Wiring snapshot: {meta.get('stack', 'full_default')}", ""]
    extra = sorted(set(meta) - set(_META_ORDER) - {"stack"})
    for key in list(_META_ORDER) + extra:
        if key in meta:
            lines.append(f"- **{key}**: {meta[key]}")
    lines += ["", "```mermaid", render_mermaid(graph).rstrip("\n"), "```", ""]
    # Compact canonical JSON (one line): keeps future baselines ~1k lines
    # instead of ~7k. Drift compares parsed graphs, not bytes, so older
    # indent=0 baselines stay valid until re-blessed.
    canonical = json.dumps(graph, sort_keys=True, separators=(",", ":"))
    lines += [_TRAILER_OPEN, canonical, _TRAILER_CLOSE, ""]
    return "\n".join(lines)


def extract_graph_from_md(text):
    """Parse the graph JSON back out of a ``wiring.md`` trailer."""
    m = _TRAILER_RE.search(text or "")
    if not m:
        raise ValueError(
            f"no `{_TRAILER_OPEN} ... {_TRAILER_CLOSE}` trailer found"
        )
    return json.loads(m.group(1))


# ── drift diffing ──────────────────────────────────────────────────────────

def _edge_key(edge):
    return f'{edge["dir"]}:{edge["node"]}:{edge["topic"]}'


def diff_graphs(expected, observed):
    """Compare two graphs in normalized form; return a drift verdict dict.

    ``verdict["identical"]`` is the pass/fail judgment; the remaining keys
    name exactly what drifted (nodes, edges, per-edge QoS, per-topic types).
    """
    exp = normalize_graph(expected)
    obs = normalize_graph(observed)

    exp_nodes, obs_nodes = set(exp["nodes"]), set(obs["nodes"])
    exp_edges = {_edge_key(e): e for e in exp["edges"]}
    obs_edges = {_edge_key(e): e for e in obs["edges"]}

    qos_mismatches = []
    for key in sorted(set(exp_edges) & set(obs_edges)):
        if exp_edges[key]["qos_profile"] != obs_edges[key]["qos_profile"]:
            qos_mismatches.append({
                "edge": key,
                "expected": exp_edges[key]["qos_profile"],
                "observed": obs_edges[key]["qos_profile"],
            })

    type_mismatches = []
    for topic in sorted(set(exp["topics"]) & set(obs["topics"])):
        exp_type = exp["topics"][topic].get("type")
        obs_type = obs["topics"][topic].get("type")
        if exp_type != obs_type:
            type_mismatches.append({
                "topic": topic, "expected": exp_type, "observed": obs_type,
            })

    verdict = {
        "identical": False,
        "missing_nodes": sorted(exp_nodes - obs_nodes),
        "extra_nodes": sorted(obs_nodes - exp_nodes),
        "missing_edges": sorted(set(exp_edges) - set(obs_edges)),
        "extra_edges": sorted(set(obs_edges) - set(exp_edges)),
        "qos_mismatches": qos_mismatches,
        "type_mismatches": type_mismatches,
    }
    verdict["identical"] = not any(
        verdict[k] for k in verdict if k != "identical"
    )
    return verdict


# ── CLI ────────────────────────────────────────────────────────────────────

def _load_graph(path):
    """Load a graph from a ``wiring.md`` (trailer) or a raw ``graph.json``."""
    with open(path) as fh:
        text = fh.read()
    if _TRAILER_OPEN in text:
        return extract_graph_from_md(text)
    return json.loads(text)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)

    render = sub.add_parser("render", help="Render a graph JSON to wiring.md")
    render.add_argument("--graph-json", required=True,
                        help="Path to the graph JSON (data model above)")
    render.add_argument("--out", required=True, help="wiring.md output path")
    render.add_argument("--meta", action="append", default=[], metavar="K=V",
                        help="Provenance entries, repeatable (e.g. sim=isaacsim)")

    diff = sub.add_parser("diff", help="Diff two snapshots; exit 0 iff identical")
    diff.add_argument("--expected", required=True,
                      help="Committed wiring.md (or graph.json)")
    diff.add_argument("--observed", required=True,
                      help="Observed wiring.md or graph.json")

    args = ap.parse_args(argv)

    if args.cmd == "render":
        graph = _load_graph(args.graph_json)
        meta = {}
        for kv in args.meta:
            key, _, value = kv.partition("=")
            meta[key] = value
        out_dir = os.path.dirname(os.path.abspath(args.out))
        os.makedirs(out_dir, exist_ok=True)
        with open(args.out, "w") as fh:
            fh.write(render_wiring_md(graph, meta))
        return 0

    verdict = diff_graphs(_load_graph(args.expected), _load_graph(args.observed))
    print(json.dumps(verdict, indent=2))
    return 0 if verdict["identical"] else 1


if __name__ == "__main__":
    sys.exit(main())

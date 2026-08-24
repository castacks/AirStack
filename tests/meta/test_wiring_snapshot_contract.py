# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Contract tests for the wiring-snapshot tool (RFC #379 §4.4).

``tests/wiring_snapshot.py`` turns ``ros2 topic info --verbose`` output into a
committed ``wiring.md`` and diffs snapshots for drift. These tests pin the
contracts the system test and the golden files depend on: the Jazzy verbose
parser, infra-noise normalization, deterministic mermaid rendering, the
render→extract round trip, drift verdicts, and the CLI exit codes.
"""
import json

import pytest

import wiring_snapshot as ws  # noqa: E402 — pytest adds tests/ to sys.path

pytestmark = pytest.mark.unit


# Realistic ROS 2 Jazzy `ros2 topic info --verbose` output: one publisher and
# one subscription, each with a full QoS block; GIDs and type hashes present
# so the parser is proven to drop them.
ODOM_INFO = """\
Type: nav_msgs/msg/Odometry

Publisher count: 1

Node name: mavros
Node namespace: /robot_1/interface/mavros
Topic type: nav_msgs/msg/Odometry
Topic type hash: RIHS01_9f3c1cb1c9e3d2b6c4a1f0e9d8c7b6a5f4e3d2c1b0a9f8e7d6c5b4a3f2e1d0
Endpoint type: PUBLISHER
GID: 01.0f.cc.d1.62.5c.9e.ac.01.00.00.00.00.00.15.03
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (10)
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 1

Node name: trajectory_control_node
Node namespace: /robot_1/trajectory_controller
Topic type: nav_msgs/msg/Odometry
Topic type hash: RIHS01_9f3c1cb1c9e3d2b6c4a1f0e9d8c7b6a5f4e3d2c1b0a9f8e7d6c5b4a3f2e1d0
Endpoint type: SUBSCRIPTION
GID: 01.0f.cc.d1.62.5c.9e.ac.01.00.00.00.00.00.16.04
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite
"""

TF_INFO = """\
Type: tf2_msgs/msg/TFMessage

Publisher count: 1

Node name: robot_state_publisher
Node namespace: /robot_1
Topic type: tf2_msgs/msg/TFMessage
Endpoint type: PUBLISHER
GID: 01.0f.cc.d1.62.5c.9e.ac.01.00.00.00.00.00.17.03
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (100)
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 0
"""

ROSOUT_INFO = """\
Type: rcl_interfaces/msg/Log

Publisher count: 1

Node name: mavros
Node namespace: /robot_1/interface/mavros
Topic type: rcl_interfaces/msg/Log
Endpoint type: PUBLISHER
GID: 01.0f.cc.d1.62.5c.9e.ac.01.00.00.00.00.00.18.03
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (1000)
  Durability: TRANSIENT_LOCAL
  Lifespan: 10000000000 nanoseconds
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 0
"""


def _sample_graph():
    """Assemble a graph the way the system test does, then normalize it."""
    topics = {}
    edges = []
    for topic, text in (
        ("/robot_1/odometry", ODOM_INFO),
        ("/tf", TF_INFO),
        ("/rosout", ROSOUT_INFO),
    ):
        entry, topic_edges = ws.parse_topic_info_verbose(topic, text)
        topics[topic] = entry
        edges.extend(topic_edges)
    nodes = [
        "/launch_ros_12345",
        "/robot_1/interface/mavros/mavros",
        "/robot_1/robot_state_publisher",
        "/robot_1/trajectory_controller/trajectory_control_node",
        "/robot_1/transform_listener_impl_55d0a1b2c3d4",
    ]
    return ws.normalize_graph(
        {"version": 1, "nodes": nodes, "topics": topics, "edges": edges}
    )


def test_parser_extracts_endpoints_types_and_qos():
    entry, edges = ws.parse_topic_info_verbose("/robot_1/odometry", ODOM_INFO)

    assert entry["type"] == "nav_msgs/msg/Odometry"
    # Topic-level QoS comes from the first publisher.
    assert entry["qos"] == {"reliability": "RELIABLE", "durability": "VOLATILE",
                            "history": "KEEP_LAST", "depth": "10"}

    assert len(edges) == 2
    pub, sub = edges
    assert pub["dir"] == "pub"
    assert pub["node"] == "/robot_1/interface/mavros/mavros"
    assert pub["type"] == "nav_msgs/msg/Odometry"
    assert sub["dir"] == "sub"
    assert sub["node"] == "/robot_1/trajectory_controller/trajectory_control_node"
    assert sub["qos_profile"] == {"reliability": "BEST_EFFORT",
                                  "durability": "VOLATILE",
                                  "history": "UNKNOWN", "depth": "UNKNOWN"}
    # GIDs are non-deterministic and must not leak into the model.
    assert "GID" not in json.dumps(edges)


def test_normalize_excludes_infra_but_keeps_tf():
    graph = _sample_graph()

    assert "/rosout" not in graph["topics"]
    assert not any(e["topic"] == "/rosout" for e in graph["edges"])
    # /tf and /tf_static are wiring — they stay.
    assert "/tf" in graph["topics"]
    # launch/CLI helper nodes (pid/hex-suffixed) are excluded.
    assert "/launch_ros_12345" not in graph["nodes"]
    assert "/robot_1/transform_listener_impl_55d0a1b2c3d4" not in graph["nodes"]
    assert "/robot_1/robot_state_publisher" in graph["nodes"]
    # Normalization is idempotent (diff_graphs re-normalizes both sides).
    assert ws.normalize_graph(graph) == graph


def test_mermaid_render_is_deterministic_and_grouped():
    graph = _sample_graph()
    first = ws.render_mermaid(graph)
    second = ws.render_mermaid(graph)

    assert first == second
    assert first.startswith("graph LR")
    # Nodes group by the namespace segment after the robot namespace.
    assert 'subgraph g0["interface"]' in first
    # pub×sub edge with topic + shortened type as the label.
    assert '-->|"/robot_1/odometry<br/>Odometry"|' in first
    # /tf has a publisher but no subscriber → dangling annotation node.
    assert "(no subscribers)" in first


def test_render_extract_round_trips_graph_exactly():
    graph = _sample_graph()
    meta = {"stack": "full_default", "generated-by": "contract-test",
            "date": "2026-08-20 00:00:00", "sim": "isaacsim",
            "num_robots": 1, "source-sha": "deadbee"}
    text = ws.render_wiring_md(graph, meta)

    assert text.startswith("# Wiring snapshot: full_default")
    assert "- **sim**: isaacsim" in text
    assert "```mermaid" in text
    assert ws.extract_graph_from_md(text) == graph


def test_diff_identical_graphs():
    graph = _sample_graph()
    verdict = ws.diff_graphs(graph, graph)

    assert verdict["identical"] is True
    assert verdict["missing_edges"] == []
    assert verdict["extra_edges"] == []
    assert verdict["qos_mismatches"] == []
    assert verdict["type_mismatches"] == []


def test_diff_reports_removed_edge_as_missing():
    expected = _sample_graph()
    observed = json.loads(json.dumps(expected))
    removed = next(e for e in observed["edges"] if e["dir"] == "sub")
    observed["edges"].remove(removed)

    verdict = ws.diff_graphs(expected, observed)

    assert verdict["identical"] is False
    assert verdict["missing_edges"] == [
        f"sub:{removed['node']}:{removed['topic']}"
    ]
    assert verdict["extra_edges"] == []


def test_main_diff_exit_codes(tmp_path, capsys):
    graph = _sample_graph()
    expected_md = tmp_path / "wiring.md"
    expected_md.write_text(ws.render_wiring_md(graph, {"stack": "full_default"}))

    same = tmp_path / "observed_same.json"
    same.write_text(json.dumps(graph))
    rc = ws.main(["diff", "--expected", str(expected_md),
                  "--observed", str(same)])
    verdict = json.loads(capsys.readouterr().out)
    assert rc == 0
    assert verdict["identical"] is True

    drifted_graph = json.loads(json.dumps(graph))
    drifted_graph["edges"] = drifted_graph["edges"][1:]
    drifted = tmp_path / "observed_drifted.json"
    drifted.write_text(json.dumps(drifted_graph))
    rc = ws.main(["diff", "--expected", str(expected_md),
                  "--observed", str(drifted)])
    verdict = json.loads(capsys.readouterr().out)
    assert rc == 1
    assert verdict["identical"] is False
    assert verdict["missing_edges"]

"""End-to-end RavenNavNode round-trip over real DDS.

Needs rclpy + the AirStack message packages, so it SKIPS on the host and runs
inside the robot container (WP-C's harness):

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    python3 -m pytest src/global/planners/raven_nav/test/integration -q

Everything the node consumes is published by a fake peer node on the same
context, and everything it publishes is captured — so this covers the parts
the pure tests cannot: parameter declaration against the real spawn argv, the
PointCloud2 field layouts, QoS matching, the frame lift, and the results file.
"""
import json
import os
import pathlib
import shutil
import tempfile
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy', reason='integration test needs ROS 2')

from geometry_msgs.msg import Point32, PolygonStamped              # noqa: E402
from nav_msgs.msg import Odometry, Path                            # noqa: E402
from rclpy.executors import SingleThreadedExecutor                 # noqa: E402
from rclpy.node import Node                                        # noqa: E402
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy)
from sensor_msgs.msg import NavSatFix, PointCloud2, PointField     # noqa: E402
from sensor_msgs_py import point_cloud2                            # noqa: E402
from std_msgs.msg import Header, String                            # noqa: E402
from visualization_msgs.msg import MarkerArray                     # noqa: E402

from raven_nav.raven_nav_node import RavenNavNode                  # noqa: E402

ROBOT = 'robot_1'
DOMAIN = os.getenv('ROS_DOMAIN_ID', '0')
RF = f'/robot_{DOMAIN}/rayfronts/msg_serv'
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
SENSOR = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST, depth=10)

QUERIES = ['person', 'sky', 'road']


def _cloud(fields, rows, frame='map'):
    hdr = Header()
    hdr.frame_id = frame
    pf = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
          for i, n in enumerate(fields)]
    return point_cloud2.create_cloud(hdr, pf, [list(map(float, r)) for r in rows])


def _flu_to_rdf(p):
    p = np.asarray(p, dtype=float).reshape(-1, 3)
    return np.stack([-p[:, 1], -p[:, 2], p[:, 0]], axis=1)


class Peer(Node):
    """Publishes raven's inputs, captures raven's outputs."""

    def __init__(self, context):
        super().__init__('raven_test_peer', context=context,
                         use_global_arguments=False)
        self.got = {}
        self.vox_pub = self.create_publisher(PointCloud2, f'{RF}/voxels_sim/all', 10)
        self.ray_pub = self.create_publisher(PointCloud2, f'{RF}/rays_sim/all', 10)
        self.fro_pub = self.create_publisher(PointCloud2, f'{RF}/frontiers', 10)
        self.odom_pub = self.create_publisher(Odometry, f'/{ROBOT}/odometry', 10)
        self.fix_pub = self.create_publisher(
            NavSatFix, f'/{ROBOT}/interface/mavros/global_position/raw/fix', SENSOR)
        self.area_pub = self.create_publisher(
            PolygonStamped, f'/{ROBOT}/raven_nav/search_area', LATCHED)
        self.lvlm_pub = self.create_publisher(
            String, f'/{ROBOT}/raven_nav/lvlm_output', 10)
        for topic, typ in (
                (f'/{ROBOT}/global_plan', Path),
                (f'/{ROBOT}/navigation_mode', String),
                (f'/{ROBOT}/completed_targets', String),
                (f'/{ROBOT}/raven_nav/discoveries', String),
                (f'/{ROBOT}/raven_nav/confirmed_targets', String),
                (f'/{ROBOT}/raven_nav/guiding_objects', String),
                (f'/{ROBOT}/filtered_rays', MarkerArray),
                (f'/{ROBOT}/voxel_clusters', MarkerArray),
                (f'{RF}/new_text_query', String)):
            self.create_subscription(
                typ, topic, (lambda m, t=topic: self.got.setdefault(t, []).append(m)),
                10)

    def odom(self, xyz):
        m = Odometry()
        m.header.frame_id = 'map'
        m.pose.pose.position.x = float(xyz[0])
        m.pose.pose.position.y = float(xyz[1])
        m.pose.pose.position.z = float(xyz[2])
        m.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(m)

    def voxels(self, pts_flu, hot_col=0, score=0.99, q=3):
        rows = []
        for p in _flu_to_rdf(pts_flu):
            sims = [0.005] * q
            sims[hot_col] = score
            rows.append(list(p) + sims)
        self.vox_pub.publish(
            _cloud(['x', 'y', 'z'] + [f'sim_{i}' for i in range(q)], rows))

    def rays(self, origins_flu, theta_deg, phi_deg, hot_col=0, score=0.99, q=3):
        rows = []
        for p, th, ph in zip(_flu_to_rdf(origins_flu), theta_deg, phi_deg):
            sims = [0.005] * q
            sims[hot_col] = score
            rows.append(list(p) + [th, ph] + sims)
        self.ray_pub.publish(_cloud(
            ['x', 'y', 'z', 'theta', 'phi'] + [f'sim_{i}' for i in range(q)], rows))

    def frontiers(self, pts_flu):
        self.fro_pub.publish(_cloud(['x', 'y', 'z'], _flu_to_rdf(pts_flu)))

    def fix(self, lat=38.0, lon=-9.0, alt=110.0):
        m = NavSatFix()
        m.status.status = 0
        m.latitude, m.longitude, m.altitude = lat, lon, alt
        self.fix_pub.publish(m)

    def polygon(self, pts_xy):
        m = PolygonStamped()
        m.header.frame_id = 'map'
        for x, y in pts_xy:
            m.polygon.points.append(Point32(x=float(x), y=float(y), z=0.0))
        self.area_pub.publish(m)


@pytest.fixture()
def harness(monkeypatch):
    results = tempfile.mkdtemp(prefix='raven-test-')
    monkeypatch.setenv('ROBOT_NAME', ROBOT)
    monkeypatch.setenv('RAVEN_LVLM', 'false')
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    # Per-node cli_args, NOT rclpy.init(args=...): global arguments belong to
    # the default context, which a private context never sees.
    cli_args = [
        '--ros-args',
        '-p', f"query_labels:=['{QUERIES[0]}','{QUERIES[1]}','{QUERIES[2]}']",
        '-p', "target_labels:=['person']",
        '-p', 'timer_period:=0.05',
        '-p', 'min_altitude_agl:=1.5',
        '-p', 'max_altitude_agl:=30.0',
        '-p', 'voxel_min_cluster_size:=8',
        '-p', 'coverage_cell_size_m:=1.0',
        '-p', 'coverage_complete_threshold:=0.15',
        '-p', 'results_dump_period_s:=0.1',
        '-p', 'lvlm_enabled:=false',
        '-p', f'results_dir:={results}',
        # every inert parameter the spawner passes, to prove they are accepted
        '-p', 'vlfm_baseline:=false', '-p', 'conavgpt_baseline:=false',
        '-p', 'bundle_len:=3', '-p', 'ray_reach_factor:=3.0',
        '-p', 'bb_release_timeout_s:=8.0', '-p', 'debug_auction:=false',
    ]
    node = None
    peer = None
    try:
        node = RavenNavNode(context=ctx, cli_args=cli_args)
        peer = Peer(ctx)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        ex.add_node(peer)

        def spin(seconds=1.0):
            end = time.time() + seconds
            while time.time() < end:
                ex.spin_once(timeout_sec=0.02)

        spin(0.5)   # discovery
        yield node, peer, spin, results
    finally:
        if node is not None:
            node.destroy_node()
        if peer is not None:
            peer.destroy_node()
        rclpy.shutdown(context=ctx)
        shutil.rmtree(results, ignore_errors=True)


def _last(peer, topic):
    msgs = peer.got.get(topic) or []
    return msgs[-1] if msgs else None


def test_node_declares_every_spawn_parameter(harness):
    node, _peer, _spin, _res = harness
    from raven_nav import params as P
    for name in P.PARAM_NAMES:
        assert node.has_parameter(name), name


def test_idle_until_odometry(harness):
    _node, peer, spin, _res = harness
    spin(0.5)
    mode = _last(peer, f'/{ROBOT}/navigation_mode')
    assert mode is not None and mode.data == 'idle'
    assert not peer.got.get(f'/{ROBOT}/global_plan')


def test_frontier_mode_produces_a_global_plan(harness):
    _node, peer, spin, _res = harness
    peer.odom([0.0, 0.0, 6.0])
    blob = np.array([30.0, 0.0, 6.0]) + np.random.default_rng(0).normal(
        scale=0.3, size=(12, 3))
    peer.frontiers(blob)
    spin(1.0)
    plan = _last(peer, f'/{ROBOT}/global_plan')
    assert plan is not None and len(plan.poses) == 2
    assert plan.header.frame_id == 'map'
    assert _last(peer, f'/{ROBOT}/navigation_mode').data == 'frontier'


def test_voxel_mode_and_detection_reporting(harness):
    _node, peer, spin, _res = harness
    peer.odom([0.0, 0.0, 6.0])
    r = np.arange(-1.0, 1.01, 0.5)
    g = np.stack(np.meshgrid(r, r, r, indexing='ij'), axis=-1).reshape(-1, 3)
    peer.voxels(np.array([20.0, 0.0, 3.0])[None, :] + g)
    spin(1.0)
    assert _last(peer, f'/{ROBOT}/navigation_mode').data == 'voxel'
    ds = json.loads(_last(peer, f'/{ROBOT}/raven_nav/discoveries').data)
    assert ds and ds[0]['label'] == 'person'
    assert {'instance_id', 'label', 'cx', 'cy', 'cz', 'status'} <= set(ds[0])
    cts = json.loads(_last(peer, f'/{ROBOT}/raven_nav/confirmed_targets').data)
    assert cts and cts[0]['label'] == 'person'
    boxes = _last(peer, f'/{ROBOT}/voxel_clusters')
    assert boxes is not None and any(m.action == 0 for m in boxes.markers)


def test_ray_mode_beats_frontier(harness):
    _node, peer, spin, _res = harness
    peer.odom([0.0, 0.0, 6.0])
    blob = np.array([30.0, 0.0, 6.0]) + np.random.default_rng(0).normal(
        scale=0.3, size=(12, 3))
    peer.frontiers(blob)
    # RDF spherical: theta about +z_rdf, phi from +z_rdf. A ray heading +x_flu
    # is +z_rdf, i.e. phi = 0.
    peer.rays(np.array([[25.0, 0.0, 6.0]]), theta_deg=[0.0], phi_deg=[0.0])
    spin(1.0)
    assert _last(peer, f'/{ROBOT}/navigation_mode').data == 'ray'
    rays_viz = _last(peer, f'/{ROBOT}/filtered_rays')
    assert rays_viz is not None and rays_viz.markers


def test_results_file_schema_after_a_gps_fix(harness):
    _node, peer, spin, results = harness
    peer.odom([0.0, 0.0, 6.0])
    spin(0.2)
    peer.fix()
    r = np.arange(-1.0, 1.01, 0.5)
    g = np.stack(np.meshgrid(r, r, r, indexing='ij'), axis=-1).reshape(-1, 3)
    peer.voxels(np.array([20.0, 0.0, 3.0])[None, :] + g)
    spin(1.5)
    path = pathlib.Path(results) / f'{ROBOT}.json'
    assert path.exists(), 'no results dump'
    data = json.loads(path.read_text())
    from raven_nav.results import RESULT_KEYS
    assert tuple(data.keys()) == RESULT_KEYS
    assert data['robot'] == ROBOT
    assert data['confirmed_targets_enu']
    # boot_enu is huge (Lisbon-referenced ENU); the detection must be lifted.
    boot = data['boot_enu']
    ct = data['confirmed_targets_enu'][0]
    assert abs(ct['center_enu'][0] - (boot[0] + 20.0)) < 1.0
    assert data['target_events']


def test_search_area_polygon_is_accepted(harness):
    node, peer, spin, _res = harness
    peer.polygon([(-50, -50), (50, -50), (50, 50), (-50, 50)])
    spin(0.5)
    assert node._search_area_xy is not None
    assert node._search_area_xy.shape == (4, 2)
    peer.polygon([])
    spin(0.5)
    assert node._search_area_xy is None


def test_coverage_completion_publishes_complete(harness):
    """9 poses over a 6x6 m polygon with 1 m cells = 9 of 36 m^2 = 25%, which
    clears the 15% threshold this harness sets.

    The thresholds are deliberately small: coverage is measured by testing each
    cell's CENTRE against the polygon, so a polygon whose edges do not fall on
    cell boundaries can never read 100% (with 2 m cells this 6x6 m square tops
    out at 44%). Irrelevant at mission scale — 0.5 m cells over a 250 m plate —
    but it decides what a nine-pose test can assert.
    """
    _node, peer, spin, _res = harness
    peer.polygon([(-3, -3), (3, -3), (3, 3), (-3, 3)])
    peer.odom([0.0, 0.0, 6.0])
    spin(0.3)
    for x in (-2.5, -0.5, 2.5):
        for y in (-2.5, -0.5, 2.5):
            peer.odom([x, y, 6.0])
            spin(0.15)
    spin(0.5)
    modes = [m.data for m in peer.got.get(f'/{ROBOT}/navigation_mode', [])]
    assert 'complete' in modes


def test_external_lvlm_answer_becomes_guiding_queries(harness):
    _node, peer, spin, _res = harness
    peer.odom([0.0, 0.0, 6.0])
    spin(0.3)
    peer.lvlm_pub.publish(String(data='a car, the roof, debris.'))
    spin(0.8)
    objs = _last(peer, f'/{ROBOT}/raven_nav/guiding_objects')
    assert objs is not None
    assert json.loads(objs.data) == ['car', 'roof', 'debris']
    sent = [m.data for m in peer.got.get(f'{RF}/new_text_query', [])]
    assert sent == ['car', 'roof', 'debris']


def test_frontier_only_baseline_still_reports_detections(monkeypatch):
    """Deviation 3 + the requirement that the baseline arm keeps detecting."""
    results = tempfile.mkdtemp(prefix='raven-fob-')
    monkeypatch.setenv('ROBOT_NAME', ROBOT)
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    cli_args = [
        '--ros-args',
        '-p', f"query_labels:=['{QUERIES[0]}','{QUERIES[1]}','{QUERIES[2]}']",
        '-p', "target_labels:=['person']",
        '-p', 'timer_period:=0.05', '-p', 'frontier_only_baseline:=true',
        '-p', 'voxel_min_cluster_size:=8', '-p', 'lvlm_enabled:=false',
        '-p', f'results_dir:={results}',
    ]
    node = peer = None
    try:
        node = RavenNavNode(context=ctx, cli_args=cli_args)
        peer = Peer(ctx)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        ex.add_node(peer)

        def spin(seconds):
            end = time.time() + seconds
            while time.time() < end:
                ex.spin_once(timeout_sec=0.02)

        spin(0.5)
        peer.odom([18.0, 0.0, 3.0])
        r = np.arange(-1.0, 1.01, 0.5)
        g = np.stack(np.meshgrid(r, r, r, indexing='ij'), axis=-1).reshape(-1, 3)
        peer.voxels(np.array([20.0, 0.0, 3.0])[None, :] + g)
        blob = np.array([60.0, 0.0, 6.0]) + np.random.default_rng(0).normal(
            scale=0.3, size=(12, 3))
        peer.frontiers(blob)
        spin(1.5)
        modes = {m.data for m in peer.got.get(f'/{ROBOT}/navigation_mode', [])}
        assert modes <= {'idle', 'frontier'}, f'baseline navigated: {modes}'
        ds = json.loads(_last(peer, f'/{ROBOT}/raven_nav/discoveries').data)
        assert ds and ds[0]['label'] == 'person'
        completed = json.loads(_last(peer, f'/{ROBOT}/completed_targets').data)
        assert completed == ['person'], 'the fly-by should count as visited'
    finally:
        if node is not None:
            node.destroy_node()
        if peer is not None:
            peer.destroy_node()
        rclpy.shutdown(context=ctx)
        shutil.rmtree(results, ignore_errors=True)

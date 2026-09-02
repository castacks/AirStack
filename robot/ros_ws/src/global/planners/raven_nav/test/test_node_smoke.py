"""RavenNavNode driven end to end against stubbed ROS modules.

This is the host-side counterpart to `test/integration/test_node_roundtrip.py`
(which needs real rclpy): it constructs the node, feeds its callbacks, runs its
tick, and asserts on what it published — catching wiring mistakes the pure
behaviour tests cannot see. It SKIPS where real ROS is installed, so it never
shadows the real integration test in the container.
"""
import json

import numpy as np
import pytest

import ros_stubs

pytestmark = pytest.mark.skipif(
    ros_stubs.ros_is_real(),
    reason='real ROS present — the DDS integration test covers this')

QUERIES = ['person', 'sky', 'road']


@pytest.fixture()
def node(monkeypatch, tmp_path):
    rec = ros_stubs.install()
    monkeypatch.setenv('ROBOT_NAME', 'robot_1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '1')
    monkeypatch.setenv('RAVEN_LVLM', 'false')
    import importlib
    import raven_nav.ros_io
    importlib.reload(raven_nav.ros_io)
    import raven_nav.raven_nav_node as rn
    importlib.reload(rn)
    n = rn.RavenNavNode()
    n._query_labels = list(QUERIES)
    n._target_objects = ['person']
    n._results_dir = str(tmp_path)
    n._results_dump_period_s = 1e-6
    n._manager.voxel_behavior.min_cluster_size = 8
    ros_stubs.set_clock(0.0)
    yield n, rec, tmp_path


def _tick(node, rec, t):
    ros_stubs.set_clock(t)
    node._timer_cb()


def _odom(node, xyz):
    from nav_msgs.msg import Odometry
    m = Odometry()
    m.pose = type('P', (), {})()
    m.pose.pose = type('Q', (), {})()
    m.pose.pose.position = type('V', (), {})()
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = \
        [float(v) for v in xyz]
    m.pose.pose.orientation = type('O', (), {'w': 1.0, 'x': 0.0, 'y': 0.0,
                                             'z': 0.0})()
    node._odometry_cb(m)


def _flu_to_rdf(p):
    p = np.asarray(p, dtype=float).reshape(-1, 3)
    return np.stack([-p[:, 1], -p[:, 2], p[:, 0]], axis=1)


def _voxels(node, pts_flu, col=0, score=0.99):
    rows = []
    for p in _flu_to_rdf(pts_flu):
        sims = [0.005] * len(QUERIES)
        sims[col] = score
        rows.append(list(p) + sims)
    node._vox_all_cb(ros_stubs.make_cloud(
        ['x', 'y', 'z'] + [f'sim_{i}' for i in range(len(QUERIES))], rows))


def _rays(node, origins_flu, theta, phi, col=0, score=0.99):
    rows = []
    for p, th, ph in zip(_flu_to_rdf(origins_flu), theta, phi):
        sims = [0.005] * len(QUERIES)
        sims[col] = score
        rows.append(list(p) + [th, ph] + sims)
    node._ray_all_cb(ros_stubs.make_cloud(
        ['x', 'y', 'z', 'theta', 'phi']
        + [f'sim_{i}' for i in range(len(QUERIES))], rows))


def _frontiers(node, pts_flu):
    node._frontiers_cb(ros_stubs.make_cloud(
        ['x', 'y', 'z'], _flu_to_rdf(pts_flu)))


def _blob(center, n=12, spread=0.3, seed=0):
    return np.asarray(center, dtype=float)[None, :] + \
        np.random.default_rng(seed).normal(scale=spread, size=(n, 3))


def _box(center, half=1.0, step=0.5):
    r = np.arange(-half, half + 1e-9, step)
    g = np.stack(np.meshgrid(r, r, r, indexing='ij'), axis=-1).reshape(-1, 3)
    return np.asarray(center, dtype=float)[None, :] + g


T = {k: f'/robot_1/{k}' for k in (
    'global_plan', 'navigation_mode', 'completed_targets', 'current_target',
    'filtered_rays', 'voxel_clusters', 'filtered_frontiers',
    'frontier_viewpoints')}
T.update({k: f'/robot_1/raven_nav/{k}' for k in (
    'discoveries', 'confirmed_targets', 'explored_area_coverage',
    'lvlm_trigger', 'guiding_objects', 'lvlm_request')})
RF = '/robot_1/rayfronts/msg_serv'


# ── construction ────────────────────────────────────────────────────────────
def test_every_contract_parameter_is_declared(node):
    n, _rec, _tmp = node
    from raven_nav import params as P
    for name in P.PARAM_NAMES:
        if P.param(name).auto:
            continue
        assert n.has_parameter(name), name


def test_startup_log_line(node):
    _n, rec, _tmp = node
    assert any('raven_nav started' in line for line in rec.logs)


def test_it_subscribes_to_every_input(node):
    _n, rec, _tmp = node
    for topic in (f'{RF}/rays_sim/all', f'{RF}/voxels_sim/all',
                  f'{RF}/frontiers', '/robot_1/odometry', '/input_prompt',
                  '/robot_1/raven_nav/clear_blacklist',
                  '/robot_1/interface/mavros/global_position/raw/fix',
                  '/robot_1/raven_nav/search_area',
                  '/robot_1/raven_nav/lvlm_output',
                  '/robot_1/sensors/front_stereo/left/image_rect'):
        assert topic in rec.subscriptions, topic


def test_it_does_not_subscribe_to_the_removed_coordination_topics(node):
    _n, rec, _tmp = node
    for topic in rec.subscriptions:
        assert 'peer_registry' not in topic
        assert 'conavgpt' not in topic


# ── the tick ────────────────────────────────────────────────────────────────
def test_idle_before_odometry(node):
    n, rec, _tmp = node
    _tick(n, rec, 1.0)
    assert rec.last(T['navigation_mode']).data == 'idle'
    assert not rec.published.get(T['global_plan'])
    assert any('waiting for odometry' in l.lower() for l in rec.logs)


def test_frontier_tick_publishes_a_two_pose_plan(node):
    n, rec, _tmp = node
    _odom(n, [0, 0, 6])
    _frontiers(n, _blob([40, 0, 6]))
    _tick(n, rec, 1.0)
    assert rec.last(T['navigation_mode']).data == 'frontier'
    plan = rec.last(T['global_plan'])
    assert plan is not None and len(plan.poses) == 2
    assert plan.header.frame_id == 'map'
    assert rec.last(T['frontier_viewpoints']) is not None
    assert rec.last(T['filtered_frontiers']) is not None


def test_voxel_tick_switches_mode_and_reports(node):
    n, rec, _tmp = node
    _odom(n, [0, 0, 3])
    _frontiers(n, _blob([40, 0, 6]))
    _tick(n, rec, 1.0)
    _voxels(n, _box([20, 0, 3]))
    _tick(n, rec, 2.0)
    assert rec.last(T['navigation_mode']).data == 'voxel'
    ds = json.loads(rec.last(T['discoveries']).data)
    assert ds and ds[0]['label'] == 'person'
    cts = json.loads(rec.last(T['confirmed_targets']).data)
    assert cts and cts[0]['status'] == 'observing'
    assert any('behavior mode: Frontier-based -> Voxel-based' in l
               for l in rec.logs)


def test_ray_tick(node):
    n, rec, _tmp = node
    _odom(n, [0, 0, 6])
    # RDF spherical: +x_flu is +z_rdf, i.e. phi = 0.
    _rays(n, np.array([[25.0, 0.0, 6.0]]), theta=[0.0], phi=[0.0])
    _tick(n, rec, 1.0)
    assert rec.last(T['navigation_mode']).data == 'ray'
    plan = rec.last(T['global_plan'])
    # wp1 = origin + 6 m along +x
    assert plan.poses[0].pose.position.x == pytest.approx(31.0, abs=1e-6)
    assert rec.last(T['current_target']).data == 'person'


def test_mode_switch_resets_the_waypoint_lock(node):
    n, rec, _tmp = node
    _odom(n, [0, 0, 3])
    _voxels(n, _box([20, 0, 3]))
    _tick(n, rec, 1.0)
    assert n._waypoint_locked is True
    _voxels(n, np.zeros((0, 3)))
    n._vox_xyz = None
    n._vox_scores = None
    _frontiers(n, _blob([40, 0, 6]))
    _tick(n, rec, 2.0)
    assert rec.last(T['navigation_mode']).data == 'frontier'


def test_status_line_survives_the_upstream_filter_every_tick(node):
    n, rec, _tmp = node
    _odom(n, [0, 0, 6])
    _frontiers(n, _blob([40, 0, 6]))
    _tick(n, rec, 1.0)
    assert any('[Frontier-based]' in l for l in rec.logs)


# ── frames + results ────────────────────────────────────────────────────────
def test_navsat_anchors_the_odom_origin_not_the_current_pose(node):
    from sensor_msgs.msg import NavSatFix
    n, rec, _tmp = node
    _odom(n, [10, 20, 5])
    fix = NavSatFix()
    fix.status = type('S', (), {'status': 0})()
    fix.latitude, fix.longitude, fix.altitude = 38.0, -9.0, 105.0
    n._navsat_cb(fix)
    # the stub gps_to_enu answers (1000, 2000, alt); the current pose backs out
    assert n._boot_enu[0] == pytest.approx(990.0)
    assert n._boot_enu[1] == pytest.approx(1980.0)
    assert n._alt_ground == pytest.approx(100.0)
    assert any('boot GPS captured' in l for l in rec.logs)


def test_results_file_is_written_with_the_contract_schema(node):
    from raven_nav.results import RESULT_KEYS
    from sensor_msgs.msg import NavSatFix
    n, rec, tmp = node
    _odom(n, [0, 0, 3])
    fix = NavSatFix()
    fix.status = type('S', (), {'status': 0})()
    fix.latitude, fix.longitude, fix.altitude = 38.0, -9.0, 103.0
    n._navsat_cb(fix)
    _voxels(n, _box([20, 0, 3]))
    _tick(n, rec, 1.0)
    path = tmp / 'robot_1.json'
    assert path.exists()
    data = json.loads(path.read_text())
    assert tuple(data.keys()) == RESULT_KEYS
    ct = data['confirmed_targets_enu'][0]
    assert ct['center_enu'][0] == pytest.approx(1000.0 + 20.0, abs=0.5)
    assert ct['center_enu'][2] == pytest.approx(3.0, abs=0.5)   # z stays AGL
    assert data['target_events'][0]['label'] == 'person'


def test_search_area_polygon_constrains_navigation(node):
    from geometry_msgs.msg import PolygonStamped
    n, rec, _tmp = node
    msg = PolygonStamped()
    msg.polygon = type('P', (), {})()
    msg.polygon.points = [type('V', (), {'x': x, 'y': y, 'z': 0.0})()
                          for x, y in ((-10, -10), (10, -10), (10, 10),
                                       (-10, 10))]
    n._search_area_cb(msg)
    assert n._search_area_xy.shape == (4, 2)
    assert any('search_area updated' in l for l in rec.logs)
    _odom(n, [0, 0, 6])
    _frontiers(n, _blob([200, 0, 6]))       # entirely outside the polygon
    _tick(n, rec, 1.0)
    assert not rec.published.get(T['global_plan'])


def test_coverage_completion_publishes_complete_and_hovers(node):
    from geometry_msgs.msg import PolygonStamped
    n, rec, tmp = node
    n._coverage.cell_size_m = 2.0
    n._coverage_threshold = 0.4
    msg = PolygonStamped()
    msg.polygon = type('P', (), {})()
    msg.polygon.points = [type('V', (), {'x': x, 'y': y, 'z': 0.0})()
                          for x, y in ((-3, -3), (3, -3), (3, 3), (-3, 3))]
    n._search_area_cb(msg)
    t = 0.0
    for x in (-2.0, 0.0, 2.0):
        for y in (-2.0, 0.0, 2.0):
            _odom(n, [x, y, 6])
            t += 1.0
            _tick(n, rec, t)
    modes = [m.data for m in rec.published[T['navigation_mode']]]
    assert 'complete' in modes
    plan = rec.last(T['global_plan'])
    assert len(plan.poses) == 1, 'complete must hover in place'


# ── the frontier-only baseline ──────────────────────────────────────────────
def test_frontier_only_baseline_navigates_frontier_but_still_detects(
        monkeypatch, tmp_path):
    rec = ros_stubs.install()
    monkeypatch.setenv('ROBOT_NAME', 'robot_1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '1')
    monkeypatch.setenv('RAVEN_LVLM', 'false')
    import importlib
    import raven_nav.ros_io
    importlib.reload(raven_nav.ros_io)
    import raven_nav.raven_nav_node as rn
    importlib.reload(rn)
    n = rn.RavenNavNode()
    n._frontier_only = True
    n._manager.frontier_only = True
    n._query_labels = list(QUERIES)
    n._target_objects = ['person']
    n._manager.voxel_behavior.min_cluster_size = 8
    _odom(n, [18, 0, 3])
    _voxels(n, _box([20, 0, 3]))
    _frontiers(n, _blob([80, 0, 6]))
    _rays(n, np.array([[25.0, 0.0, 6.0]]), theta=[0.0], phi=[0.0])
    _tick(n, rec, 1.0)
    _tick(n, rec, 2.0)
    modes = {m.data for m in rec.published[T['navigation_mode']]}
    assert modes == {'frontier'}, f'the baseline navigated semantically: {modes}'
    ds = json.loads(rec.last(T['discoveries']).data)
    assert ds and ds[0]['label'] == 'person'
    assert json.loads(rec.last(T['completed_targets']).data) == ['person']


# ── LVLM plumbing ───────────────────────────────────────────────────────────
def test_external_lvlm_answer_registers_queries(node):
    from std_msgs.msg import String
    n, rec, _tmp = node
    n._lvlm_output_cb(String(data='a car, the roof, debris.'))
    assert json.loads(rec.last(T['guiding_objects']).data) == \
        ['car', 'roof', 'debris']
    sent = [m.data for m in rec.published[f'{RF}/new_text_query']]
    assert sent == ['car', 'roof', 'debris']
    latched = json.loads(rec.last(f'{RF}/guiding_queries').data)
    assert latched == ['car', 'roof', 'debris']


def test_a_repeated_answer_does_not_re_register_queries(node):
    from std_msgs.msg import String
    n, rec, _tmp = node
    n._lvlm_output_cb(String(data='car'))
    n._lvlm_output_cb(String(data='car'))
    assert len(rec.published[f'{RF}/new_text_query']) == 1


def test_shared_mode_keeps_guiding_labels_off_new_text_query(monkeypatch):
    # RAYFRONTS_MODE=shared: the shared server pins every new_text_query
    # label forever (GuidingQueryRegistry.pin), so guiding objects must
    # travel ONLY on the latched guiding_queries list or they can never be
    # deleted when the LVLM moves on.
    rec = ros_stubs.install()
    monkeypatch.setenv('ROBOT_NAME', 'robot_1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '1')
    monkeypatch.setenv('RAVEN_LVLM', 'false')
    monkeypatch.setenv('RAYFRONTS_MODE', 'shared')
    import importlib
    import raven_nav.ros_io
    importlib.reload(raven_nav.ros_io)
    import raven_nav.raven_nav_node as rn
    importlib.reload(rn)
    from std_msgs.msg import String
    n = rn.RavenNavNode()
    n._lvlm_output_cb(String(data='a car, the roof'))
    assert rec.published.get(f'{RF}/new_text_query', []) == []
    assert json.loads(rec.last(f'{RF}/guiding_queries').data) == \
        ['car', 'roof']


def test_lvlm_mode_fires_on_a_guiding_column(monkeypatch, tmp_path):
    from std_msgs.msg import String
    rec = ros_stubs.install()
    monkeypatch.setenv('ROBOT_NAME', 'robot_1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '1')
    monkeypatch.setenv('RAVEN_LVLM', 'true')
    import importlib
    import raven_nav.ros_io
    importlib.reload(raven_nav.ros_io)
    import raven_nav.raven_nav_node as rn
    importlib.reload(rn)
    n = rn.RavenNavNode()
    n._vlm = None                       # no HTTP in this test
    n._query_labels = ['person', 'sky', 'roof']
    n._target_objects = ['person']
    n._lvlm_output_cb(String(data='roof'))
    _odom(n, [0, 0, 6])
    _rays(n, np.array([[10.0, 0.0, 6.0]]), theta=[0.0], phi=[0.0], col=2)
    _tick(n, rec, 1.0)
    assert rec.last(T['navigation_mode']).data == 'lvlm'
    assert rec.last(T['lvlm_trigger']).data is True
    plan = rec.last(T['global_plan'])
    assert len(plan.poses) == 2
    assert plan.poses[1].pose.position.x == pytest.approx(15.0, abs=1e-6)
    assert any('[LVLM-guided]' in l for l in rec.logs)


def test_column_order_comes_from_the_rayfronts_topic_names(node):
    n, rec, _tmp = node
    rec.topics.extend([f'{RF}/rays_sim/q0_person',
                       f'{RF}/rays_sim/q1_sky',
                       f'{RF}/rays_sim/q2_fire_truck'])
    n._detected_query_labels = None
    n._manager.lvlm_behavior.set_guiding_objects('fire truck')
    n._refresh_columns(3)
    # the sanitised topic name is mapped back to the label that produced it
    assert n._query_labels == ['person', 'sky', 'fire truck']


def test_inert_parameters_are_logged_once_when_set(monkeypatch, tmp_path):
    rec = ros_stubs.install()
    monkeypatch.setenv('ROBOT_NAME', 'robot_1')
    monkeypatch.setenv('RAVEN_LVLM', 'false')
    import importlib
    import raven_nav.ros_io
    importlib.reload(raven_nav.ros_io)
    import raven_nav.raven_nav_node as rn
    importlib.reload(rn)

    real_declare = ros_stubs._Node.declare_parameter

    def fake_declare(self, name, default):
        if name == 'conavgpt_baseline':
            default = True
        return real_declare(self, name, default)
    monkeypatch.setattr(ros_stubs._Node, 'declare_parameter', fake_declare)
    rn.RavenNavNode()
    hits = [l for l in rec.logs if 'IGNORED' in l]
    assert len(hits) == 1 and 'conavgpt_baseline' in hits[0]


# ── env-resolved LVLM knobs reach the behaviour ─────────────────────────────
@pytest.mark.parametrize('env,interval,threshold', [
    ({}, 30.0, 0.9),
    ({'RAVEN_LVLM_INTERVAL_S': '7.5', 'RAVEN_LVLM_RAY_THRESHOLD': '0.35'},
     7.5, 0.35),
    ({'RAVEN_LVLM_INTERVAL_S': 'nonsense'}, 30.0, 0.9),
])
def test_lvlm_env_knobs_reach_the_behaviour(monkeypatch, env, interval,
                                            threshold):
    ros_stubs.install()
    monkeypatch.setenv('ROBOT_NAME', 'robot_1')
    monkeypatch.setenv('RAVEN_LVLM', 'false')
    for k in ('RAVEN_LVLM_INTERVAL_S', 'RAVEN_LVLM_RAY_THRESHOLD'):
        monkeypatch.delenv(k, raising=False)
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    import importlib
    import raven_nav.ros_io
    importlib.reload(raven_nav.ros_io)
    import raven_nav.raven_nav_node as rn
    importlib.reload(rn)
    n = rn.RavenNavNode()
    assert n._manager.lvlm_behavior.request_interval_s == interval
    assert n._manager.lvlm_behavior.ray_threshold == threshold

"""Map fusion, no-BB leakage, and actual ROS serialization integration."""
import json
from types import SimpleNamespace as S
import numpy as np
import pytest
from raven_nav.semantic_map import SemanticMapFusion


def test_overlapping_sources_do_not_duplicate_voxels_or_repeat_votes():
    f = SemanticMapFusion(.5)
    labels = ['car', 'background']
    f.update('r1', [[.1, .1, 5]], [[.8, .2]], labels, 1)
    f.update('r2', [[.2, .1, 5]], [[.4, .6]], labels, 10)
    assert not f.update('r2', [[.2, .1, 5]], [[1, 0]], labels, 10)
    for stamp in range(11, 15):
        f.update('r2', [[.2, .1, 5]], [[.4, .6]], labels, stamp)
    xyz, scores = f.arrays()
    assert len(xyz) == 1
    np.testing.assert_allclose(scores, [[.6, .4]])
    np.testing.assert_allclose(xyz, [[.15, .1, 5]])


def test_accumulation_retains_all_cells_and_full_score_vector():
    f = SemanticMapFusion()
    f.update('r1', [[0, 0, 5]], [[.1, .9]], ['car', 'wall'], 1)
    f.update('r1', [[10, 0, 5]], [[.05, .95]], ['car', 'wall'], 2)
    xyz, scores = f.arrays()
    assert xyz.shape == (2, 3) and scores.shape == (2, 2)
    assert scores[:, 1].min() > .8  # background cells not discarded
    with pytest.raises(ValueError, match='vocabulary'):
        f.update('r2', [[0, 0, 5]], [[.9, .1]], ['wall', 'car'], 3)


def test_peer_only_map_evidence_produces_bb_through_existing_clusterer():
    from raven_nav.behaviors.voxel_behavior import VoxelBehavior
    f = SemanticMapFusion(.5)
    labels = ['car', 'wall']
    f.update('own', [[0, 0, 5]], [[.01, .99]], labels, 1)
    xyz = np.array([[10+x*.5, 10+y*.5, 8+z*.5]
                    for x in range(5) for y in range(5) for z in range(3)])
    f.update('peer', xyz, np.tile([.99, .01], (len(xyz), 1)), labels, 2)
    fused_xyz, scores = f.arrays()
    vb = VoxelBehavior(get_clock=lambda: None, score_threshold=.5,
                       min_cluster_size=10, confirm_hits=1)
    vb.condition_check(fused_xyz, scores, labels, ['car'],
                       committed_origin=None, committed_dir=None)
    assert vb.target_voxel_clusters
    assert set(vb.cluster_query_map.values()) == {'car'}
    assert all(bb[0] > 9 for bb in vb.target_voxel_clusters.values())


def test_ros_map_roundtrip_fuses_peer_geometry_in_local_frame():
    from raven_nav.raven_nav_node import RavenNavNode
    from coordination_msgs.msg import PeerProfile, PeerProfilePayload, SemanticVoxelMap
    from rclpy.serialization import serialize_message, deserialize_message
    published = []
    node = S(_boot_enu=np.array([100., 200., 0.]), _robot_name='robot_1',
             _query_labels=['car', 'wall'], _semantic_fusion=SemanticMapFusion(),
             _own_semantic_snapshot=(np.array([[.1, .1, 5.]], dtype=np.float32),
                                     np.array([[.8, .2]], dtype=np.float32), 1),
             _semantic_map_pub=S(publish=published.append), _semantic_map_rejections=0)
    from builtin_interfaces.msg import Time
    node.get_clock = lambda: S(now=lambda: S(to_msg=lambda: Time(sec=50)))
    node.get_logger = lambda: S(error=lambda *a, **kw: pytest.fail(str(a)))
    node._refresh_semantic_map = lambda **kw: RavenNavNode._refresh_semantic_map(node, **kw)
    node._refresh_semantic_map(publish=True)
    sent = deserialize_message(serialize_message(published[0]), SemanticVoxelMap)
    assert sent.cloud.width == 1 and sent.labels == ['car', 'wall']
    # Receiver has a different spawn origin, but sees the same world geometry.
    node._boot_enu = np.array([90., 190., 0.])
    own_snapshot = node._own_semantic_snapshot
    node._own_semantic_snapshot = None  # peer map works even with no own data
    profile = PeerProfile(robot_name='robot_2', payloads=[PeerProfilePayload(
        payload_name='semantic_voxel_map', payload_type='coordination_msgs/msg/SemanticVoxelMap',
        payload_data=list(serialize_message(sent)))])
    RavenNavNode._accept_semantic_map(node, profile)
    assert 'robot_2' in node._semantic_fusion.sources
    assert np.any(np.linalg.norm(node._vox_xyz - [10.1, 10.1, 5], axis=1) < .01)
    # Re-publishing must still contain only the own raw cell, never the union.
    node._own_semantic_snapshot = own_snapshot
    node._refresh_semantic_map(publish=True)
    assert published[-1].cloud.width == 1


def test_map_mode_gossip_excludes_unvisited_bb_channels():
    from coordination_bringup.bandwidth import completed_only_payload, payload_sizes
    from coordination_bringup.peer_profile import PeerProfile
    from std_msgs.msg import String
    boxes = String(data=json.dumps([{'label': 'car', 'status': s} for s in ['observing', 'visited']]))
    out = completed_only_payload('confirmed_targets', boxes)
    assert [b['status'] for b in json.loads(out.data)] == ['visited']
    table = String(data=json.dumps({'visited_bbs': [{'label': 'car'}], 'available': [
        {'status': 'bb-observing', 'size': [1, 2, 3]}, {'status': 'ray'}]}))
    filtered = completed_only_payload('auction_table', table)
    assert json.loads(filtered.data)['available'] == [{'status': 'ray'}]
    p = PeerProfile('robot_1')
    p.add_payload(out, name='confirmed_targets')
    p.add_payload(filtered, name='auction_table')
    from rclpy.serialization import serialize_message
    sizes = payload_sizes(p.to_ros_msg())
    assert sizes['confirmed_targets'] == len(serialize_message(out))
    assert sizes['auction_table'] == len(serialize_message(filtered))


def test_live_gossip_transports_large_map_and_records_exact_payload_bytes(monkeypatch):
    import time
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.serialization import serialize_message
    from pathlib import Path
    from coordination_msgs.msg import SemanticVoxelMap
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header, String
    from rosgraph_msgs.msg import Clock
    from builtin_interfaces.msg import Time
    monkeypatch.setenv('RAVEN_SEMANTIC_MAP_SHARING', 'true')
    monkeypatch.setenv('GOSSIP_MEASURE_BYTES', 'true')
    from coordination_bringup.gossip_node import GossipNode
    rclpy.init(args=[])
    nodes = []
    executor = SingleThreadedExecutor()
    try:
        # Resolve by repo landmark rather than relying on workspace depth.
        repo = next(p for p in Path(__file__).resolve().parents if (p / 'airstack.sh').exists())
        config = repo / 'common/ros_packages/coordination/coordination_bringup/config/gossip_payloads.yaml'
        for name in ['robot_81', 'robot_82']:
            monkeypatch.setenv('ROBOT_NAME', name)
            n = GossipNode()
            n._boot_pos = (0., 0., 0.)
            n._setup_payload_subscriptions(str(config))
            executor.add_node(n)
            nodes.append(n)
        n = nodes[0]
        pub = n.create_publisher(SemanticVoxelMap, '/robot_81/raven_nav/semantic_voxel_map', 1)
        boxpub = n.create_publisher(String, '/robot_81/raven_nav/confirmed_targets', 1)
        clockpub = n.create_publisher(Clock, '/clock', 1)
        matrix = np.zeros((10000, 43), dtype='<f4')
        matrix[:, 0] = np.arange(10000) * .5
        matrix[:, 2] = 5
        matrix[:, 3:] = .025
        names = ['x', 'y', 'z'] + [f'sim_{i}' for i in range(40)]
        msg = SemanticVoxelMap(labels=[f'label{i}' for i in range(40)], voxel_size=.5,
            cloud=PointCloud2(header=Header(stamp=Time(sec=42), frame_id='world_xy_agl'),
                height=1, width=len(matrix), point_step=172, row_step=172*len(matrix),
                fields=[PointField(name=name, offset=i*4, datatype=7, count=1)
                        for i, name in enumerate(names)], data=matrix.tobytes(), is_dense=True))
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            pub.publish(msg)
            boxpub.publish(String(data='[{"status":"observing"},{"status":"visited"}]'))
            clockpub.publish(Clock(clock=Time(sec=42)))
            for _ in range(10):
                executor.spin_once(timeout_sec=.02)
            if nodes[1]._registry.get('robot_81') is not None:
                from coordination_bringup.peer_profile import PeerProfile
                received = PeerProfile.from_ros_msg(nodes[1]._registry['robot_81'])
                cloud = received.get_payload_by_name('semantic_voxel_map')
                if cloud is not None:
                    assert bytes(cloud.cloud.data) == matrix.tobytes()
                    boxes = received.get_payload_by_name('confirmed_targets')
                    assert json.loads(boxes.data) == [{'status': 'visited'}]
                    lines = Path('/tmp/gossip_bandwidth_robot_81.jsonl').read_text().splitlines()
                    assert any(json.loads(line)['payload_bytes'].get('semantic_voxel_map') == len(serialize_message(msg))
                               for line in lines)
                    break
        else:
            pytest.fail('full semantic map was not delivered through live gossip')
    finally:
        for n in nodes:
            executor.remove_node(n)
            n.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_bandwidth_summary_uses_each_robots_own_search_window(tmp_path):
    import importlib.util
    from pathlib import Path
    repo = next(p for p in Path(__file__).resolve().parents if (p / 'airstack.sh').exists())
    spec = importlib.util.spec_from_file_location('byte_summary', repo / 'osmo/workspace/summarize_gossip_bytes.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    (tmp_path / 'raven_results').mkdir()
    for robot, start in [('robot_1', 100), ('robot_2', 200)]:
        (tmp_path / 'raven_results' / f'{robot}.json').write_text(json.dumps({'mission_start_ts': start}))
        ledger = tmp_path / f'gossip_bandwidth_{robot}.jsonl'
        entries = [dict(direction='tx', wall_ns=i*1_000_000_000,
                        sim_ns=None if t is None else t*1e9, cdr_bytes=100,
                        payload_bytes={'confirmed_targets': 50})
                   for i, t in enumerate([None, 50, 150, 250, 450, 600])]
        ledger.write_text('\n'.join(json.dumps(e) for e in entries))
    summary = module.summarize(tmp_path)
    assert summary['robots']['robot_1']['tx']['messages'] == 2
    assert summary['robots']['robot_2']['tx']['messages'] == 2
    assert summary['fleet']['tx']['cdr_bytes'] == 400
    assert summary['fleet']['tx']['payload_bytes']['confirmed_targets'] == 200

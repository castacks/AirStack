"""Raw sender -> gossip CDR -> receiver fusion -> unchanged local filtering."""
from types import SimpleNamespace as S
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import PointCloud2
from coordination_msgs.msg import PeerProfile as PeerProfileMsg
from coordination_bringup.peer_profile import PeerProfile
from coordination_bringup.frame_utils import transform_point_cloud2
from raven_nav.raven_nav_node import RavenNavNode
from raven_nav.peer_state import PeerState
from raven_nav.ray_groups import compute_ray_groups


def sender(raw=False, vlfm=False):
    sent = []
    node = S(_share_raw_rays=raw, _vlfm_baseline=vlfm,
             _query_labels=['car', 'wall'], _target_objects=['car'],
             _score_threshold=.65, _shared_rays_pub=S(publish=sent.append),
             _ray_share_stats={'source_rows': 0, 'wire_rows': 0, 'snapshots': 0})
    return node, sent


def publish(node, origins, scores):
    dirs = np.tile([1., 0., 0.], (len(origins), 1)).astype(np.float32)
    RavenNavNode._publish_shared_rays(node, origins, dirs, scores,
                                    [f'sim_{i}' for i in range(scores.shape[1])], Time(sec=42))
    return dirs


@pytest.mark.parametrize('raw,vlfm,count', [(False, False, 2), (True, False, 4), (False, True, 4)])
def test_sender_mode_preserves_schema_and_local_arrays(raw, vlfm, count):
    node, sent = sender(raw, vlfm)
    origins = np.array([[0, 0, 10], [1, 0, 10], [2, 0, 10], [3, 0, 10]], np.float32)
    scores = np.array([[.9, .1], [.1, .9], [.7, .8], [.4, .3]], np.float32)
    before = scores.copy()
    dirs = publish(node, origins, scores)
    msg = deserialize_message(serialize_message(sent[0]), PointCloud2)
    assert msg.width == count and msg.point_step == 32
    assert [f.name for f in msg.fields] == ['x','y','z','dx','dy','dz','sim_0','sim_1']
    rows = np.frombuffer(msg.data, np.float32).reshape(count, 8)
    keep = np.arange(4) if raw or vlfm else np.array([0, 2])
    np.testing.assert_array_equal(rows, np.column_stack([origins, dirs, scores])[keep])
    np.testing.assert_array_equal(scores, before)
    assert node._ray_share_stats['source_rows'] == 4
    assert node._ray_share_stats['wire_rows'] == count


def test_raw_peer_cloud_fuses_then_filters_on_each_robot_without_rebroadcast():
    node, sent = sender(True)
    origins = np.array([[0,0,10], [1,0,10], [2,0,10], [3,0,30]], np.float32)
    scores = np.array([[.9,.1], [.1,.9], [.7,.8], [.95,.05]], np.float32)
    dirs = publish(node, origins, scores)
    global_cloud = transform_point_cloud2(sent[0], 100., 200., 0.)
    profile = PeerProfile(robot_name='robot_1')
    profile.add_payload(global_cloud, name='shared_rays', stamp=Time(sec=42))
    wire = deserialize_message(serialize_message(profile.to_ros_msg()), PeerProfileMsg)
    received = PeerProfile.from_ros_msg(wire).get_payload('sensor_msgs/msg/PointCloud2')
    for boot in (np.array([90.,190.,0.]), np.array([120.,210.,0.])):
        peer = PeerState()._parse_shared_rays(received, boot, None)
        assert len(peer.origins) == 4  # below threshold/background rays reach the receiver
        np.testing.assert_allclose(peer.origins, origins + [100,200,0] - boot)
        np.testing.assert_array_equal(peer.dirs, dirs)
        np.testing.assert_array_equal(peer.scores, scores)
        receiver = S(_ray_origins=np.array([[0,0,10]], np.float32),
                     _ray_dirs=np.array([[0,1,0]], np.float32),
                     _ray_scores=np.array([[.8,.2]], np.float32),
                     _share_rays=True, _peer_state=S(peer_rays={'robot_1': peer}))
        merged = RavenNavNode._merge_own_and_peer_rays(receiver)
        assert len(merged[0]) == 5
        groups = compute_ray_groups(*merged, ['car','wall'], ['car'], .65,
                                    np.array([0,0,10]), min_altitude=5, max_altitude=20)
        assert sum(g.num_rays for g in groups) == 2  # own + one valid peer ray
        # Sender-filtered transport produces identical local grouped evidence.
        peer.scores = peer.scores[[0,2,3]]
        peer.origins = peer.origins[[0,2,3]]
        peer.dirs = peer.dirs[[0,2,3]]
        filtered = compute_ray_groups(*RavenNavNode._merge_own_and_peer_rays(receiver),
                       ['car','wall'], ['car'], .65, np.array([0,0,10]),
                       min_altitude=5, max_altitude=20)
        assert [g.num_rays for g in filtered] == [g.num_rays for g in groups]
        for a,b in zip(groups,filtered):
            np.testing.assert_array_equal(a.ray_origins,b.ray_origins)
            np.testing.assert_array_equal(a.ray_scores,b.ray_scores)
    assert len(sent) == 1 and sent[0].width == 4  # never publish the fused cloud


def test_large_raw_cloud_keeps_every_score_column_and_row():
    node, sent = sender(True)
    n, k = 40000, 50
    origins = np.zeros((n,3), np.float32); origins[:,2] = 10
    scores = np.random.default_rng(17).random((n,k), dtype=np.float32)
    publish(node,origins,scores)
    wire = serialize_message(sent[0])
    assert len(wire) > 8_000_000
    msg = deserialize_message(wire, PointCloud2)
    assert msg.width == n
    rows = np.frombuffer(msg.data,np.float32).reshape(n,6+k)
    np.testing.assert_array_equal(rows[:,6:],scores)
    assert node._ray_share_stats['source_rows'] == node._ray_share_stats['wire_rows'] == n


def test_peer_freshness_tracks_new_ray_cloud_not_heartbeat():
    node, sent = sender(True)
    publish(node, np.array([[0,0,10]],np.float32), np.array([[.9,.1]],np.float32))
    profile = PeerProfile(robot_name='robot_1')
    profile.add_payload(sent[0],name='shared_rays',stamp=Time(sec=42))
    peer = PeerState(verbose_bids=False)
    for now in (100.,101.):
        peer.update(profile.to_ros_msg(),np.zeros(3),None,now)
    assert peer.peer_ray_updates == {'robot_1':1}
    first = peer.peer_ray_received_at['robot_1']
    sent[0].header.stamp.sec += 1
    profile.clear_payloads()
    profile.add_payload(sent[0],name='shared_rays',stamp=Time(sec=43))
    peer.update(profile.to_ros_msg(),np.zeros(3),None,102.)
    assert peer.peer_ray_updates == {'robot_1':2}
    assert peer.peer_ray_received_at['robot_1'] >= first

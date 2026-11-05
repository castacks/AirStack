import time
import threading
import numpy as np
from coordination_bringup.semantic_delta import encode_delta
from coordination_bringup.map_stream import MapStream
from coordination_msgs.msg import SemanticVoxelMap
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs_py import point_cloud2
from raven_nav.semantic_map import SemanticMapFusion


def message(matrix, stamp=1):
    matrix = np.ascontiguousarray(matrix, dtype='<f4')
    names = ['x', 'y', 'z'] + [f'sim_{i}' for i in range(matrix.shape[1]-3)]
    return SemanticVoxelMap(source_robot='robot_1', voxel_size=.5,
        labels=[f'label{i}' for i in range(matrix.shape[1]-3)],
        cloud=PointCloud2(header=Header(stamp=Time(sec=stamp), frame_id='world_xy_agl'),
            height=1, width=len(matrix), point_step=matrix.shape[1]*4,
            row_step=matrix.nbytes, fields=[PointField(name=n, offset=i*4, datatype=7, count=1)
                                          for i,n in enumerate(names)], data=matrix.tobytes(), is_dense=True))


def test_lossless_updates_include_new_and_changed_cells_only():
    initial = np.array([[0,0,5,.8,.2], [1,0,5,.2,.8]], dtype=np.float32)
    data, base = encode_delta(serialize_message(message(initial)), None)
    full = deserialize_message(data, SemanticVoxelMap)
    assert not full.is_delta and full.cloud.width == 2
    updated = np.vstack([initial, [2,0,5,.9,.1]]).astype(np.float32)
    updated[0,3:] = [.7,.3]
    delta, new_base = encode_delta(serialize_message(message(updated, 2)), base)
    update = deserialize_message(delta, SemanticVoxelMap)
    assert update.is_delta and update.base_revision == 1_000_000_000 and update.cloud.width == 2
    fusion = SemanticMapFusion()
    for msg in [full, update]:
        arr = point_cloud2.read_points_numpy(msg.cloud)
        fusion.update('robot_1', arr[:,:3], arr[:,3:], msg.labels, msg.cloud.header.stamp.sec)
    xyz, scores = fusion.arrays()
    np.testing.assert_allclose(np.c_[xyz,scores], updated)
    # Unchanged map advances only the revision: zero voxel payload rows.
    empty, _ = encode_delta(serialize_message(message(updated, 3)), new_base)
    assert deserialize_message(empty, SemanticVoxelMap).cloud.width == 0
    # An unacknowledged peer still receives the complete initial state.
    retry, _ = encode_delta(serialize_message(message(updated, 3)), None)
    assert not deserialize_message(retry, SemanticVoxelMap).is_delta
    assert deserialize_message(retry, SemanticVoxelMap).cloud.width == 3


def test_three_streams_deliver_large_maps_and_recover_failed_ack():
    got = [[], [], []]
    traffic = []
    reject_first = [True]
    def receiver(i):
        def receive(data):
            if i == 1 and reject_first[0]:
                reject_first[0] = False
                raise ValueError('simulate missing base / failed acknowledgement')
            got[i].append(data)
            return 'peer'
        return receive
    streams = [MapStream(receiver(i), lambda *args: traffic.append(args), lambda name: True,
                         host='127.0.0.1', port=0, interval=.05) for i in range(3)]
    try:
        # Larger than the failed DDS samples: 24 MiB, byte-for-byte verified.
        data = bytes(range(256)) * (96*1024)
        for i, stream in enumerate(streams):
            for j, other in enumerate(streams):
                if i != j:
                    stream.peer(str(j), other.endpoint())
            stream.snapshot(data)
        deadline = time.monotonic()+15
        while time.monotonic() < deadline and any(len(g) < 2 for g in got):
            time.sleep(.02)
        assert all(len(g) >= 2 for g in got)
        assert all(d == data for g in got for d in g)
        assert any(not row[-1] for row in traffic)
        assert all(row[3] == row[2]+8 for row in traffic)
    finally:
        for stream in streams:
            stream.close()


def test_stream_deltas_resynchronize_after_lost_ack():
    received, traffic = [], []
    reject = [True]
    fusion = SemanticMapFusion()
    def receive(data):
        msg = deserialize_message(data, SemanticVoxelMap)
        arr = point_cloud2.read_points_numpy(msg.cloud)
        fusion.update('robot_1', arr[:,:3], arr[:,3:], msg.labels, msg.cloud.header.stamp.sec)
        received.append(msg)
        if msg.is_delta and reject[0]:
            reject[0] = False
            raise ValueError('ACK lost after applying update')
        return 'robot_1'
    target = MapStream(receive, lambda *a: None, lambda name: True, host='127.0.0.1', port=0)
    sender = MapStream(lambda data: 'unused', lambda *a: traffic.append(a), lambda name: True,
                       host='127.0.0.1', port=0, interval=.05, encode=encode_delta)
    def wait_for(predicate):
        deadline = time.monotonic()+5
        while not predicate():
            assert time.monotonic() < deadline
            time.sleep(.01)
    try:
        sender.peer('target', target.endpoint())
        initial = np.array([[0,0,5,.8,.2], [1,0,5,.2,.8]], dtype=np.float32)
        sender.snapshot(serialize_message(message(initial)))
        wait_for(lambda: len(received) == 1)
        changed = np.vstack([initial, [2,0,5,.9,.1]]).astype(np.float32)
        sender.snapshot(serialize_message(message(changed, 2)))
        wait_for(lambda: len(received) >= 3)
        assert received[1].is_delta and received[1].cloud.width == 1
        assert not received[2].is_delta and received[2].cloud.width == 3
        xyz,scores = fusion.arrays()
        np.testing.assert_allclose(np.c_[xyz,scores], changed)
        assert any(not row[-1] for row in traffic)
    finally:
        sender.close()
        target.close()


def test_planner_requires_base_and_acknowledges_empty_updates():
    from types import SimpleNamespace as S
    from raven_nav.raven_nav_node import RavenNavNode
    acks = []
    matrix = np.array([[0,0,5,.8,.2], [1,0,5,.2,.8]], dtype=np.float32)
    raw, base = encode_delta(serialize_message(message(matrix)), None)
    full = deserialize_message(raw, SemanticVoxelMap)
    changed = matrix.copy()
    changed[0,3:] = [.7,.3]
    raw, newbase = encode_delta(serialize_message(message(changed,2)), base)
    delta = deserialize_message(raw, SemanticVoxelMap)
    n = S(_boot_enu=None, _robot_name='robot_2', _query_labels=list(full.labels),
          _semantic_fusion=SemanticMapFusion(), _semantic_map_received_at={},
          _semantic_map_updates={}, _semantic_map_rejections=0,
          _semantic_map_ack_pub=S(publish=acks.append), _refresh_semantic_map=lambda: None,
          get_logger=lambda: S(error=lambda *a,**k: (_ for _ in ()).throw(AssertionError(str(a)))))
    RavenNavNode._on_stream_semantic_map(n, full)
    assert not acks  # not ready: the transport must resync later
    n._boot_enu = np.zeros(3)
    RavenNavNode._on_stream_semantic_map(n, delta)
    assert not acks  # restarted planner has no base for the delta
    RavenNavNode._on_stream_semantic_map(n, full)
    RavenNavNode._on_stream_semantic_map(n, delta)
    assert len(acks) == 2
    raw, _ = encode_delta(serialize_message(message(changed,3)), newbase)
    empty = deserialize_message(raw, SemanticVoxelMap)
    assert empty.cloud.width == 0
    RavenNavNode._on_stream_semantic_map(n, empty)
    assert len(acks) == 3 and n._semantic_map_updates['robot_1'] == 3

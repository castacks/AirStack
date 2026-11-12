"""Run in the deployment image with /opt/rayfronts on PYTHONPATH."""
import threading
import time
from types import SimpleNamespace

import numpy as np

from semantic_search_task.query_delivery import QueryDelivery


def test_snapshot_acknowledgement(tmp_path, monkeypatch):
    monkeypatch.setenv('RAVEN_MAP_SNAPSHOT_DIR', str(tmp_path))
    delivery = QueryDelivery('robot_1', ['ship', 'sidewalk', 'ship'], interval_s=5)
    assert delivery.missing_query(0) is None  # Mapper not ready.
    np.savez(delivery.path, query_labels=['ship'])
    assert delivery.missing_query(4) is None
    assert delivery.missing_query(5) == 'sidewalk'
    assert delivery.missing_query(6) is None
    np.savez(delivery.path, query_labels=['sidewalk', 'ship'])
    assert delivery.missing_query(10) is None  # Order-independent acknowledgement.
    np.savez(delivery.path, query_labels=['ship'])  # Mapper restart.
    assert delivery.missing_query(15) == 'sidewalk'


def test_real_mapper_receiver_recovers_overwritten_queries(tmp_path, monkeypatch):
    """Real upstream depth-5 ROS subscriber and add_queries, delayed test encoder.

    Proves DDS history loss under callback load, then repairs only missing labels.
    Encoder delay is synthetic; ROS receiver and upstream deduplication are real.
    """
    import torch
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from rayfronts.mapping_server import MappingServer
    from rayfronts.messaging_services.ros import Ros2MessagingService

    labels = [f'label_{i:02}' for i in range(45)]

    class Encoder:
        def encode_labels(self, queries):
            time.sleep(0.25)
            return torch.tensor([[float(labels.index(q)), 1.0] for q in queries])

    server = MappingServer.__new__(MappingServer)
    server.encoder = Encoder()
    server.cfg = SimpleNamespace(querying=SimpleNamespace(
        text_query_mode='labels', compressed=False))
    server.feat_compressor = None
    server._query_lock = threading.RLock()
    server._queries_labels_history = set()
    server._queries_feats = None
    server._queries_labels = None
    server._queries_updated = False
    receiver = Ros2MessagingService('/query_delivery_test', server.add_queries)
    node = Node('query_delivery_sender')
    publisher = node.create_publisher(String, '/query_delivery_test', 10)
    try:
        deadline = time.monotonic() + 5
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        assert publisher.get_subscription_count() == 1
        time.sleep(0.3)
        for label in labels:
            publisher.publish(String(data=label))
            time.sleep(0.1)  # Existing task cadence.
        time.sleep(2.0)  # Drain depth-5 receiver, including its active callback.
        with server._query_lock:
            before = list(server._queries_labels['text'])
        print('Initial burst encoded', len(before), 'of', len(labels), flush=True)
        assert len(before) < len(labels), 'Fixture must reproduce receiver history loss'

        monkeypatch.setenv('RAVEN_MAP_SNAPSHOT_DIR', str(tmp_path))
        delivery = QueryDelivery('robot_1', labels, interval_s=0.4)
        retries = []
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            with server._query_lock:
                observed = list(server._queries_labels['text'])
            np.savez(delivery.path, query_labels=observed)
            missing = delivery.missing_query(time.monotonic())
            if missing is not None:
                assert missing not in observed
                retries.append(missing)
                publisher.publish(String(data=missing))
            if set(observed) == set(labels):
                break
            time.sleep(0.05)
        with server._query_lock:
            final = list(server._queries_labels['text'])
            features = server._queries_feats['text'].numpy()
        assert set(final) == set(labels) and len(final) == len(labels)
        np.testing.assert_array_equal(features,
            np.array([[float(labels.index(q)), 1.0] for q in final], dtype=np.float32))
        print('Recovered all45 unique labels; retries', len(retries), flush=True)
    finally:
        receiver._ros_executor.shutdown(timeout_sec=2)
        receiver._spin_thread.join(timeout=2)
        receiver._rosnode.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

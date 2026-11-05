"""Count serialized application traffic at gossip publish/receive boundaries.

These bytes exclude RTPS/TCP/Ethernet headers and retransmissions. A transmit
is counted once, never multiplied by the number of peers. Actual receptions
are counted separately, before range filtering and application deduplication.
"""
import json
import time
import threading


def payload_sizes(msg):
    result = {}
    for p in msg.payloads:
        result[p.payload_name] = result.get(p.payload_name, 0) + len(p.payload_data)
    return result


def completed_only_payload(name, msg):
    """Remove unvisited BB geometry from both possible gossip paths."""
    if name not in ('confirmed_targets', 'auction_table'):
        return msg
    from std_msgs.msg import String
    data = json.loads(msg.data)
    if name == 'confirmed_targets':
        data = [box for box in data if str(box.get('status', '')).lower() == 'visited']
    else:
        data['available'] = [t for t in data.get('available', [])
                             if str(t.get('status', '')).startswith('ray')]
    return String(data=json.dumps(data))


class BandwidthRecorder:
    def __init__(self, path, robot):
        self.robot = robot
        self.file = open(path, 'a', buffering=1)
        self._lock = threading.Lock()

    def _write(self, row):
        with self._lock:
            self.file.write(json.dumps(row) + '\n')

    def record(self, direction, msg, serialized_bytes, sim_ns, ros_ns):
        self._write(dict(
            direction=direction, robot=self.robot, source=msg.robot_name,
            wall_ns=time.time_ns(), sim_ns=sim_ns, ros_ns=ros_ns,
            source_stamp_ns=(msg.gps_fix.header.stamp.sec * 1_000_000_000
                             + msg.gps_fix.header.stamp.nanosec),
            source_kind=int(msg.source), relay_hops=int(msg.relay_hops),
            transport='dds_gossip', cdr_bytes=serialized_bytes, payload_bytes=payload_sizes(msg)))

    def record_map(self, direction, peer, payload_bytes, stream_bytes, complete, sim_ns, ros_ns):
        self._write(dict(direction=direction, robot=self.robot, source=(self.robot if direction == 'tx' else peer),
            peer=peer, wall_ns=time.time_ns(), sim_ns=sim_ns, ros_ns=ros_ns,
            source_stamp_ns=0, source_kind=0, relay_hops=0, transport='tcp_map',
            cdr_bytes=payload_bytes, payload_bytes={'semantic_voxel_map': payload_bytes},
            stream_bytes=stream_bytes, complete=complete))

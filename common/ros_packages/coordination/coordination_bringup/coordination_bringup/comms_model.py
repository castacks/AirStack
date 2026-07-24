"""Communication-range model for the gossip layer.

Simulates a unit-disk radio: a transmission is heard iff the last-hop transmitter
is within COMMS_RANGE_M. Multi-hop relaying (up to MAX_RELAY_HOPS) extends reach;
relay loops/storms are prevented by per-origin stamp deduplication in
gossip_node, not by the hop limit (which only bounds scope).
"""

import os

from coordination_bringup.frame_utils import gps_to_enu

COMMS_RANGE_M = 50.0#15.0
MAX_RELAY_HOPS = max(0, int(os.environ.get("NUM_ROBOTS", "1")) - 1)

SOURCE_RELAYED = 1


def _last_hop_xy(msg):
    if msg.source == SOURCE_RELAYED:
        x, y, _ = gps_to_enu(msg.relay_lat, msg.relay_lon, 0.0)
    else:
        x, y, _ = gps_to_enu(msg.gps_fix.latitude, msg.gps_fix.longitude,
                             msg.gps_fix.altitude)
    return x, y


def should_accept(msg, rx_xy, range_m=COMMS_RANGE_M):
    if msg.source != SOURCE_RELAYED and msg.gps_fix.status.status < 0:
        return True
    tx, ty = _last_hop_xy(msg)
    dx, dy = tx - rx_xy[0], ty - rx_xy[1]
    return dx * dx + dy * dy <= range_m * range_m

"""Communication-range model for the gossip layer.

Simulates an isotropic radio: a transmission is heard iff the receiver lies
inside the sphere of radius COMMS_RANGE_M centred on the last-hop transmitter.
Range is Euclidean in 3-D ENU metres, so altitude separation spends the same
budget as ground distance (a peer 60 m overhead is as far away as one 60 m
downrange). Multi-hop relaying (up to MAX_RELAY_HOPS) extends reach; relay
loops/storms are prevented by per-origin stamp deduplication in gossip_node,
not by the hop limit (which only bounds scope).
"""

import os

from coordination_bringup.frame_utils import gps_to_enu

COMMS_RANGE_M = 5000#50.0#15.0
MAX_RELAY_HOPS = 1 #max(0, int(os.environ.get("NUM_ROBOTS", "1")) - 1)

SOURCE_RELAYED = 1


def _last_hop_xyz(msg):
    """ENU (x, y, z) of whoever put this message on the air last: the relay for
    a forwarded copy, the originator for a direct one."""
    if msg.source == SOURCE_RELAYED:
        return gps_to_enu(msg.relay_lat, msg.relay_lon, msg.relay_alt)
    return gps_to_enu(msg.gps_fix.latitude, msg.gps_fix.longitude,
                      msg.gps_fix.altitude)


def should_accept(msg, rx_xyz, range_m=COMMS_RANGE_M):
    """True iff the receiver at ENU rx_xyz=(x, y, z) is within range_m of the
    last-hop transmitter. Direct messages from a robot with no GPS fix are
    accepted unconditionally — there is no position to range against."""
    if msg.source != SOURCE_RELAYED and msg.gps_fix.status.status < 0:
        return True
    tx, ty, tz = _last_hop_xyz(msg)
    dx, dy, dz = tx - rx_xyz[0], ty - rx_xyz[1], tz - rx_xyz[2]
    return dx * dx + dy * dy + dz * dz <= range_m * range_m

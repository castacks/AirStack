"""
gcs_utils.py
============
Shared utilities for GCS visualizer nodes.

Coordinate system
-----------------
All visualizers transform data from each robot's local odometry frame into
a common ENU (East-North-Up) frame anchored at the first GPS fix seen on the
ground (``alt_ground``). This ENU frame is published as the ``map`` frame in
Foxglove.

The key concept is the **boot offset**: the ENU coordinates of a robot's first
GPS fix, which equals that robot's odometry frame origin. Any data expressed in
the robot's local odometry frame must be translated by this boot offset to
display correctly alongside other robots in the global map frame.
"""

import math
import copy
import struct

# Shared frame utilities live in coordination_bringup so both robot and GCS
# packages can import them.  Re-export the ones callers expect from here.
from coordination_bringup.frame_utils import (
    gps_to_enu as _gps_to_enu_abs,
    heading_to_quat,
    rotate_vector,
    transform_marker_array,
    transform_point_cloud2 as _transform_pc2,
)

# ── Origin ────────────────────────────────────────────────────────────────────
ORIGIN_LAT = 38.736832
ORIGIN_LON = -9.137977
ORIGIN_ALT = 90.0


# ── GPS / coordinate helpers ──────────────────────────────────────────────────

def gps_to_enu(lat, lon, alt, alt_ground):
    """Convert GPS lat/lon/alt to ENU metres relative to the world origin.

    z is relative to ``alt_ground`` (the altitude of the first GPS fix seen).
    """
    x = (lon - ORIGIN_LON) * 111320.0 * math.cos(math.radians(ORIGIN_LAT))
    y = (lat - ORIGIN_LAT) * 111320.0
    z = alt - alt_ground
    return x, y, z


def multiply_quaternions(q1, q2):
    """Hamilton product q1 * q2 (apply q2 first, then q1). Both are (x,y,z,w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def point_cloud2_to_cube_marker(cloud, bx, by, bz, ns, marker_id, stamp, lifetime,
                                fallback_color=None, scale=0.2):
    """Convert a PointCloud2 to a CUBE_LIST Marker translated into the map frame.

    Extracts x, y, z fields and applies the boot offset (bx, by, bz).
    If the cloud has an ``rgb`` field (PCL-packed uint32: 0x00RRGGBB stored as
    float32 bytes), per-point colors are written to ``colors[]``.
    Otherwise ``fallback_color`` (r, g, b, a) is used as a uniform color.

    pose.orientation.w is set to 1.0 — required for CUBE_LIST so the per-point
    positions are not distorted by an invalid default quaternion.

    Returns a Marker, or None if the cloud has no x/y/z fields.
    """
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point as GPoint
    from std_msgs.msg import ColorRGBA

    field_map = {f.name: f.offset for f in cloud.fields}
    if not all(k in field_map for k in ('x', 'y', 'z')):
        return None

    ox, oy, oz = field_map['x'], field_map['y'], field_map['z']
    o_rgb = field_map.get('rgb')
    ps = cloud.point_step
    n_points = cloud.width * cloud.height
    data = bytes(cloud.data)

    m = Marker()
    m.header.frame_id = 'map'
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.pose.orientation.w = 1.0   # identity — must be set or CUBE_LIST distorts positions
    m.type = Marker.CUBE_LIST
    m.action = Marker.ADD
    m.scale.x = scale
    m.scale.y = scale
    m.scale.z = scale
    m.lifetime = lifetime

    for idx in range(n_points):
        base = idx * ps
        x, = struct.unpack_from('<f', data, base + ox)
        y, = struct.unpack_from('<f', data, base + oy)
        z, = struct.unpack_from('<f', data, base + oz)
        m.points.append(GPoint(x=x + bx, y=y + by, z=z + bz))
        if o_rgb is not None:
            packed, = struct.unpack_from('<I', data, base + o_rgb)
            r = ((packed >> 16) & 0xFF) / 255.0
            g = ((packed >> 8)  & 0xFF) / 255.0
            b = (packed         & 0xFF) / 255.0
            m.colors.append(ColorRGBA(r=r, g=g, b=b, a=1.0))

    if not m.colors:
        fc = fallback_color or (1.0, 1.0, 1.0, 1.0)
        m.color = ColorRGBA(r=fc[0], g=fc[1], b=fc[2], a=fc[3])

    return m


# transform_point_cloud2 is re-exported from coordination_bringup.frame_utils
transform_point_cloud2 = _transform_pc2

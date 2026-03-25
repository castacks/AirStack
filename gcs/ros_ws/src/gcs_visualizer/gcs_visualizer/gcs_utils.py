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

# ── Origin ────────────────────────────────────────────────────────────────────
# Matches Pegasus configs.yaml and gps_utils.py DEFAULT_WORLD_ORIGIN (Lisbon).
# Update this if your simulation/field site uses a different origin.
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


def rotate_vector(v, q):
    """Rotate vector v=(vx,vy,vz) by quaternion q=(x,y,z,w). Returns (x,y,z)."""
    vx, vy, vz = v
    qx, qy, qz, qw = q
    cx = qy * vz - qz * vy
    cy = qz * vx - qx * vz
    cz = qx * vy - qy * vx
    return (
        vx + 2.0 * (qw * cx + qy * cz - qz * cy),
        vy + 2.0 * (qw * cy + qz * cx - qx * cz),
        vz + 2.0 * (qw * cz + qx * cy - qy * cx),
    )


# ── Marker / PointCloud2 transform helpers ────────────────────────────────────

def transform_marker_array(marker_array, bx, by, bz, ns, id_base, stamp, lifetime):
    """Deep-copy a MarkerArray and translate all point coordinates by (bx, by, bz).

    Sets frame_id='map', the given namespace, sequential IDs starting at
    ``id_base``, and the provided stamp and lifetime on every marker.

    Only translates ``points[]`` — does NOT touch ``pose.position``, which is
    intentional: LINE_STRIP and ARROW markers encode geometry in ``points``.
    Returns a list of Marker messages.
    """
    from geometry_msgs.msg import Point

    result = []
    for k, orig in enumerate(marker_array.markers):
        m = copy.deepcopy(orig)
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.ns = ns
        m.id = id_base + k
        m.lifetime = lifetime
        for pt in m.points:
            pt.x += bx
            pt.y += by
            pt.z += bz
        result.append(m)
    return result


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


def transform_point_cloud2(cloud, bx, by, bz):
    """Return a copy of a PointCloud2 with all x,y,z values offset by (bx, by, bz).

    Assumes the cloud has float32 x, y, z fields (standard ROS convention).
    Reads field offsets from the message so non-standard field orderings work.
    Sets frame_id='map' on the returned message.
    """
    # Find byte offsets for x, y, z fields
    field_offsets = {}
    for f in cloud.fields:
        if f.name in ('x', 'y', 'z'):
            field_offsets[f.name] = f.offset

    if not all(k in field_offsets for k in ('x', 'y', 'z')):
        return cloud  # can't transform without x,y,z fields

    ox = field_offsets['x']
    oy = field_offsets['y']
    oz = field_offsets['z']

    new_cloud = copy.copy(cloud)
    new_cloud.header = copy.copy(cloud.header)
    new_cloud.header.frame_id = 'map'
    data = bytearray(cloud.data)
    n_points = cloud.width * cloud.height
    ps = cloud.point_step
    for i in range(n_points):
        base = i * ps
        x, = struct.unpack_from('<f', data, base + ox)
        y, = struct.unpack_from('<f', data, base + oy)
        z, = struct.unpack_from('<f', data, base + oz)
        struct.pack_into('<f', data, base + ox, x + bx)
        struct.pack_into('<f', data, base + oy, y + by)
        struct.pack_into('<f', data, base + oz, z + bz)
    new_cloud.data = bytes(data)
    return new_cloud

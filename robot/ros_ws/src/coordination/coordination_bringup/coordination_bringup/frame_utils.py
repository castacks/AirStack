"""
frame_utils.py
==============
Coordinate frame utilities shared between gossip_node (robot) and
gcs_visualizer (GCS).

Robots transform their payloads into global ENU before attaching them to
PeerProfile, so every consumer receives world-frame data directly.

ENU world origin (what GPS (0,0,0) maps to in metres):
  Must match gcs_utils.py ORIGIN_* and the simulation's GPS home position.
  Override via the 'enu_origin_lat/lon/alt' ROS parameters in gossip_node,
  or leave at the Lisbon default used everywhere else in AirStack.
"""

import copy
import math
import struct

# Default world origin — Lisbon (matches gcs_utils.py and gps_utils.py)
DEFAULT_ORIGIN_LAT = 38.736832
DEFAULT_ORIGIN_LON = -9.137977
DEFAULT_ORIGIN_ALT = 90.0


def gps_to_enu(lat, lon, alt,
               origin_lat=DEFAULT_ORIGIN_LAT,
               origin_lon=DEFAULT_ORIGIN_LON,
               origin_alt=DEFAULT_ORIGIN_ALT):
    """Convert GPS lat/lon/alt to ENU metres relative to the world origin."""
    x = (lon - origin_lon) * 111320.0 * math.cos(math.radians(origin_lat))
    y = (lat - origin_lat) * 111320.0
    z = alt - origin_alt
    return x, y, z


def heading_to_quat(heading_deg):
    """Compass heading (degrees CW from North) → ENU yaw quaternion (x,y,z,w).

    ENU: yaw=0 → East (+x), yaw=90° → North (+y).
    heading=0 (North) → yaw=90° → q=(0,0,sin45,cos45)
    heading=90 (East)  → yaw=0°  → q=(0,0,0,1)
    """
    yaw_enu = math.radians(90.0 - heading_deg)
    return (0.0, 0.0, math.sin(yaw_enu / 2.0), math.cos(yaw_enu / 2.0))


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


def transform_marker_array(marker_array, bx, by, bz, q=(0.0, 0.0, 0.0, 1.0)):
    """Deep-copy a MarkerArray and transform all points: p_map = R(q)*p + (bx,by,bz).

    Transforms every point in points[] (LINE_LIST, LINE_STRIP, CUBE_LIST,
    SPHERE_LIST, POINTS, ARROW-with-2-points, TRIANGLE_LIST).

    Note: pose.position is intentionally NOT translated here.  In the ROS
    marker convention, points[] are expressed in the marker's local frame
    (defined by pose).  These markers typically use an identity pose so that
    points[] are already in the parent frame; translating pose.position as
    well would double-apply the offset.  If you have single-geometry markers
    (SPHERE, CUBE, TEXT, MESH_RESOURCE) whose position lives in pose.position
    rather than points[], translate pose.position at the call site instead.

    Sets frame_id='map'.  Returns a new MarkerArray (does not modify in-place).
    """
    from visualization_msgs.msg import MarkerArray as MA
    out = MA()
    for orig in marker_array.markers:
        m = copy.deepcopy(orig)
        m.header.frame_id = 'map'
        for pt in m.points:
            rx, ry, rz = rotate_vector((pt.x, pt.y, pt.z), q)
            pt.x = rx + bx
            pt.y = ry + by
            pt.z = rz + bz
        out.markers.append(m)
    return out


def transform_point_cloud2(cloud, bx, by, bz, q=(0.0, 0.0, 0.0, 1.0)):
    """Return a copy of PointCloud2 with all xyz transformed: p_map = R(q)*p + (bx,by,bz).

    Sets frame_id='map'.  Reads field offsets from the message header so
    non-standard field orderings work.
    """
    field_offsets = {f.name: f.offset for f in cloud.fields if f.name in ('x', 'y', 'z')}
    if not all(k in field_offsets for k in ('x', 'y', 'z')):
        return cloud

    ox, oy, oz = field_offsets['x'], field_offsets['y'], field_offsets['z']
    ps = cloud.point_step
    n_points = cloud.width * cloud.height
    if ps == 0 or len(cloud.data) < n_points * ps:
        return cloud  # malformed cloud — skip rather than raise

    new_cloud = copy.copy(cloud)
    new_cloud.header = copy.copy(cloud.header)
    new_cloud.header.frame_id = 'map'
    data = bytearray(cloud.data)
    for i in range(n_points):
        base = i * ps
        x, = struct.unpack_from('<f', data, base + ox)
        y, = struct.unpack_from('<f', data, base + oy)
        z, = struct.unpack_from('<f', data, base + oz)
        rx, ry, rz = rotate_vector((x, y, z), q)
        struct.pack_into('<f', data, base + ox, rx + bx)
        struct.pack_into('<f', data, base + oy, ry + by)
        struct.pack_into('<f', data, base + oz, rz + bz)
    new_cloud.data = bytes(data)
    return new_cloud

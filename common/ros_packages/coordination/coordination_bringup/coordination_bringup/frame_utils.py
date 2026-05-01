"""Coordinate frame utilities shared between gossip_node (robot) and gcs_visualizer (GCS)."""

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

    ENU: yaw=0 → East (+x). heading=0 (North) → yaw=90° → q=(0,0,sin45,cos45).
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
    """Deep-copy a MarkerArray and transform all points[]: p_map = R(q)*p + (bx,by,bz).

    Transforms points[] only, not pose.position — LINE_STRIP/POINTS markers store
    geometry in points[] with an identity pose, so translating pose.position would
    double-apply the offset. Sets frame_id='map'. Returns a new MarkerArray.
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

    Sets frame_id='map'. Reads field offsets from the message header.
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

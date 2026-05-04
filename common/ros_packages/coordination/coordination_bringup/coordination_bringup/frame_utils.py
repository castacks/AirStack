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


def global_enu_to_local(point_xyz, boot_enu_xyz,
                        local_alt_ground=None,
                        gossip_origin_alt=DEFAULT_ORIGIN_ALT):
    """Inverse of gossip_node._publish_own translation.

    Converts a point in the gossip frame (global ENU, alt datum=DEFAULT_ORIGIN_ALT)
    into this robot's local 'map' frame (ENU relative to this robot's boot GPS,
    z=0 at this robot's ground).

    point_xyz:        (x, y, z) in gossip frame.
    boot_enu_xyz:     this robot's boot ENU position (gps_to_enu of first fix).
    local_alt_ground: this robot's first-fix MSL altitude. If None, only
                      subtracts boot z (only correct when both robots booted at
                      exactly the same altitude).
    """
    import numpy as np
    bx, by, bz = boot_enu_xyz
    px, py, pz = point_xyz
    lx = px - bx
    ly = py - by
    if local_alt_ground is None:
        lz = pz - bz
    else:
        # gossip used: gossip_z = msl_alt - DEFAULT_ORIGIN_ALT
        # so absolute MSL = gossip_z + DEFAULT_ORIGIN_ALT
        # local AGL = absolute MSL - local ground
        lz = pz + gossip_origin_alt - local_alt_ground
    return np.array([lx, ly, lz], dtype=np.float64)


def global_enu_to_local_batch(points_Nx3, boot_enu_xyz,
                              local_alt_ground=None,
                              gossip_origin_alt=DEFAULT_ORIGIN_ALT):
    """Vectorized global_enu_to_local. Accepts (N,3) ndarray, returns (N,3)."""
    import numpy as np
    pts = np.asarray(points_Nx3, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"points_Nx3 must be (N,3), got {pts.shape}")
    bx, by, bz = boot_enu_xyz
    out = np.empty_like(pts)
    out[:, 0] = pts[:, 0] - bx
    out[:, 1] = pts[:, 1] - by
    if local_alt_ground is None:
        out[:, 2] = pts[:, 2] - bz
    else:
        out[:, 2] = pts[:, 2] + gossip_origin_alt - local_alt_ground
    return out


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

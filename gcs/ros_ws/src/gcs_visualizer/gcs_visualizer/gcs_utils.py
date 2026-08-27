"""Shared utilities for GCS visualizer nodes."""

import colorsys
import math
import copy
import struct

from coordination_bringup.frame_utils import (
    gps_to_enu as _gps_to_enu_abs,
    heading_to_quat,
    rotate_vector,
    transform_marker_array,
    transform_point_cloud2 as _transform_pc2,
)

ORIGIN_LAT = 38.736832
ORIGIN_LON = -9.137977
ORIGIN_ALT = 90.0

ROBOT_COLORS = [
    (0.90, 0.10, 0.10),
    (0.10, 0.70, 0.20),
    (0.20, 0.40, 1.00),
    (1.00, 0.55, 0.00),
    (0.70, 0.30, 0.90),
    (0.00, 0.80, 0.85),
    (1.00, 0.85, 0.10),
    (1.00, 0.40, 0.70),
    (0.40, 0.80, 0.40),
    (0.55, 0.27, 0.07),
    (0.30, 0.30, 0.30),
    (0.95, 0.95, 0.95),
]


def gps_to_enu(lat, lon, alt, alt_ground):
    """Convert GPS lat/lon/alt to ENU metres. z is relative to alt_ground."""
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

    Extracts xyz and applies boot offset (bx, by, bz). If the cloud has an 'rgb'
    field (PCL-packed uint32 0x00RRGGBB stored as float32 bytes), per-point colors
    are written to colors[]; otherwise fallback_color (r,g,b,a) is used.

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
    m.pose.orientation.w = 1.0  # must be set or CUBE_LIST distorts positions
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


# Low end of the voxel similarity gradient — dark purple #54005e. Keep in sync
# with render_layout.py VOXEL_GRADIENT_LOW (styles the voxels_sim/all cloud).
VOXEL_GRADIENT_LOW = (0x54 / 255, 0.0, 0x5e / 255)


def fluorescent(color):
    """Fluorescent take on a color: full value, boosted saturation. Keep in
    sync with render_layout.py _robot_color_hex."""
    h, s, _ = colorsys.rgb_to_hsv(*color)
    return colorsys.hsv_to_rgb(h, min(1.0, s * 1.25), 1.0)


def _voxel_sim_points(cloud, bx, by, bz, q, robot_color, sim_min, max_voxels):
    """Extract (xyz, rgb) arrays from a rayfronts per-query (x, y, z, sim)
    PointCloud2: rotate xyz by quaternion q (x, y, z, w), translate by the boot
    offset (bx, by, bz), and color on a VOXEL_GRADIENT_LOW → robot_color
    gradient with sim mapped over [sim_min, 1.0] (clamped). If the cloud has
    more than max_voxels points (and max_voxels > 0), keep the top-max_voxels
    by sim. Returns (xyz Nx3, rgb Nx3, total points) or None if the cloud
    lacks x/y/z/sim fields."""
    import numpy as np

    field_map = {f.name: f.offset for f in cloud.fields}
    if not all(k in field_map for k in ('x', 'y', 'z', 'sim')):
        return None

    n = cloud.width * cloud.height
    dt = np.dtype({'names': ['x', 'y', 'z', 'sim'],
                   'formats': [np.float32] * 4,
                   'offsets': [field_map[k] for k in ('x', 'y', 'z', 'sim')],
                   'itemsize': cloud.point_step})
    rec = np.frombuffer(bytes(cloud.data), dtype=dt, count=n)
    sim = rec['sim'].astype(np.float64)
    xyz = np.stack([rec['x'], rec['y'], rec['z']], axis=1).astype(np.float64)

    if max_voxels and n > max_voxels:
        keep = np.argpartition(sim, -max_voxels)[-max_voxels:]
        sim, xyz = sim[keep], xyz[keep]

    qx, qy, qz, qw = q
    rot = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])
    xyz = xyz @ rot.T + (bx, by, bz)

    span = max(1.0 - sim_min, 1e-6)
    t = np.clip((sim - sim_min) / span, 0.0, 1.0)
    rgb = np.outer(1.0 - t, VOXEL_GRADIENT_LOW) + np.outer(t, robot_color)
    return xyz, rgb, n


def voxel_sim_cloud_to_scene_update(cloud, bx, by, bz, q, entity_id, stamp,
                                    robot_color, sim_min, scale, max_voxels=0):
    """Convert a rayfronts per-query PointCloud2 into a foxglove_msgs
    SceneUpdate of cube primitives in the global map frame (see
    _voxel_sim_points for the transform/gradient/cap semantics). Foxglove
    outlines scene-entity cubes by default, unlike CUBE_LIST markers.

    Returns (SceneUpdate, total points), or None if fields are missing."""
    from foxglove_msgs.msg import SceneUpdate, SceneEntity, CubePrimitive, Color
    from geometry_msgs.msg import Pose, Point as GPoint, Quaternion, Vector3

    res = _voxel_sim_points(cloud, bx, by, bz, q, robot_color, sim_min, max_voxels)
    if res is None:
        return None
    xyz, rgb, total = res

    entity = SceneEntity()
    entity.timestamp = stamp
    entity.frame_id = 'map'
    entity.id = entity_id
    size = Vector3(x=scale, y=scale, z=scale)
    entity.cubes = [
        CubePrimitive(
            pose=Pose(position=GPoint(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                      orientation=Quaternion(w=1.0)),
            size=size,
            color=Color(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0))
        for p, c in zip(xyz, rgb)]
    update = SceneUpdate()
    update.entities = [entity]
    return update, total


def voxel_sim_cloud_to_cube_marker(cloud, bx, by, bz, q, ns, stamp,
                                   robot_color, sim_min, scale, max_voxels=0):
    """CUBE_LIST-marker fallback for voxel_sim_cloud_to_scene_update, for GCS
    images without foxglove_msgs. Same visuals minus the cube outlines.

    Returns (Marker, total points), or None if fields are missing."""
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point as GPoint
    from std_msgs.msg import ColorRGBA

    res = _voxel_sim_points(cloud, bx, by, bz, q, robot_color, sim_min, max_voxels)
    if res is None:
        return None
    xyz, rgb, total = res

    m = Marker()
    m.header.frame_id = 'map'
    m.header.stamp = stamp
    m.ns = ns
    m.id = 0
    m.pose.orientation.w = 1.0  # must be set or CUBE_LIST distorts positions
    m.type = Marker.CUBE_LIST
    m.action = Marker.ADD
    m.scale.x = m.scale.y = m.scale.z = scale
    m.points = [GPoint(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in xyz]
    m.colors = [ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0)
                for c in rgb]
    return m, total


transform_point_cloud2 = _transform_pc2


# ── robot-local `map` -> GCS global-ENU `map` ────────────────────────────────
#
# Every robot's `map` is anchored at ITS OWN TAKEOFF POINT (PX4's local origin
# comes from the GPS home, which the sim derives from the spawn pose). The
# GCS's `map` is global ENU — the frame the sim ground, the GT boxes and the
# robot meshes are drawn in. A robot-local message drawn as-is on the GCS
# therefore lands at the world origin instead of next to the drone, offset by
# exactly the spawn point. These two helpers are the pure-translation versions
# of `transform_marker_array` for the message types the search planner
# publishes; both are vectorised because `voxel_map` carries up to 120k points
# at ~1 Hz and `frame_utils.transform_point_cloud2` unpacks per point.

def translate_point_cloud2(cloud, bx, by, bz):
    """Return a copy of `cloud` with x/y/z shifted by (bx, by, bz), frame 'map'.

    Only the three float32 coordinate fields are touched; rgb / intensity /
    anything else passes through byte-for-byte. Falls back to the original
    message if the layout is not the plain float32 xyz the planner emits.
    """
    import numpy as np

    offs = {f.name: (f.offset, f.datatype) for f in cloud.fields
            if f.name in ('x', 'y', 'z')}
    if len(offs) != 3 or any(dt != 7 for _, dt in offs.values()):   # 7 = FLOAT32
        return cloud
    n = int(cloud.width) * int(cloud.height)
    ps = int(cloud.point_step)
    if n == 0 or ps == 0 or len(cloud.data) < n * ps:
        return cloud
    buf = np.frombuffer(bytes(cloud.data), dtype=np.uint8)[:n * ps].copy()
    view = buf.reshape(n, ps)
    for name, delta in (('x', bx), ('y', by), ('z', bz)):
        o = offs[name][0]
        col = view[:, o:o + 4].view(np.float32).reshape(n)
        col += np.float32(delta)          # in place: `col` is a view into buf
    out = copy.copy(cloud)
    out.header = copy.copy(cloud.header)
    out.header.frame_id = 'map'
    out.data = buf.tobytes()
    return out


def translate_occupancy_grid(grid, bx, by, bz):
    """Return a copy of an OccupancyGrid whose origin is shifted by (bx, by, bz).

    A grid is positioned entirely by `info.origin`, so this is the whole
    transform; the cell data is shared, not copied.
    """
    out = copy.copy(grid)
    out.header = copy.copy(grid.header)
    out.header.frame_id = 'map'
    out.info = copy.copy(grid.info)
    out.info.origin = copy.deepcopy(grid.info.origin)
    out.info.origin.position.x += bx
    out.info.origin.position.y += by
    out.info.origin.position.z += bz
    return out

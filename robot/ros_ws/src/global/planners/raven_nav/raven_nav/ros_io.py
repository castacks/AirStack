"""The one module that knows about ROS message types.

`raven_nav.behaviors.*`, `coverage`, `detection_memory`, `results`, `params`
and `lvlm_client` are all pure Python + numpy so they can be tested on a host
without ROS. Everything that builds or parses a message lives here, behind
guarded imports: `import raven_nav.ros_io` succeeds without ROS installed and
each helper raises a clear error if actually called.
"""
from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

HAVE_ROS = True
_IMPORT_ERROR = ''
try:  # pragma: no cover - exercised implicitly in the container
    from std_msgs.msg import ColorRGBA, Header
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import Path
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
    from visualization_msgs.msg import Marker, MarkerArray
except Exception as _exc:  # noqa: BLE001
    HAVE_ROS = False
    _IMPORT_ERROR = f'{type(_exc).__name__}: {_exc}'


def _require_ros() -> None:
    if not HAVE_ROS:
        raise RuntimeError(
            f'raven_nav.ros_io needs ROS 2 message packages ({_IMPORT_ERROR})')


# ── inbound: PointCloud2 -> numpy ───────────────────────────────────────────
def sim_field_names(msg) -> List[str]:
    """`sim_0..sim_{Q-1}` in column order."""
    names = [f.name for f in msg.fields]
    return sorted([n for n in names if n.startswith('sim_')],
                  key=lambda s: int(s.split('_', 1)[1]))


def _read(msg, fields) -> Optional[np.ndarray]:
    _require_ros()
    pts = list(point_cloud2.read_points(msg, field_names=tuple(fields),
                                        skip_nans=True))
    if not pts:
        return None
    return np.array([list(p) for p in pts], dtype=np.float64)


def rdf_to_flu(a: np.ndarray) -> np.ndarray:
    """(N,3) RDF -> FLU. Same permutation the OG behaviours applied inline."""
    return np.stack([a[:, 2], -a[:, 0], -a[:, 1]], axis=1)


def parse_ray_cloud(msg):
    """rays_sim/all fields: x,y,z,theta,phi,sim_0..  -> (origins, dirs, scores)
    in FLU. theta/phi are degrees in the mapper's RDF frame."""
    sims = sim_field_names(msg)
    if not sims:
        return None, None, None
    arr = _read(msg, ('x', 'y', 'z', 'theta', 'phi') + tuple(sims))
    if arr is None:
        return None, None, None
    rdf_orig = arr[:, :3]
    theta = np.deg2rad(arr[:, 3])
    phi = np.deg2rad(arr[:, 4])
    rdf_dirs = np.stack([np.cos(theta) * np.sin(phi),
                         np.sin(theta) * np.sin(phi),
                         np.cos(phi)], axis=1)
    return rdf_to_flu(rdf_orig), rdf_to_flu(rdf_dirs), arr[:, 5:]


def parse_voxel_cloud(msg):
    """voxels_sim/all fields: x,y,z,sim_0..  -> (xyz_flu, scores)."""
    sims = sim_field_names(msg)
    if not sims:
        return None, None
    arr = _read(msg, ('x', 'y', 'z') + tuple(sims))
    if arr is None:
        return None, None
    return rdf_to_flu(arr[:, :3]), arr[:, 3:]


def parse_frontier_cloud(msg) -> Optional[np.ndarray]:
    """(N,6) [x,y,z,empty_cnt,unobserved_cnt,occupied_cnt]; counts 0-filled when
    the message lacks them. Columns 0:3 stay RDF — the frontier behaviour flips
    them itself, exactly as the OG did."""
    names = {f.name for f in msg.fields}
    has_cnts = {'empty_cnt', 'unobserved_cnt', 'occupied_cnt'} <= names
    cols = (('x', 'y', 'z', 'empty_cnt', 'unobserved_cnt', 'occupied_cnt')
            if has_cnts else ('x', 'y', 'z'))
    arr = _read(msg, cols)
    if arr is None:
        return None
    if has_cnts:
        return arr
    padded = np.zeros((arr.shape[0], 6), dtype=np.float64)
    padded[:, :3] = arr
    return padded


def image_to_bgr(msg) -> Optional[np.ndarray]:
    """sensor_msgs/Image -> BGR uint8, without cv_bridge (which pulls OpenCV's
    ROS wrapper into the tick). Returns None for an encoding we do not handle."""
    enc = str(msg.encoding).lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    h, w = int(msg.height), int(msg.width)
    if h <= 0 or w <= 0:
        return None
    if enc in ('rgb8', 'bgr8'):
        if buf.size < h * w * 3:
            return None
        img = buf[:h * w * 3].reshape(h, w, 3)
        return img[:, :, ::-1].copy() if enc == 'rgb8' else img.copy()
    if enc in ('rgba8', 'bgra8'):
        if buf.size < h * w * 4:
            return None
        img = buf[:h * w * 4].reshape(h, w, 4)[:, :, :3]
        return img[:, :, ::-1].copy() if enc == 'rgba8' else img.copy()
    if enc == 'mono8':
        if buf.size < h * w:
            return None
        g = buf[:h * w].reshape(h, w)
        return np.stack([g, g, g], axis=2)
    return None


# ── outbound: numpy -> messages ─────────────────────────────────────────────
def make_header(stamp, frame_id: str = 'map'):
    _require_ros()
    hdr = Header()
    hdr.stamp = stamp
    hdr.frame_id = frame_id
    return hdr


def make_path(stamp, points: Sequence[np.ndarray], frame_id: str = 'map'):
    _require_ros()
    path = Path()
    path.header = make_header(stamp, frame_id)
    for p in points:
        ps = PoseStamped()
        ps.header = make_header(stamp, frame_id)
        ps.pose.position.x = float(p[0])
        ps.pose.position.y = float(p[1])
        ps.pose.position.z = float(p[2])
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path


def make_xyz_cloud(stamp, xyz: np.ndarray, frame_id: str = 'map'):
    """PointCloud2 with x/y/z float32 fields — the OG
    `create_pointcloud2_msg` (frontier_behavior.py:112-129)."""
    _require_ros()
    fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    return point_cloud2.create_cloud(make_header(stamp, frame_id), fields,
                                     [[float(x), float(y), float(z)]
                                      for x, y, z in xyz])


# OG ray_behavior.py:197-207
ARROW_COLORS = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
                (1.0, 1.0, 0.0), (0.0, 1.0, 1.0), (1.0, 0.0, 1.0),
                (0.5, 0.5, 0.5), (1.0, 0.5, 0.0), (0.5, 0.0, 1.0),
                (0.0, 0.5, 0.5))
ARROW_LENGTH_M = 2.0        # OG ray_behavior.py:195


def make_ray_markers(stamp, origins: np.ndarray, dirs: np.ndarray,
                     group_ids, prev_count: int, frame_id: str = 'map'):
    """(MarkerArray, new_count). Deletes last tick's ids first, exactly like
    OG `clear_filtered_rays` (ray_behavior.py:238-249)."""
    _require_ros()
    arr = MarkerArray()
    for i in range(int(prev_count)):
        m = Marker()
        m.header = make_header(stamp, frame_id)
        m.ns = 'arrows'
        m.id = i
        m.action = Marker.DELETE
        arr.markers.append(m)
    j = 0
    origins = np.asarray(origins, dtype=np.float64).reshape(-1, 3)
    dirs = np.asarray(dirs, dtype=np.float64).reshape(-1, 3)
    for i in range(origins.shape[0]):
        gid = int(group_ids[i]) if group_ids is not None else 0
        if gid < 0:
            continue
        rr, gg, bb = ARROW_COLORS[gid % len(ARROW_COLORS)]
        p0 = origins[i]
        p1 = p0 + ARROW_LENGTH_M * dirs[i]
        m = Marker()
        m.header = make_header(stamp, frame_id)
        m.ns = 'arrows'
        m.id = j
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.points = [Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])),
                    Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2]))]
        m.scale.x, m.scale.y, m.scale.z = 0.6, 1.2, 0.75
        m.color.r, m.color.g, m.color.b, m.color.a = rr, gg, bb, 0.5
        arr.markers.append(m)
        j += 1
    return arr, j


def make_cluster_markers(stamp, clusters, prev_count: int,
                         frame_id: str = 'map'):
    """(MarkerArray, new_count) of green translucent CUBEs — OG
    `visualize_voxel_cluster_bbox` (voxel_behavior.py:202-234)."""
    _require_ros()
    arr = MarkerArray()
    for i in range(int(prev_count)):
        m = Marker()
        m.header = make_header(stamp, frame_id)
        m.ns = 'ccl_boxes'
        m.id = i
        m.action = Marker.DELETE
        arr.markers.append(m)
    j = 0
    for c in clusters:
        m = Marker()
        m.header = make_header(stamp, frame_id)
        m.ns = 'ccl_boxes'
        m.id = j
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(c.center[0])
        m.pose.position.y = float(c.center[1])
        m.pose.position.z = float(c.center[2])
        m.pose.orientation.w = 1.0
        m.scale.x = float(c.size[0])
        m.scale.y = float(c.size[1])
        m.scale.z = float(c.size[2])
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.2)
        m.lifetime.sec = 1
        arr.markers.append(m)
        j += 1
    return arr, j


def make_coverage_grid(packed) -> object:
    """`coverage.CoverageTracker.packed_grid()` -> coordination_msgs/CoverageGrid."""
    from coordination_msgs.msg import CoverageGrid  # noqa: PLC0415
    resolution, origin_x, origin_y, width, height, data = packed
    out = CoverageGrid()
    out.resolution = float(resolution)
    out.origin_x = float(origin_x)
    out.origin_y = float(origin_y)
    out.width = int(width)
    out.height = int(height)
    out.data = data
    return out

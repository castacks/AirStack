"""Pure stereo geometry preview, without teacher data or execution authority.

Operator regions identify pixels, not verified semantic objects. A returned point
describes a representative visible surface; it is never an approach waypoint or a
C02 ``localized`` fact. Only standard-library dependencies are used.
"""
from dataclasses import dataclass
import math
import statistics


@dataclass(frozen=True)
class PixelRegion:
    label: str
    x: int
    y: int
    width: int
    height: int
    image_sha256: str


@dataclass(frozen=True)
class StereoCalibration:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    baseline_m: float
    min_disparity: float
    max_disparity: float
    disparity_step: float
    valid_window: tuple[int, int, int, int]


@dataclass(frozen=True)
class SensorBinding:
    episode_id: str
    image_sha256: str
    image_stamp_ns: int
    right_image_stamp_ns: int
    calibration_stamp_ns: int
    disparity_stamp_ns: int
    received_monotonic_s: float


@dataclass(frozen=True)
class CameraMapPose:
    parent_frame: str
    child_frame: str
    source_stamp_ns: int
    translation_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class LocalizationPolicy:
    minimum_samples: int = 32
    minimum_region_fraction: float = 0.2
    maximum_relative_depth_mad: float = 0.15
    minimum_depth_m: float = 0.2
    maximum_depth_m: float = 30.0
    maximum_age_s: float = 1.0


def _finite(values):
    return all(type(value) in (int, float) and math.isfinite(value) for value in values)


def _hold(reason, **diagnostics):
    return dict(status='HOLD', reason=reason, execution_dispatch=False,
                evidence_scope='OPERATOR_REGION_GEOMETRY_ONLY', **diagnostics)


def _rotate(point, quaternion):
    x, y, z, w = quaternion
    px, py, pz = point
    tx, ty, tz = 2*(y*pz-z*py), 2*(z*px-x*pz), 2*(x*py-y*px)
    return (px+w*tx+y*tz-z*ty, py+w*ty+z*tx-x*tz, pz+w*tz+x*ty-y*tx)


def localize_region(disparity, *, region: PixelRegion, calibration: StereoCalibration,
                    binding: SensorBinding, pose: CameraMapPose, current_episode_id: str,
                    now_monotonic_s: float, policy: LocalizationPolicy = LocalizationPolicy()):
    """Return a sensor-only surface preview or an explicit no-authority HOLD."""
    if (not binding.episode_id or binding.episode_id != current_episode_id):
        return _hold('episode_mismatch')
    if (not isinstance(binding.image_sha256, str) or len(binding.image_sha256) != 64
            or any(c not in '0123456789abcdef' for c in binding.image_sha256)
            or region.image_sha256 != binding.image_sha256):
        return _hold('image_binding_mismatch')
    stamps = (binding.image_stamp_ns, binding.right_image_stamp_ns,
              binding.calibration_stamp_ns, binding.disparity_stamp_ns, pose.source_stamp_ns)
    if any(type(stamp) is not int or stamp <= 0 for stamp in stamps) or len(set(stamps)) != 1:
        return _hold('stamp_mismatch')
    if (pose.parent_frame != 'map' or pose.child_frame != 'camera_left'):
        return _hold('frame_mismatch')
    if (not _finite((now_monotonic_s, binding.received_monotonic_s, policy.maximum_age_s))
            or policy.maximum_age_s <= 0 or binding.received_monotonic_s < 0
            or not 0 <= now_monotonic_s-binding.received_monotonic_s <= policy.maximum_age_s):
        return _hold('stale_observation')
    if (not _finite((calibration.fx, calibration.fy, calibration.cx, calibration.cy,
                     calibration.baseline_m, calibration.min_disparity,
                     calibration.max_disparity, calibration.disparity_step))
            or min(calibration.fx, calibration.fy, calibration.baseline_m, calibration.disparity_step) <= 0
            or calibration.max_disparity <= max(0, calibration.min_disparity)):
        return _hold('invalid_calibration')
    if len(calibration.valid_window) != 4:
        return _hold('invalid_disparity_window')
    integers = (calibration.width, calibration.height, region.x, region.y,
                region.width, region.height, *calibration.valid_window)
    if any(type(value) is not int for value in integers):
        return _hold('invalid_dimensions')
    width, height = calibration.width, calibration.height
    if (width <= 0 or height <= 0 or len(disparity) != height
            or any(len(row) != width for row in disparity)):
        return _hold('image_shape_mismatch')
    if (not 0 <= calibration.cx < width or not 0 <= calibration.cy < height
            or not math.isfinite(calibration.fx*calibration.baseline_m)):
        return _hold('invalid_calibration')
    x, y, rw, rh = calibration.valid_window
    if x < 0 or y < 0 or rw <= 0 or rh <= 0 or x+rw > width or y+rh > height:
        return _hold('invalid_disparity_window')
    if (not region.label or region.x < 0 or region.y < 0 or region.width <= 0 or region.height <= 0
            or region.x+region.width > width or region.y+region.height > height):
        return _hold('invalid_pixel_region')
    if (len(pose.translation_m) != 3 or len(pose.quaternion_xyzw) != 4
            or not _finite((*pose.translation_m, *pose.quaternion_xyzw))
            or abs(sum(q*q for q in pose.quaternion_xyzw)-1) > 1e-6):
        return _hold('invalid_pose')
    if (type(policy.minimum_samples) is not int or policy.minimum_samples < 1
            or not _finite((policy.minimum_region_fraction, policy.maximum_relative_depth_mad,
                            policy.minimum_depth_m, policy.maximum_depth_m))
            or not 0 < policy.minimum_region_fraction <= 1 or policy.maximum_relative_depth_mad < 0
            or not 0 < policy.minimum_depth_m < policy.maximum_depth_m):
        return _hold('invalid_policy')
    samples = []
    for v in range(max(y,region.y), min(y+rh,region.y+region.height)):
        for u in range(max(x,region.x), min(x+rw,region.x+region.width)):
            try:
                value = float(disparity[v][u])
            except (TypeError,ValueError,OverflowError):
                return _hold('invalid_disparity_sample')
            if not math.isfinite(value) or not max(0,calibration.min_disparity) < value < calibration.max_disparity:
                continue
            z = calibration.fx*calibration.baseline_m/value
            if policy.minimum_depth_m <= z <= policy.maximum_depth_m:
                samples.append(((u-calibration.cx)*z/calibration.fx,
                                (v-calibration.cy)*z/calibration.fy, z))
    fraction = len(samples)/(region.width*region.height)
    metrics = dict(region_label=region.label, sample_count=len(samples), region_fraction=fraction)
    if len(samples) < policy.minimum_samples or fraction < policy.minimum_region_fraction:
        return _hold('insufficient_region_depth', **metrics)
    point = tuple(statistics.median(row[i] for row in samples) for i in range(3))
    if not _finite(point):
        return _hold('nonfinite_geometry', **metrics)
    depth_mad = statistics.median(abs(row[2]-point[2]) for row in samples)
    if depth_mad/point[2] > policy.maximum_relative_depth_mad:
        return _hold('incoherent_region_depth', median_depth_m=point[2], depth_mad_m=depth_mad, **metrics)
    rotated = _rotate(point,pose.quaternion_xyzw)
    map_point = tuple(rotated[i]+pose.translation_m[i] for i in range(3))
    if not _finite(map_point):
        return _hold('nonfinite_geometry', **metrics)
    return dict(status='SURFACE_POINT_PREVIEW', execution_dispatch=False,
                evidence_scope='OPERATOR_REGION_GEOMETRY_ONLY', association_verified=False,
                accuracy_validated=False, frame_id='map', camera_frame_id='camera_left',
                episode_id=binding.episode_id, image_sha256=binding.image_sha256,
                source_stamp_ns=binding.image_stamp_ns, camera_surface_point_m=point,
                map_surface_point_m=map_point,
                coordinate_mad_m=tuple(statistics.median(abs(row[i]-point[i]) for row in samples) for i in range(3)),
                disparity_half_bin_depth_m=point[2]**2/(calibration.fx*calibration.baseline_m)*calibration.disparity_step/2,
                **metrics)

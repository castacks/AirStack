"""Experimental stationary body-twist interval projection; never an admission.

Uses only a completed native motion and requested timestamps. Acquisition clock,
physical frame and controller deadline verification belong to a separate layer.
The principal rotation logarithm assumes no unobserved rotation winding.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation

from Scripts.Setup.arbiter_motion_output import interval, rigid


def se3(matrix):
    raw = matrix.tolist() if isinstance(matrix, np.ndarray) else matrix
    rigid(raw)
    return np.asarray(raw, dtype=np.float64)


def hat(w):
    x, y, z = w
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def jacobian(w):
    theta = float(np.linalg.norm(w)); omega = hat(w)
    if theta < 1e-4:
        t2 = theta*theta
        a = .5-t2/24+t2*t2/720
        b = 1/6-t2/120+t2*t2/5040
    else:
        a = (1-math.cos(theta))/(theta*theta)
        b = (theta-math.sin(theta))/(theta**3)
    return np.eye(3)+a*omega+b*(omega@omega)


def scale_motion(matrix, ratio):
    """Exp(ratio*Log(matrix)), including SE3 translation/rotation coupling."""
    if type(ratio) not in (int, float) or not math.isfinite(ratio) or ratio <= 0:
        raise ValueError('positive finite duration ratio required')
    delta = se3(matrix)
    w = Rotation.from_matrix(delta[:3, :3]).as_rotvec()
    # Solving J*v=t avoids the inverse-J cancellation near zero and pi.
    velocity = np.linalg.solve(jacobian(w), delta[:3, 3])
    scaled = w*ratio
    out = np.eye(4)
    out[:3, :3] = Rotation.from_rotvec(scaled).as_matrix()
    out[:3, 3] = jacobian(scaled)@(velocity*ratio)
    return se3(out)


def project(native_delta, native_interval_ns, target_interval_ns):
    """Fixed one-native-duration horizon; no reference or quality labels read."""
    a, b = interval(native_interval_ns); r0, r1 = interval(target_interval_ns)
    duration = b-a
    outside = max(a-r0, r0-b, a-r1, r1-b, 0)
    result = {'model': 'stationary_body_twist_v1', 'native_interval_ns': [a, b],
              'target_interval_ns': [r0, r1], 'duration_ratio': (r1-r0)/duration,
              'max_outside_native_ns': outside, 'horizon_ns': duration,
              'valid_model_projection': False, 'delta_model': None,
              'admission_ready': False, 'physical_body_transform_verified': False,
              'sensor_clock_verified': False, 'controller_deadline_verified': False,
              'reference_used_for_prediction': False, 'reason': 'horizon_exceeded'}
    if outside > duration:
        return result
    result.update(valid_model_projection=True, reason='model_projection_only',
                  delta_model=scale_motion(native_delta, result['duration_ratio']).tolist())
    return result

"""Conditional offline SE3 transport. No clock verification or benchmark admission.

Closed-form stationary body-twist equations and one-duration horizon from EXP088
(Scripts/Setup/causal_interval_model.py). Principal logarithm assumes no winding.
"""
import math

import numpy as np
from scipy.spatial.transform import Rotation

from selected_motion_trajectory import integer, name, rigid

MODEL = 'stationary_body_twist_v1'
ASSUMPTION = 'provided_ns_identity_assumption_v1'


def interval(value):
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError('two exact nonnegative integer endpoints required')
    a, b = (integer(t) for t in value)
    if a >= b:
        raise ValueError('positive interval required')
    return a, b


def jacobian(w):
    x, y, z = w
    omega = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    theta = float(np.linalg.norm(w))
    if theta < 1e-4:
        t2 = theta*theta
        a = .5-t2/24+t2*t2/720
        b = 1/6-t2/120+t2*t2/5040
    else:
        a = (1-math.cos(theta))/(theta*theta)
        b = (theta-math.sin(theta))/(theta**3)
    return np.eye(3)+a*omega+b*(omega@omega)


def scale_motion(delta, ratio):
    if type(ratio) not in (int, float) or not math.isfinite(ratio) or ratio <= 0:
        raise ValueError('positive finite ratio required')
    d, _ = rigid(delta)
    w = Rotation.from_matrix(d[:3, :3]).as_rotvec()
    v = np.linalg.solve(jacobian(w), d[:3, 3])
    out = np.eye(4)
    out[:3, :3] = Rotation.from_rotvec(w*ratio).as_matrix()
    out[:3, 3] = jacobian(w*ratio)@(v*ratio)
    return rigid(out)[0]


def identity_assumption(source_clock, target_clock):
    return {'kind': ASSUMPTION, 'source_clock_id': name(source_clock),
            'target_clock_id': name(target_clock), 'offset_ns': 0,
            'rate_numerator': 1, 'rate_denominator': 1,
            'evidence_ref': 'explicit_development_assumption_no_physical_clock_evidence',
            'physically_verified': False}


def transport(delta_camera, native_interval_ns, target_interval_ns,
              source_clock, target_clock, target_from_camera, clock_contract=None):
    """Conjugate a completed native delta, then conditionally transport its duration.

    Shifted start times have no effect only under the declared stationary twist.
    A contract is an assumption about two domains, never proof of synchronization.
    """
    a, b = interval(native_interval_ns); r0, r1 = interval(target_interval_ns)
    name(source_clock); name(target_clock)
    d, dc = rigid(delta_camera); e, ec = rigid(target_from_camera)
    outside = max(a-r0, r0-b, a-r1, r1-b, 0)
    out = {'model': MODEL, 'native_interval_ns': [a, b],
           'target_interval_ns': [r0, r1], 'source_clock_id': source_clock,
           'target_clock_id': target_clock, 'clock_contract': clock_contract,
           'target_from_camera': e.tolist(), 'delta_camera': d.tolist(),
           'rotation_roundoff_max_abs': max(dc, ec),
           'duration_ratio': (r1-r0)/(b-a), 'max_outside_native_ns': outside,
           'horizon_ns': b-a, 'accepted': False, 'delta_target': None,
           'transport_kind': None, 'admission_ready': False,
           'physical_frame_verified': False, 'sensor_clock_verified': False,
           'controller_deadline_verified': False, 'benchmark_execution_ready': False,
           'reference_used': False, 'online_output_latency_available': False,
           'reason': 'clock_contract_required'}
    if source_clock != target_clock:
        if clock_contract is None:
            return out
        c = clock_contract
        if (c.get('kind') != ASSUMPTION or c.get('source_clock_id') != source_clock
                or c.get('target_clock_id') != target_clock
                or any(type(c.get(k)) is not int or c[k] != v for k, v in
                       [('offset_ns', 0), ('rate_numerator', 1), ('rate_denominator', 1)])
                or c.get('physically_verified') is not False):
            raise ValueError('only explicit unfitted identity clock assumption supported')
        name(c.get('evidence_ref'))
    elif clock_contract is not None:
        raise ValueError('same clock requires no cross-domain assumption')
    if outside > b-a:
        out['reason'] = 'horizon_exceeded'
        return out
    common = rigid(e@d@np.linalg.inv(e))[0]
    exact = source_clock == target_clock and [a, b] == [r0, r1]
    out.update(accepted=True, reason='exact_native' if exact else 'conditional_model_only',
               transport_kind='exact_native' if exact else 'conditional_model',
               delta_target=(common if exact else scale_motion(common, (r1-r0)/(b-a))).tolist())
    return out


def integrate(records):
    """Offline model gauge only; rejected edges split segments and leave holes."""
    poses, segments = [np.eye(4).tolist()], [0]
    roots = [{'segment': 0, 'edge': 0, 'published_gauge_anchor': True}]
    current, active, segment = np.eye(4), True, 0
    for i, row in enumerate(records):
        if not row['accepted']:
            poses.append(None); segments.append(None); active = False
            continue
        if not active:
            segment += 1; current = np.eye(4); active = True
            roots.append({'segment': segment, 'edge': i, 'published_gauge_anchor': False})
        current = current@rigid(row['delta_target'])[0]
        poses.append(current.tolist()); segments.append(segment)
    return {'poses': poses, 'segments': segments, 'gauge_roots': roots,
            'initialization': 'arbitrary_model_gauge_not_measured_at_target_time',
            'scope': 'offline conditional engineering trajectory',
            'benchmark_execution_ready': False, 'physical_frame_or_clock_verified': False,
            'online_output_latency_available': False, 'reference_used': False}

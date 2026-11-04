"""Reference-free exact-interval publication of an already selected motion stream.

Frame/clock evidence names are caller declarations, not physical verification.
No selector, endpoint search, time transport, reference reader or fallback is here.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def integer(value):
    if type(value) is not int or value < 0:
        raise ValueError('nonnegative exact integer required')
    return value


def name(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('nonempty evidence/frame/clock/identity name required')
    return value


def rigid(value):
    """EXP133 serialization screen, with recorded proper-SO3 roundoff handling."""
    raw = np.asarray(value)
    if any(isinstance(x, (bool, np.bool_)) for x in np.asarray(value, dtype=object).flat):
        raise ValueError('boolean transform entries are not numeric measurements')
    if raw.dtype.kind not in 'iuf' or raw.shape != (4, 4):
        raise ValueError('numeric 4x4 transform required')
    matrix = raw.astype(float)
    if not np.isfinite(matrix).all() or not np.array_equal(matrix[3], [0, 0, 0, 1]):
        raise ValueError('invalid homogeneous transform')
    rotation = matrix[:3, :3]
    if (np.linalg.norm(rotation.T@rotation-np.eye(3)) > 1e-5
            or abs(np.linalg.det(rotation)-1) > 1e-5):
        raise ValueError('nonrigid transform')
    u, _, vt = np.linalg.svd(rotation)
    result = matrix.copy()
    result[:3, :3] = u@np.diag([1., 1., np.linalg.det(u@vt)])@vt
    return result, float(np.max(np.abs(result-matrix)))


def publish(contract):
    times = contract['time_ns']
    ids = contract['source_ids']
    if (len(times) < 2 or len(times) != len(ids)
            or any(type(t) is not int or t < 0 for t in times)
            or any(a >= b for a, b in zip(times, times[1:]))
            or len(set(ids)) != len(ids) or any(not isinstance(s, str) or not s for s in ids)):
        raise ValueError('exact chronological source axis required')
    target = name(contract['target_frame'])
    sensor_clock = name(contract['sensor_clock_id'])
    controller_clock = name(contract['controller_clock_id'])
    name(contract['sensor_clock_evidence_ref'])
    gap_limit = integer(contract['max_source_gap_ns'])
    if gap_limit == 0 or len(contract['slots']) != len(times)-1:
        raise ValueError('positive gap bound and exactly one decision per source edge required')
    transforms = {}
    for camera, frame in contract['frame_contracts'].items():
        if camera not in ('rgb', 'thermal'):
            raise ValueError('unknown frame-contract camera')
        name(frame['source_frame']); name(frame['frame_evidence_ref'])
        if frame['target_frame'] != target:
            raise ValueError('frame contract has wrong target')
        transforms[camera] = rigid(frame['target_from_camera'])
    poses, segments, records = [None], [None], []
    segment = -1
    active = False
    current = np.eye(4)
    previous_deadline = -1
    seen = set()
    max_roundoff = max([v[1] for v in transforms.values()] + [0.])
    initialization = contract.get('initialization')
    roots = []
    if initialization is not None:
        previous_deadline = integer(initialization['deadline_ns'])
        received = integer(initialization['received_ns'])
        if (initialization['controller_clock_id'] != controller_clock
                or type(initialization['source_time_ns']) is not int
                or initialization['source_time_ns'] != times[0]
                or initialization['target_frame'] != target or received > previous_deadline):
            raise ValueError('invalid explicit initialization')
        name(initialization['evidence_ref'])
        poses[0], segments[0] = current.tolist(), 0
        segment, active = 0, True
        roots.append({'segment': 0, 'root_time_ns': times[0], 'root_published': True,
                      'reason': 'explicit_initialization_gauge'})
    epochs = {}
    for index, slot in enumerate(contract['slots']):
        interval = [times[index], times[index+1]]
        value = slot['target_interval_ns']
        if (not isinstance(value, list) or len(value) != 2
                or any(type(t) is not int for t in value) or value != interval):
            raise ValueError('decision target differs from exact source interval')
        deadline = integer(slot['deadline_ns'])
        if deadline < previous_deadline:
            raise ValueError('backwards controller deadline')
        previous_deadline = deadline
        action = slot['action']
        if action not in ('RGB', 'THERMAL', 'ABSTAIN'):
            raise ValueError('unknown preselected action')
        result = {'source_indices': [index, index+1], 'source_ids': ids[index:index+2],
                  'target_interval_ns': interval, 'action': action, 'deadline_ns': deadline,
                  'accepted': False, 'reason': 'advisory_abstain', 'source': None,
                  'delta_target': None, 'segment': None}

        def admit():
            nonlocal max_roundoff
            if action == 'ABSTAIN':
                return None
            envelope = slot.get('selected')
            if envelope is None:
                result['reason'] = 'missing_selected_motion'; return None
            if envelope['controller_clock_id'] != controller_clock:
                result['reason'] = 'controller_clock_mismatch'; return None
            received = integer(envelope['received_ns'])
            if received > deadline:
                # A future payload's identity, frame and geometry remain unobserved.
                result['reason'] = 'late_selected_motion'; return None
            payload = envelope['payload']
            identity = name(payload['id'])
            if identity in seen:
                raise ValueError('duplicate received motion identity')
            seen.add(identity)
            camera = action.lower()
            if payload['camera'] != camera:
                result['reason'] = 'selected_camera_mismatch'; return None
            frame = contract['frame_contracts'].get(camera)
            if frame is None:
                result['reason'] = 'missing_frame_contract'; return None
            if payload.get('available') is not True:
                result['reason'] = 'selected_motion_unavailable'; return None
            if payload.get('reference_transport_used') is not False:
                result['reason'] = 'reference_transport_not_excluded'; return None
            source_interval = payload['native_interval_ns']
            if (not isinstance(source_interval, list) or len(source_interval) != 2
                    or any(type(t) is not int for t in source_interval)
                    or source_interval != interval):
                result['reason'] = 'native_interval_mismatch'; return None
            if payload['sensor_clock_id'] != sensor_clock:
                result['reason'] = 'sensor_clock_mismatch'; return None
            if payload['source_frame'] != frame['source_frame']:
                result['reason'] = 'source_frame_mismatch'; return None
            epoch = integer(payload['native_segment_id'])
            if type(payload['reset_before']) is not bool:
                raise ValueError('explicit boolean reset flag required')
            try:
                native, correction = rigid(payload['delta_camera'])
            except (ValueError, TypeError, OverflowError):
                result['reason'] = 'invalid_selected_geometry'; return None
            transform = transforms[camera][0]
            delta, composed_correction = rigid(transform@native@np.linalg.inv(transform))
            max_roundoff = max(max_roundoff, correction, composed_correction)
            reset = payload['reset_before'] or (camera in epochs and epochs[camera] != epoch)
            epochs[camera] = epoch
            result['source'] = {'id': identity, 'camera': camera,
                                'native_interval_ns': source_interval.copy(),
                                'received_ns': received, 'native_segment_id': epoch,
                                'frame_evidence_ref': frame['frame_evidence_ref'],
                                'sensor_clock_evidence_ref': contract['sensor_clock_evidence_ref']}
            return delta, reset

        accepted = admit()
        if accepted is None:
            poses.append(None); segments.append(None)
            active = False
        else:
            delta, reset = accepted
            source_gap = times[index+1]-times[index] > gap_limit
            if not active or reset or source_gap:
                segment += 1
                current = np.eye(4)
                roots.append({'segment': segment, 'root_time_ns': times[index],
                              'root_published': False,
                              'reason': 'reset' if reset else 'source_gap' if source_gap else 'after_missing_motion'})
            current, correction = rigid(current@delta)
            max_roundoff = max(max_roundoff, correction)
            active = True
            poses.append(current.tolist()); segments.append(segment)
            result.update(accepted=True, reason='accepted_exact_selected_motion',
                          delta_target=delta.tolist(), segment=segment)
        records.append(result)
    return {'schema_version': 1, 'time_ns': times.copy(), 'source_ids': ids.copy(),
            'poses': poses, 'segments': segments, 'segment_roots': roots,
            'target_frame': target, 'sensor_clock_id': sensor_clock,
            'controller_clock_id': controller_clock, 'decisions': records,
            'terminal_success': contract['terminal_success'] is True,
            'source_frames': len(times), 'pose_frames': sum(p is not None for p in poses),
            'accepted_motions': sum(r['accepted'] for r in records),
            'segment_count': segment+1, 'max_rotation_roundoff_correction': max_roundoff,
            'reference_used': False, 'time_transport_used': False,
            'physical_frame_or_clock_verified': False,
            'scope': 'declared-frame/clock exact-native publication only'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contract', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    payload = args.contract.read_bytes()
    result = publish(json.loads(payload))
    result['input_sha256'] = hashlib.sha256(payload).hexdigest()
    result['publisher_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')

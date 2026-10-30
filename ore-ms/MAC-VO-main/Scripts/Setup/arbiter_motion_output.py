"""Opt-in motion admission for existing advisory decisions; no time transport.

Frame verification/evidence are caller attestations. This module checks their
presence and numerical consistency, not the physical truth of a calibration.
Controller receipt clocks are separate from sensor and worker clocks.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from Scripts.Setup.pair_arbiter_events import camera_record, index_events, integer
from Scripts.Setup.shared_arbiter_replay import POSE_FRAME

CAMERAS = ('rgb', 'thermal')


def interval(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError('two exact integer sensor endpoints required')
    a, b = map(integer, value)
    if b <= a:
        raise ValueError('nonincreasing target interval')
    return [a, b]


def multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rigid(matrix):
    if not isinstance(matrix, list) or len(matrix) != 4 or any(not isinstance(r, list) or len(r) != 4 for r in matrix):
        raise ValueError('4x4 transform required')
    if any(type(x) not in (float, int) or not math.isfinite(x) for row in matrix for x in row):
        raise ValueError('finite numeric transform required')
    if any(abs(x-y) > 1e-8 for x, y in zip(matrix[3], [0, 0, 0, 1])):
        raise ValueError('invalid homogeneous row')
    r = matrix
    for i in range(3):
        for j in range(3):
            if abs(sum(r[k][i]*r[k][j] for k in range(3)) - (i == j)) > 1e-8:
                raise ValueError('nonorthogonal rotation')
    det = r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])-r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])+r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0])
    if abs(det-1) > 1e-8:
        raise ValueError('improper rotation')
    return matrix


def inverse(a):
    return [[a[j][i] for j in range(3)]+[-sum(a[k][i]*a[k][3] for k in range(3))] for i in range(3)]+[[0, 0, 0, 1]]


def pose_matrix(pose):
    if not isinstance(pose, list) or len(pose) != 7 or any(type(x) not in (int, float) or not math.isfinite(x) for x in pose):
        raise ValueError('finite xyz+xyzw pose required')
    tx, ty, tz, x, y, z, w = pose
    if abs(x*x+y*y+z*z+w*w-1) > 1e-6:
        raise ValueError('unit quaternion required')
    # Normalize only bounded quaternion serialization roundoff.
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    x, y, z, w = (v/norm for v in (x, y, z, w))
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), tx],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), ty],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), tz], [0, 0, 0, 1]]


def admit(advice, candidates, *, target_interval_ns, target_frame, deadline_ns, controller_clock_id, frame_contracts):
    """Accept only the chosen expert. Rejection never triggers a new preference.

    candidates[c] contains record (camera_record), received_ns and clock_id.
    receipt is when the controller had BOTH health and the first motion write.
    frame_contracts[c] must name source_frame, target_frame, verified=True,
    evidence_ref, and target_from_camera (4x4). No approximate time matching.
    """
    target = interval(target_interval_ns)
    integer(deadline_ns)
    if deadline_ns < 0 or not isinstance(controller_clock_id, str) or not controller_clock_id.strip() or not isinstance(target_frame, str) or not target_frame.strip():
        raise ValueError('declared controller clock, nonnegative deadline and target frame required')
    action = advice['action']
    if action not in ('RGB', 'THERMAL', 'ABSTAIN'):
        raise ValueError('unknown advice action')
    out = {'schema_version': 1, 'advisory_action': action, 'action': 'ABSTAIN',
           'sensor_interval_ns': target, 'target_frame': target_frame,
           'controller_clock_id': controller_clock_id, 'deadline_ns': deadline_ns,
           'delta_target': None, 'reason': 'advisory_abstain', 'source': None,
           'reference_transport_used': False, 'physical_accuracy_validated': False}
    if action == 'ABSTAIN':
        return out
    camera = action.lower()
    candidate = candidates.get(camera)
    def reject(reason):
        out['reason'] = reason
        return out
    if candidate is None:
        return reject('missing_expert_record')
    record = candidate['record']
    if record.get('health_status') != 'COMPLETE' or record.get('motion_available_camera_local') is not True:
        return reject('expert_unavailable')
    motion = record.get('motion')
    if motion is None:
        return reject('missing_motion')
    if interval(record['sensor_interval_ns']) != target or interval([motion['time0_ns'], motion['time1_ns']]) != target:
        return reject('interval_mismatch')
    if candidate.get('clock_id') != controller_clock_id:
        return reject('controller_clock_mismatch')
    received = integer(candidate['received_ns'])
    if received < 0:
        raise ValueError('nonnegative controller receipt required')
    if received > deadline_ns:
        return reject('late_result')
    frame = frame_contracts.get(camera)
    if frame is None or frame.get('verified') is not True or not isinstance(frame.get('evidence_ref'), str) or not frame['evidence_ref'].strip():
        return reject('unverified_frame_contract')
    if frame.get('source_frame') != record.get('pose_frame') or frame.get('target_frame') != target_frame:
        return reject('frame_mismatch')
    try:
        transform = rigid(frame['target_from_camera'])
        native = pose_matrix(motion['derived_camera_delta_xyzw'])
        converted = rigid(multiply(multiply(transform, native), inverse(transform)))
    except (KeyError, ValueError, OverflowError):
        return reject('invalid_geometry')
    out.update(action=action, delta_target=converted, reason='accepted_exact_interval',
               source={'camera': camera, 'motion_event_sequence': motion['sequence'],
                       'received_ns': received, 'native_interval_ns': target.copy(),
                       'frame_evidence_ref': frame['evidence_ref']})
    return out


def replay(protocol, timeline, frame_contracts=None):
    """Incremental controller-prefix adapter for shared_arbiter_replay journals.

    Uses source advice unchanged and RGB acquisition interval as the explicit
    requested interval. Empty contracts are the correct default for current data.
    Local event clocks are never used as controller receipt times.
    """
    rows = protocol['input_rows']
    events = {c: [] for c in CAMERAS}
    receipts = {c: {} for c in CAMERAS}
    previous = -1
    for sequence, entry in enumerate(timeline):
        now = integer(entry['controller_monotonic_ns'])
        if integer(entry['sequence']) != sequence or now < 0 or now < previous:
            raise ValueError('nonsequential timeline or backwards controller clock')
        previous = now
        if entry['kind'] == 'observer_event':
            c, event = entry['camera'], entry['event']
            if c not in CAMERAS or integer(event['sequence']) != len(events[c]):
                raise ValueError('invalid event identity')
            events[c].append(event)
            receipts[c][event['sequence']] = now
        elif entry['kind'] == 'decision':
            from Scripts.Setup.shared_arbiter_replay import advisory
            pair = integer(entry['pair_ordinal'])
            if not 0 <= pair < len(rows)-1:
                raise ValueError('invalid target pair')
            advice = advisory(rows, pair, events)
            for key in ('action', 'availability', 'evidence', 'observed_event_counts'):
                if entry[key] != advice[key]:
                    raise ValueError('source advice differs from observed prefix')
            candidates = {}
            for c in CAMERAS:
                a, b = [integer(rows[i][c+'_left']['time_ns']) for i in (pair, pair+1)]
                health, motions, _ = index_events(events[c])
                record = camera_record({'coordinate_frame': POSE_FRAME}, health, motions, a, b)
                if record['motion'] is not None and (a, b) in health:
                    received = max(receipts[c][record['motion']['sequence']], receipts[c][health[(a, b)]['sequence']])
                    candidates[c] = {'record': record, 'received_ns': received, 'clock_id': 'shared-controller'}
            target = [integer(rows[i]['rgb_left']['time_ns']) for i in (pair, pair+1)]
            result = admit(advice, candidates, target_interval_ns=target, target_frame='vehicle',
                           deadline_ns=now, controller_clock_id='shared-controller', frame_contracts=frame_contracts or {})
            result.update(pair_ordinal=pair, source_decision_sequence=sequence)
            yield result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    paths = [args.run/'protocol.json', args.run/'timeline.jsonl']
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    protocol = json.loads(paths[0].read_text())
    with paths[1].open() as stream:
        outputs = list(replay(protocol, (json.loads(line) for line in stream)))
    if hashes != {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}:
        raise ValueError('source changed during replay')
    payload = {'scope': 'strict exact-interval admission; no frame contracts supplied; not a quality experiment',
               'input_sha256': hashes, 'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), 'outputs': outputs}
    with args.output.open('x') as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
        stream.write('\n')


if __name__ == '__main__':
    main()

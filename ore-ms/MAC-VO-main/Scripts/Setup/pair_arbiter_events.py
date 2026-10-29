"""Pair verified event journals while preserving distinct sensor/frame contracts."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo

HEALTH_FIELDS = ('selected_count', 'inbound_count', 'retained_count', 'occupied_3x3_cells',
                 'grid_count', 'grid_positive_uu_vv_count', 'grid_trace_q10_q50_q90_px2',
                 'original_need_interp', 'execution_error', 'motion_status')


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def integer(value):
    if type(value) is not int:
        raise ValueError('exact integer timestamp/index required')
    return value


def interval(event):
    first, last = integer(event['time0_ns']), integer(event['time1_ns'])
    if last <= first:
        raise ValueError('nonincreasing motion interval')
    return first, last


def index_events(events):
    health, submissions, firsts = {}, {}, {}
    repeats, previous_clock = 0, None
    for i, event in enumerate(events):
        if integer(event['sequence']) != i:
            raise ValueError('invalid event sequence')
        clock = integer(event['observation_monotonic_ns'])
        if previous_clock is not None and clock < previous_clock:
            raise ValueError('observer clock went backwards')
        previous_clock = clock
        if event['kind'] == 'pair_end':
            key = interval(event)
            if key in health:
                raise ValueError('duplicate pair-end interval')
            health[key] = event
        elif event['kind'] == 'optimization_submitted':
            identity = integer(event['submission_id'])
            if identity in submissions:
                raise ValueError('duplicate submission ID')
            interval(event)
            submissions[identity] = event
        elif event['kind'] == 'motion_result_written':
            if not event['first_write_for_submission']:
                repeats += 1
                continue
            identity = integer(event['submission_id'])
            if identity not in submissions or identity in firsts:
                raise ValueError('orphan or duplicate first motion write')
            submitted = submissions[identity]
            if interval(event) != interval(submitted):
                raise ValueError('motion write interval differs from submission')
            if event['from_idx'] != submitted['from_idx'] or event['frame_idx'] != submitted['frame_idx']:
                raise ValueError('motion write graph indices differ from submission')
            firsts[identity] = event
    return health, list(firsts.values()), repeats


def camera_record(report, health, firsts, first, last):
    end = health.get((first, last))
    # A graph's parent can differ from the input pair (e.g. skipped keyframes).
    # Preserve that mismatch rather than pretending the deltas share an interval.
    candidates = [e for e in firsts if e['time1_ns'] == last]
    motion = candidates[0] if len(candidates) == 1 else None
    matches = motion is not None and interval(motion) == (first, last)
    complete = end is not None and all(end.get(k) is not None for k in HEALTH_FIELDS if k not in ('execution_error', 'grid_trace_q10_q50_q90_px2'))
    values = {k: end.get(k) if end is not None else None for k in HEALTH_FIELDS}
    if complete:
        counts = [integer(values[k]) for k in ('selected_count','inbound_count','retained_count','occupied_3x3_cells','grid_count','grid_positive_uu_vv_count')]
        if any(n < 0 for n in counts) or not counts[0] >= counts[1] >= counts[2] or counts[3] > 9 or counts[5] > counts[4]:
            raise ValueError('invalid health count ordering')
        if type(values['original_need_interp']) is not bool:
            raise ValueError('original support state must be boolean')
    return {'sensor_interval_ns': [first, last], 'pose_frame': report['coordinate_frame'],
        'health_status': 'COMPLETE' if complete else 'MISSING_PAIR_END' if end is None else 'PARTIAL_HEALTH',
        'health': values, 'health_observation_monotonic_ns': end['observation_monotonic_ns'] if end else None,
        'motion_status': 'AMBIGUOUS_FIRST_WRITES' if len(candidates) > 1 else 'NO_FIRST_WRITE' if motion is None else 'INTERVAL_MATCH' if matches else 'INTERVAL_MISMATCH',
        'motion_candidate_count': len(candidates), 'motion_interval_matches_pair': matches,
        'motion_available_camera_local': bool(matches and end is not None and end.get('original_need_interp') is False
            and motion.get('original_need_interp') is False and end.get('execution_error') is None
            and motion['derived_camera_delta_xyzw'] is not None),
        'motion': None if motion is None else {k: motion[k] for k in (
            'sequence', 'submission_id', 'from_idx', 'frame_idx', 'time0_ns', 'time1_ns', 'stage',
            'observation_monotonic_ns', 'parent_pose_camera_map_xyzw', 'committed_pose_camera_map_xyzw',
            'derived_camera_delta_xyzw', 'derived_delta_unavailable_reason')},
        'body_transform_verified': False}


def build_pairs(rgb_report, rgb_events, thermal_report, thermal_events):
    if rgb_report['modality'] != 'rgb' or thermal_report['modality'] != 'thermal':
        raise ValueError('wrong expert modality')
    if rgb_report['input_rows'] != thermal_report['input_rows']:
        raise ValueError('paired manifest rows differ between experts')
    source = rgb_report['input_rows']
    if len(source) < 2:
        raise ValueError('at least two source rows required')
    clocks = {c: [integer(row[c+'_left']['time_ns']) for row in source] for c in ('rgb', 'thermal')}
    if any(b <= a for times in clocks.values() for a, b in zip(times, times[1:])):
        raise ValueError('nonincreasing source camera timestamps')
    indexed = {c: index_events(events) for c, events in [('rgb', rgb_events), ('thermal', thermal_events)]}
    rows = []
    for i in range(len(source)-1):
        row = {'pair_ordinal': i, 'cameras': {}}
        for c, report in [('rgb', rgb_report), ('thermal', thermal_report)]:
            health, motion, _ = indexed[c]
            row['cameras'][c] = camera_record(report, health, motion, clocks[c][i], clocks[c][i+1])
        a, b = row['cameras']['rgb']['sensor_interval_ns'], row['cameras']['thermal']['sensor_interval_ns']
        row['rgb_minus_thermal_endpoint_ns'] = [a[0]-b[0], a[1]-b[1]]
        row['exact_same_interval'] = a == b
        rows.append(row)
    health_complete = all(cam['health_status'] == 'COMPLETE' for row in rows for cam in row['cameras'].values())
    return {'schema_version': 1, 'capture_mode': 'separate_replay_processes', 'pairs': rows,
        'alignment': {'paired_source_rows_identical': True, 'pair_count': len(rows),
            'exact_same_intervals': sum(row['exact_same_interval'] for row in rows),
            'max_abs_endpoint_offset_ns': max(abs(x) for row in rows for x in row['rgb_minus_thermal_endpoint_ns'])},
        'availability': {c: {'complete_health_pairs': sum(r['cameras'][c]['health_status'] == 'COMPLETE' for r in rows),
            'camera_local_motion_pairs': sum(r['cameras'][c]['motion_available_camera_local'] for r in rows),
            'repeated_writes_not_counted_as_new_motion': indexed[c][2]} for c in ('rgb','thermal')},
        'eligibility': {'health_schema_replay_complete': health_complete, 'pose_gate_training_ready': False,
            'reasons': ['no verified common-body frame transport', 'no declared asynchronous comparison policy',
                'no applicable independent pose-loss labels', 'separate replays do not establish concurrent result availability']},
        'feature_contract': {'allowed_health_fields': HEALTH_FIELDS,
            'observation_times': 'local monotonic observation timestamps retained separately; never subtract sensor epoch times or treat replay scheduling as simultaneous availability',
            'provenance_only': 'run-level status, whole-clip summaries and source outcomes are metadata, not per-pair predictor inputs'}}


def load_verified_run(path, proof):
    validation = json.loads(proof.read_text())
    summary_path, events_path = path/'arbiter-events/summary.json', path/'arbiter-events/events.jsonl'
    summary = json.loads(summary_path.read_text())
    if validation['status'] != 'PASS' or summary['status'] != 'COMPLETE' or summary['observer_errors']:
        raise ValueError('event journal is not complete and independently validated')
    if sha(events_path) != validation['events_sha256'] or sha(summary_path) != validation['summary_sha256']:
        raise ValueError('event journal/proof hash mismatch')
    return json.loads((path/'report.json').read_text()), [json.loads(line) for line in events_path.read_text().splitlines()]


def run(rgb, thermal, rgb_proof, thermal_proof, output):
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    files = [p for base in (rgb,thermal) for p in (base/'report.json',base/'arbiter-events/events.jsonl',base/'arbiter-events/summary.json')]+[rgb_proof,thermal_proof]
    hashes = {str(p): sha(p) for p in files}
    r, re = load_verified_run(rgb,rgb_proof);t,te=load_verified_run(thermal,thermal_proof)
    report=build_pairs(r,re,t,te)
    protocol={'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'script_sha256':sha(__file__),
        'input_sha256':hashes,'rgb_run':str(rgb),'thermal_run':str(thermal),
        'checkpoint_sha256':{'rgb':r['checkpoint_sha256'],'thermal':t['checkpoint_sha256']},
        'scope':'manifest-row pairing of existing separate event replays; no time correction, frame conversion, motion fusion, labels or gate fit'}
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    (output/'paired-events.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    assert all(sha(p)==h for p,h in hashes.items())
    print(json.dumps({k:report[k] for k in ('alignment','availability','eligibility')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['rgb','thermal','rgb-proof','thermal-proof','output']:p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();run(a.rgb,a.thermal,a.rgb_proof,a.thermal_proof,a.output)

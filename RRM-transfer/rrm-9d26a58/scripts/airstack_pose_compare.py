#!/usr/bin/env python3
"""Offline receipt-stamp pose comparison; never estimates acquisition time or control."""
import argparse
import bisect
import hashlib
from datetime import datetime, timezone
import json
import math
import re
from pathlib import Path
import statistics


def vector(value):
    if (not isinstance(value, list) or len(value) != 3
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in value)):
        raise ValueError('expected finite three-component position')
    return value


def statistics_xyz(values):
    return {'count': len(values),
            'median_xyz_m': [statistics.median(v[i] for v in values) for i in range(3)],
            'min_xyz_m': [min(v[i] for v in values) for i in range(3)],
            'max_xyz_m': [max(v[i] for v in values) for i in range(3)],
            'rms_norm_m': math.sqrt(sum(sum(x*x for x in v) for v in values)/len(values))}


def compare(truth, events, baseline_start_s, baseline_end_s, *, max_gap_s=.25):
    """Axes assumed aligned by caller; fixed grounded translation is descriptive."""
    if (not truth or not math.isfinite(max_gap_s) or max_gap_s <= 0
            or not math.isfinite(baseline_start_s) or not math.isfinite(baseline_end_s)
            or baseline_start_s >= baseline_end_s):
        raise ValueError('invalid capture, baseline interval or interpolation gap')
    identities = set()
    times, bodies, sensors = [], [], []
    for i, r in enumerate(truth):
        if (r.get('schema') != 'airstack-physical-truth/v1'
                or type(r.get('sequence')) is not int or r['sequence'] != i):
            raise ValueError('truth schema/sequence mismatch')
        identity = (r['capture_id'], r['vehicle_path'], r['recorder_sha256'])
        if (any(not isinstance(v, str) or not v for v in identity)
                or not re.fullmatch(r'[0-9a-f]{64}', identity[2])):
            raise ValueError('invalid truth identity or source hash')
        identities.add(identity)
        t = r['sim_time_s']
        if type(t) not in (int, float) or not math.isfinite(t) or (times and t <= times[-1]):
            raise ValueError('truth clock must advance without resets or duplicates')
        times.append(t); bodies.append(vector(r['rigid_body_position_m']))
        sensors.append(vector(r['sensor_state_position_m']))
    if len(identities) != 1:
        raise ValueError('mixed capture, vehicle or recorder identities')
    odometry = [r for r in events if r.get('channel') == 'odom']
    if not odometry:
        raise ValueError('odometry missing')
    rows, skipped = [], {'outside_overlap': 0, 'wide_bracket': 0}
    frames, previous = set(), None
    for r in odometry:
        stamp = r['source_stamp_ns']
        if type(stamp) is not int or stamp < 0 or (previous is not None and stamp < previous):
            raise ValueError('odometry clock invalid or reset')
        previous = stamp; t = stamp/1e9
        msg = r['message']
        header_stamp = msg['header']['stamp']
        if header_stamp['sec'] * 1_000_000_000 + header_stamp['nanosec'] != stamp:
            raise ValueError('event/header odometry timestamp mismatch')
        frames.add((msg['header']['frame_id'],msg['child_frame_id']))
        position = vector([msg['pose']['pose']['position'][a] for a in 'xyz'])
        if t < times[0] or t > times[-1]:
            skipped['outside_overlap'] += 1; continue
        hi = bisect.bisect_left(times,t)
        if hi < len(times) and times[hi] == t:
            body = bodies[hi]; gap = 0.0
        else:
            lo = hi-1; gap = times[hi]-times[lo]
            if gap > max_gap_s:
                skipped['wide_bracket'] += 1; continue
            f = (t-times[lo])/gap
            body = [a+(b-a)*f for a,b in zip(bodies[lo],bodies[hi])]
        rows.append({'sim_receipt_stamp_s':t,'bracket_s':gap,
                     'raw_body_minus_odometry_xyz_m':[a-b for a,b in zip(body,position)]})
    if len(frames) != 1 or frames != {('map','base_link')}:
        raise ValueError('expected consistent map/base_link odometry labels')
    baseline = [v for v in rows if baseline_start_s <= v['sim_receipt_stamp_s'] <= baseline_end_s]
    if len(baseline) < 10:
        raise ValueError('baseline has fewer than ten overlapping samples')
    # Caller must establish grounded/disarmed baseline independently. Never fit flight data.
    offset = statistics_xyz([v['raw_body_minus_odometry_xyz_m'] for v in baseline])['median_xyz_m']
    after = [v for v in rows if v['sim_receipt_stamp_s'] > baseline_end_s]
    for row in rows:
        row['grounded_translation_removed_xyz_m'] = [a-b for a,b in
            zip(row['raw_body_minus_odometry_xyz_m'],offset)]
    return {'schema':'rrm-pose-comparison/v1','execution_dispatch':False,
            'alignment':'linear interpolation at converted odometry header; receipt-phase only',
            'assumptions':['world/map axes aligned; not proven by relabeled map header',
                           'same simulator epoch established externally; overlapping numeric clocks do not prove identity',
                           'baseline externally verified grounded/disarmed'],
            'limitations':'Not acquisition-time registration, estimator accuracy, collision containment or flight acceptance. No lag fitting. Twist velocities are not compared (body/world frames differ).',
            'truth_identity':list(next(iter(identities))),
            'sensor_state_field_meaning':'Pegasus vehicle.state physical input to simulated sensors, not noisy sensor output or PX4 belief; equality checks capture paths only',
            'body_sensor_max_distance_m':max(math.dist(a,b) for a,b in zip(bodies,sensors)),
            'baseline_interval_s':[baseline_start_s,baseline_end_s],
            'baseline_raw':statistics_xyz([v['raw_body_minus_odometry_xyz_m'] for v in baseline]),
            'raw_all':statistics_xyz([v['raw_body_minus_odometry_xyz_m'] for v in rows]),
            'after_baseline_translation_removed':statistics_xyz([v['grounded_translation_removed_xyz_m'] for v in after]) if after else None,
            'matched_samples':len(rows),'skipped':skipped,'max_bracket_s':max(v['bracket_s'] for v in rows),
            'samples':rows}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--truth',type=Path,required=True);p.add_argument('--control',type=Path,required=True)
    p.add_argument('--baseline-start-s',type=float,required=True);p.add_argument('--baseline-end-s',type=float,required=True)
    p.add_argument('--assume-aligned-axes',action='store_true',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    paths=[args.truth,args.control];data=[path.read_bytes() for path in paths]
    report=compare(*[[json.loads(x) for x in content.splitlines()] for content in data],args.baseline_start_s,args.baseline_end_s)
    report['created_at_utc']=datetime.now(timezone.utc).isoformat()
    report['comparison_source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    report['inputs']=[{'path':str(path),'sha256':hashlib.sha256(content).hexdigest()} for path,content in zip(paths,data)]
    with args.output.open('x') as out:json.dump(report,out,indent=2,allow_nan=False);out.write('\n')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))


if __name__=='__main__':main()

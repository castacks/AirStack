#!/usr/bin/env python3
"""Relative map/PhysX consistency after preflight-only yaw/translation calibration.

Diagnostic only: truth-assisted registration and wall pairing cannot qualify SLAM.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import statistics


def yaw(q):
    x, y, z, w = q
    norm = math.sqrt(sum(v*v for v in q))
    if norm < .5 or abs(norm-1) > .01:
        raise ValueError('invalid quaternion')
    x, y, z, w = (v/norm for v in q)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def rotate(p, angle):
    c, s = math.cos(angle), math.sin(angle)
    return [c*p[0]-s*p[1], s*p[0]+c*p[1], p[2]]


def metrics(values):
    values = sorted(values)
    return dict(samples=len(values), rmse=math.sqrt(statistics.mean(x*x for x in values)),
                median=statistics.median(values), p95=values[math.ceil(.95*len(values))-1],
                maximum=max(values))


def assess(events, truth):
    times = [t['wall_time_unix_s'] for t in truth]
    if not times or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('missing/nonmonotonic truth')
    odometry = [e for e in events if e['kind'] == 'odom']
    if any(b['wall_unix'] <= a['wall_unix'] for a, b in zip(odometry, odometry[1:])):
        raise ValueError('nonmonotonic odometry receipt')
    pairs = []
    unpaired = 0
    for e in odometry:
        i = bisect.bisect_left(times, e['wall_unix'])
        t = min(truth[max(0, i-1):min(len(truth), i+1)],
                key=lambda t: abs(t['wall_time_unix_s']-e['wall_unix']))
        if abs(t['wall_time_unix_s']-e['wall_unix']) > .15:
            unpaired += 1
            continue
        d = e['data']
        if d['header']['frame_id'] != 'map' or d['child_frame_id'] != 'base_link':
            raise ValueError('unexpected odometry frames')
        p = [d['pose']['pose']['position'][k] for k in 'xyz']
        q = [d['pose']['pose']['orientation'][k] for k in 'xyzw']
        physical = t['rigid_body_position_m']
        delta = yaw(t['rigid_body_quaternion_xyzw'])-yaw(q)
        epoch = d['header']['stamp']['sec']+d['header']['stamp']['nanosec']*1e-9-t['sim_time_s']
        if not all(math.isfinite(v) for v in p+physical+[delta, epoch]):
            raise ValueError('nonfinite pose/time')
        pairs.append(dict(event=e, truth=t, p=p, delta=delta, epoch=epoch))
    calibration = [v for v in pairs if v['event']['phase'] == 'preflight']
    if len(calibration) < 10:
        raise ValueError('insufficient preflight registration')
    angle = math.atan2(statistics.mean(math.sin(v['delta']) for v in calibration),
                       statistics.mean(math.cos(v['delta']) for v in calibration))
    offsets = [[b-a for a, b in zip(rotate(v['p'], angle), v['truth']['rigid_body_position_m'])]
               for v in calibration]
    offset = [statistics.median(v[k] for v in offsets) for k in range(3)]
    held = [v for v in pairs if v['event']['phase'] != 'preflight']
    if not held:
        raise ValueError('no held-out flight data')
    phases = {}
    for v in held:
        predicted = [a+b for a, b in zip(rotate(v['p'], angle), offset)]
        v['error'] = math.dist(predicted, v['truth']['rigid_body_position_m'])
        phases.setdefault(v['event']['phase'], []).append(v['error'])
    return dict(verdict='NOT DEMONSTRATED',
                scope='relative localization diagnostic; truth-assisted initial registration, not SLAM qualification',
                calibration_samples=len(calibration), yaw_map_to_world_rad=angle,
                translation_map_to_world_m=offset, held_out_position_error_m=metrics([v['error'] for v in held]),
                by_phase_position_error_m={k: metrics(v) for k, v in phases.items()},
                final_registered_error_m=held[-1]['error'], unpaired_odometry=unpaired,
                source_epoch_minus_truth_s=metrics([abs(v['epoch']) for v in pairs]),
                limitations=['150ms wall-receipt pairing is not acquisition synchronization',
                             'body/IMU lever arm conventions and gravity-axis alignment assumed',
                             'no loop closure, map correctness or GNSS-denial test'])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for key in ('events', 'truth', 'output'):
        ap.add_argument('--'+key, required=True, type=Path)
    args = ap.parse_args()
    if args.output.exists():
        ap.error('output must be new')
    result = assess(*[[json.loads(line) for line in path.read_text().splitlines()]
                      for path in (args.events, args.truth)])
    result['assessor_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    result['input_sha256'] = {k: hashlib.sha256(getattr(args, k).read_bytes()).hexdigest()
                              for k in ('events', 'truth')}
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

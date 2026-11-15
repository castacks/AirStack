#!/usr/bin/env python3
"""Assess restricted observation-conditioned viewpoint from raw frames and PhysX.

Bbox growth is an image-resolution proxy, not semantic information gain. Requires
a matching complete-flight contact assessment and immutable evidence inputs.
"""
import argparse
import bisect
import hashlib
import inspect
import json
import math
from pathlib import Path
import statistics

import cv2
import numpy as np
from rrm.uav_marker import detect_red_surface
from rrm.uav_viewpoint import optical_forward_in_map, select_viewpoint


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for key in ('events', 'truth', 'contact-assessment', 'raw-dir', 'output'):
        ap.add_argument('--'+key, required=True, type=Path)
    args = ap.parse_args()
    if args.output.exists():
        ap.error('output must be new')
    events = [json.loads(line) for line in args.events.read_text().splitlines()]
    truth = [json.loads(line) for line in args.truth.read_text().splitlines()]
    contact = json.loads(args.contact_assessment.read_text())
    checks = dict(complete_flight=contact.get('verdict') == 'PASS',
                  evidence_case_bound=all(contact.get('input_sha256', {}).get(k) == digest(getattr(args, k))
                                          for k in ('events', 'truth')),
                  raw_observations_match=False, exact_fresh_pairs=False,
                  policy_source_matches=False, policy_transform_and_goal_bound=False,
                  measured_resolution_gain=False, measured_standoff=False,
                  physical_viewpoint_motion=False, observed_navigation_and_recovery=False)
    metrics = {}
    errors = []
    records = {name: [e for e in events if e['kind'] == 'marker_observation' and e['label'] == name]
               for name in ('before', 'after')}
    decisions = [e for e in events if e['kind'] == 'marker_viewpoint_decision']
    qualities = [e for e in events if e['kind'] == 'marker_viewpoint_quality']
    goals = [e for e in events if e['kind'] == 'viewpoint_goal']
    provenance = [e for e in events if e['kind'] == 'marker_policy_provenance']
    summaries = [e['data'] for e in events if e['kind'] == 'summary']
    raw_hashes = {}
    recomputed = {}
    try:
        if any(len(v) != 3 for v in records.values()) or len(decisions) != 1 or len(goals) != 1:
            raise ValueError('requires one decision/goal and three before/after raw observations')
        valid = True
        paired = True
        for label, values in records.items():
            recomputed[label] = []
            stamps = []
            for record in values:
                prefix = record['raw_prefix']
                if Path(prefix).name != prefix:
                    raise ValueError('invalid raw filename')
                png, npz = (args.raw_dir/(prefix+suffix) for suffix in ('.png', '.npz'))
                raw_hashes[png.name] = digest(png)
                raw_hashes[npz.name] = digest(npz)
                image = cv2.imread(str(png), cv2.IMREAD_COLOR)
                with np.load(npz, allow_pickle=False) as saved:
                    depth = saved['depth_m']
                    intrinsic = [float(v) for v in saved['intrinsic']]
                    observation = detect_red_surface(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), depth, intrinsic)
                valid &= observation == record['observation'] and observation['status'] == 'OBSERVED'
                valid &= intrinsic == record['intrinsic'] and [image.shape[1], image.shape[0]] == record['image_size']
                recomputed[label].append(observation)
                stamp = (record['stamp_sec'], record['stamp_nanosec'])
                stamps.append(stamp)
                paired &= record['exact_stamp_pair'] and record['depth_encoding'] == '32FC1'
                paired &= 0 <= record['wall_unix']-record['image_receipt_wall_unix'] <= .5
                for header in record['headers'].values():
                    paired &= header['frame_id'] == record['frame'] and (header['stamp']['sec'], header['stamp']['nanosec']) == stamp
                paired &= set(record['headers']) == {'rgb', 'depth', 'info'}
            paired &= len(set(stamps)) == 3 and stamps == sorted(stamps)
        checks['raw_observations_match'] = bool(valid)
        checks['exact_fresh_pairs'] = bool(paired)
        if not valid:
            raise ValueError('raw target observation missing or mismatch')
        before, after = records['before'], records['after']
        event = decisions[0]
        checks['policy_source_matches'] = len(provenance) == 1 and (
            provenance[0]['policy_sha256'] == digest(Path(inspect.getfile(select_viewpoint))) and
            provenance[0]['detector_sha256'] == digest(Path(inspect.getfile(detect_red_surface))))
        transform = event['transform']
        q = transform['transform']['rotation']
        axis = optical_forward_in_map([q[k] for k in 'xyzw'])
        decision = select_viewpoint(recomputed['before'], before[0]['image_size'], axis)
        selected = before[decision['selected_observation_index']]
        ts = transform['header']['stamp']
        previous_odom = [e for e in events if e['kind'] == 'odom' and e['wall_unix'] < goals[0]['wall_unix']][-1]
        anchor = previous_odom['data']['pose']['pose']['position']
        commanded = goals[0]['target_map_m']
        checks['policy_transform_and_goal_bound'] = (
            decision == event['decision'] and decision['status'] == 'SELECTED' and
            transform['header']['frame_id'] == 'map' and transform['child_frame_id'] == selected['frame'] and
            event['transform_observation_index'] == decision['selected_observation_index'] and
            (ts['sec'], ts['nanosec']) == (selected['stamp_sec'], selected['stamp_nanosec']) and
            all(abs(a-b) < 1e-9 for a, b in zip(axis, event['optical_axis_map'])) and
            all(0 <= event['wall_unix']-r['image_receipt_wall_unix'] < 2 for r in before) and
            event['wall_unix'] < goals[0]['wall_unix'] and commanded[2] == 2 and
            all(abs(commanded[i]-anchor[k]-decision['delta_map_m'][i]) < 1e-8 for i, k in enumerate('xy')))
        areas = {k: statistics.median(v['bbox_xywh'][2]*v['bbox_xywh'][3] for v in values)
                 for k, values in recomputed.items()}
        depths = {k: statistics.median(v['surface_depth_z_m'] for v in values)
                  for k, values in recomputed.items()}
        ratio = areas['after']/areas['before']
        reduction = depths['before']-depths['after']
        calibration_fixed = all(r['intrinsic'] == before[0]['intrinsic'] and r['frame'] == before[0]['frame'] and
                                r['image_size'] == before[0]['image_size'] for r in before+after)
        checks['measured_resolution_gain'] = calibration_fixed and ratio >= 1.2
        checks['measured_standoff'] = reduction >= .2 and depths['after'] >= 2.4
        metrics.update(bbox_area_ratio=ratio, before_surface_depth_z_m=depths['before'],
                       after_surface_depth_z_m=depths['after'], optical_depth_reduction_m=reduction,
                       planned_translation_m=decision['translation_m'])
        times = [t['wall_time_unix_s'] for t in truth]
        if not times or any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError('missing/nonmonotonic independent truth')
        matched = []
        for record in (selected, after[1]):
            wall = record['image_receipt_wall_unix']
            i = bisect.bisect_left(times, wall)
            sample = min(truth[max(0, i-1):min(len(truth), i+1)], key=lambda t: abs(t['wall_time_unix_s']-wall))
            if abs(sample['wall_time_unix_s']-wall) > .15:
                raise ValueError('physical capture pairing gap')
            matched.append(sample)
        displacement = math.dist(*(v['rigid_body_position_m'] for v in matched))
        speeds = [math.sqrt(sum(x*x for x in v['rigid_body_linear_velocity_m_s'])) for v in matched]
        checks['physical_viewpoint_motion'] = .2 <= displacement <= 1 and all(v < .2 for v in speeds)
        metrics.update(physical_capture_displacement_m=displacement, physical_capture_speeds_m_s=speeds)
        checks['observed_navigation_and_recovery'] = bool(summaries) and (
            summaries[-1].get('marker_viewpoint_action') is True and summaries[-1].get('error') is None and
            summaries[-1].get('pending_goal_unreconciled') is False and
            all(summaries[-1].get(k) is True for k in ('navigate_action', 'land_action', 'final_disarmed', 'final_on_ground')))
    except Exception as exc:
        errors.append(repr(exc))
    verdict = 'PASS' if all(checks.values()) else 'NOT DEMONSTRATED'
    if contact.get('verdict') == 'FAIL' or (checks['raw_observations_match'] and not errors and
        any(not checks[k] for k in ('policy_transform_and_goal_bound', 'measured_resolution_gain', 'measured_standoff', 'physical_viewpoint_motion'))):
        verdict = 'FAIL'
    result = dict(verdict=verdict, scope='one static red-fixture resolution-seeking viewpoint; oracle depth, not semantic information gain',
                  checks=checks, metrics=metrics, errors=errors, raw_sha256=raw_hashes,
                  assessor_sha256=digest(Path(__file__)),
                  input_sha256={key: digest(getattr(args, key)) for key in ('events', 'truth', 'contact_assessment')})
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

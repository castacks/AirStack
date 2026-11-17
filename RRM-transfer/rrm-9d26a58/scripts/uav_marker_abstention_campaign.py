#!/usr/bin/env python3
"""Offline restricted-marker intervention campaign; no ROS or actuation.

Synthetic calibrated images qualify software dispositions, never physical safety,
semantic identity, renderer synchronization or real sensor fault handling.
"""
import argparse
import hashlib
import inspect
import json
from pathlib import Path
import platform

import cv2
import numpy as np

from rrm.uav_marker import detect_red_surface
from rrm.uav_viewpoint import select_viewpoint


def cases():
    """Authored interventions and expected observable dispositions, not accuracy labels."""
    rgb = np.zeros((120, 160, 3), dtype=np.uint8)
    rgb[40:80, 60:100] = [255, 0, 0]
    depth = np.full((120, 160), 3.5, dtype=np.float32)
    yield 'baseline', rgb.copy(), depth.copy(), 'OBSERVED', 'SELECTED', 'single centered red fixture'
    for name in ('duplicate', 'absent', 'occluded_split', 'occluded_partial',
                 'occluded_full', 'depth_zero', 'depth_nan', 'depth_inf',
                 'depth_negative', 'depth_far', 'depth_sparse', 'cropped'):
        image, distance = rgb.copy(), depth.copy()
        status, policy = 'OBSERVED', 'SELECTED'
        limitation = 'offline authored intervention'
        if name == 'duplicate':
            image[15:30, 15:30] = [255, 0, 0]
            status, policy = 'AMBIGUOUS', 'ABSTAIN'
        elif name == 'absent':
            image[:] = [0, 0, 255]
            status, policy = 'ABSENT', 'ABSTAIN'
        elif name == 'occluded_split':
            image[56:64, 55:105] = 0
            status, policy = 'AMBIGUOUS', 'ABSTAIN'
        elif name == 'occluded_partial':
            image[40:55, 60:100] = 0
            limitation = 'one visible component remains; partial occlusion is NOT detected'
        elif name == 'occluded_full':
            image[40:80, 60:100] = 0
            status, policy = 'ABSENT', 'ABSTAIN'
        elif name.startswith('depth_'):
            value = dict(depth_zero=0., depth_nan=np.nan, depth_inf=np.inf,
                         depth_negative=-1., depth_far=20., depth_sparse=np.nan)[name]
            distance[:] = value
            if name == 'depth_sparse':
                distance[50:53, 70:73] = 3.5
            status, policy = 'DEPTH_UNKNOWN', 'ABSTAIN'
        elif name == 'cropped':
            image[:] = 0
            image[40:80, :25] = [255, 0, 0]
            policy = 'ABSTAIN'
        yield name, image, distance, status, policy, limitation


def run(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    intrinsic = [100., 0., 80., 0., 100., 60., 0., 0., 1.]
    results = []
    hashes = {}
    for name, rgb, depth, expected, expected_policy, limitation in cases():
        png, npz = output/(name+'.png'), output/(name+'.npz')
        if not cv2.imwrite(str(png), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
            raise RuntimeError('image write failed')
        np.savez_compressed(npz, depth_m=depth, intrinsic=np.array(intrinsic))
        # Exercise the persisted inputs, not just the generator's in-memory values.
        saved_rgb = cv2.cvtColor(cv2.imread(str(png)), cv2.COLOR_BGR2RGB)
        with np.load(npz, allow_pickle=False) as saved:
            observation = detect_red_surface(saved_rgb, saved['depth_m'], saved['intrinsic'].tolist())
        decision = select_viewpoint([observation]*3, [160, 120], [1., 0., 0.])
        passed = (observation['status'] == expected and decision['status'] == expected_policy
                  and (expected_policy != 'ABSTAIN' or decision['delta_map_m'] is None))
        results.append(dict(case=name, expected_observation=expected,
                            expected_policy=expected_policy, observation=observation,
                            decision=decision, passed=passed, limitation=limitation))
        for path in (png, npz):
            hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    source_hashes = {}
    for obj in (run, detect_red_surface, select_viewpoint):
        path = Path(inspect.getfile(obj))
        source_hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
        # Retain assessor/generator/policy bytes so later changes do not erase provenance.
        (output/path.name).write_bytes(path.read_bytes())
    report = dict(schema_version=1, scope='synthetic offline software-contract development campaign',
                  physical_execution=False, learned_reasoning=False,
                  independent_observation_window=False,
                  notes=['Three repeated observations exercise disposition propagation only.',
                         'Partial connected occlusion can remain SELECTED; no visibility model is claimed.',
                         'Image/depth edits do not establish a real or simulated sensor-fault response.'],
                  versions=dict(python=platform.python_version(), cv2=cv2.__version__, numpy=np.__version__),
                  source_sha256=source_hashes, input_sha256=hashes, cases=results,
                  passed=sum(r['passed'] for r in results), total=len(results))
    (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must be new; earlier evidence is never overwritten')
    report = run(args.output)
    print(json.dumps(dict(passed=report['passed'], total=report['total'], output=str(args.output))))
    raise SystemExit(0 if report['passed'] == report['total'] else 1)


if __name__ == '__main__':
    main()

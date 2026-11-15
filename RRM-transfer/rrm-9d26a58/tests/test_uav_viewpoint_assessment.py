"""Synthetic raw-frame fixtures for downstream viewpoint-evidence assessment."""
import hashlib
import inspect
import json
import math
from pathlib import Path
import subprocess
import sys

import unittest

try:
    import cv2
    import numpy as np
    __import__("pytest")
except ImportError as error:
    raise unittest.SkipTest("optional UAV perception tests: " + str(error))

from rrm.uav_marker import detect_red_surface
from rrm.uav_viewpoint import optical_forward_in_map, select_viewpoint

SCRIPT = Path(__file__).parents[1]/'scripts/assess_uav_viewpoint.py'


def assess(tmp_path, mode='good'):
    k = [184., 0., 240., 0., 184., 150., 0., 0., 1.]
    records = {'before': [], 'after': []}
    events = []
    for label in records:
        for i in range(3):
            color = np.zeros((300, 480, 3), dtype=np.uint8)
            if label == 'before' or mode == 'bad_gain':
                color[175:205, 220:252] = (255, 0, 0)
            else:
                color[170:208, 216:256] = (255, 0, 0)
            if mode == 'unknown_target' and label == 'after':
                color[:] = 0
            depth = np.full((300, 480), 3.5 if label == 'before' else 2.85, dtype=np.float32)
            intrinsic = k.copy()
            if mode == 'changed_calibration' and label == 'after':
                intrinsic[0] = 200.
            prefix = f'{label}-{i}'
            cv2.imwrite(str(tmp_path/(prefix+'.png')), cv2.cvtColor(color, cv2.COLOR_RGB2BGR))
            np.savez_compressed(tmp_path/(prefix+'.npz'), depth_m=depth, intrinsic=np.array(intrinsic))
            stamp = dict(sec=30 if label == 'before' else 40, nanosec=i*30000000)
            wall = (10 if label == 'before' else 20)+i*.1
            record = dict(kind='marker_observation', label=label, raw_prefix=prefix,
                          wall_unix=wall+.01, image_receipt_wall_unix=wall,
                          observation=detect_red_surface(color, depth, intrinsic),
                          frame='camera_left', image_size=[480, 300], intrinsic=intrinsic,
                          stamp_sec=stamp['sec'], stamp_nanosec=stamp['nanosec'],
                          headers={name: dict(stamp=stamp, frame_id='camera_left') for name in ('rgb', 'depth', 'info')},
                          depth_encoding='32FC1', exact_stamp_pair=True)
            records[label].append(record)
            events.append(record)
    q = [0., math.sqrt(.5), 0., math.sqrt(.5)]
    axis = optical_forward_in_map(q)
    decision = select_viewpoint([r['observation'] for r in records['before']], [480, 300], axis)
    events.extend([
        dict(kind='marker_policy_provenance', wall_unix=9.,
             policy_sha256=hashlib.sha256(Path(inspect.getfile(select_viewpoint)).read_bytes()).hexdigest(),
             detector_sha256=hashlib.sha256(Path(inspect.getfile(detect_red_surface)).read_bytes()).hexdigest()),
        dict(kind='marker_viewpoint_decision', wall_unix=10.3, decision=decision,
             optical_axis_map=axis if mode != 'bad_axis_binding' else [0., 1., 0.],
             transform_observation_index=0, transform=dict(
                 header=dict(frame_id='map', stamp=dict(sec=30, nanosec=0)), child_frame_id='camera_left',
                 transform=dict(rotation=dict(zip('xyzw', q))))),
        dict(kind='odom', wall_unix=10.35, data=dict(pose=dict(pose=dict(position=dict(x=0., y=0., z=2.))))),
        dict(kind='viewpoint_goal', wall_unix=10.4,
             target_map_m=[decision['delta_map_m'][0]+(.2 if mode == 'bad_goal' else 0), 0., 2.]),
        dict(kind='summary', wall_unix=21., data=dict(marker_viewpoint_action=True, error=None,
             pending_goal_unreconciled=False, navigate_action=True, land_action=True,
             final_disarmed=True, final_on_ground=True))])
    truth = [dict(wall_time_unix_s=9.9+i*.1,
                  rigid_body_position_m=[0. if i < 15 else decision['translation_m'], 0., 2.],
                  rigid_body_linear_velocity_m_s=[0., 0., 0.]) for i in range(113)]
    if mode == 'missing_physics':
        truth = []
    files = {}
    for name, values in [('events', sorted(events, key=lambda e: e['wall_unix'])), ('truth', truth)]:
        path = tmp_path/(name+'.jsonl')
        path.write_text(''.join(json.dumps(v)+'\n' for v in values))
        files[name] = path
    contact = dict(verdict='FAIL' if mode == 'contact_failure' else 'PASS',
                   input_sha256={k: hashlib.sha256(v.read_bytes()).hexdigest() for k, v in files.items()})
    if mode == 'unbound_contacts':
        contact['input_sha256']['events'] = 'wrong'
    path = tmp_path/'contact.json'
    path.write_text(json.dumps(contact))
    files['contact-assessment'] = path
    output = tmp_path/'assessment.json'
    command = [sys.executable, str(SCRIPT), '--raw-dir', str(tmp_path), '--output', str(output)]
    for key, path in files.items():
        command.extend(['--'+key, str(path)])
    subprocess.run(command, check=True, capture_output=True, text=True)
    return json.loads(output.read_text())


def test_raw_viewpoint_evidence_passes(tmp_path):
    assert assess(tmp_path)['verdict'] == 'PASS'


def test_no_resolution_gain_fails(tmp_path):
    assert assess(tmp_path, 'bad_gain')['verdict'] == 'FAIL'


def test_changed_calibration_cannot_supply_gain(tmp_path):
    assert assess(tmp_path, 'changed_calibration')['verdict'] == 'FAIL'


def test_goal_must_match_selected_motion(tmp_path):
    assert assess(tmp_path, 'bad_goal')['verdict'] == 'FAIL'


def test_axis_must_match_acquisition_transform(tmp_path):
    assert assess(tmp_path, 'bad_axis_binding')['verdict'] == 'FAIL'


def test_unknown_target_withholds_credit(tmp_path):
    assert assess(tmp_path, 'unknown_target')['verdict'] == 'NOT DEMONSTRATED'


def test_missing_physics_withholds_credit(tmp_path):
    assert assess(tmp_path, 'missing_physics')['verdict'] == 'NOT DEMONSTRATED'


def test_other_case_contact_assessment_cannot_pass(tmp_path):
    assert assess(tmp_path, 'unbound_contacts')['verdict'] == 'NOT DEMONSTRATED'


def test_contact_failure_overrides_quality(tmp_path):
    assert assess(tmp_path, 'contact_failure')['verdict'] == 'FAIL'

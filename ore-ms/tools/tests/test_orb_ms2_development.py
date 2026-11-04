"""Analytic adapter contracts; no network/checkpoint/reference dataset required."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import orb_ms2_development as adapter


def plan():
    return {'time_ns': [10**18 + i * 100_000_000 for i in range(5)],
            'right_time_ns': [10**18 + i * 100_000_000 + 9 for i in range(5)], 'modality': 'rgb'}


def journal(p=None):
    p = p or plan()
    rows = []
    for i, time in enumerate(p['time_ns']):
        matrix = np.eye(4)
        matrix[0, 3] = -i * .25
        rows.append({'index': i, 'time_ns': time, 'right_time_ns': p['right_time_ns'][i],
                     'state': 2, 'map_id': 5, 'T_camera_from_world': matrix.tolist()})
    return rows + [{'terminal_success': True, 'processed_frames': len(rows)}]


def test_rigid_inverse_direction_and_metric_scale():
    tcw = np.array([[0., -1, 0, 2], [1, 0, 0, -3], [0, 0, 1, 4], [0, 0, 0, 1]])
    got = np.array(adapter.invert_native(tcw))
    np.testing.assert_array_equal(got[:3, 3], [3, 2, -4])
    np.testing.assert_array_equal(got @ tcw, np.eye(4))
    output = adapter.publish(plan(), journal(), 0)
    assert output['terminal_success']
    assert output['poses'][-1][0][3] == 1.0
    assert output['segments'] == [0] * 5


@pytest.mark.parametrize('index,entry', [(0, np.nan), (0, 2.0), (15, 0.0)])
def test_bad_native_pose_is_rejected(index, entry):
    matrix = np.eye(4).reshape(-1)
    matrix[index] = entry
    with pytest.raises(ValueError, match='nonrigid'):
        adapter.invert_native(matrix.reshape(4, 4))


def test_failure_does_not_bridge_or_erase_source():
    rows = journal()
    rows[1].update(T_camera_from_world=None, state=3)
    rows[4]['map_id'] = 9
    got = adapter.publish(plan(), rows, 0)
    assert got['poses'][1] is None
    assert got['segments'] == [0, None, 1, 1, 2]
    assert got['time_ns'] == plan()['time_ns']


def test_crash_retains_unprocessed_tail_as_missing():
    got = adapter.publish(plan(), journal()[:2], -11)
    assert not got['terminal_success']
    assert got['segments'] == [0, 0, None, None, None]
    assert got['pose_frames'] == 2 and got['source_frames'] == 5


def test_shutdown_failure_retains_processed_outputs():
    got = adapter.publish(plan(), journal()[:-1], 1)
    assert not got['terminal_success'] and got['pose_frames'] == 5


@pytest.mark.parametrize('field,value', [('time_ns', 1e18), ('time_ns', True),
                                       ('index', False), ('right_time_ns', 1e18)])
def test_equal_valued_float_bool_clock_or_index_rejected(field, value):
    rows = journal()
    rows[0][field] = value
    with pytest.raises(ValueError, match='clock mismatch'):
        adapter.publish(plan(), rows, 0)


def test_reordered_or_duplicate_outputs_rejected():
    rows = journal()
    rows[0], rows[1] = rows[1], rows[0]
    with pytest.raises(ValueError): adapter.publish(plan(), rows, 0)
    with pytest.raises(ValueError): adapter.publish(plan(), journal() + journal(), 0)


def test_untracked_identity_is_not_a_pose():
    rows = journal()
    rows[0]['state'] = 1
    with pytest.raises(ValueError, match='tracking/map'):
        adapter.publish(plan(), rows, 0)


def test_source_time_gap_breaks_segment_without_interpolation():
    p = plan()
    p['time_ns'][2:] = [t + 200_000_000 for t in p['time_ns'][2:]]
    got = adapter.publish(p, journal(p), 0)
    assert got['segments'] == [0, 0, 1, 1, 1]


@pytest.fixture
def source(tmp_path):
    records = []
    for i in range(3):
        row = {'stem': f'{i:06}', 'cameras': {},
               'references': {'never_read': '/nonexistent/forbidden-reference.txt'}}
        for sensor in ('rgb', 'thr'):
            camera = {'K': [[40., 0, 30.], [0, 40., 15.], [0, 0, 1]], 'baseline_m': .3}
            for side in ('left', 'right'):
                raw = (np.arange(2048, dtype=np.uint16).reshape(32, 64) + 3000 + i)
                if sensor == 'rgb': raw = np.repeat((raw % 256).astype(np.uint8)[..., None], 3, axis=2)
                path = tmp_path / sensor / side / (row['stem'] + '.png')
                path.parent.mkdir(parents=True, exist_ok=True)
                assert cv2.imwrite(str(path), raw)
                camera[side] = {'path': str(path.relative_to(tmp_path)), 'sha256': adapter.sha(path),
                                'dtype': str(raw.dtype), 'shape': list(raw.shape),
                                'recorded_time_ns': 10**18 + i*100_000_000 + (9 if side == 'right' else 0)}
            row['cameras'][sensor] = camera
        records.append(row)
    value = {'sequence': '_2021-08-13-21-18-04', 'records': records}
    manifest = tmp_path / 'source.json'
    manifest.write_text(json.dumps(value))
    return tmp_path, manifest, value


def test_rgb_prepare_exact_pixels_and_calibrated_settings(source):
    root, manifest, source = source
    output = root / 'rgb-prepared'
    p = adapter.prepare(root, manifest, 'rgb', output, 0)
    src = root / source['records'][0]['cameras']['rgb']['left']['path']
    assert src.read_bytes() == (output / 'left/000000.png').read_bytes()
    np.testing.assert_array_equal(cv2.imread(str(src)), cv2.imread(str(output / 'left/000000.png')))
    fs = cv2.FileStorage(str(output / 'settings.yaml'), cv2.FILE_STORAGE_READ)
    assert fs.getNode('Stereo.b').real() == .3
    assert fs.getNode('Camera1.fx').real() * fs.getNode('Stereo.b').real() == 12.
    assert fs.getNode('loopClosing').real() == 0
    fs.release()
    assert p['time_ns'][1] - p['time_ns'][0] == 100_000_000
    assert not p['reference_payload_read']
    with pytest.raises(FileExistsError): adapter.prepare(root, manifest, 'rgb', output, 0)


def test_thermal_pixels_match_independent_order_statistics(source):
    root, manifest, source = source
    output = root / 'thr-prepared'
    p = adapter.prepare(root, manifest, 'thr', output, 0)
    raw = cv2.imread(str(root / source['records'][0]['cameras']['thr']['left']['path']), -1)
    lo, hi = round(raw.size*.01), round(raw.size*.99)-1
    bounds = np.partition(raw.flatten(), [lo, hi])[[lo, hi]]
    expected = ((np.clip(raw, *bounds)-bounds[0]) / (int(bounds[1])-int(bounds[0])) * 255).astype(np.uint8)
    expected = cv2.createCLAHE(clipLimit=2., tileGridSize=(8, 8)).apply(expected)
    expected = cv2.bilateralFilter(expected, 5, 20, 15)
    np.testing.assert_array_equal(expected, cv2.imread(str(output / 'left/000000.png'), -1))
    assert p['thermal_recipe'] == adapter.RECIPE


@pytest.mark.parametrize('mutation', ['reserve', 'bool_time', 'reorder', 'outside', 'baseline', 'hash'])
def test_invalid_source_refused_before_preparation(source, mutation):
    root, manifest, value = source
    camera = value['records'][0]['cameras']['rgb']
    if mutation == 'reserve': value['sequence'] = '_2021-08-13-22-16-02'
    elif mutation == 'bool_time': camera['left']['recorded_time_ns'] = True
    elif mutation == 'reorder': value['records'].reverse()
    elif mutation == 'outside': camera['left']['path'] = '../forbidden.png'
    elif mutation == 'baseline': camera['baseline_m'] = -.3
    elif mutation == 'hash': camera['left']['sha256'] = '0'*64
    manifest.write_text(json.dumps(value))
    with pytest.raises(ValueError): adapter.prepare(root, manifest, 'rgb', root / 'bad', 0)
    assert not (root / 'bad').exists()

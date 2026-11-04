"""Independent reference-free audit of EXP134 inputs and received native outputs."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def audit(root, protocol, output):
    root = Path(root).resolve()
    specification = json.loads(Path(protocol).read_text())
    checks = 0

    def check(value, description):
        nonlocal checks
        if not value:
            raise ValueError(description)
        checks += 1

    for name, expected in specification['source_sha256'].items():
        check(digest(root / name) == expected, 'frozen source: ' + name)
    summaries = []
    for case in specification['cases']:
        source = json.loads((root / case['manifest']).read_text())
        calibration = json.loads((root / case['calibration']).read_text())
        check(digest(root / case['manifest']) == case['manifest_sha256'], 'source manifest pin')
        check(digest(root / case['calibration']) == case['calibration_sha256'], 'calibration pin')
        check('ms2/' + source['sequence'] in specification['exposed_recordings'], 'development identity')
        for modality in ('rgb', 'thr'):
            directory = root / specification['output'] / (case['condition'] + '-' + modality)
            plan = json.loads((directory / 'plan.json').read_text())
            estimate = json.loads((directory / 'estimate.json').read_text())
            native = [json.loads(line) for line in (directory / 'native.jsonl').read_text().splitlines()]
            native_rows = [row for row in native if 'index' in row]
            check(plan['recording'] == 'ms2/' + source['sequence'], 'plan source identity')
            for path, expected in plan['frozen_files'].items():
                check(digest(path) == expected, 'source/prepared hash')
            k = np.asarray(calibration['K_' + modality + 'L']['values'], dtype=float)
            baseline = -calibration['T_' + modality + 'R']['values'][0][0] / 1000
            check(plan['calibration']['baseline_m'] == baseline, 'provided mm-to-m baseline')
            check(plan['calibration']['bf'] == baseline*k[0, 0], 'provided fx times baseline')
            fs = cv2.FileStorage(str(directory / 'settings.yaml'), cv2.FILE_STORAGE_READ)
            check(fs.getNode('Stereo.b').real() == baseline, 'settings baseline')
            check(fs.getNode('Camera1.fx').real() == k[0, 0], 'settings fx')
            check(fs.getNode('loopClosing').real() == 0, 'loop closure disabled')
            fs.release()
            times = [row['cameras'][modality]['left']['recorded_time_ns'] for row in source['records']]
            check(estimate['time_ns'] == times and all(type(x) is int for x in estimate['time_ns']), 'exact source clocks')
            check(len(estimate['poses']) == len(estimate['segments']) == len(times), 'all source slots retained')
            previous_segment = None
            previous_map = None
            closed_segments = set()
            for index, row in enumerate(source['records']):
                check(plan['stems'][index] == row['stem'], 'official source stem')
                for side in ('left', 'right'):
                    entry = row['cameras'][modality][side]
                    source_path = root / entry['path']
                    image_path = directory / side / (row['stem'] + '.png')
                    check(digest(source_path) == entry['sha256'], 'original image pin')
                    original = cv2.imread(str(source_path), cv2.IMREAD_UNCHANGED)
                    prepared = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
                    if modality == 'rgb':
                        check(source_path.read_bytes() == image_path.read_bytes(), 'RGB encoded bytes preserved')
                        check(np.array_equal(original, prepared), 'RGB pixels preserved')
                    else:
                        # Independent order statistics, not adapter/producer calls.
                        ranks = [round(original.size*.01), round(original.size*.99)-1]
                        lo, hi = np.partition(original.reshape(-1), ranks)[ranks]
                        expected = ((np.clip(original, lo, hi)-lo)/(int(hi)-int(lo))*255).astype(np.uint8)
                        expected = cv2.createCLAHE(clipLimit=2., tileGridSize=(8, 8)).apply(expected)
                        expected = cv2.bilateralFilter(expected, 5, 20, 15)
                        check(original.dtype == np.uint16, 'native thermal precision')
                        check(np.array_equal(expected, prepared), 'independent thermal recipe pixels')
                n = native_rows[index] if index < len(native_rows) else None
                if n is not None:
                    check(type(n['time_ns']) is int and n['time_ns'] == times[index], 'native integer timestamp')
                    check(n['right_time_ns'] == row['cameras'][modality]['right']['recorded_time_ns'], 'right timestamp')
                    check(n['index'] == index, 'native chronological index')
                pose = estimate['poses'][index]
                segment = estimate['segments'][index]
                if n is None or n['T_camera_from_world'] is None:
                    check(pose is None and segment is None, 'failed/missing output remains null')
                    if previous_segment is not None:
                        closed_segments.add(previous_segment)
                    previous_segment = None
                else:
                    check(n['state'] == 2, 'only tracking-OK poses admitted')
                    inverse = np.linalg.inv(np.asarray(n['T_camera_from_world']))
                    check(np.allclose(pose, inverse, rtol=0, atol=1e-5), 'independent Tcw inverse')
                    check(type(segment) is int, 'valid segment identity')
                    check(segment not in closed_segments, 'closed segment never resumes')
                    if previous_segment is not None and previous_segment != segment:
                        closed_segments.add(previous_segment)
                    if previous_segment is not None and n['map_id'] == previous_map and times[index]-times[index-1] <= 150_000_000:
                        check(segment == previous_segment, 'within-map continuity')
                    elif index and estimate['poses'][index-1] is not None:
                        check(segment > estimate['segments'][index-1], 'map/time break')
                    previous_segment, previous_map = segment, n['map_id']
            check(estimate['reference_payload_read'] is False, 'no reference use')
            check(estimate['common_rgb_frame_verified'] is (modality == 'rgb'), 'no thermal common-frame overclaim')
            summaries.append({'condition': case['condition'], 'modality': modality,
                              'source_frames': len(times), 'processed_frames': len(native_rows),
                              'pose_frames': sum(p is not None for p in estimate['poses']),
                              'segments': estimate['segment_count'],
                              'terminal_success': estimate['terminal_success'],
                              'native_calibration_verified': estimate['native_calibration_verified'],
                              'estimate_sha256': digest(directory / 'estimate.json')})
    result = {'pass': True, 'independent_checks': checks, 'runs': summaries,
              'all_native_calls_completed': all(r['terminal_success'] and r['processed_frames'] == r['source_frames'] for r in summaries),
              'all_native_calibrations_verified': all(r['native_calibration_verified'] for r in summaries),
              'all_runs_have_tracking_output': all(r['pose_frames'] > 0 for r in summaries),
              'benchmark_execution_ready': False, 'reference_payload_read': False,
              'scope': 'bounded exposed-development adapter evidence only'}
    with Path(output).open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--protocol', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    audit(args.root, args.protocol, args.output)

"""Pinned ORB stereo adapter on explicitly exposed MS² development inputs only.

This adapter publishes camera-local received poses, not a final benchmark result.
It never reads manifest reference paths or estimates a scale, clock or extrinsic.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil
import re
import subprocess
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ORB_REVISION = '4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4'
RECIPE = 'td3-firestereo-v2-clahe-bilateral-v1'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def thermal_function():
    # Execute only the unchanged pure function; avoid importing Torch/VO code.
    source = ROOT / 'MAC-VO-main/DataLoader/Dataset/MS2.py'
    tree = ast.parse(source.read_text())
    function, = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                 and node.name == 'thermal_producer_replay']
    module = ast.Module(body=[function], type_ignores=[])
    namespace = {'np': np, 'cv2': cv2}
    exec(compile(module, str(source), 'exec'), namespace)
    return namespace['thermal_producer_replay']


def safe_path(root, value):
    path = (root / value).resolve()
    if not path.is_relative_to(root):
        raise ValueError('source path outside root')
    return path


def calibration(camera):
    k = np.asarray(camera['K'], dtype=float)
    baseline = camera['baseline_m']
    shape = camera['left']['shape'][:2]
    if (k.shape != (3, 3) or not np.isfinite(k).all()
            or not np.array_equal(k[2], [0, 0, 1])
            or k[0, 1] != 0 or k[1, 0] != 0 or k[0, 0] <= 0 or k[1, 1] <= 0
            or type(baseline) not in (int, float) or not np.isfinite(baseline) or baseline <= 0
            or len(shape) != 2 or any(type(x) is not int or x <= 0 for x in shape)):
        raise ValueError('invalid rectified pinhole calibration')
    return k, float(baseline), shape


def settings(camera, fps):
    k, baseline, (height, width) = calibration(camera)
    return '\n'.join([
        '%YAML:1.0', 'File.version: "1.0"', 'Camera.type: "Rectified"',
        f'Camera1.fx: {float(k[0, 0])}', f'Camera1.fy: {float(k[1, 1])}',
        f'Camera1.cx: {float(k[0, 2])}', f'Camera1.cy: {float(k[1, 2])}',
        f'Camera.width: {width}', f'Camera.height: {height}', f'Camera.fps: {fps}',
        'Camera.RGB: 0', f'Stereo.b: {baseline}', 'Stereo.ThDepth: 35.0',
        'ORBextractor.nFeatures: 2000', 'ORBextractor.scaleFactor: 1.2',
        'ORBextractor.nLevels: 8', 'ORBextractor.iniThFAST: 20',
        'ORBextractor.minThFAST: 7', 'loopClosing: 0',
        'Viewer.KeyFrameSize: 0.6', 'Viewer.KeyFrameLineWidth: 2.0',
        'Viewer.GraphLineWidth: 1.0', 'Viewer.PointSize: 2.0',
        'Viewer.CameraSize: 0.7', 'Viewer.CameraLineWidth: 3.0',
        'Viewer.ViewpointX: 0.0', 'Viewer.ViewpointY: -100.0',
        'Viewer.ViewpointZ: -0.1', 'Viewer.ViewpointF: 2000.0', ''])


def prepare(root, manifest, modality, output, seed):
    if modality not in ('rgb', 'thr') or type(seed) is not int or seed not in (0, 1, 2):
        raise ValueError('explicit modality and seed 0/1/2 required')
    root, manifest, output = Path(root).resolve(), Path(manifest).resolve(), Path(output).resolve()
    source = json.loads(manifest.read_text())
    protocol_path = ROOT / 'experiments/independent-benchmark-2026-11-04.json'
    protocol = json.loads(protocol_path.read_text())
    recording = 'ms2/' + source['sequence']
    if recording not in protocol['partitions']['development'] or recording in protocol['reported_reserves']:
        raise ValueError('only previously exposed development recordings allowed')
    records = source['records']
    stems = [r['stem'] for r in records]
    if (len(records) < 2 or any(not isinstance(s, str) or len(s) != 6 or not s.isdigit() for s in stems)
            or stems != sorted(set(stems))):
        raise ValueError('at least two chronological distinct official stems required')
    frozen = {str(manifest): sha(manifest), str(protocol_path): sha(protocol_path),
              str(Path(__file__).resolve()): sha(__file__),
              str(ROOT / 'MAC-VO-main/DataLoader/Dataset/MS2.py'):
              sha(ROOT / 'MAC-VO-main/DataLoader/Dataset/MS2.py')}
    clocks = {key: [] for key in ('rgb_left', 'rgb_right', 'thr_left', 'thr_right')}
    signatures = {}
    # Check all four streams; never silently drop a missing/bad source row.
    for row in records:
        for sensor in ('rgb', 'thr'):
            camera = row['cameras'][sensor]
            k, baseline, shape = calibration(camera)
            signature = (k.tolist(), baseline, shape)
            if sensor in signatures and signatures[sensor] != signature:
                raise ValueError('variable source geometry')
            signatures[sensor] = signature
            for side in ('left', 'right'):
                rec = camera[side]
                stamp = rec['recorded_time_ns']
                previous = clocks[sensor + '_' + side]
                if type(stamp) is not int or not 0 <= stamp < 2**63 or (previous and stamp <= previous[-1]):
                    raise ValueError('strict increasing integer nanosecond source clocks required')
                previous.append(stamp)
                path = safe_path(root, rec['path'])
                if path.stem != row['stem'] or path.suffix != '.png' or sha(path) != rec['sha256']:
                    raise ValueError('source stem or hash mismatch')
                frozen[str(path)] = rec['sha256']
    output.mkdir(parents=True, exist_ok=False)
    times = clocks[modality + '_left']
    fps = max(1, int(round(1e9 / np.median(np.diff(np.asarray(times, dtype=np.int64))))))
    (output / 'settings.yaml').write_text(settings(records[0]['cameras'][modality], fps))
    thermal = thermal_function()
    prepared = []
    with (output / 'inputs.tsv').open('x') as stream:
        for index, row in enumerate(records):
            paths = []
            for side in ('left', 'right'):
                rec = row['cameras'][modality][side]
                raw = cv2.imread(str(safe_path(root, rec['path'])), cv2.IMREAD_UNCHANGED)
                expected = 'uint8' if modality == 'rgb' else 'uint16'
                if (raw is None or str(raw.dtype) != expected or rec['dtype'] != expected
                        or list(raw.shape) != rec['shape']
                        or raw.ndim != (3 if modality == 'rgb' else 2)
                        or (modality == 'rgb' and raw.shape[2] != 3)):
                    raise ValueError('decoded source image schema mismatch')
                image = raw if modality == 'rgb' else thermal(raw)
                path = output / side / (row['stem'] + '.png')
                path.parent.mkdir(exist_ok=True)
                if modality == 'rgb':
                    shutil.copyfile(safe_path(root, rec['path']), path)
                elif not cv2.imwrite(str(path), image):
                    raise ValueError('cannot write canonical image')
                if not np.array_equal(cv2.imread(str(path), cv2.IMREAD_UNCHANGED), image):
                    raise ValueError('prepared PNG pixel roundtrip mismatch')
                paths.append(str(path))
                prepared.append({'path': str(path), 'sha256': sha(path),
                                 'pixel_sha256': hashlib.sha256(image.tobytes()).hexdigest()})
            fields = [str(times[index]), str(clocks[modality + '_right'][index]), row['stem'], *paths]
            if any('\t' in field or '\n' in field for field in fields):
                raise ValueError('tab/newline in TSV fields')
            stream.write('\t'.join(fields) + '\n')
    for name in ('settings.yaml', 'inputs.tsv'):
        frozen[str(output / name)] = sha(output / name)
    for item in prepared:
        frozen[item['path']] = item['sha256']
    plan = {'recording': recording, 'modality': modality, 'seed': seed,
            'orb_revision': ORB_REVISION, 'time_ns': times,
            'right_time_ns': clocks[modality + '_right'], 'stems': stems,
            'source_manifest_sha256': sha(manifest), 'frozen_files': frozen,
            'prepared_images': prepared, 'fps_setting_from_rounded_median_source_cadence': fps,
            'rectification_or_resize': False, 'reference_payload_read': False,
            'thermal_recipe': RECIPE if modality == 'thr' else None,
            'native_tracking_time': 'left clock minus first left clock, seconds',
            'scope': 'bounded exposed development smoke, camera-local only',
            'opencv_version': cv2.__version__, 'numpy_version': np.__version__}
    k, baseline, _ = calibration(records[0]['cameras'][modality])
    plan['calibration'] = {'fx': float(k[0, 0]), 'baseline_m': baseline,
                           'bf': float(k[0, 0] * baseline)}
    write(output / 'plan.json', plan)
    return plan


def invert_native(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.array_equal(matrix[3], [0, 0, 0, 1])
            or np.linalg.norm(matrix[:3, :3].T @ matrix[:3, :3] - np.eye(3)) > 1e-5
            or abs(np.linalg.det(matrix[:3, :3]) - 1) > 1e-5):
        raise ValueError('nonrigid native pose')
    # Exact rigid inverse; retain harmless native float rotation roundoff.
    result = np.eye(4)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -matrix[:3, :3].T @ matrix[:3, 3]
    return result.tolist()


def publish(plan, journal, returncode):
    frames = [r for r in journal if 'index' in r]
    terminal = [r for r in journal if 'terminal_success' in r]
    if len(frames) > len(plan['time_ns']) or len(terminal) > 1:
        raise ValueError('duplicate/excess native output')
    poses, segments, states = [], [], []
    segment, active, last_map = -1, False, None
    for i, stamp in enumerate(plan['time_ns']):
        row = frames[i] if i < len(frames) else None
        if row is not None and (type(row['index']) is not int or row['index'] != i
                or type(row['time_ns']) is not int or row['time_ns'] != stamp
                or type(row['right_time_ns']) is not int
                or row['right_time_ns'] != plan['right_time_ns'][i]):
            raise ValueError('native source index/clock mismatch')
        value = row.get('T_camera_from_world') if row else None
        if value is None:
            poses.append(None); segments.append(None); active = False
        else:
            if row['state'] != 2 or type(row['map_id']) is not int or row['map_id'] < 0:
                raise ValueError('native pose without successful tracking/map identity')
            value = invert_native(value)
            if not active or row['map_id'] != last_map or (i and stamp-plan['time_ns'][i-1] > 150_000_000):
                segment += 1
            poses.append(value); segments.append(segment)
            active, last_map = True, row['map_id']
        states.append(row)
    success = (returncode == 0 and len(frames) == len(poses) and len(terminal) == 1
               and terminal[0]['terminal_success'] is True
               and terminal[0]['processed_frames'] == len(poses))
    return {'time_ns': plan['time_ns'], 'poses': poses, 'segments': segments,
            'terminal_success': success, 'native_returncode': returncode,
            'coordinate_frame': plan['modality'] + '_rectified_left_camera',
            'pose_convention': 'T_world_from_camera, metres; native arbitrary map origin',
            'common_rgb_frame_verified': plan['modality'] == 'rgb',
            'reference_payload_read': False, 'rows': states,
            'source_frames': len(poses), 'pose_frames': sum(p is not None for p in poses),
            'segment_count': segment + 1,
            'scope': 'received camera-local development output; no final benchmark scoring'}


def validate_build_flags(flags):
    for component, value in flags.items():
        standards = re.findall(r'-std=(?:gnu\+\+|c\+\+)(\d+)', value)
        if not standards or standards[-1] != '11' or '-march=native' not in value:
            raise ValueError('incompatible Eigen build ABI: ' + component)


def run(directory, timeout=300):
    directory = Path(directory).resolve()
    plan = json.loads((directory / 'plan.json').read_text())
    for path, expected in plan['frozen_files'].items():
        if sha(path) != expected:
            raise ValueError('prepared/source file changed: ' + path)
    runner = Path('/opt/ore-ms-adapter/build/orb_ms2_runner')
    vocabulary = Path('/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt')
    revision = subprocess.check_output(['git', '-C', '/opt/ORB_SLAM3', 'rev-parse', 'HEAD'], text=True).strip()
    if revision != ORB_REVISION:
        raise ValueError('wrong ORB revision')
    flag_paths = {
        'orb': '/opt/ORB_SLAM3/build/CMakeFiles/ORB_SLAM3.dir/flags.make',
        'g2o': '/opt/ORB_SLAM3/Thirdparty/g2o/build/CMakeFiles/g2o.dir/flags.make',
        'adapter': '/opt/ore-ms-adapter/build/CMakeFiles/orb_ms2_runner.dir/flags.make'}
    flags = {name: Path(path).read_text() for name, path in flag_paths.items()}
    validate_build_flags(flags)
    command = [str(runner), str(vocabulary), str(directory / 'settings.yaml'),
               str(directory / 'inputs.tsv'), str(directory / 'native.jsonl'), str(plan['seed']), '1']
    identity = {'command': command, 'runner_sha256': sha(runner),
                'vocabulary_sha256': sha(vocabulary),
                'library_sha256': sha('/opt/ORB_SLAM3/lib/libORB_SLAM3.so'),
                'plan_sha256': sha(directory / 'plan.json'), 'orb_revision': revision}
    upstream_diff = subprocess.check_output(['git', '-C', '/opt/ORB_SLAM3', 'diff'], text=True)
    identity['upstream_patch'] = upstream_diff
    identity['upstream_patch_sha256'] = hashlib.sha256(upstream_diff.encode()).hexdigest()
    identity['compile_flags'] = flags
    write(directory / 'execution-lock.json', identity)
    start = time.monotonic_ns()
    with (directory / 'native.log').open('x') as stream:
        try:
            result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, timeout=timeout)
            returncode = result.returncode
        except subprocess.TimeoutExpired:
            returncode = 124
    journal = directory / 'native.jsonl'
    rows = [json.loads(line) for line in journal.read_text().splitlines()] if journal.exists() else []
    estimate = publish(plan, rows, returncode)
    configurations = [row['configuration'] for row in rows if 'configuration' in row]
    estimate['native_configuration'] = configurations
    estimate['native_calibration_verified'] = (len(configurations) == 1
        and configurations[0]['rerectifies'] is False and configurations[0]['resizes'] is False
        and all(np.isclose(configurations[0][key], value, rtol=5e-7, atol=1e-7)
                for key, value in plan['calibration'].items()))
    if not estimate['native_calibration_verified']:
        estimate['terminal_success'] = False
    estimate['wall_runtime_ns_including_startup_shutdown'] = time.monotonic_ns() - start
    estimate['execution_lock_sha256'] = sha(directory / 'execution-lock.json')
    write(directory / 'estimate.json', estimate)
    return estimate


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    prep = sub.add_parser('prepare')
    for name in ('root', 'manifest', 'output'):
        prep.add_argument('--' + name, type=Path, required=True)
    prep.add_argument('--modality', choices=['rgb', 'thr'], required=True)
    prep.add_argument('--seed', type=int, choices=[0, 1, 2], default=0)
    execute = sub.add_parser('run')
    execute.add_argument('--directory', type=Path, required=True)
    execute.add_argument('--timeout', type=int, default=300)
    args = parser.parse_args()
    if args.command == 'prepare':
        value = prepare(args.root, args.manifest, args.modality, args.output, args.seed)
        print(json.dumps({'recording': value['recording'], 'frames': len(value['time_ns'])}))
    else:
        value = run(args.directory, args.timeout)
        print(json.dumps({k: value[k] for k in ['terminal_success', 'source_frames', 'pose_frames', 'segment_count']}))
        raise SystemExit(0 if value['terminal_success'] else 1)

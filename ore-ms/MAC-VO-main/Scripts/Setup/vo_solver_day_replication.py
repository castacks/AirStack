"""Host-side frozen plan and bounded six-run day-window replication orchestration."""
import argparse
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import json
from pathlib import Path
import subprocess
import time

DATASET = 'results/long-salon/prepared-001/datasets/middle'


def read(p): return json.loads(Path(p).read_text())
def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda: f.read(1024*1024), b''): h.update(b)
    return h.hexdigest()
def write(p, value): Path(p).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def array_parity(first, second):
    import numpy as np
    paths = sorted(first.rglob('*.npz')); other = sorted(second.rglob('*.npz'))
    if [p.relative_to(first) for p in paths] != [p.relative_to(second) for p in other]: raise ValueError('NPZ population differs')
    count = 0
    for p in paths:
        with np.load(p, allow_pickle=False) as a, np.load(second/p.relative_to(first), allow_pickle=False) as b:
            if set(a.files) != set(b.files): raise ValueError('NPZ keys differ')
            for key in a.files:
                x, y = a[key], b[key]
                if x.dtype != y.dtype or x.shape != y.shape or x.tobytes() != y.tobytes(): raise ValueError('array parity failure: '+str(p.relative_to(first))+'/'+key)
                count += 1
    return {'status': 'PASS', 'npz_files': len(paths), 'arrays_bitwise_equal': count}


def prepare(root, output):
    output.mkdir(parents=True, exist_ok=False); code = root/'MAC-VO-main'
    files = []
    for directory in ('DataLoader', 'Odometry', 'Module', 'Utility', 'Config', 'Scripts/Setup', 'Scripts/UnitTest'):
        files += [p for p in (code/directory).rglob('*') if p.is_file() and p.suffix in ('.py', '.yaml', '.yml', '.json')]
    files = [p for p in files if p.name not in ('evaluate_vo_solver_day_replication.py', 'validate_vo_solver_day_replication.py')]
    files += [code/'Docker/Dockerfile.solver-vo']
    images = sorted(p for p in (root/DATASET).rglob('*') if p.is_file())
    checkpoints = [root/'models/macslam'/name for name in ('MACVO_FrontendCov.pth', 'MACT_Offroad.pth')]
    expected = ['bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a', 'eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a']
    if [sha(p) for p in checkpoints] != expected: raise ValueError('checkpoint identity differs')
    evaluation = [root/'results/history-day-replication/proxy-001'/name for name in ('pose-proxy.json', 'protocol.json', 'validation.json')]
    reference = root/'data/tartandrive3/20251027/0_salon_so_day/super_odometry/odometry'
    evaluation += [reference/name for name in ('data.txt', 'info.yaml', 'interp_data.txt', 'interp_timestamps.txt', 'timestamps.txt')]
    image_id = subprocess.check_output(['docker', 'image', 'inspect', 'macvo:solver-vo', '--format', '{{.Id}}'], text=True).strip()
    if image_id != 'sha256:e1e72a2dc43630ffe47cf45c29278bd879d36b9bfb9fd6d621c1e88f88455b82': raise ValueError('frozen EXP084/085 image changed')
    rows = read(root/DATASET/'manifest.json')['pairs']
    if len(rows) != 100: raise ValueError('insufficient prepared interval')
    runs = []
    for camera in ('rgb', 'thermal'):
        for arm in ('original', 'passthrough', 'repair'):
            target = output/'runs'/arm
            module = 'Scripts.Setup.run_td3_execution_v3' if arm == 'original' else 'Scripts.Setup.run_td3_solver_replication'
            command = ['docker', 'run', '--rm', '--gpus', 'all', '--ipc=host', '-e', 'TORCHDYNAMO_DISABLE=1', '-e', 'OMP_NUM_THREADS=1', '-e', 'OPENBLAS_NUM_THREADS=1',
                '-v', str(root)+':/workspace', '-w', '/workspace/MAC-VO-main', image_id, 'python', '-m', module,
                '--dataset', '/workspace/'+DATASET, '--manifest', '/workspace/'+DATASET+'/manifest.json', '--output', '/workspace/'+str(target.relative_to(root)),
                '--modality', camera, '--frames', '100', '--start', '0', '--record-uncertainty', '--record-arbiter-events', '--acknowledge-provisional-geometry']
            if arm != 'original': command += ['--arm', arm]
            runs.append({'camera': camera, 'arm': arm, 'output_relative': str((target/camera).relative_to(root)), 'command': command})
    write(output/'protocol.json', {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(), 'base_commit': '0e4dc683f', 'image_id': image_id,
        'frames': 100, 'start': 0, 'input_rows': rows[:100], 'dataset_relative': DATASET, 'runs': runs, 'repair_delta_whitened': .1,
        'production_defaults_changed': False, 'reserved_data_used': False, 'window': 'day-middle', 'scope': 'same-runtime sequential bounded original/pass-through/repair VO; qualified proxy evaluated separately',
        'source_sha256': {str(p.relative_to(root)): sha(p) for p in sorted(set(files))},
        'input_sha256': {str(p.relative_to(root)): sha(p) for p in images},
        'checkpoint_sha256': {str(p.relative_to(root)): sha(p) for p in checkpoints},
        'evaluation_input_sha256': {str(p.relative_to(root)): sha(p) for p in evaluation}})
    print(json.dumps({'frozen_runs': len(runs), 'source_files': len(set(files)), 'prepared_files': len(images), 'image_id': image_id}))


def verify(root, protocol, evaluation=False):
    for name in ('source_sha256', 'input_sha256', 'checkpoint_sha256')+ (('evaluation_input_sha256',) if evaluation else ()):
        for path, digest in protocol[name].items():
            if sha(root/path) != digest: raise ValueError('frozen input changed: '+path)


def run(root, output):
    protocol = read(output/'protocol.json'); verify(root, protocol)
    if (output/'execution.json').exists(): raise FileExistsError('execution already started')
    logs = output/'logs'; logs.mkdir(); record = {'protocol_sha256': sha(output/'protocol.json'), 'status': 'RUNNING', 'runs': [], 'parity': {}}
    write(output/'execution.json', record)
    try:
        for spec in protocol['runs']:
            verify(root, protocol); log = logs/(spec['arm']+'-'+spec['camera']+'.log'); began = time.monotonic_ns()
            with log.open('wb') as f: process = subprocess.run(spec['command'], stdout=f, stderr=subprocess.STDOUT, timeout=900)
            entry = {'camera': spec['camera'], 'arm': spec['arm'], 'exit_code': process.returncode, 'host_elapsed_ns': time.monotonic_ns()-began,
                'log_relative': str(log.relative_to(root)), 'log_sha256': sha(log)}
            record['runs'].append(entry); write(output/'execution.json', record)
            if process.returncode: raise RuntimeError('bounded run failed: '+spec['arm']+'/'+spec['camera'])
            report = read(root/spec['output_relative']/'report.json')
            if report['status'] != 'PASS_EXECUTION_ONLY' or report['completed_frames'] != 100: raise ValueError('incomplete camera execution')
            if spec['arm'] == 'passthrough':
                # Delegate parity check in a CPU NumPy runtime so host needs only stdlib.
                first = '/workspace/'+str((output/'runs/original'/spec['camera']).relative_to(root)); second = '/workspace/'+spec['output_relative']
                source = 'import json;from pathlib import Path;from Scripts.Setup.vo_solver_day_replication import array_parity;print(json.dumps(array_parity(Path('+repr(first)+'),Path('+repr(second)+'))))'
                command = ['docker', 'run', '--rm', '--runtime=runc', '-v', str(root)+':/workspace', 'macvo:solver-contract', '-c', source]
                parity = json.loads(subprocess.check_output(command, text=True)); record['parity'][spec['camera']] = parity; write(output/'execution.json', record)
            print(json.dumps(entry), flush=True)
        verify(root, protocol); record['status'] = 'PASS_EXECUTION_CONTROLS_ONLY'
    except BaseException as error:
        record['status'] = 'FAILED'; record['error_type'] = type(error).__name__; record['error'] = str(error); raise
    finally: write(output/'execution.json', record)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('action', choices=('prepare', 'run')); p.add_argument('--root', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); {'prepare': prepare, 'run': run}[a.action](a.root,a.output)

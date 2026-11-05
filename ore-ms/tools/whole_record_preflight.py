"""EXP140 source/capacity preflight. No native tracking, reference values or clock fit."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'results/whole-record-preflight/source-001'
OUT = ROOT/'results/whole-record-preflight/run-001'
PLAN = ROOT/'experiments/whole-record-preflight-2026-11-04.json'
CASES = [('day', 'day-replication', 'ms2-day', 'exp-048.tar.gz'),
         ('night', 'night-pilot', 'ms2-night', 'exp-041.tar.gz'),
         ('rain', 'rain-replication', 'ms2-rain', 'exp-054.tar.gz')]


def read(p): return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()
def write(p, v):
    p = Path(p); p.parent.mkdir(parents=True, exist_ok=True)
    with p.open('x') as f: json.dump(v, f, indent=2, allow_nan=False); f.write('\n')


def restore():
    SOURCE.mkdir(parents=True, exist_ok=False)
    prior = read(ROOT/'experiments/droid-longer-prefix-2026-11-04.json')
    rows = []
    for condition, suffix, recovery, filename in CASES:
        base = Path('/root/ore-ms-recovery')/recovery
        manifest = read(base/'manifest.json')
        packet = base/filename
        if 'archive_sha256' in manifest: expected = manifest['archive_sha256']
        elif 'files' in manifest: expected = manifest['files'][filename]['sha256']
        else: expected = manifest['file_sha256'][filename]
        if sha(packet) != expected: raise ValueError('recovery packet mismatch')
        copied = SOURCE/(condition+'-recovery-manifest.json'); shutil.copyfile(base/'manifest.json', copied)
        case = next(r for r in prior['runs'] if r['condition'] == condition)
        sequence = case['recording'].split('/')[-1]
        prefix = 'ore-ms/results/ms2-'+suffix
        selected = {prefix+'/sync_data-tar-index.json', prefix+'/archives/sync_data-block-offsets.json',
                    prefix+'/archive-manifest.json'}
        selected.update(prefix+'/pilot/sync_data/'+sequence+'/'+m+'/img_'+s+'_timestamp.txt'
                        for m in ['rgb', 'thr'] for s in ['left', 'right'])
        found = []
        with tarfile.open(packet, 'r:gz') as tar:
            for member in tar:
                if member.name not in selected: continue
                if not member.isfile(): raise ValueError('nonregular metadata member')
                data = tar.extractfile(member).read()
                expected_member = manifest['member_sha256'][member.name]
                if hashlib.sha256(data).hexdigest() != expected_member: raise ValueError('metadata member mismatch')
                target = ROOT.parent/member.name
                if target.exists():
                    if sha(target) != expected_member: raise ValueError('conflicting restored metadata')
                else: target.parent.mkdir(parents=True, exist_ok=True); target.write_bytes(data)
                found.append({'path': str(target.relative_to(ROOT)), 'sha256': expected_member, 'bytes': len(data)})
        if len(found) != len(selected): raise ValueError('missing required metadata')
        rows.append({'condition': condition, 'sequence': sequence, 'source_base': 'results/ms2-'+suffix,
                     'packet_sha256': expected, 'restored': found, 'manifest': str(copied.relative_to(ROOT))})
    # Existing EXP114 packet proves which previously exposed windows can be recovered.
    base = Path('/root/ore-ms-recovery/ms2-broader-range')
    packet = base/'ms2-broader-range.tar.gz'
    if sha(packet) != 'be8efa7e58d0eb98359bff85c0a659efa172e04a211664d65765995e39c658f9':
        raise ValueError('prior broader packet mismatch')
    shutil.copyfile(base/'manifest.json', SOURCE/'broader-recovery-manifest.json')
    native = Path('/root/ore-ms-recovery/droid-source/droid_slam')
    for name in ['depth_video.py', 'trajectory_filler.py']:
        shutil.copyfile(native/name, SOURCE/name)
    write(SOURCE/'recovery.json', {'cases': rows, 'broader_packet_sha256': sha(packet),
          'selected_metadata_members': sum(len(c['restored']) for c in rows),
          'reference_values_parsed': False, 'new_camera_payload_restored': False})
    print('restored', sum(len(c['restored']) for c in rows), 'visual/index metadata members; no camera/reference extraction')


def freeze():
    recovery = read(SOURCE/'recovery.json')
    p = read(ROOT/'experiments/droid-longer-prefix-2026-11-04.json')
    files = list(SOURCE.iterdir()) + [Path(__file__), ROOT/'tools/whole_record_capacity_probe.py',
            ROOT/'tools/whole_record_preflight_audit.py', ROOT/'tools/droid_longer_worker.py',
            ROOT/'tools/droid_ms2_development.py', ROOT/'experiments/droid-longer-prefix-2026-11-04.json',
            ROOT/'experiments/ms2-broader-range-2026-11-02.json']
    for c in recovery['cases']:
        files.extend(ROOT/r['path'] for r in c['restored'])
        case = next(r for r in p['runs'] if r['condition'] == c['condition'])
        files.append(ROOT/case['manifest'])
        for row in read(ROOT/case['manifest'])['records']:
            for m in ['rgb', 'thr']:
                for s in ['left', 'right']: files.append(ROOT/row['cameras'][m][s]['path'])
    plan = {'date': '2026-11-04 America/New_York', 'parent_commit': '826e633b938f38c23ffc9392dfeb8372f035f70e',
            'scope': 'whole-record provided-same-ID engineering source and capacity preflight, not tracking or accuracy',
            'cases': recovery['cases'], 'target_modalities': ['rgb', 'thermal'],
            'source_rule': 'every common original stem across four camera TAR lists, joined to original ordinal timestamps; fail on mismatch',
            'settings': p['settings'], 'runtime_image': p['runtime_image'],
            'geometry': {'rgb': [248, 784], 'thermal': [256, 640]}, 'probe_buffers': [64, 128],
            'allocation': 'max(64,next_power_of_two(source_frames+16+1))',
            'capacity_scope': 'persistent state lower bound only; model/backend/correlation/workspace excluded',
            'reference_scoring': False, 'physical_clock_verified': False,
            'native_tracking': False, 'benchmark_execution_ready': False,
            'source_sha256': {str(f.relative_to(ROOT)): sha(f) for f in files}}
    OUT.mkdir(parents=True, exist_ok=False); write(PLAN, plan); write(OUT/'plan.json', plan)
    print('frozen', len(plan['source_sha256']), 'pins before full inventory and GPU probes')


def verify():
    p = read(PLAN)
    if sha(PLAN) != sha(OUT/'plan.json'): raise ValueError('plan changed')
    for path, h in p['source_sha256'].items():
        if sha(ROOT/path) != h: raise ValueError('changed source '+path)
    return p


def inventory():
    p = verify(); parent = read(ROOT/'experiments/droid-longer-prefix-2026-11-04.json')
    historical = read(SOURCE/'broader-recovery-manifest.json')['member_sha256']
    summaries = []; all_rows = []; archived = []
    for c in p['cases']:
        base = ROOT/c['source_base']; sequence = c['sequence']
        index = read(base/'sync_data-tar-index.json')
        cameras = {m+'_'+s: {} for m in ['rgb', 'thr'] for s in ['left', 'right']}
        for entry in index:
            for key in cameras:
                m, s = key.split('_'); prefix = sequence+'/'+m+'/img_'+s+'/'
                if entry['regular'] and entry['name'].startswith(prefix) and entry['name'].endswith('.png'):
                    stem = Path(entry['name']).stem
                    if stem in cameras[key]: raise ValueError('duplicate source stem')
                    cameras[key][stem] = entry
        stems = sorted(set.intersection(*(set(v) for v in cameras.values())))
        if not stems or any(set(v) != set(stems) for v in cameras.values()): raise ValueError('unmatched camera member lists')
        if stems != [f'{i:06}' for i in range(len(stems))]: raise ValueError('source IDs are not original complete ordinal axis')
        times = {}
        for key in cameras:
            m, s = key.split('_')
            values = (base/'pilot/sync_data'/sequence/m/('img_'+s+'_timestamp.txt')).read_text().split()
            if any(not v.isdecimal() for v in values): raise ValueError('noninteger original timestamp')
            times[key] = [int(v) for v in values]
            if len(values) != len(stems) or any(a >= b for a, b in zip(times[key], times[key][1:])):
                raise ValueError('timestamps do not match complete camera stem axis')
        prefix_case = next(r for r in parent['runs'] if r['condition'] == c['condition'])
        resident = {}
        for row in read(ROOT/prefix_case['manifest'])['records']:
            for key in cameras:
                m, s = key.split('_'); rec = row['cameras'][m][s]
                if sha(ROOT/rec['path']) != rec['sha256']: raise ValueError('resident source hash')
                if rec['recorded_time_ns'] != times[key][int(row['stem'])]: raise ValueError('prefix timestamp identity')
                resident[(key, row['stem'])] = rec
        recovered = {}
        for path, digest in historical.items():
            for key in cameras:
                m, s = key.split('_'); prefix = 'ore-ms/results/ms2-broader-range/run-001/source/'+sequence+'/'+m+'/img_'+s+'/'
                if path.startswith(prefix) and path.endswith('.png'):
                    stem = Path(path).stem
                    if stem not in cameras[key]: raise ValueError('archived stem outside full inventory')
                    recovered[(key, stem)] = digest
                    archived.append({'condition': c['condition'], 'camera': key, 'stem': stem,
                                     'archive_member': path, 'sha256': digest})
        resident_groups = 0; recoverable_groups = 0; missing_bytes = 0
        for ordinal, stem in enumerate(stems):
            existing = all((key, stem) in resident for key in cameras)
            recoverable = all((key, stem) in resident or (key, stem) in recovered for key in cameras)
            resident_groups += existing; recoverable_groups += recoverable
            row = {'condition': c['condition'], 'stem': stem, 'resident_group': existing,
                   'recoverable_prior_group': recoverable}
            for key in cameras:
                entry = cameras[key][stem]
                row[key+'_time_ns'] = times[key][ordinal]; row[key+'_png_bytes'] = entry['size']
                if (key, stem) in resident and (key, stem) in recovered:
                    if resident[(key, stem)]['sha256'] != recovered[(key, stem)]: raise ValueError('archived/resident overlap differs')
                if (key, stem) not in resident and (key, stem) not in recovered: missing_bytes += entry['size']
            all_rows.append(row)
        archive = next(a for a in read(base/'archive-manifest.json') if a['kind'] == 'sync_data')
        summaries.append({'condition': c['condition'], 'sequence': sequence, 'source_groups': len(stems),
                          'camera_png_members': 4*len(stems), 'resident_complete_groups': resident_groups,
                          'resident_or_prior_recoverable_groups': recoverable_groups,
                          'still_missing_groups': len(stems)-recoverable_groups,
                          'indexed_camera_png_bytes': sum(e['size'] for v in cameras.values() for e in v.values()),
                          'missing_camera_png_bytes_after_prior_recovery': missing_bytes,
                          'full_sync_archive_bytes': archive['bytes'],
                          'historical_archive_sha256': archive['sha256'],
                          'current_remote_archive_hash_verified': False})
    with (OUT/'source-axis.csv').open('x', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(all_rows[0])); w.writeheader(); w.writerows(all_rows)
    write(OUT/'prior-recoverable-camera-members.json', archived)
    write(OUT/'inventory.json', {'cases': summaries, 'source_groups': len(all_rows),
          'physical_clock_verified': False, 'reference_scoring': False, 'benchmark_execution_ready': False})
    print(json.dumps(summaries, indent=2))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('command', choices=['restore', 'freeze', 'inventory', 'verify'])
    command = ap.parse_args().command
    if command == 'verify': print('verified', len(verify()['source_sha256']), 'pins')
    else: globals()[command]()

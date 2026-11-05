"""Independent EXP140 source inventory and capacity arithmetic; no producer imports."""
import csv
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/whole-record-preflight/run-001'
CHECKS = 0


def read(p): return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()
def check(x, label):
    global CHECKS
    CHECKS += 1
    if not x: raise ValueError(label)


def run():
    p = read(ROOT/'experiments/whole-record-preflight-2026-11-04.json')
    check(sha(ROOT/'experiments/whole-record-preflight-2026-11-04.json') == sha(OUT/'plan.json'), 'plan')
    for path, digest in p['source_sha256'].items(): check(sha(ROOT/path) == digest, 'pin '+path)
    inventory = read(OUT/'inventory.json'); probe = read(OUT/'capacity-probe.json')
    with (OUT/'source-axis.csv').open() as f: rows = list(csv.DictReader(f))
    parent = read(ROOT/'experiments/droid-longer-prefix-2026-11-04.json')
    historical = read(ROOT/'results/whole-record-preflight/source-001/broader-recovery-manifest.json')['member_sha256']
    archived = read(OUT/'prior-recoverable-camera-members.json')
    check(len(rows) == inventory['source_groups'], 'whole population')
    for flag in ['physical_clock_verified', 'reference_scoring', 'benchmark_execution_ready']:
        check(inventory[flag] is False, 'no readiness '+flag)
    capacities = []; expected_archived = []
    for case, saved in zip(p['cases'], inventory['cases']):
        condition, seq = case['condition'], case['sequence']
        check(saved['condition'] == condition and saved['sequence'] == seq, 'recording identity')
        base = ROOT/case['source_base']
        members = {e['name']: e for e in read(base/'sync_data-tar-index.json') if e['regular']}
        streams = {}; times = {}
        for camera in ['rgb', 'thr']:
            for side in ['left', 'right']:
                key = camera+'_'+side; prefix = seq+'/'+camera+'/img_'+side+'/'
                streams[key] = {Path(n).stem: e for n, e in members.items() if n.startswith(prefix) and n.endswith('.png')}
                times[key] = [int(x) for x in (base/'pilot/sync_data'/seq/camera/('img_'+side+'_timestamp.txt')).read_text().split()]
        n = len(times['rgb_left']); stems = [f'{i:06d}' for i in range(n)]
        for key in streams:
            check(set(streams[key]) == set(stems) and len(times[key]) == n, 'full source axis')
            check(all(a < b for a, b in zip(times[key], times[key][1:])), 'chronology')
        chosen = [r for r in rows if r['condition'] == condition]
        check([r['stem'] for r in chosen] == stems, 'no windowing/reordering')
        check(saved['source_groups'] == n and saved['camera_png_members'] == 4*n, 'complete cardinality')
        pc = next(r for r in parent['runs'] if r['condition'] == condition)
        resident = read(ROOT/pc['manifest'])['records']
        have = {(m+'_'+s, r['stem']): r['cameras'][m][s]['sha256'] for r in resident for m in ['rgb', 'thr'] for s in ['left', 'right']}
        recover = {}
        for path, digest in historical.items():
            for key in streams:
                m, s = key.split('_'); prefix = 'ore-ms/results/ms2-broader-range/run-001/source/'+seq+'/'+m+'/img_'+s+'/'
                if path.startswith(prefix) and path.endswith('.png'):
                    stem = Path(path).stem; recover[(key, stem)] = digest
                    expected_archived.append({'condition': condition, 'camera': key, 'stem': stem, 'archive_member': path, 'sha256': digest})
        totals = [0, 0, 0, 0]
        for i, row in enumerate(chosen):
            stem = stems[i]
            present = all((k, stem) in have for k in streams)
            available = all((k, stem) in have or (k, stem) in recover for k in streams)
            check(row['resident_group'] == str(present) and row['recoverable_prior_group'] == str(available), 'availability population')
            totals[0] += present; totals[1] += available
            for key in streams:
                check(row[key+'_time_ns'] == str(times[key][i]), 'exact original integer time')
                size = streams[key][stem]['size']; totals[2] += size
                check(row[key+'_png_bytes'] == str(size), 'original TAR size')
                if (key, stem) not in have and (key, stem) not in recover: totals[3] += size
                if (key, stem) in have and (key, stem) in recover:
                    check(have[(key, stem)] == recover[(key, stem)], 'prefix/archive identity')
        check(saved['resident_complete_groups'] == totals[0] and saved['resident_or_prior_recoverable_groups'] == totals[1], 'coverage counts')
        check(saved['still_missing_groups'] == n-totals[1], 'missing groups')
        check(saved['indexed_camera_png_bytes'] == totals[2] and saved['missing_camera_png_bytes_after_prior_recovery'] == totals[3], 'whole byte totals')
        archive = next(a for a in read(base/'archive-manifest.json') if a['kind'] == 'sync_data')
        check(saved['full_sync_archive_bytes'] == archive['bytes'] and saved['historical_archive_sha256'] == archive['sha256'], 'historical archive identity')
        check(saved['current_remote_archive_hash_verified'] is False, 'not current download claim')
        for camera, (h, w) in p['geometry'].items():
            buffer = 64
            while buffer < n+17: buffer *= 2
            state_bytes = (7*h*w + 1032*(h//8)*(w//8) + 50)*buffer
            capacities.append({'condition': condition, 'camera': camera, 'source_frames': n,
                               'unchanged_wrapper_buffer': buffer, 'persistent_state_lower_bound_bytes': state_bytes,
                               'gpu_total_bytes': probe['gpu_total_bytes'],
                               'exceeds_total_gpu_memory': state_bytes > probe['gpu_total_bytes'],
                               'status': 'blocked_by_state_lower_bound' if state_bytes > probe['gpu_total_bytes'] else 'state_only_fits_full_run_unproven'})
    check(sorted(archived, key=lambda r: r['archive_member']) == sorted(expected_archived, key=lambda r: r['archive_member']), 'all prior recoverable members')
    check(len(probe['probes']) == 4, 'full probe population')
    for row in probe['probes']:
        h, w = p['geometry'][row['camera']]; b = row['buffer']; cells = (h//8)*(w//8)
        check(row['shape'] == [h, w] and b in p['probe_buffers'], 'probe shape/capacity')
        expected = {'tstamp': (b, 4), 'images': (b*3*h*w, 1), 'dirty': (b, 1), 'red': (b, 1),
                    'poses': (b*7, 4), 'disps': (b*cells, 4), 'disps_sens': (b*cells, 4),
                    'disps_up': (b*h*w, 4), 'intrinsics': (b*4, 4),
                    'fmaps': (b*2*128*cells, 2), 'nets': (b*128*cells, 2), 'inps': (b*128*cells, 2)}
        check(set(row['tensors']) == set(expected), 'all persistent tensors')
        for key, (elements, size) in expected.items():
            tensor = row['tensors'][key]
            check(tensor['elements'] == elements and tensor['element_bytes'] == size and tensor['bytes'] == elements*size, 'tensor bytes '+key)
        exact = sum(a*s for a, s in expected.values())
        check(row['tensor_bytes'] == exact == b*(7*h*w+1032*cells+50), 'independent state formula')
        check(exact <= row['cuda_allocated_delta_bytes'] <= exact+512*len(expected), 'measured allocation rounding bound')
    for camera in p['geometry']:
        a, b = [r for r in probe['probes'] if r['camera'] == camera]
        check(b['tensor_bytes']*a['buffer'] == a['tensor_bytes']*b['buffer'], 'measured per-slot slope')
    for flag in ['weights_loaded', 'images_read', 'tracking_executed', 'backend_executed', 'physical_sync_verified']:
        check(probe[flag] is False, 'probe-only '+flag)
    check(probe['depth_video_sha256'] == sha(ROOT/'results/whole-record-preflight/source-001/depth_video.py'), 'actual native source')
    prior = read(ROOT/'experiments/ms2-clock-evidence-2026-11-04.json')
    for path, digest in prior['source_sha256'].items(): check(sha(ROOT/path) == digest, 'parent unchanged '+path)
    result = {'passed': True, 'independent_checks': CHECKS, 'source_groups': len(rows),
              'all_target_recordings_retained': len(capacities) == 6,
              'capacity_decisions': capacities, 'parent_pins_unchanged': len(prior['source_sha256']),
              'full_record_tracking_executed': False, 'whole_record_inputs_complete': False,
              'physical_sync_verified': False, 'benchmark_execution_ready': False,
              'scope': 'verified source/capacity preflight; not complete-run peak-memory/performance/accuracy evidence'}
    with (OUT/'audit.json').open('x') as f: json.dump(result, f, indent=2); f.write('\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__': run()

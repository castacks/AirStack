"""EXP139 fixed original-prefix recorded-time diagnostics; never physical timing proof."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT/'experiments/ms2-clock-evidence-2026-11-04.json'
OUT = ROOT/'results/clock-evidence/run-001'
FIELDS = ['rgb_left', 'rgb_right', 'thr_left', 'thr_right']


def read(p): return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()
def write(p, value):
    with Path(p).open('x') as f: json.dump(value, f, indent=2, allow_nan=False); f.write('\n')


def freeze():
    parent = read(ROOT/'experiments/common-time-model-2026-11-04.json')
    registry = ROOT/'results/clock-evidence/source-001/registry.json'
    files = [Path(__file__), ROOT/'tools/ms2_clock_sources.py', ROOT/'tools/ms2_clock_evidence_audit.py',
             ROOT/'experiments/common-time-model-2026-11-04.json', registry,
             ROOT/'results/clock-evidence/source-001/evidence-ledger.json']
    cases = [{k: c[k] for k in ['condition', 'recording', 'manifest']} for c in parent['cases']]
    files += [ROOT/c['manifest'] for c in cases]
    sources = read(registry)
    files += [ROOT/s['path'] for s in sources['sources']]
    plan = {'date': '2026-11-04 America/New_York', 'parent_commit': '91ec26b49118beb80246226db1620ec376a78424',
            'cases': cases, 'original_stems': [f'{i:06}' for i in range(101)],
            'timestamp_fields': FIELDS, 'legacy_stereo_limit_ns': 2_000_000,
            'legacy_cross_left_limit_ns': 30_000_000,
            'threshold_scope': 'inherited recorded-time diagnostics only; not physical eligibility',
            'diagnostics': 'all same-ID stereo/cross-left signed offsets; cadence; exact endpoint equality',
            'clock_fit': False, 'timestamp_search': False, 'new_images': False,
            'native_inference': False, 'reference_scoring': False,
            'benchmark_execution_ready': False,
            'source_sha256': {str(p.relative_to(ROOT)): sha(p) for p in files}}
    OUT.mkdir(parents=True, exist_ok=False); write(PLAN, plan); write(OUT/'plan.json', plan)
    print('frozen', len(plan['source_sha256']), 'pins')


def verify():
    p = read(PLAN)
    if sha(PLAN) != sha(OUT/'plan.json'): raise ValueError('plan changed')
    for path, expected in p['source_sha256'].items():
        if sha(ROOT/path) != expected: raise ValueError('changed pin '+path)
    return p


def summary(values, limit=None):
    ordered = sorted(values); n = len(values)
    result = {'count': n, 'minimum_ns': min(values), 'maximum_ns': max(values),
              'median_twice_ns': ordered[(n-1)//2]+ordered[n//2],
              'negative_count': sum(x < 0 for x in values),
              'zero_count': sum(x == 0 for x in values), 'positive_count': sum(x > 0 for x in values),
              'sign_changes_adjacent': sum(a*b < 0 for a, b in zip(values, values[1:]))}
    if limit is not None:
        result.update(legacy_limit_ns=limit, within_legacy_limit=sum(abs(x) <= limit for x in values),
                      constant_shift_worst_residual_lower_bound_twice_ns=max(values)-min(values))
    return result


def run():
    p = verify(); rows = []; summaries = []
    for case in p['cases']:
        source = read(ROOT/case['manifest']); records = source['records']
        if [r['stem'] for r in records] != p['original_stems']: raise ValueError('changed original IDs')
        times = {k: [] for k in FIELDS}
        current = []
        for record in records:
            row = {'condition': case['condition'], 'recording': case['recording'], 'stem': record['stem']}
            for camera in ['rgb', 'thr']:
                for side in ['left', 'right']:
                    k = camera+'_'+side; t = record['cameras'][camera][side]['recorded_time_ns']
                    if type(t) is not int or t < 0: raise ValueError('exact nonnegative timestamp required')
                    row[k+'_ns'] = t; times[k].append(t)
            for label, a, b in [('rgb_stereo', 'rgb_right', 'rgb_left'),
                                ('thr_stereo', 'thr_right', 'thr_left'),
                                ('cross_left', 'thr_left', 'rgb_left')]:
                row[label+'_offset_ns'] = row[a+'_ns']-row[b+'_ns']
            rows.append(row); current.append(row)
        cadence = {}
        for k, ts in times.items():
            differences = [b-a for a, b in zip(ts, ts[1:])]
            if any(d <= 0 for d in differences): raise ValueError('nonchronological source axis')
            cadence[k] = summary(differences)
        offsets = {label: summary([r[label+'_offset_ns'] for r in current],
                                  p['legacy_cross_left_limit_ns'] if label == 'cross_left' else p['legacy_stereo_limit_ns'])
                   for label in ['rgb_stereo', 'thr_stereo', 'cross_left']}
        equal_points = sum(a == b for a, b in zip(times['rgb_left'], times['thr_left']))
        equal_edges = sum(times['rgb_left'][i:i+2] == times['thr_left'][i:i+2] for i in range(100))
        summaries.append({'condition': case['condition'], 'recording': case['recording'],
                          'source_rows': len(current), 'offsets': offsets, 'cadence': cadence,
                          'equal_rgb_thermal_left_points': equal_points,
                          'equal_rgb_thermal_left_intervals': equal_edges})
    with (OUT/'timestamp-rows.csv').open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    write(OUT/'metadata-summary.json', {'cases': summaries, 'total_rows': len(rows),
          'total_original_timestamps': 4*len(rows), 'sensor_clock_verified': False,
          'scope': 'provided recorded-time metadata only; neither exposure offsets nor a physical sync test',
          'reference_scoring': False, 'benchmark_execution_ready': False})
    print(json.dumps(summaries, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('command', choices=['freeze', 'run', 'verify'])
    command = parser.parse_args().command
    if command == 'verify': print('verified pins', len(verify()['source_sha256']))
    else: globals()[command]()

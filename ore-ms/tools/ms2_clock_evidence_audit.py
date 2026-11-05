"""Independent scalar audit of EXP139 CSV/summary/source snapshots; no producer imports."""
import csv
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/clock-evidence/run-001'
COUNT = 0


def read(p): return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()
def check(x, label):
    global COUNT
    COUNT += 1
    if not x: raise ValueError(label)


def stats(values, output, limit=None):
    check(output['count'] == len(values), 'count')
    check(output['minimum_ns'] == min(values) and output['maximum_ns'] == max(values), 'extrema')
    exact = statistics.median_low(values)+statistics.median_high(values)
    check(output['median_twice_ns'] == exact, 'integer median')
    for key, predicate in [('negative_count', lambda x: x < 0), ('zero_count', lambda x: x == 0), ('positive_count', lambda x: x > 0)]:
        check(output[key] == len([x for x in values if predicate(x)]), key)
    switches = 0
    for i in range(1, len(values)):
        if (values[i-1] < 0 and values[i] > 0) or (values[i-1] > 0 and values[i] < 0): switches += 1
    check(output['sign_changes_adjacent'] == switches, 'sign changes')
    if limit is not None:
        check(output['legacy_limit_ns'] == limit, 'inherited limit')
        check(output['within_legacy_limit'] == len([v for v in values if -limit <= v <= limit]), 'within diagnostic bound')
        bound = Fraction(max(values), 2)-Fraction(min(values), 2)
        check(Fraction(output['constant_shift_worst_residual_lower_bound_twice_ns'], 2) == bound, 'analytical no-fit bound')


def audit():
    plan = read(ROOT/'experiments/ms2-clock-evidence-2026-11-04.json')
    check(sha(ROOT/'experiments/ms2-clock-evidence-2026-11-04.json') == sha(OUT/'plan.json'), 'plan')
    for path, expected in plan['source_sha256'].items(): check(sha(ROOT/path) == expected, 'pin '+path)
    reg = read(ROOT/'results/clock-evidence/source-001/registry.json')
    for s in reg['sources']:
        path = ROOT/s['path']; check(path.stat().st_size == s['bytes'] and sha(path) == s['sha256'], 'public snapshot')
    with (OUT/'timestamp-rows.csv').open() as f: rows = list(csv.DictReader(f))
    result = read(OUT/'metadata-summary.json')
    check(len(rows) == result['total_rows'] == 303 and result['total_original_timestamps'] == 1212, 'complete population')
    check(len(result['cases']) == len(plan['cases']) == 3, 'case population')
    for flag in ['sensor_clock_verified', 'reference_scoring', 'benchmark_execution_ready']:
        check(result[flag] is False, 'not physical readiness')
    for case, saved in zip(plan['cases'], result['cases']):
        records = read(ROOT/case['manifest'])['records']; subset = [r for r in rows if r['condition'] == case['condition']]
        check([r['stem'] for r in records] == [r['stem'] for r in subset] == plan['original_stems'], 'exact stem population')
        check(saved['condition'] == case['condition'] and saved['recording'] == case['recording'] and saved['source_rows'] == 101, 'case lineage')
        source_times = {k: [] for k in ['rgb_left', 'rgb_right', 'thr_left', 'thr_right']}
        offsets = {k: [] for k in ['rgb_stereo', 'thr_stereo', 'cross_left']}
        for original, row in zip(records, subset):
            check(row['recording'] == case['recording'], 'recording')
            for k in source_times:
                camera, side = k.split('_'); t = original['cameras'][camera][side]['recorded_time_ns']
                check(type(t) is int and t >= 0 and row[k+'_ns'] == str(t), 'exact original timestamp')
                source_times[k].append(t)
            for label, a, b in [('rgb_stereo', 'rgb_right', 'rgb_left'), ('thr_stereo', 'thr_right', 'thr_left'), ('cross_left', 'thr_left', 'rgb_left')]:
                value = original['cameras'][a.split('_')[0]][a.split('_')[1]]['recorded_time_ns']-original['cameras'][b.split('_')[0]][b.split('_')[1]]['recorded_time_ns']
                check(row[label+'_offset_ns'] == str(value), 'scalar offset')
                offsets[label].append(value)
        for k, values in offsets.items(): stats(values, saved['offsets'][k], 30_000_000 if k == 'cross_left' else 2_000_000)
        for k, times in source_times.items():
            diffs = [times[i+1]-times[i] for i in range(100)]
            check(all(d > 0 for d in diffs), 'chronology'); stats(diffs, saved['cadence'][k])
        eq = [source_times['rgb_left'][i] == source_times['thr_left'][i] for i in range(101)]
        check(saved['equal_rgb_thermal_left_points'] == sum(eq), 'equal points')
        check(saved['equal_rgb_thermal_left_intervals'] == sum(eq[i] and eq[i+1] for i in range(100)), 'equal intervals')
    parent = read(ROOT/'experiments/common-time-model-2026-11-04.json')
    for path, digest in parent['source_sha256'].items(): check(sha(ROOT/path) == digest, 'unchanged parent '+path)
    report = {'passed': True, 'independent_scalar_checks': COUNT, 'original_rows': 303,
              'original_timestamps': 1212, 'parent_pins_unchanged': len(parent['source_sha256']),
              'physical_sync_verified': False, 'reference_scoring': False, 'benchmark_execution_ready': False}
    with (OUT/'audit.json').open('x') as f: json.dump(report, f, indent=2); f.write('\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__': audit()

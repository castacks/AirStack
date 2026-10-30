"""Inventory existing qualified labels; never fit a selector or open new captures."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import statistics


CASES = [('early', 'end-to-end-solver-control', 19),
         ('middle', 'solver-collapse-replication', 99),
         ('day', 'solver-day-replication', 99)]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def describe(values):
    return dict(count=len(values), minimum=min(values), median=statistics.median(values), maximum=max(values))


def audit(root):
    frozen = {}

    def read(path):
        frozen[str(path.relative_to(root))] = sha(path)
        return json.loads(path.read_text())

    windows = {}
    for window, name, count in CASES:
        base = root/'results'/name/'run-001'
        report = read(base/'evaluation.json')
        proof = read(base/'validation.json')
        declaration = read(base/'evaluation-protocol.json')
        protocol = read(base/'protocol.json')
        assert proof['status'] == 'PASS'
        assert proof['evaluation_sha256'] == sha(base/'evaluation.json')
        assert proof['evaluation_protocol_sha256'] == report['evaluation_protocol_sha256'] == sha(base/'evaluation-protocol.json')
        assert proof['execution_protocol_sha256'] == sha(base/'protocol.json')
        assert report['independent_ground_truth'] is False and report['fit_performed'] is False
        assert declaration['physical_body_transform_verified'] is False
        assert declaration['independent_ground_truth'] is False
        # Verify existing reference dependency hashes, not a new provenance claim.
        refs = {p: h for p, h in declaration['prior_evaluation_sha256'].items()
                if p.startswith('data/tartandrive3/') and '/super_odometry/odometry/' in p}
        assert len(refs) == 5
        recordings = {p.split('/super_odometry/')[0] for p in refs}
        assert len(recordings) == 1
        for p, h in refs.items():
            assert sha(root/p) == h
            frozen[p] = h
        rows = {(r['camera'], r['pair_ordinal']): r for r in report['rows'] if r['arm'] == 'original'}
        assert len(rows) == count*2
        assert len(protocol['input_rows']) == count+1
        endpoints = []; durations = {c: [] for c in ('rgb', 'thermal')}
        available = Counter(); stages = {c: Counter() for c in durations}
        transforms = {c: set() for c in durations}; both = processing = equal = 0
        for i in range(count):
            pair = {c: rows[c, i] for c in durations}
            for c, row in pair.items():
                own = row['sensor_interval_ns']
                assert all(type(t) is int for t in own) and own[1] > own[0]
                assert own == [protocol['input_rows'][j][c+'_left']['time_ns'] for j in (i, i+1)]
                assert row['common_interval_ns'] == pair['rgb']['sensor_interval_ns']
                durations[c].append(own[1]-own[0])
                transforms[c].add(json.dumps(row['nominal_body_from_camera'], sort_keys=True))
                valid = row['common_loss'] is not None
                assert valid == (row['own_body_delta'] is not None)
                assert valid == (row['motion_stage'] is not None)
                if valid:
                    assert row['original_need_interp'] is False
                    available[c] += 1
                    stages[c][row['motion_stage']] += 1
            equal += pair['rgb']['sensor_interval_ns'] == pair['thermal']['sensor_interval_ns']
            both += all(r['common_loss'] is not None for r in pair.values())
            processing += all(r['motion_stage'] == 'pair_processing' for r in pair.values())
        for i in range(count+1):
            frame = protocol['input_rows'][i]
            endpoints.append(frame['rgb_left']['time_ns']-frame['thermal_left']['time_ns'])
        assert all(len(t) == 1 for t in transforms.values())
        windows[window] = dict(recording=next(iter(recordings)), slots=count,
            common_span_ns=[rows['rgb', 0]['sensor_interval_ns'][0], rows['rgb', count-1]['sensor_interval_ns'][1]],
            exact_cross_camera_intervals=equal, rgb_minus_thermal_endpoint_ns=describe(endpoints),
            endpoint_offset_sign_counts=dict(negative=sum(x < 0 for x in endpoints), zero=endpoints.count(0), positive=sum(x > 0 for x in endpoints)),
            sensor_duration_ns={c: describe(v) for c, v in durations.items()},
            sensor_intervals_over_125ms={c: sum(x > 125_000_000 for x in v) for c, v in durations.items()},
            available={c: available[c] for c in durations}, both_supported=both,
            both_processing_stage=processing, stages={c: dict(v) for c, v in stages.items()},
            nominal_transform_sha256={c: hashlib.sha256(next(iter(v)).encode()).hexdigest() for c, v in transforms.items()},
            reference_metadata=declaration['reference_metadata'],
            gradient_audit_exceptions=proof.get('gradient_component_exceptions', []))
    records = sorted({v['recording'] for v in windows.values()})
    assert windows['early']['recording'] == windows['middle']['recording']
    assert windows['early']['common_span_ns'][1] < windows['middle']['common_span_ns'][0]
    assert len(records) == 2
    assert all(sha(root/p) == h for p, h in frozen.items())
    return dict(status='PASS_INVENTORY_ONLY', source_sha256=sha(Path(__file__)), input_sha256=frozen,
        windows=windows, distinct_recordings=records, total_paired_slots=sum(v['slots'] for v in windows.values()),
        physical_reference_ready=False, prospective_recording_holdout_ready=False,
        reason='Both recordings repeatedly inspected; checkpoint membership unknown; nominal rig and reference-assisted interval transport only.',
        no_new_inference=True, no_gate_fit=True, no_reserved_access=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    result = audit(a.root)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False); f.write('\n')
    print(json.dumps({k: result[k] for k in ('status', 'total_paired_slots', 'distinct_recordings')}))

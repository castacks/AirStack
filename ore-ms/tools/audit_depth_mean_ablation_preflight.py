"""Independent saved-event identity, native replay and depth inventory checks."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/depth-mean-ablation/preflight-001'


def read(p):
    return json.loads(p.read_text())


def audit():
    plan = read(ROOT/'experiments/depth-mean-ablation-preflight-2026-11-09.json')
    for rel, digest in plan['pins'].items():
        assert hashlib.sha256((ROOT/rel).read_bytes()).hexdigest() == digest, rel
    expected = {(c['condition'], c['camera'], c['pair_ordinal']): c for c in plan['cases']}
    seen, streams, exact = set(), {}, 0
    for condition in ['day', 'night', 'rain']:
        report = read(OUT/f'parity-{condition}.json')
        assert report['depth_changing_arms_executed'] == report['new_frontend_calls'] == 0
        for row in report['rows']:
            key = (condition, row['camera'], row['pair'])
            assert key not in seen
            seen.add(key); case = expected[key]
            relative = case['expert_relative']
            if relative not in streams:
                events = [json.loads(s) for s in (ROOT/relative/'arbiter-events/events.jsonl').read_text().splitlines()]
                streams[relative] = {e['sequence']: e for e in events}
            event = streams[relative][case['motion_event_sequence']]
            assert event['kind'] == 'motion_result_written' and event['first_write_for_submission']
            assert [event['from_idx'], event['frame_idx']] == [row['pair'], row['pair']+1]
            assert case['initial_pose_xyzw'] == event['parent_pose_camera_map_xyzw']
            native = event['optimizer_pose_before_write_native_parameterization'][0]
            assert case['recorded_native_pose_xyzw'] == native
            # Exact parameter-array equality is stronger than the declared pose tolerance.
            assert row['native_solver_result']['pose_xyzw'] == native
            assert row['parity_error'] == {'translation_m': 0., 'rotation_deg': 0.}
            assert row['parity_pass'] is True
            exact += 1
    assert seen == set(expected)
    names = set()
    for number in [147, 151]:
        for name in read(OUT/f'exp{number}-manifest.json')['member_sha256']:
            if '/labels/' in name:
                names.add(name.split('/labels/', 1)[1])
    required = set()
    for condition, coverage in plan['label_coverage'].items():
        expected_names = {f"{coverage['sequence']}/{cam}/depth/{stem}.png" for cam in ['rgb', 'thr'] for stem in coverage['source_stems']}
        assert len(expected_names) == coverage['expected_own_camera_maps'] == 42
        assert not expected_names.intersection(names)
        assert set(coverage['missing_exact_maps']) == expected_names
        assert not coverage['available_exact_maps']
        required.update(expected_names)
    assert plan['label_gate_pass'] is False
    return {'status': 'PASS', 'producer_imported': False, 'source_hashes_verified': len(plan['pins']),
            'native_event_and_parameter_array_checks': exact, 'native_case_coverage': len(seen),
            'required_exact_depth_maps': len(required), 'inventoried_exact_depth_maps': 0,
            'interpretation': 'parity passes; label prerequisite fails; four-arm experiment not executed'}


if __name__ == '__main__':
    result = audit()
    with (OUT/'audit.json').open('x') as f:
        json.dump(result, f, indent=2)
    print(result)

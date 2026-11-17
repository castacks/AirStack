"""Offline integrity and episode-history check; no ROS or motion endpoints."""
import hashlib
import json
from pathlib import Path


def main():
    root = Path(__file__).parent
    report = json.loads(root.with_suffix('.json').read_text())
    for name, expected in report['input_sha256'].items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == expected, name
    events = json.loads((root/'events.json').read_text())
    summary = json.loads((root/'summary.json').read_text())
    baseline = json.loads((root/'baseline-summary.json').read_text())
    assert summary['verdict'] == 'PASS' and len(summary['checks']) == 28
    assert summary['checks'] == report['checks'] and all(summary['checks'].values())
    assert baseline['baseline_unsupported'] is True and all(baseline['checks'].values())
    assert hashlib.sha256((root/'harness.py').read_bytes()).hexdigest() == summary['harness_sha256']
    assert hashlib.sha256((root/'baseline-harness.py').read_bytes()).hexdigest() == baseline['harness_sha256']
    snapshots = [event['value'] for event in events if event['kind'] == 'status']
    assert snapshots
    assert len({value['monitor_instance_id'] for value in snapshots}) == 1
    assert all(b['sequence'] > a['sequence'] for a, b in zip(snapshots, snapshots[1:]))
    assert all(value['physical_stop'] == 'UNVERIFIED' and
               value['authority_scope'] == 'monitor_local_requests_only' for value in snapshots)
    assert all(value['escalation_required'] == (value['active'] and value['deadline_exceeded'])
               for value in snapshots)
    for generation, expected in report['episode_metrics'].items():
        values = [value for value in snapshots if value['generation'] == int(generation)]
        assert values and not values[-1]['active'] and not values[-1]['escalation_required']
        assert values[-1]['deadline_exceeded'] is expected['deadline_exceeded']
        assert values[-1]['elapsed_wall_s'] == expected['final_elapsed_s']
        assert all(b['elapsed_wall_s'] >= a['elapsed_wall_s'] for a, b in zip(values, values[1:]))
        required = [value for value in values if value['escalation_required']]
        assert (required[0]['elapsed_wall_s'] if required else None) == expected['first_observed_escalation_elapsed_s']
        assert all(value['elapsed_wall_s'] >= value['acceptance_budget_wall_s'] for value in required)
    assert report['live_executable_sha256_before'] == report['live_executable_sha256_after']
    print(json.dumps(dict(verdict='PASS', snapshots=len(snapshots),
                          checked_input_hashes=len(report['input_sha256']), checks=28,
                          scope='saved trace integrity and local episode history only')))


if __name__ == '__main__':
    main()

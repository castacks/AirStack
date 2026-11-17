"""Offline input-integrity and controller response/state checks; no ROS startup."""
import hashlib
import json
from pathlib import Path


def main():
    root = Path(__file__).parent
    report = json.loads(root.with_suffix('.json').read_text())
    for name, digest in report['input_sha256'].items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == digest, name
    summary = json.loads((root/'summary.json').read_text())
    assert summary['verdict'] == 'PASS' and all(summary['checks'].values())
    assert summary['checks'] == report['checks'] and len(summary['checks']) == 60
    events = json.loads((root/'events.json').read_text())
    responses = [event for event in events if event['kind'] == 'response']
    assert len({value['instance'] for value in responses}) == 2
    delayed = [value for value in responses
               if value['phase'] == 'old_commands_after_stop' and
               value['request_generation'] != value['generation']]
    assert len(delayed) == 4 and all(not value['success'] for value in delayed)
    assert any(value['reason'] == 'controller_instance_mismatch' for value in responses
               if value['phase'] == 'restart')
    assert any(value['success'] and value['duplicate'] for value in responses)
    assert any(value['reason'] == 'sequence_conflict' for value in responses)
    old_sequence = None
    old_instance = None
    for value in responses:
        if value['instance'] == old_instance:
            assert value['last_sequence'] >= old_sequence
        old_instance, old_sequence = value['instance'], value['last_sequence']
    failed = json.loads((root/'failed-summary.json').read_text())
    assert failed['verdict'] == 'FAIL'
    assert [key for key, value in failed['checks'].items() if not value] == report['retained_failed_checks']
    for prefix, value in (('', summary), ('failed-', failed)):
        assert hashlib.sha256((root/(prefix+'harness.py')).read_bytes()).hexdigest() == value['harness_sha256']
    print(json.dumps(dict(verdict='PASS', checked_hashes=len(report['input_sha256']),
                          responses=len(responses), contract_checks=60,
                          scope='saved protocol/reference trace integrity; no physical claim')))


if __name__ == '__main__':
    main()

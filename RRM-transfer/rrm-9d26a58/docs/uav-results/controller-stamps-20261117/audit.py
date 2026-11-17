"""Offline malformed-stamp evidence check; no ROS or process execution."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).parent
report = json.loads(root.with_suffix('.json').read_text())
for name, digest in report['input_sha256'].items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest() == digest
for label in ('baseline', 'candidate'):
    summary = json.loads((root/label/'summary.json').read_text())
    assert hashlib.sha256((root/'harness.py').read_bytes()).hexdigest() == summary['harness_sha256']
    for case in summary['cases']:
        if 'wire_file' in case:
            assert hashlib.sha256((root/label/case['wire_file']).read_bytes()).hexdigest() == case['wire_sha256']
        if label == 'candidate':
            assert case['rejected_without_exit'] and case['valid_zero_stamp_accepted']
            assert case['process_exit_before_cleanup'] is None
assert len(report['baseline_process_aborts']) == 4
assert report['candidate_summary']['passed'] == 8
print(json.dumps(dict(verdict='PASS', candidate_cases=8, baseline_aborts=4,
                      baseline_incomplete=len(report['baseline_discovery_incomplete']))))

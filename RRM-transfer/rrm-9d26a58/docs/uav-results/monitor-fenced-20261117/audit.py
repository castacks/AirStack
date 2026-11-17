"""Verify saved trace and raw held-RPC ordering without starting ROS."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).parent
report = json.loads(root.with_suffix('.json').read_text())
for name, digest in report['input_sha256'].items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest() == digest
summary = json.loads((root/'summary.json').read_text())
assert summary['verdict'] == 'PASS' and all(summary['checks'].values())
assert summary['checks'] == report['checks'] and len(summary['checks']) == 25
assert summary['nonfinite_completion_samples'] == 0
failed = json.loads((root/'failed-summary.json').read_text())
assert failed['nonfinite_completion_samples'] == 34
for prefix, value in (('', summary), ('failed-', failed)):
    assert hashlib.sha256((root/(prefix+'harness.py')).read_bytes()).hexdigest() == value['harness_sha256']
events = json.loads((root/'events.json').read_text())
a, b, c, d = [next(value for value in events if value['kind'] == kind)
              for kind in ('proxy_received', 'release_proxy', 'proxy_forwarding', 'proxy_reply')]
assert a['monotonic'] < b['monotonic'] < c['monotonic'] < d['monotonic']
assert a['generation'] < b['generation'] and not d['success'] and d['latched']
assert d['reason'] == 'stop_generation_mismatch'
assert hashlib.sha256((root/a['wire_file']).read_bytes()).hexdigest() == a['wire_sha256']
print(json.dumps(dict(verdict='PASS', checks=25, input_hashes=len(report['input_sha256']),
                      held_rpc_ordering=True, scope='saved integration evidence only')))

"""Offline lease evidence integrity; no ROS node or clock assumptions on replay."""
import hashlib
import json
from pathlib import Path

root = Path(__file__).parent
report = json.loads(root.with_suffix('.json').read_text())
for name, digest in report['input_sha256'].items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest() == digest
for case in report['cases']:
    directory = root/case['case']
    summary = json.loads((directory/'summary.json').read_text())
    assert summary == case['summary'] and summary['verdict'] == 'PASS'
    assert len(summary['checks']) == 26 and all(summary['checks'].values())
    assert hashlib.sha256((root/'harness.py').read_bytes()).hexdigest() == summary['harness_sha256']
    events = json.loads((directory/'events.json').read_text())
    received, release, forward, reply = [next(e for e in events if e['kind'] == kind)
                                        for kind in ('proxy_received', 'release_proxy',
                                                     'proxy_forwarding', 'proxy_reply')]
    assert received['monotonic'] < release['monotonic'] < forward['monotonic'] < reply['monotonic']
    assert hashlib.sha256((directory/'delayed-command.cdr').read_bytes()).hexdigest() == received['wire_sha256']
    assert not reply['success'] and reply['reason'] == 'lease_expired'
print(json.dumps(dict(verdict='PASS', cases=3, expectations=78,
                      input_hashes=len(report['input_sha256']), scope='saved lease evidence')))

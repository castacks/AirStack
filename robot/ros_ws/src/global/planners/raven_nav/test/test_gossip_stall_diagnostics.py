"""A blocked ROS callback must still produce a saved diagnostic stack."""
from pathlib import Path
import subprocess
import sys


def test_watchdog_rearms_then_captures_blocked_callback(tmp_path):
    trace = tmp_path / 'gossip.log'
    child = r'''
import faulthandler, sys, time
from types import SimpleNamespace
from coordination_bringup.gossip_node import GossipNode
with open(sys.argv[1], 'w') as stream:
    node = SimpleNamespace(_stall_dump_timeout=.4, _diag_file=stream)
    for _ in range(8):
        GossipNode._arm_stall_dump(node)
        time.sleep(.075)
    stream.flush()
    assert not open(sys.argv[1]).read(), 'healthy progress produced a stall dump'
    def blocked_callback():
        time.sleep(.7)
    blocked_callback()
    faulthandler.cancel_dump_traceback_later()
'''
    result = subprocess.run([sys.executable, '-c', child, str(trace)],
                            capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'blocked_callback' in trace.read_text()

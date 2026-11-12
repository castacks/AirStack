import importlib.util
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location('byte_summary', Path(__file__).parents[1] / 'summarize_gossip_bytes.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_canonical_complete_ledger_accepts_only_exact_diagnostic_prefix(tmp_path):
    canonical = tmp_path / 'logs/airstack-robot-desktop-1/gossip_bandwidth_robot_1.jsonl'
    canonical.parent.mkdir(parents=True)
    canonical.write_bytes(b'first\nsecond\n')
    copied = tmp_path / 'logs/diagnostic/gossip_bandwidth_robot_1.jsonl'
    copied.parent.mkdir()
    copied.write_bytes(b'first\n')
    assert module.select_ledger(tmp_path, 'robot_1') == canonical
    copied.write_bytes(b'first\ndifferent\n')
    with pytest.raises(ValueError, match='divergent duplicate'):
        module.select_ledger(tmp_path, 'robot_1')


def test_multiple_noncanonical_ledgers_remain_ambiguous(tmp_path):
    for name in ['a', 'b']:
        folder = tmp_path / name
        folder.mkdir()
        (folder / 'gossip_bandwidth_robot_1.jsonl').write_bytes(b'first\n')
    with pytest.raises(ValueError, match='expected one byte ledger'):
        module.select_ledger(tmp_path, 'robot_1')

#!/usr/bin/env python3
"""Post-search check, run in every robot container for the map/BB comparison."""
import json
import os
from pathlib import Path

root = Path('/root/.cache/raven_results')
robot = os.environ['ROBOT_NAME']
mode = os.getenv('RAVEN_SEMANTIC_MAP_SHARING', 'false').lower() == 'true'
expected = int(os.environ.get('NUM_ROBOTS', '3'))
rows = [json.loads(line) for line in (root / f'{robot}_auction_solve.jsonl').read_text().splitlines()]
assert rows, 'missing auction trace'
assert all(bool(r['params']['semantic_map_sharing']) == mode for r in rows), 'wrong sharing mode'
if mode:
    assert all(r['params']['semantic_map_rejections'] == 0 for r in rows), 'rejected semantic map'
    assert any(sum(n > 0 for n in r['params']['semantic_map_cells_by_source'].values()) >= expected
               for r in rows), 'did not fuse nonempty maps from the whole fleet'
ledger = Path(f'/tmp/gossip_bandwidth_{robot}.jsonl')
traffic = [json.loads(line) for line in ledger.read_text().splitlines()]
start = json.loads((root / f'{robot}.json').read_text())['mission_start_ts'] * 1e9
traffic = [r for r in traffic if r['sim_ns'] is not None and start <= r['sim_ns'] <= start + 300e9]
assert any(r['direction'] == 'tx' for r in traffic), 'no search-window byte measurements'
if mode:
    assert any(r['direction'] == 'tx' and r['payload_bytes'].get('semantic_voxel_map', 0) > 0
               for r in traffic), 'no transmitted map bytes'
    assert any(r['direction'] == 'rx' and r['payload_bytes'].get('semantic_voxel_map', 0) > 0
               for r in traffic), 'no received map bytes'
else:
    assert not any(r['payload_bytes'].get('semantic_voxel_map', 0) for r in traffic), 'map leaked into BB control'
print(f'{robot}: sharing mode and search-window bandwidth measurements verified')

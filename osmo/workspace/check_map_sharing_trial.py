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
    peers = [f'robot_{n}' for n in range(1, expected+1) if f'robot_{n}' != robot]
    assert all(max(r['params'].get('semantic_map_peer_updates', {}).get(peer, 0) for r in rows) >= 3
               for peer in peers), 'did not receive repeated updates from every peer'
    active = [r for r in rows if r['ts'] >= rows[0]['ts'] + 60]
    assert active, 'no post-warmup map freshness observations'
    fresh = lambda r: all(r['params'].get('semantic_map_peer_age_wall_s', {}).get(peer, 1e9) <= 15
                          for peer in peers)
    assert sum(fresh(r) for r in active) / len(active) >= .7, 'peer maps were stale during most of the search'
    assert any(fresh(r) for r in rows if r['ts'] >= rows[-1]['ts']-30), 'no fresh fleet maps near mission end'
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

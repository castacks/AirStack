#!/usr/bin/env python3
"""Verify the raw-ray ablation, fleet ingestion, freshness and byte evidence."""
import json
import os
from pathlib import Path
root = Path('/root/.cache/raven_results')
robot = os.environ['ROBOT_NAME']
expected = int(os.environ.get('NUM_ROBOTS', '3'))
peers = [f'robot_{i}' for i in range(1, expected+1) if f'robot_{i}' != robot]
result = json.loads((root / f'{robot}.json').read_text())
assert result.get('share_raw_rays') is True, 'raw-ray switch not active'
stats = result['ray_share_stats']
assert stats['source_rows'] == stats['wire_rows'] > 0, 'sender pruned raw rays'
assert stats['snapshots'] > 1, 'missing repeated own ray clouds'
rows = [json.loads(s) for s in (root / f'{robot}_auction_solve.jsonl').read_text().splitlines()]
assert rows and all(r['params'].get('share_raw_rays') for r in rows), 'wrong runtime mode'
assert all(r['params']['share_rays'] and r['params']['share_bbs'] and not r['params']['semantic_map_sharing']
           for r in rows), 'other sharing settings changed'
assert any(all(r['params'].get('peer_ray_rows', {}).get(p, 0) > 0 for p in peers)
           and r['params']['merged_ray_rows'] == r['params']['own_ray_rows'] + sum(r['params']['peer_ray_rows'].values())
           for r in rows), 'no full-fleet ray fusion'
assert all(max(r['params'].get('peer_ray_updates', {}).get(p, 0) for r in rows) >= 3
           for p in peers), 'no repeated new ray clouds from every peer'
active = [r for r in rows if r['ts'] >= rows[0]['ts'] + 60]
fresh = lambda r: all(r['params'].get('peer_ray_age_wall_s', {}).get(p, 1e9) <= 15 for p in peers)
assert active and sum(fresh(r) for r in active) / len(active) >= .7, 'rays stale through most of search'
assert any(fresh(r) for r in rows if r['ts'] >= rows[-1]['ts']-30), 'no fresh fleet rays near mission end'
# Actual movement/trajectory scoring is also audited after download; this rejects gross missing/stationary output.
metrics = json.loads((root / 'metrics.json').read_text())
assert metrics['mission_reason'] == 'sim_time_budget', 'search did not finish its 300-second sim budget'
assert result['path_length_m'] > 1 and result['num_odom_samples'] > 10, 'missing movement evidence'
traffic = [json.loads(s) for s in Path(f'/tmp/gossip_bandwidth_{robot}.jsonl').read_text().splitlines()]
start = result['mission_start_ts'] * 1e9
traffic = [r for r in traffic if r['sim_ns'] is not None and start <= r['sim_ns'] <= start+300e9]
assert any(r['direction']=='tx' and r['payload_bytes'].get('shared_rays',0)>0 for r in traffic), 'no ray TX bytes'
assert all(any(r['direction']=='rx' and r['source']==p and r['payload_bytes'].get('shared_rays',0)>0 for r in traffic)
           for p in peers), 'missing peer ray RX bytes'
assert not any(r['payload_bytes'].get('semantic_voxel_map',0) for r in traffic), 'map-sharing leaked into raw-ray ablation'
print(f'{robot}: unpruned raw rays, fleet fusion, freshness, search duration, motion and bytes verified')

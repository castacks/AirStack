#!/usr/bin/env python3
"""Summarize a saved trial's gossip bytes over each robot's 300s search.

Usage: python3 osmo/workspace/summarize_gossip_bytes.py <iteration-dir>
Writes gossip_bandwidth_summary.json next to iteration.json. Sum TX once
across robots for publisher traffic; sum RX separately for actual receptions.
Neither is an Ethernet/RTPS packet capture or a claim about retransmissions.
"""
import argparse
import collections
import json
from pathlib import Path


def summarize(run, duration=300):
    run = Path(run)
    robots = {}
    for result in sorted((run / 'raven_results').glob('robot_[0-9]*.json')):
        if not result.stem.removeprefix('robot_').isdigit():
            continue
        robot = result.stem
        start = json.loads(result.read_text())['mission_start_ts'] * 1e9
        files = list(run.rglob(f'gossip_bandwidth_{robot}.jsonl'))
        if len(files) != 1:
            raise ValueError(f'{robot}: expected one byte ledger, found {len(files)}')
        directions = {}
        for line in files[0].read_text().splitlines():
            row = json.loads(line)
            if row['sim_ns'] is None or not start <= row['sim_ns'] <= start + duration * 1e9:
                continue
            d = directions.setdefault(row['direction'], dict(messages=0, cdr_bytes=0,
                payload_bytes=collections.Counter(), first_wall_ns=row['wall_ns'], last_wall_ns=row['wall_ns']))
            d['messages'] += 1
            d['cdr_bytes'] += row['cdr_bytes']
            d['payload_bytes'].update(row['payload_bytes'])
            d['last_wall_ns'] = row['wall_ns']
        if not directions.get('tx', {}).get('messages'):
            raise ValueError(f'{robot}: no TX measurements in the search window')
        for d in directions.values():
            span = (d['last_wall_ns'] - d['first_wall_ns']) / 1e9
            d['wall_span_s'] = span
            d['cdr_MiB'] = d['cdr_bytes'] / 2**20
            d['mean_cdr_Mbit_s'] = d['cdr_bytes'] * 8 / span / 1e6 if span else None
        robots[robot] = directions
    if not robots:
        raise ValueError('no robot results found')
    fleet = {}
    for direction in ('tx', 'rx'):
        entries = [r[direction] for r in robots.values() if direction in r]
        counts = collections.Counter()
        for e in entries:
            counts.update(e['payload_bytes'])
        fleet[direction] = dict(messages=sum(e['messages'] for e in entries),
            cdr_bytes=sum(e['cdr_bytes'] for e in entries), payload_bytes=counts)
    return dict(measurement='serialized ROS application bytes; excludes DDS/transport overhead',
                duration_sim_s=duration, robots=robots, fleet=fleet)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('iteration', type=Path)
    args = parser.parse_args()
    result = summarize(args.iteration)
    dest = args.iteration / 'gossip_bandwidth_summary.json'
    dest.write_text(json.dumps(result, indent=2))
    print(json.dumps(result['fleet'], indent=2))

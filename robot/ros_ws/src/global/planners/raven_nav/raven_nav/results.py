"""The per-robot result dump: `<results_dir>/<robot>.json`.

The schema is a contract with two offline readers that parse it by key name:
`raven_nav.compile_results` (which re-runs the same
`discoveries.merge_confirmed_targets` over `confirmed_targets_enu`) and, via
`compiled_results.json`, `raven_nav.compare_to_groundtruth`.  Keep every key.

`intent_events` stays as an empty list: it only ever carried the VLFM
baseline's pursue/release records, and that baseline is gone, but
`compile_results` and any archived run still expect the key.
"""
from __future__ import annotations

import json
import os
from typing import Callable, Iterable, List, Optional

import numpy as np

# Every top-level key `_write_results` has ever produced. `test_results_schema`
# asserts the built dict matches this exactly, and that it covers every key
# compile_results.py reads.
RESULT_KEYS = (
    'robot', 'boot_enu', 'alt_ground', 'mission_start_ts', 'dump_ts',
    'mission_duration_s', 'completion_reason', 'coverage_fraction',
    'coverage_threshold', 'path_length_m', 'num_odom_samples',
    'query_labels', 'target_labels', 'confirmed_targets_enu',
    'target_events', 'intent_events',
)

TARGET_EVENT_KEYS = (
    'label', 'instance_id', 'pos_enu', 'first_discovered_ts',
    'first_confirmed_ts', 'first_visited_ts', 'first_discovered_rel_s',
    'first_confirmed_rel_s', 'first_visited_rel_s',
)

CONFIRMED_TARGET_KEYS = ('label', 'center_enu', 'size', 'status', 'confidence')


def build_results_dict(*, robot: str, boot_enu, alt_ground: Optional[float],
                       mission_start_ts: Optional[float], now: float,
                       completion_reason: str, coverage_fraction: float,
                       coverage_threshold: float, path_length_m: float,
                       num_odom_samples: int, query_labels: Iterable[str],
                       target_labels: Iterable[str],
                       confirmed_targets: Iterable,
                       to_world: Callable, target_events: Iterable[dict],
                       intent_events: Optional[List[dict]] = None) -> dict:
    targets = []
    for ct in confirmed_targets:
        targets.append({
            'label': ct.label,
            'center_enu': np.asarray(to_world(ct.center), dtype=float).tolist(),
            'size': np.asarray(ct.size, dtype=float).tolist(),
            'status': ct.status,
            'confidence': float(ct.confidence),
        })

    start = mission_start_ts

    def _rel(t):
        return (t - start) if (t is not None and start is not None) else None

    events = []
    for ev in target_events:
        events.append({
            'label': ev['label'],
            'instance_id': ev['instance_id'],
            'pos_enu': ev['pos_enu'],
            'first_discovered_ts': ev['first_discovered_ts'],
            'first_confirmed_ts': ev['first_confirmed_ts'],
            'first_visited_ts': ev['first_visited_ts'],
            'first_discovered_rel_s': _rel(ev['first_discovered_ts']),
            'first_confirmed_rel_s': _rel(ev['first_confirmed_ts']),
            'first_visited_rel_s': _rel(ev['first_visited_ts']),
        })

    return {
        'robot': robot,
        'boot_enu': np.asarray(boot_enu, dtype=float).tolist(),
        'alt_ground': alt_ground,
        'mission_start_ts': start,
        'dump_ts': float(now),
        'mission_duration_s': (float(now) - start) if start is not None else None,
        'completion_reason': completion_reason or 'in_progress',
        'coverage_fraction': float(coverage_fraction),
        'coverage_threshold': float(coverage_threshold),
        'path_length_m': float(path_length_m),
        'num_odom_samples': int(num_odom_samples),
        'query_labels': list(query_labels),
        'target_labels': list(target_labels),
        'confirmed_targets_enu': targets,
        'target_events': events,
        # VLFM baseline leftovers; the key stays for the readers.
        'intent_events': list(intent_events or []),
    }


def write_results_atomic(results_dir: str, robot: str, payload: dict) -> str:
    """`.{robot}.json.tmp` -> os.replace, so a reader never sees a half file."""
    os.makedirs(results_dir, exist_ok=True)
    tmp = os.path.join(results_dir, f'.{robot}.json.tmp')
    final = os.path.join(results_dir, f'{robot}.json')
    with open(tmp, 'w') as f:
        json.dump(payload, f, indent=2)
    os.replace(tmp, final)
    return final

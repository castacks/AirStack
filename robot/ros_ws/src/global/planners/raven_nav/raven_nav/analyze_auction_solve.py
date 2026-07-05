"""Offline knob-redundancy analysis of /debug/auction_solve traces.

Replays every logged consensus solve with one cost term disabled and diffs
the resulting assignment against the logged one — a term that never flips a
decision is dead weight in these scenes. Also reports decline behavior,
ownership churn, and cross-robot solve divergence.

Usage (inside the robot container, or anywhere with rclpy for bag input):
    ros2 run raven_nav analyze_auction_solve <bag_dir | traces.jsonl>
        [--variants no_geo,no_repulsion,...] [--dt 0.3] [--json out.json]

Input: a rosbag2 directory (reads every topic ending in /debug/auction_solve)
or a JSONL file (one trace per line, e.g. from `ros2 topic echo`).
Replay needs the full-precision fields (pos/size/w/prev) recorded by
bid_manager.last_debug; traces without them fall back to trace-only metrics.
"""
import argparse
import json
from collections import defaultdict
from contextlib import contextmanager
from pathlib import Path

import numpy as np

from raven_nav import bid_manager
from raven_nav.bid_manager import ConsensusAssigner, Task

# variant -> module-constant overrides / replay flags.
VARIANTS = {
    'no_geo':        {'GEO_PRIOR_W': 0.0},
    'no_repulsion':  {'PEER_REPULSION_W': 0.0},
    'no_retention':  {'switch_margin': 0.0},
    'no_age_weight': {'unit_weights': True},
    'no_detour_cap': {'BUNDLE_MAX_DETOUR_M': 1e9},
    'no_decline':    {'no_explore': True},
    'no_markup':     {'EXPLORE_MARKUP_M': 0.0},
}
_CONST_KEYS = ('GEO_PRIOR_W', 'PEER_REPULSION_W', 'BUNDLE_MAX_DETOUR_M',
               'EXPLORE_MARKUP_M')


def _key(k):
    return tuple(k)


def can_replay(trace):
    return (trace.get('prev') is not None
            and all('size' in t for t in trace.get('tasks', [])))


def build_tasks(trace):
    out = []
    for t in trace['tasks']:
        out.append(Task(
            key=_key(t['key']), label=str(t['label']),
            centroid=np.array([t['xy'][0], t['xy'][1], 0.0]),
            size=np.array([t['size'][0], t['size'][1], 0.0]),
            status=str(t['status'])))
    return out


@contextmanager
def _patched(consts):
    saved = {k: getattr(bid_manager, k) for k in consts}
    for k, v in consts.items():
        setattr(bid_manager, k, v)
    try:
        yield
    finally:
        for k, v in saved.items():
            setattr(bid_manager, k, v)


def replay_solve(trace, variant=None):
    """Re-run one logged solve (optionally with a variant); returns the
    normalized assignment {aid: [key tuples] | 'explore'}."""
    v = dict(VARIANTS.get(variant, {})) if variant else {}
    consts = {k: v.pop(k) for k in list(v) if k in _CONST_KEYS}
    agents = trace['agents']
    pos = {int(a): np.array([d['pos'][0], d['pos'][1], 0.0])
           for a, d in agents.items()}
    w = ({int(a): 1.0 for a in agents} if v.get('unit_weights')
         else {int(a): float(d['w']) for a, d in agents.items()})
    ex = ({} if v.get('no_explore') else
          {int(a): float(d['explore_dist']) for a, d in agents.items()
           if d.get('explore_dist') is not None})
    p = trace['params']
    ca = ConsensusAssigner(int(p['bundle_len']))
    ca._prev = {int(a): [np.array([c[0], c[1], 0.0]) for c in cs]
                for a, cs in (trace.get('prev') or {}).items()}
    ca._prev_explored = set(int(a) for a in trace.get('prev_explored') or [])
    sm = v.get('switch_margin', float(p['switch_margin']))
    with _patched(consts):
        assigned = ca.assign(build_tasks(trace), pos, sm,
                             float(p['match_m']),
                             agent_weight=w, explore_dist=ex)
    return norm_assigned(assigned)


def norm_assigned(assigned):
    return {int(a): ('explore' if not ks else [_key(k) for k in ks])
            for a, ks in assigned.items()}


def norm_logged(trace):
    out = {}
    for a, ks in trace.get('result', {}).items():
        out[int(a)] = 'explore' if ks == 'explore' else [_key(k) for k in ks]
    return out


def _heads(m):
    return {a: (ks[0] if isinstance(ks, list) else ks) for a, ks in m.items()}


# ── loading ──────────────────────────────────────────────────────────────────

def load_traces(path):
    p = Path(path)
    if p.is_dir():
        msgs = _read_bag(p)
    else:
        msgs = []
        with open(p) as f:
            for line in f:
                line = line.strip()
                if line:
                    msgs.append(json.loads(line))
    msgs = [m for m in msgs if 'result' in m and 'tasks' in m]
    msgs.sort(key=lambda m: (str(m.get('robot', '')), float(m.get('ts', 0))))
    return msgs


def _read_bag(bag_dir):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from std_msgs.msg import String
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_dir)), ConverterOptions('', ''))
    wanted = {t.name for t in reader.get_all_topics_and_types()
              if t.name.endswith('/debug/auction_solve')}
    out = []
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic not in wanted:
            continue
        try:
            out.append(json.loads(deserialize_message(data, String).data))
        except (ValueError, TypeError):
            pass
    return out


# ── metrics ──────────────────────────────────────────────────────────────────

def term_decisiveness(traces, variants):
    """Per variant: how often disabling the term changes the solve."""
    rows = {}
    replayable = [t for t in traces if can_replay(t)]
    base_ok = sum(1 for t in replayable
                  if replay_solve(t) == norm_logged(t))
    for name in variants:
        head_flips = any_flips = decline_delta = 0
        for t in replayable:
            logged = norm_logged(t)
            alt = replay_solve(t, name)
            if _heads(alt) != _heads(logged):
                head_flips += 1
            if alt != logged:
                any_flips += 1
            decline_delta += (
                sum(1 for v in alt.values() if v == 'explore')
                - sum(1 for v in logged.values() if v == 'explore'))
        n = max(len(replayable), 1)
        rows[name] = {'ticks': len(replayable),
                      'head_flip_pct': 100.0 * head_flips / n,
                      'any_flip_pct': 100.0 * any_flips / n,
                      'decline_delta': decline_delta}
    return {'replay_fidelity_pct':
            100.0 * base_ok / max(len(replayable), 1),
            'replayable_ticks': len(replayable),
            'total_ticks': len(traces),
            'variants': rows}


def decline_behavior(traces):
    """Per robot: decline rate + explore<->task flap transitions."""
    out = {}
    by_robot = defaultdict(list)
    for t in traces:
        by_robot[str(t.get('robot', '?'))].append(t)
    for robot, ts in by_robot.items():
        declines = flaps = 0
        prev_state = {}
        for t in ts:
            logged = norm_logged(t)
            for a, v in logged.items():
                s = 'explore' if v == 'explore' else 'task'
                if v == 'explore':
                    declines += 1
                if prev_state.get(a) is not None and prev_state[a] != s:
                    flaps += 1
                prev_state[a] = s
        out[robot] = {'ticks': len(ts), 'decline_ticks': declines,
                      'explore_task_transitions': flaps}
    return out


def ownership_churn(traces):
    """Per task key: distinct owners + owner changes over the run."""
    hist = defaultdict(list)   # key -> [(ts, owner)]
    for t in traces:
        for task in t.get('tasks', []):
            hist[_key(task['key'])].append(
                (float(t.get('ts', 0)), task.get('owner')))
    rows = []
    for k, seq in hist.items():
        seq.sort()
        owners = [o for _, o in seq]
        changes = sum(1 for a, b in zip(owners, owners[1:]) if a != b)
        span = max(seq[-1][0] - seq[0][0], 1e-6)
        rows.append({'key': list(k), 'samples': len(seq),
                     'distinct_owners': len(set(owners)),
                     'owner_changes': changes,
                     'changes_per_min': 60.0 * changes / span})
    rows.sort(key=lambda r: -r['owner_changes'])
    return rows


def dropped_bb_summary(traces):
    """Task-table drops by reason + the distinct visited-dedup victims —
    directly answers 'why was this BB never assigned'."""
    by_reason = defaultdict(int)
    victims = {}
    for t in traces:
        for d in t.get('dropped_bbs', []):
            by_reason[d.get('reason', '?')] += 1
            if d.get('reason') == 'visited-dedup':
                k = (d.get('label'), round(d['xy'][0]), round(d['xy'][1]))
                v = victims.setdefault(k, {**d, 'ticks': 0})
                v['ticks'] += 1
    return {'drops_by_reason': dict(by_reason),
            'visited_dedup_victims': sorted(
                victims.values(), key=lambda v: -v['ticks'])}


def cross_robot_divergence(traces, dt):
    """Same-tick result diffs across robots, attributed to input diffs."""
    buckets = defaultdict(list)
    for t in traces:
        buckets[round(float(t.get('ts', 0)) / dt)].append(t)
    compared = agree = 0
    attribution = defaultdict(int)
    for _b, ts in sorted(buckets.items()):
        if len({str(t.get('robot')) for t in ts}) < 2:
            continue
        base, rest = ts[0], ts[1:]
        for other in rest:
            compared += 1
            shared = (set(norm_logged(base)) & set(norm_logged(other)))
            if all(norm_logged(base)[a] == norm_logged(other)[a]
                   for a in shared):
                agree += 1
                continue
            deltas = {'task_set': _task_set_delta(base, other),
                      'pos': _agent_delta(base, other, 'pos'),
                      'w': _agent_delta(base, other, 'w'),
                      'explore_dist': _agent_delta(base, other,
                                                   'explore_dist')}
            attribution[max(deltas, key=lambda k: deltas[k])] += 1
    return {'pairs_compared': compared, 'pairs_agree': agree,
            'agreement_pct': 100.0 * agree / max(compared, 1),
            'disagreement_attribution': dict(attribution)}


def _task_set_delta(a, b):
    ka = {_key(t['key']) for t in a.get('tasks', [])}
    kb = {_key(t['key']) for t in b.get('tasks', [])}
    return float(len(ka ^ kb))


def _agent_delta(a, b, field):
    da, db = a.get('agents', {}), b.get('agents', {})
    worst = 0.0
    for aid in set(da) & set(db):
        va, vb = da[aid].get(field), db[aid].get(field)
        if va is None or vb is None:
            continue
        if isinstance(va, list):
            worst = max(worst, float(np.linalg.norm(
                np.asarray(va) - np.asarray(vb))))
        else:
            worst = max(worst, abs(float(va) - float(vb)))
    return worst


# ── report ───────────────────────────────────────────────────────────────────

def analyze(traces, variants, dt):
    return {
        'term_decisiveness': term_decisiveness(traces, variants),
        'decline_behavior': decline_behavior(traces),
        'dropped_bbs': dropped_bb_summary(traces),
        'ownership_churn_top': ownership_churn(traces)[:10],
        'cross_robot_divergence': cross_robot_divergence(traces, dt),
    }


def print_report(rep):
    td = rep['term_decisiveness']
    print(f"\nreplayable ticks: {td['replayable_ticks']}/{td['total_ticks']}"
          f"   replay fidelity: {td['replay_fidelity_pct']:.1f}%"
          f"   (must be ~100 for the flip rates to mean anything)")
    print('\nTERM DECISIVENESS — % of solves that change with the term off')
    print(f"  {'variant':<15}{'head flip %':>12}{'any flip %':>12}"
          f"{'decline delta':>15}")
    for name, r in td['variants'].items():
        print(f"  {name:<15}{r['head_flip_pct']:>11.1f} "
              f"{r['any_flip_pct']:>11.1f} {r['decline_delta']:>14d}")
    print('\nDECLINE BEHAVIOR')
    for robot, r in rep['decline_behavior'].items():
        print(f"  {robot}: ticks={r['ticks']} "
              f"decline_ticks={r['decline_ticks']} "
              f"explore<->task transitions={r['explore_task_transitions']}")
    db = rep['dropped_bbs']
    print('\nTASK-TABLE DROPS (BBs filtered before the auction)')
    if db['drops_by_reason']:
        print(f"  by reason: {db['drops_by_reason']}")
        for v in db['visited_dedup_victims'][:10]:
            print(f"  visited-dedup victim {v.get('label')}@"
                  f"({v['xy'][0]:.0f},{v['xy'][1]:.0f}) size={v['size']} "
                  f"for {v['ticks']} ticks — matched fragment @"
                  f"({v.get('match_xy', ['?', '?'])[0]},"
                  f"{v.get('match_xy', ['?', '?'])[1]}) "
                  f"centre_dist={v.get('centre_dist', float('nan')):.1f} "
                  f"surface_gap={v.get('surface_gap', float('nan')):.1f}")
    else:
        print('  none recorded')
    print('\nOWNERSHIP CHURN (top 10 by owner changes)')
    for r in rep['ownership_churn_top']:
        print(f"  {tuple(r['key'])}: owners={r['distinct_owners']} "
              f"changes={r['owner_changes']} "
              f"({r['changes_per_min']:.2f}/min over {r['samples']} samples)")
    dv = rep['cross_robot_divergence']
    print(f"\nCROSS-ROBOT DIVERGENCE: {dv['pairs_agree']}/"
          f"{dv['pairs_compared']} same-tick pairs agree "
          f"({dv['agreement_pct']:.1f}%)")
    if dv['disagreement_attribution']:
        print(f"  disagreements attributed to largest input delta: "
              f"{dv['disagreement_attribution']}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('path', help='rosbag2 dir or JSONL file of traces')
    ap.add_argument('--variants', default=','.join(VARIANTS),
                    help='comma list of variants to replay '
                         f'(default: all = {",".join(VARIANTS)})')
    ap.add_argument('--dt', type=float, default=0.3,
                    help='same-tick bucket width (s) for divergence')
    ap.add_argument('--json', dest='json_out', default=None,
                    help='also dump the full report to this JSON file')
    args = ap.parse_args()

    traces = load_traces(args.path)
    if not traces:
        raise SystemExit('no auction_solve traces found')
    variants = [v.strip() for v in args.variants.split(',') if v.strip()]
    unknown = [v for v in variants if v not in VARIANTS]
    if unknown:
        raise SystemExit(f'unknown variants {unknown}; '
                         f'choose from {list(VARIANTS)}')
    rep = analyze(traces, variants, args.dt)
    print_report(rep)
    if args.json_out:
        with open(args.json_out, 'w') as f:
            json.dump(rep, f, indent=1)
        print(f'\nfull report -> {args.json_out}')


if __name__ == '__main__':
    main()

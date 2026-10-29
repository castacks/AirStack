"""Frozen diagnostic preferences, then a separate qualified pose-proxy join.

No inference, fitting, new reference transport or adopted gate. Features are
offline reconstructions; original write deadlines are bounds, not measured
feature-computation latency.
"""
import argparse
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.stats import rankdata

FEATURES = ('median_reprojection_px', 'median_abs_disparity_px',
            'whitened_mean_per_component', 'mixed_units_condition_number')
OLD = {'flow_trace_px2': 'lower', 'ray_trace_rad2': 'lower', 'retained_fraction': 'higher',
       'occupied_cells': 'higher', 'prior_supported': 'higher'}
CASES = {'early': 'shared-arbiter-events', 'middle': 'arbiter-support-collapse', 'day-middle': 'history-day-replication'}
CAMERAS = ('rgb', 'thermal')
LOSSES = ('translation_m', 'rotation_deg')
CONTROLS = ('rgb_only', 'thermal_only', 'fixed_rgb_preference', 'history')
AUDIT = 'results/optimizer-factor-audit/run-005'
QUALITY = 'results/arbiter-quality-signals/run-001'
HISTORY = ('results/arbiter-history/run-001', 'results/history-day-replication/history-001')
LABELS = ('results/arbiter-pose-proxy/run-002', 'results/history-day-replication/proxy-001')


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p): return json.loads(Path(p).read_text())
def save(p, value): Path(p).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def preference(values, available, direction='lower'):
    eligible = [c for c in CAMERAS if available[c]]
    if not eligible: return 'ABSTAIN', 'no_available_expert'
    if len(eligible) == 1: return eligible[0].upper(), 'sole_available_fallback'
    a, b = (values[c] for c in CAMERAS)
    if any(v is None or not np.isfinite(v) for v in (a, b)): return 'ABSTAIN', 'missing_feature'
    if a == b: return 'RGB', 'exact_tie_rgb'
    return ('RGB' if (a < b if direction == 'lower' else a > b) else 'THERMAL'), 'decisive'


def stats(values):
    a = np.asarray(values, dtype=float)
    return {'count': len(a), 'mean': float(a.mean()), 'rmse': float(np.sqrt(np.mean(a*a)))} if len(a) else {'count': 0, 'mean': None, 'rmse': None}


def association(x, y):
    if len(x) < 3: return {'count': len(x), 'rho': None, 'reason': 'fewer_than_three'}
    a, b = rankdata(x), rankdata(y)
    if np.ptp(a) == 0 or np.ptp(b) == 0: return {'count': len(x), 'rho': None, 'reason': 'constant_ranks'}
    return {'count': len(x), 'rho': float(np.corrcoef(a, b)[0, 1]), 'reason': None}


def verify(root, protocol):
    if protocol['features'] != list(FEATURES) or protocol['old_features'] != OLD or protocol['directions'] != 'all_lower' or protocol['labels_used_for_prediction'] or protocol['fitted']:
        raise ValueError('frozen diagnostic protocol changed')
    for path, digest in protocol['input_sha256'].items():
        if sha(root/path) != digest: raise ValueError('frozen input changed: '+path)


def prepare(root, run):
    run.mkdir(parents=True, exist_ok=False)
    files = [root/AUDIT/f for f in ('audit.json', 'validation.json')]
    audit = read(root/AUDIT/'audit.json')
    files += [root/p for p in audit['input_sha256']]
    files += [root/QUALITY/f for f in ('features.json', 'protocol.json', 'validation.json')]
    for history in HISTORY: files += [root/history/f for f in ('predictions.json', 'protocol.json', 'validation.json')]
    files += [Path(__file__)]
    save(run/'protocol.json', {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(), 'base_commit': 'f803da2ee',
         'features': list(FEATURES), 'directions': 'all_lower', 'old_features': OLD, 'fitted': False,
         'labels_used_for_prediction': False, 'exact_feature_ties': 'RGB', 'reference_tie_absolute_tolerance': 1e-12,
         'missing_feature': 'diagnostic abstention when both available', 'sole_available': 'availability fallback',
         'primary': 'translation_m', 'secondary': 'rotation_deg', 'cases': CASES,
         'scope': 'retrospective fixed diagnostics on already inspected recordings, no independent GT/heldout/latency claim',
         'input_sha256': {str(p.relative_to(root)): sha(p) for p in sorted(set(files))}})


def predict(root, run):
    if (run/'predictions.json').exists(): raise FileExistsError('predictions already exist')
    protocol = read(run/'protocol.json'); verify(root, protocol)
    audit = read(root/AUDIT/'audit.json'); proof = read(root/AUDIT/'validation.json')
    if proof['status'] != 'PASS' or proof['audit_sha256'] != sha(root/AUDIT/'audit.json') or audit['labels_loaded']:
        raise ValueError('unverified factor audit')
    factors = {(r['case'], r['camera'], r['pair_ordinal']): r for r in audit['rows']}
    if len(factors) != len(audit['rows']) or len(factors) != 434: raise ValueError('factor population changed')
    quality = read(root/QUALITY/'features.json'); qp = read(root/QUALITY/'validation.json')
    if qp['status'] != 'PASS' or qp['features_sha256'] != sha(root/QUALITY/'features.json') or quality['labels_loaded']:
        raise ValueError('unverified prior features')
    old = {(r['window'], r['pair_ordinal']): r for r in quality['rows']}
    histories = {}
    for rel in HISTORY:
        p = read(root/rel/'predictions.json'); validation = read(root/rel/'validation.json')
        if validation['status'] != 'PASS' or validation['predictions_sha256'] != sha(root/rel/'predictions.json') or p['protocol_sha256'] != sha(root/rel/'protocol.json'):
            raise ValueError('unverified history')
        for d in p['decisions']:
            key = d['window'], d['pair_ordinal']
            if key in histories or d['reference_or_loss_used']: raise ValueError('invalid history population')
            histories[key] = d
    rows = []
    for window, directory in CASES.items():
        source = root/'results'/directory/'run-001'
        for line in (source/'timeline.jsonl').read_text().splitlines():
            d = json.loads(line)
            if d['kind'] != 'decision': continue
            pair = d['pair_ordinal']; key = window, pair; available = d['availability']
            previous = old[key]; history = histories[key]
            if previous['availability'] != available or history['base_availability'] != available or previous['source_decision_sequence'] != d['sequence'] or history['source_decision_sequence'] != d['sequence'] or history['source_deadline_monotonic_ns'] != d['controller_monotonic_ns']:
                raise ValueError('source/feature/history deadline mismatch')
            values = {}; motion_ids = {}
            for c in CAMERAS:
                f = factors[directory, c, pair]
                if f['available_at_original_deadline'] != available[c]: raise ValueError('factor deadline mismatch')
                if available[c]:
                    if f['motion_event_sequence'] != d['evidence'][c]['motion_event_sequence'] or f['motion_observation_monotonic_ns'] > d['controller_monotonic_ns'] or f['diagnostics'] is None:
                        raise ValueError('future or wrong motion feature')
                    values[c] = {name: f['diagnostics'][name] for name in FEATURES}; motion_ids[c] = f['motion_event_sequence']
                else: values[c] = {name: None for name in FEATURES}; motion_ids[c] = None
            choices = {name: dict(zip(('action', 'reason'), preference({c: values[c][name] for c in CAMERAS}, available))) for name in FEATURES}
            older = {name: dict(zip(('action', 'reason'), preference({c: previous['experts'][c]['values'][name] for c in CAMERAS}, available, direction))) for name, direction in OLD.items()}
            fixed = 'RGB' if available['rgb'] else 'THERMAL' if available['thermal'] else 'ABSTAIN'
            controls = {'rgb_only': 'RGB' if available['rgb'] else 'ABSTAIN', 'thermal_only': 'THERMAL' if available['thermal'] else 'ABSTAIN', 'fixed_rgb_preference': fixed, 'history': history['action']}
            if history['action'] in ('RGB', 'THERMAL') and not history['confirmed_availability'][history['action'].lower()]: raise ValueError('unconfirmed history')
            rows.append({'window': window, 'pair_ordinal': pair, 'source_decision_sequence': d['sequence'],
                         'deadline_monotonic_ns': d['controller_monotonic_ns'], 'availability': available,
                         'motion_event_sequences': motion_ids, 'values': values, 'choices': choices, 'old_choices': older, 'controls': controls})
    if len(rows) != 217 or {(r['window'], r['pair_ordinal']) for r in rows} != set(old) or set(old) != set(histories): raise ValueError('missing/extra deadlines')
    verify(root, protocol)
    save(run/'predictions.json', {'protocol_sha256': sha(run/'protocol.json'), 'labels_loaded': False, 'rows': rows})
    print(json.dumps({'deadlines': len(rows), 'both_available': sum(all(r['availability'].values()) for r in rows), 'labels_loaded': False}))


def losses_for(row, policy):
    if policy in CONTROLS: action = row['controls'][policy]
    elif policy in FEATURES: action = row['choices'][policy]['action']
    else: action = row['old_choices'][policy]['action']
    return row['losses'].get(action.lower())


def comparison(rows, first, second):
    matched = [r for r in rows if losses_for(r, first) is not None and losses_for(r, second) is not None]
    return {'pair_ordinals': [r['pair_ordinal'] for r in matched], 'count': len(matched),
            'first': {k: stats([losses_for(r, first)[k] for r in matched]) for k in LOSSES},
            'second': {k: stats([losses_for(r, second)[k] for r in matched]) for k in LOSSES}}


def summarize(rows):
    policies = (*FEATURES, *CONTROLS)
    full = {}
    for name in policies:
        values = [r for r in rows if losses_for(r, name) is not None]
        full[name] = {'available': len(values), 'abstain': len(rows)-len(values), 'pair_ordinals': [r['pair_ordinal'] for r in values],
                      **{k: stats([losses_for(r, name)[k] for r in values]) for k in LOSSES}}
    both = [r for r in rows if all(r['availability'].values())]; features = {}
    for name in FEATURES:
        common = [r for r in both if losses_for(r, name) is not None]
        decisive = [r for r in common if r['choices'][name]['reason'] == 'decisive']
        accuracy = {}
        for loss in LOSSES:
            def correct(r):
                a, b = (r['losses'][c][loss] for c in CAMERAS)
                return None if abs(a-b) <= 1e-12 else (r['choices'][name]['action'] == ('RGB' if a < b else 'THERMAL'))
            accuracy[loss] = {'reference_ties': sum(correct(r) is None for r in common),
                              'correct_including_feature_ties': sum(correct(r) is True for r in common),
                              'decisive_correct': sum(correct(r) is True for r in decisive)}
        disagreements = {}
        for old in OLD:
            eligible = [r for r in common if losses_for(r, old) is not None]
            diff = [r for r in eligible if r['choices'][name]['action'] != r['old_choices'][old]['action']]
            disagreements[old] = {'agreed': len(eligible)-len(diff), 'missing': len(common)-len(eligible),
                'comparison': comparison(diff, name, old), 'differences': {k: dict(Counter('better' if losses_for(r, name)[k] < losses_for(r, old)[k]-1e-12 else 'worse' if losses_for(r, name)[k] > losses_for(r, old)[k]+1e-12 else 'tie' for r in diff)) for k in LOSSES}}
        features[name] = {'pair_ordinals': [r['pair_ordinal'] for r in common], 'comparable': len(common), 'missing': len(both)-len(common),
                          'feature_reasons': dict(Counter(r['choices'][name]['reason'] for r in common)), 'accuracy': accuracy,
                          'matched_controls': {c: comparison(common, name, c) for c in CONTROLS},
                          'oracle': {k: stats([min(r['losses'][c][k] for c in CAMERAS) for r in common]) for k in LOSSES},
                          'per_camera_spearman': {c: {k: association([r['values'][c][name] for r in common], [r['losses'][c][k] for r in common]) for k in LOSSES} for c in CAMERAS},
                          'old_signal_disagreements': disagreements}
    return {'deadlines': len(rows), 'availability_patterns': dict(Counter('both' if all(r['availability'].values()) else 'rgb_only' if r['availability']['rgb'] else 'thermal_only' if r['availability']['thermal'] else 'none' for r in rows)),
            'full_available': full, 'both_available': len(both), 'features': features,
            'full_matched_controls': {n: {c: comparison(rows, n, c) for c in CONTROLS} for n in FEATURES}}


def score(root, run):
    if (run/'analysis.json').exists(): raise FileExistsError('analysis already exists')
    protocol = read(run/'protocol.json'); verify(root, protocol); predictions = read(run/'predictions.json')
    if predictions['protocol_sha256'] != sha(run/'protocol.json') or predictions['labels_loaded']: raise ValueError('prediction contract changed')
    labels = {}; label_hashes = {}
    for rel in LABELS:
        p = root/rel; proof = read(p/'validation.json'); declared = read(p/'protocol.json')
        if proof['status'] != 'PASS' or proof['report_sha256'] != sha(p/'pose-proxy.json') or proof['protocol_sha256'] != sha(p/'protocol.json') or declared['independent_ground_truth'] or declared['physical_body_transform_verified']:
            raise ValueError('expected validated qualified proxy')
        for f in ('pose-proxy.json', 'protocol.json', 'validation.json'): label_hashes[str((p/f).relative_to(root))] = sha(p/f)
        for label in read(p/'pose-proxy.json')['rows']:
            key = label['window'], label['pair_ordinal']
            if key in labels: raise ValueError('duplicate label interval')
            labels[key] = label
    rows = []
    for d in predictions['rows']:
        label = labels[d['window'], d['pair_ordinal']]
        if label['source_decision_sequence'] != d['source_decision_sequence']: raise ValueError('label decision mismatch')
        losses = {c: label['experts'][c]['common_loss'] if label['experts'][c] else None for c in CAMERAS}
        if any((losses[c] is not None) != d['availability'][c] for c in CAMERAS): raise ValueError('label availability mismatch')
        if any(not np.isfinite(v) or v < 0 for c in CAMERAS if losses[c] for v in losses[c].values()): raise ValueError('invalid proxy loss')
        rows.append(d | {'losses': losses})
    if {(r['window'], r['pair_ordinal']) for r in rows} != set(labels): raise ValueError('population differs from labels')
    report = {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(), 'predictions_sha256': sha(run/'predictions.json'),
              'protocol_sha256': sha(run/'protocol.json'), 'scorer_sha256': sha(Path(__file__)), 'label_sha256': label_hashes,
              'windows': {w: summarize([r for r in rows if r['window'] == w]) for w in CASES}, 'rows': rows,
              'scope': 'Retrospective fixed-direction conditional SLAM-proxy diagnostics; no fitted/adopted selector, independent GT or generalization claim.'}
    verify(root, protocol)
    save(run/'analysis.json', report)
    print(json.dumps({w: {'both': s['both_available'], 'diagnostics': {n: {'correct_translation': f['accuracy']['translation_m']['correct_including_feature_ties'], 'selected_rmse_m': f['matched_controls']['rgb_only']['first']['translation_m']['rmse'], 'rgb_rmse_m': f['matched_controls']['rgb_only']['second']['translation_m']['rmse']} for n, f in s['features'].items()}} for w, s in report['windows'].items()}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('action', choices=('prepare', 'predict', 'score'))
    p.add_argument('--root', type=Path, required=True); p.add_argument('--run', type=Path, required=True)
    a = p.parse_args(); {'prepare': prepare, 'predict': predict, 'score': score}[a.action](a.root, a.run)

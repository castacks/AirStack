"""Independent fixed-diagnostic, population and qualified-label reconstruction."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.stats import spearmanr

NEW = ('median_reprojection_px', 'median_abs_disparity_px', 'whitened_mean_per_component', 'mixed_units_condition_number')
OLD = {'flow_trace_px2': 'lower', 'ray_trace_rad2': 'lower', 'retained_fraction': 'higher', 'occupied_cells': 'higher', 'prior_supported': 'higher'}
CONTROL = ('rgb_only', 'thermal_only', 'fixed_rgb_preference', 'history')
LOSSES = ('translation_m', 'rotation_deg')
CASES = {'early': 'shared-arbiter-events', 'middle': 'arbiter-support-collapse', 'day-middle': 'history-day-replication'}


def load(p): return json.loads(Path(p).read_text())
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def equal(a, b):
    if isinstance(b, dict):
        assert isinstance(a, dict) and set(a) == set(b)
        for k in b: equal(a[k], b[k])
    elif isinstance(b, list):
        assert isinstance(a, list) and len(a) == len(b)
        for x, y in zip(a, b): equal(x, y)
    elif b is None or isinstance(b, (str, bool, int)): assert a == b
    else: assert math.isclose(a, b, rel_tol=2e-10, abs_tol=1e-12)


def moments(x):
    if not x: return {'count': 0, 'mean': None, 'rmse': None}
    return {'count': len(x), 'mean': math.fsum(x)/len(x), 'rmse': math.sqrt(math.fsum(v*v for v in x)/len(x))}


def pick(values, availability, high=False):
    if not availability['rgb'] and not availability['thermal']: return {'action': 'ABSTAIN', 'reason': 'no_available_expert'}
    if not all(availability.values()): return {'action': 'RGB' if availability['rgb'] else 'THERMAL', 'reason': 'sole_available_fallback'}
    a, b = values['rgb'], values['thermal']
    if a is None or b is None or not math.isfinite(a) or not math.isfinite(b): return {'action': 'ABSTAIN', 'reason': 'missing_feature'}
    if a == b: return {'action': 'RGB', 'reason': 'exact_tie_rgb'}
    return {'action': 'RGB' if ((a > b) if high else (a < b)) else 'THERMAL', 'reason': 'decisive'}


def selected(r, policy):
    if policy in CONTROL: action = r['controls'][policy]
    elif policy in NEW: action = r['choices'][policy]['action']
    else: action = r['old_choices'][policy]['action']
    return None if action == 'ABSTAIN' else r['losses'][action.lower()]


def matched(rows, first, second):
    ids = []; first_values = {k: [] for k in LOSSES}; second_values = {k: [] for k in LOSSES}
    for r in rows:
        a, b = selected(r, first), selected(r, second)
        if a is None or b is None: continue
        ids.append(r['pair_ordinal'])
        for k in LOSSES: first_values[k].append(a[k]); second_values[k].append(b[k])
    return {'pair_ordinals': ids, 'count': len(ids), 'first': {k: moments(v) for k, v in first_values.items()}, 'second': {k: moments(v) for k, v in second_values.items()}}


def summary(rows):
    full = {}
    for name in (*NEW, *CONTROL):
        subset = [r for r in rows if selected(r, name) is not None]
        full[name] = {'available': len(subset), 'abstain': len(rows)-len(subset), 'pair_ordinals': [r['pair_ordinal'] for r in subset], **{k: moments([selected(r, name)[k] for r in subset]) for k in LOSSES}}
    common = [r for r in rows if all(r['availability'].values())]; features = {}
    for name in NEW:
        subset = [r for r in common if selected(r, name) is not None]; correctness = {}; correlations = {}
        for loss in LOSSES:
            tied = right = decisive_right = 0
            for r in subset:
                a, b = r['losses']['rgb'][loss], r['losses']['thermal'][loss]
                if abs(a-b) <= 1e-12: tied += 1; continue
                chosen_loss = selected(r, name)[loss]
                ok = chosen_loss == min(a, b); right += int(ok)
                if ok and r['choices'][name]['reason'] == 'decisive': decisive_right += 1
            correctness[loss] = {'reference_ties': tied, 'correct_including_feature_ties': right, 'decisive_correct': decisive_right}
        for c in ('rgb', 'thermal'):
            correlations[c] = {}
            for loss in LOSSES:
                x = [r['values'][c][name] for r in subset]; y = [r['losses'][c][loss] for r in subset]
                reason = 'fewer_than_three' if len(x) < 3 else 'constant_ranks' if min(x) == max(x) or min(y) == max(y) else None
                correlations[c][loss] = {'count': len(x), 'rho': None if reason else float(spearmanr(x, y).statistic), 'reason': reason}
        disagreements = {}
        for old in OLD:
            eligible = [r for r in subset if selected(r, old) is not None]
            diff = [r for r in eligible if r['choices'][name]['action'] != r['old_choices'][old]['action']]
            differences = {}
            for loss in LOSSES:
                names = []
                for r in diff:
                    a, b = selected(r, name)[loss], selected(r, old)[loss]
                    names.append('better' if a < b-1e-12 else 'worse' if a > b+1e-12 else 'tie')
                differences[loss] = dict(Counter(names))
            disagreements[old] = {'agreed': len(eligible)-len(diff), 'missing': len(subset)-len(eligible), 'comparison': matched(diff, name, old), 'differences': differences}
        features[name] = {'pair_ordinals': [r['pair_ordinal'] for r in subset], 'comparable': len(subset), 'missing': len(common)-len(subset),
            'feature_reasons': dict(Counter(r['choices'][name]['reason'] for r in subset)), 'accuracy': correctness,
            'matched_controls': {c: matched(subset, name, c) for c in CONTROL},
            'oracle': {k: moments([min(r['losses']['rgb'][k], r['losses']['thermal'][k]) for r in subset]) for k in LOSSES},
            'per_camera_spearman': correlations, 'old_signal_disagreements': disagreements}
    patterns = Counter()
    for r in rows:
        a = r['availability']; patterns['both' if all(a.values()) else 'rgb_only' if a['rgb'] else 'thermal_only' if a['thermal'] else 'none'] += 1
    return {'deadlines': len(rows), 'availability_patterns': dict(patterns), 'full_available': full, 'both_available': len(common), 'features': features,
            'full_matched_controls': {n: {c: matched(rows, n, c) for c in CONTROL} for n in NEW}}


def validate(root, run):
    p = load(run/'protocol.json'); prediction = load(run/'predictions.json'); result = load(run/'analysis.json')
    assert p['features'] == list(NEW) and p['directions'] == 'all_lower' and p['old_features'] == OLD and p['cases'] == CASES
    assert p['labels_used_for_prediction'] is False and p['fitted'] is False and prediction['labels_loaded'] is False
    assert prediction['protocol_sha256'] == result['protocol_sha256'] == sha(run/'protocol.json')
    assert result['predictions_sha256'] == sha(run/'predictions.json')
    assert result['scorer_sha256'] == sha(root/'MAC-VO-main/Scripts/Setup/compare_factor_proxy.py')
    for rel, digest in p['input_sha256'].items(): assert sha(root/rel) == digest
    for rel, digest in result['label_sha256'].items(): assert sha(root/rel) == digest
    assert not any('/pose-proxy/' in rel or '/arbiter-pose-proxy/' in rel or '/proxy-001/' in rel for rel in p['input_sha256'])
    audit = load(root/'results/optimizer-factor-audit/run-005/audit.json'); ap = load(root/'results/optimizer-factor-audit/run-005/validation.json')
    assert ap['status'] == 'PASS' and ap['audit_sha256'] == sha(root/'results/optimizer-factor-audit/run-005/audit.json')
    factors = {(r['case'], r['camera'], r['pair_ordinal']): r for r in audit['rows']}
    old = {(r['window'], r['pair_ordinal']): r for r in load(root/'results/arbiter-quality-signals/run-001/features.json')['rows']}
    history = {}
    for rel in ('results/arbiter-history/run-001', 'results/history-day-replication/history-001'):
        q = load(root/rel/'validation.json'); h = load(root/rel/'predictions.json')
        assert q['status'] == 'PASS' and q['predictions_sha256'] == sha(root/rel/'predictions.json')
        history.update({(r['window'], r['pair_ordinal']): r for r in h['decisions']})
    labels = {}
    for rel in ('results/arbiter-pose-proxy/run-002', 'results/history-day-replication/proxy-001'):
        q = load(root/rel/'validation.json'); lp = load(root/rel/'protocol.json')
        assert q['status'] == 'PASS' and q['report_sha256'] == sha(root/rel/'pose-proxy.json') and q['protocol_sha256'] == sha(root/rel/'protocol.json')
        assert not lp['independent_ground_truth'] and not lp['physical_body_transform_verified']
        labels.update({(r['window'], r['pair_ordinal']): r for r in load(root/rel/'pose-proxy.json')['rows']})
    timelines = {}
    for window, directory in CASES.items():
        for line in (root/'results'/directory/'run-001/timeline.jsonl').read_text().splitlines():
            d = json.loads(line)
            if d['kind'] == 'decision': timelines[window, d['pair_ordinal']] = d
    assert len(prediction['rows']) == len(result['rows']) == len(timelines) == 217
    seen = set(); output_rows = []
    for r in prediction['rows']:
        key = r['window'], r['pair_ordinal']; assert key not in seen; seen.add(key); d = timelines[key]
        assert r['source_decision_sequence'] == d['sequence'] and r['deadline_monotonic_ns'] == d['controller_monotonic_ns'] and r['availability'] == d['availability']
        assert old[key]['availability'] == r['availability'] == history[key]['base_availability']
        for c in ('rgb', 'thermal'):
            f = factors[CASES[r['window']], c, r['pair_ordinal']]
            assert f['available_at_original_deadline'] == r['availability'][c]
            if r['availability'][c]:
                assert r['motion_event_sequences'][c] == f['motion_event_sequence'] == d['evidence'][c]['motion_event_sequence']
                assert f['motion_observation_monotonic_ns'] <= r['deadline_monotonic_ns']
                assert r['values'][c] == {name: f['diagnostics'][name] for name in NEW}
            else: assert r['motion_event_sequences'][c] is None and r['values'][c] == {name: None for name in NEW}
        for name in NEW: assert r['choices'][name] == pick({c: r['values'][c][name] for c in ('rgb', 'thermal')}, r['availability'])
        for name, direction in OLD.items(): assert r['old_choices'][name] == pick({c: old[key]['experts'][c]['values'][name] for c in ('rgb', 'thermal')}, r['availability'], direction == 'higher')
        a = r['availability']; h = history[key]
        assert h['source_decision_sequence'] == d['sequence'] and h['source_deadline_monotonic_ns'] == d['controller_monotonic_ns'] and not h['reference_or_loss_used']
        assert r['controls'] == {'rgb_only': 'RGB' if a['rgb'] else 'ABSTAIN', 'thermal_only': 'THERMAL' if a['thermal'] else 'ABSTAIN', 'fixed_rgb_preference': 'RGB' if a['rgb'] else 'THERMAL' if a['thermal'] else 'ABSTAIN', 'history': h['action']}
        label = labels[key]; assert label['source_decision_sequence'] == d['sequence']
        losses = {c: label['experts'][c]['common_loss'] if label['experts'][c] else None for c in ('rgb', 'thermal')}
        assert all((losses[c] is not None) == a[c] for c in a)
        output_rows.append(r | {'losses': losses})
    assert seen == set(timelines) == set(labels) == set(old) == set(history)
    assert output_rows == result['rows']
    for window in CASES: equal(result['windows'][window], summary([r for r in output_rows if r['window'] == window]))
    verification = {'status': 'PASS', 'rows': len(seen), 'both_available': sum(all(r['availability'].values()) for r in output_rows),
        'new_diagnostic_choices': 4*len(seen), 'old_diagnostic_choices': 5*len(seen), 'disagreement_panels': 3*4*5,
        'analysis_sha256': sha(run/'analysis.json'), 'predictions_sha256': sha(run/'predictions.json'), 'protocol_sha256': sha(run/'protocol.json'),
        'validator_sha256': sha(Path(__file__)), 'scope': 'independent choice/population/label/statistic audit; not independent physical truth or generalization'}
    (run/'validation.json').write_text(json.dumps(verification, indent=2)+'\n'); print(json.dumps(verification))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--root', type=Path, required=True); p.add_argument('--run', type=Path, required=True)
    a = p.parse_args(); validate(a.root, a.run)

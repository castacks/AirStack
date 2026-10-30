"""Retrospective selection opportunity on frozen ORIGINAL-arm motion proxies.

Oracle/best-fixed labels are analysis only. No model, gate, inference or fitting.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

CASES = [('early', 'end-to-end-solver-control'), ('middle', 'solver-collapse-replication'), ('day', 'solver-day-replication')]
LOSSES = ('translation_m', 'rotation_deg')
CAMERAS = ('rgb', 'thermal')

def read(path): return json.loads(path.read_text())
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def write(path, value): path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
def stats(values):
    return {'count': len(values), 'mean': math.fsum(values)/len(values), 'rmse': math.sqrt(math.fsum(x*x for x in values)/len(values))} if values else {'count': 0, 'mean': None, 'rmse': None}


def persistence(rows, loss):
    labels = []
    for row in rows:
        a,b = [row['cameras'][c]['loss'][loss] for c in CAMERAS]
        labels.append((row['pair_ordinal'], None if a == b else 'rgb' if a < b else 'thermal'))
    runs = []; adjacent = same = 0; previous = None; length = 0
    for ordinal, winner in labels:
        contiguous = previous is not None and ordinal == previous[0]+1 and winner is not None and previous[1] is not None
        if contiguous: adjacent += 1; same += winner == previous[1]
        if contiguous and winner == previous[1]: length += 1
        else:
            if length: runs.append(length)
            length = int(winner is not None)
        previous = (ordinal, winner)
    if length: runs.append(length)
    return {'adjacent_strict_pairs': adjacent, 'same_winner': same, 'same_fraction': same/adjacent if adjacent else None, 'strict_run_lengths': runs, 'ties_break_runs': True}


def population(rows):
    ids = [r['pair_ordinal'] for r in rows]; result = {'pair_ordinals': ids, 'count': len(ids), 'losses': {}}
    for loss in LOSSES:
        fixed = {c: stats([r['cameras'][c]['loss'][loss] for r in rows if r['cameras'][c]['loss'] is not None]) for c in CAMERAS}
        decisions = []
        for row in rows:
            available = [(row['cameras'][c]['loss'][loss], i, c) for i,c in enumerate(CAMERAS) if row['cameras'][c]['loss'] is not None]
            value, _, chosen = min(available); decisions.append({'pair_ordinal': row['pair_ordinal'], 'camera': chosen, 'loss': value})
        oracle = stats([d['loss'] for d in decisions]); complete = [c for c in CAMERAS if rows and fixed[c]['count'] == len(rows)]
        best = min(complete, key=lambda c: (fixed[c]['rmse'], CAMERAS.index(c))) if complete else None
        comparisons = {}
        for c in CAMERAS:
            subset = [i for i,r in enumerate(rows) if r['cameras'][c]['loss'] is not None]
            baseline = stats([rows[i]['cameras'][c]['loss'][loss] for i in subset]); bound = stats([decisions[i]['loss'] for i in subset])
            comparisons[c] = {'matched_pair_ordinals': [ids[i] for i in subset], 'fixed': baseline, 'oracle': bound}
        ties = sum(all(r['cameras'][c]['loss'] is not None for c in CAMERAS) and r['cameras']['rgb']['loss'][loss] == r['cameras']['thermal']['loss'][loss] for r in rows)
        result['losses'][loss] = {'fixed_own_available': fixed, 'oracle': oracle, 'oracle_choices': decisions, 'exact_ties': ties,
            'hindsight_best_complete_fixed_camera': best, 'oracle_rmse_reduction_vs_best_complete_fixed': fixed[best]['rmse']-oracle['rmse'] if best else None,
            'oracle_rmse_reduction_percent': 100*(1-oracle['rmse']/fixed[best]['rmse']) if best and fixed[best]['rmse'] else None,
            'matched_fixed_comparisons': comparisons}
    return result


def run(root, output):
    output.mkdir(parents=True, exist_ok=False); inputs = {}; source = Path(__file__).resolve(); proofs = {}
    for name, directory in CASES:
        b = root/'results'/directory/'run-001'; p,e,v = [read(b/x) for x in ('protocol.json','evaluation.json','validation.json')]
        assert v['status'] == 'PASS' and v['evaluation_sha256'] == sha(b/'evaluation.json')
        assert v['execution_protocol_sha256'] == sha(b/'protocol.json') and e['evaluation_protocol_sha256'] == v['evaluation_protocol_sha256'] == sha(b/'evaluation-protocol.json')
        assert not e['fit_performed'] and not e['independent_ground_truth'] and not p['production_defaults_changed'] and not p['reserved_data_used']
        for filename in ('protocol.json','execution.json','evaluation-protocol.json','evaluation.json','validation.json'):
            path = b/filename; inputs[str(path.relative_to(root))] = sha(path)
        if name == 'day':
            extra = read(b/'roundoff-analysis.json'); assert extra['status'] == 'PASS' and extra['native_exact_H_g_cost_attempts'] == v['repair_attempts_checked']
            inputs[str((b/'roundoff-analysis.json').relative_to(root))] = sha(b/'roundoff-analysis.json')
        proofs[name] = {'frames': p['frames'], 'gradient_component_exceptions': v.get('gradient_component_exceptions', []), 'source_result': str(b.relative_to(root))}
    write(output/'protocol.json', {'created_at': datetime.now(timezone.utc).isoformat(), 'base_commit': 'a177ba60c', 'input_sha256': inputs, 'analysis_source_sha256': sha(source),
        'original_arm_only': True, 'oracle_for_prediction': False, 'fitted': False, 'independent_ground_truth': False, 'tie_preference': 'rgb; ties excluded from strict persistence',
        'populations': ['both','both_processing_only','union'], 'terminal_policy': 'included primary; excluded from processing-only sensitivity, not a shared-clock availability claim'})
    rows = []; summaries = {}
    for name, directory in CASES:
        source_rows = [r for r in read(root/'results'/directory/'run-001/evaluation.json')['rows'] if r['arm'] == 'original']; n = proofs[name]['frames']-1
        lookup = {(r['camera'], r['pair_ordinal']): r for r in source_rows}; assert len(lookup) == len(source_rows) == 2*n
        local = []
        for i in range(n):
            r = {'window': name, 'pair_ordinal': i, 'common_interval_ns': lookup['rgb',i]['common_interval_ns'], 'cameras': {}}
            for c in CAMERAS:
                a = lookup[c,i]; assert a['common_interval_ns'] == r['common_interval_ns']
                r['cameras'][c] = {'loss': a['common_loss'], 'stage': a['motion_stage'], 'unavailable_reason': a['unavailable_reason']}
            local.append(r)
        both = [r for r in local if all(r['cameras'][c]['loss'] is not None for c in CAMERAS)]; union = [r for r in local if any(r['cameras'][c]['loss'] is not None for c in CAMERAS)]
        processing = [r for r in both if all(r['cameras'][c]['stage'] == 'pair_processing' for c in CAMERAS)]
        summaries[name] = {'slots': n, 'available': {c: sum(r['cameras'][c]['loss'] is not None for r in local) for c in CAMERAS},
            'both': population(both), 'both_processing_only': population(processing), 'union': population(union), 'neither': n-len(union),
            'strict_oracle_persistence': {loss: persistence(both, loss) for loss in LOSSES}}
        rows += local
    for path,digest in inputs.items(): assert sha(root/path) == digest
    assert sha(source) == read(output/'protocol.json')['analysis_source_sha256']
    write(output/'headroom.json', {'created_at': datetime.now(timezone.utc).isoformat(), 'protocol_sha256': sha(output/'protocol.json'), 'windows': summaries, 'rows': rows, 'source_proofs': proofs,
        'fitted': False, 'oracle_for_prediction': False, 'independent_ground_truth': False,
        'scope': 'retrospective original-arm nominal-body/reference-assisted SLAM-proxy opportunity bound; not causal selection, stitched trajectory or heldout physical accuracy'})
    print(json.dumps({name:{p:{loss:s[p]['losses'][loss]['oracle_rmse_reduction_percent'] for loss in LOSSES} for p in ('both','both_processing_only')} for name,s in summaries.items()}))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.root,a.output)

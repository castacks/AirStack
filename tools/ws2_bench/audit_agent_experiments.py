"""Check executed agent decisions, evidence lineage, pairing and diagnostics."""
import argparse
import json
import math
from pathlib import Path
from agent_campaign import _candidate
from audit_results import audit_episode
from campaign import clean_twin
from episode import fingerprint, patch_active
from expanded_space import action_key
from experiment_protocol import validate
from mission import SUCCESSES
from vulnerability_report import analyze


def read(path):
    return json.loads(Path(path).read_text())


def audit(root):
    root = Path(root)
    config, history = read(root/'config.json'), read(root/'history.json')
    errors, flights, rounds = [], [], []
    if len(history)*2 > config['budget']:
        errors.append('Pair slots exceed budget')
    for index, entry in enumerate(history):
        decision = entry['decision']; problems = []
        folder = root/f'round_{index+1:02d}'
        if read(folder/'decision.json') != decision:
            problems.append('Saved/history decisions differ')
        try:
            validate(decision['experiment'], decision['action'], history[:index], action_key)
            resolved_decision, candidate = _candidate(decision, config, index)
            if resolved_decision != decision:
                problems.append('Resolved decision differs')
            call = read(root/'llm_calls'/(decision['llm_call_id']+'.json'))
            output = call['attempts'][-1]['output']
            if any(output[k] != decision[k] for k in ('action','experiment','reason','hypothesis')):
                problems.append('Executed decision differs from Claude output')
            if len(call['request']['input']['history']) != index:
                problems.append('Claude did not receive the preceding history')
        except (ValueError, KeyError, OSError) as exc:
            problems.append(str(exc)); candidate = None
        pair = entry['pairs'][0]; evidence = []
        for role in ('clean','perturbed'):
            result = pair[role]
            if result.get('executed') is False:
                if role != 'perturbed' or pair['clean']['outcome'] in SUCCESSES:
                    problems.append('Invalid skipped flight')
                continue
            run = Path(result['result_dir'])
            flights.append(audit_episode(run))
            scenario = read(run/'scenario.json')
            if candidate and fingerprint(clean_twin(candidate) if role=='clean' else candidate) != result['configuration_hash']:
                problems.append(role+': config/action fingerprint mismatch')
            ev = read(run/'condition_evidence.json'); evidence.append(ev)
            initial = dict(scenario['condition'],patch_enabled=patch_active(scenario,0))
            if ev['scene']['condition'] != initial:
                problems.append(role+': initial scene differs from requested condition/schedule')
            for ck, dk in [('rgb_noise','rgb_noise_stddev'),('delay','fixed_delay_s')]:
                if scenario['condition'][ck] != ev['frame']['sensor_disturbance'][dk]:
                    problems.append(role+': sensor mismatch '+ck)
            diag = read(run/'diagnostics.json')
            if diag != result.get('diagnostics') or not diag['sample_count'] or not diag['unique_worker_frames']:
                problems.append(role+': missing/stale diagnostic evidence')
            if not all((run/f['file']).is_file() for f in diag['preview_frames']):
                problems.append(role+': preview missing')
            if config['planners'][0]=='mononav' and scenario['condition']['patch_enabled']:
                problems.append('Unsupported MonoNav patch')
        if len(evidence)==2:
            if evidence[0]['scene']['realized_layout'] != evidence[1]['scene']['realized_layout']:
                problems.append('Paired geometry differs')
            if math.dist(*(e['scene']['position'] for e in evidence))>.01:
                problems.append('Initial positions differ >1cm')
        rounds.append({'round':index+1,'experiment':decision['experiment'],
                       'passed':not problems,'errors':problems})
    report = read(root/'vulnerability_report.json')
    expected = analyze(history, [], config, report['phase'])
    if report != expected:
        errors.append('Report differs from independently recomputed measurements')
    return {'passed':not errors and all(x['passed'] for x in [*rounds,*flights]),
            'errors':errors,'rounds':rounds,'flights':flights,
            'executed_flights':len(flights),'reserved_flight_slots':len(history)*2}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);args=parser.parse_args()
    result=audit(args.root)
    (args.root/'agent_artifact_audit.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)

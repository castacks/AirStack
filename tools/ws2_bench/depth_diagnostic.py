"""Compare learned/privileged depth on identical saved clean scenarios.

This diagnostic is separate from agent evaluation. It preserves mission, speed,
layout and controller settings. GT depth bypasses perception and can also reduce
latency; a successful GT flight alone does not establish the cause of RGB failure.
"""
import argparse
import fcntl
import json
from pathlib import Path

from episode import RUNTIME, atomic, fingerprint, resolved, run_episode
from campaign import clean_twin
from diagnostics import numeric_stats


def prepare(path, source):
    config = clean_twin(resolved(json.loads(Path(path).read_text())))
    config['diagnostic_depth_source'] = source
    config['name'] = f"{config['planner']} depth diagnostic: {source}"
    return resolved(config)


def trace_summary(folder):
    records = json.loads((folder/'diagnostic_trace.json').read_text())
    workers = {r['worker']['sequence']: r['worker'] for r in records if r.get('worker')}
    values = list(workers.values())
    stamps = sorted(w['sim_stamp'] for w in values)
    return {'unique_frames': len(values),
            'depth_sources': sorted({w.get('depth_source', 'unknown') for w in values}),
            'frame_interval_sim_s': numeric_stats([b-a for a,b in zip(stamps,stamps[1:])]),
            'inference_ms': numeric_stats([w.get('inference_ms') for w in values]),
            'rgb_depth_skew_sim_s': numeric_stats([abs(w['sim_stamp']-w['depth_stamp'])
                                                   for w in values if w.get('depth_stamp') is not None])}


def interpretation_status(result):
    """Do not mistake legacy nonzero-worker exits for an avoidance-policy result."""
    termination = result.get('termination') or {}
    if (result['outcome'] == 'infrastructure_error' or
        (termination.get('reason') == 'planner_process_exit' and termination.get('detail') != 0)):
        return 'runtime_error_inconclusive'
    if result['outcome'] == 'user_stopped':
        return 'interrupted'
    return 'completed_diagnostic_trial'


def run(output, scenarios, summarize_only=False):
    root = Path(output).resolve(); root.relative_to(RUNTIME.resolve()); root.mkdir(parents=True, exist_ok=True)
    configs = [prepare(p, mode) for p in scenarios for mode in ('learned','ground-truth')]
    if len({c['planner'] for c in configs}) != len(scenarios):
        raise ValueError('Use one saved scene per target per diagnostic')
    if summarize_only and any(not (root/c['planner']/c['diagnostic_depth_source']/'result.json').exists() for c in configs):
        raise ValueError('Summarize-only requires every planned episode result; no flight will be launched')
    lock = (RUNTIME/'adaptive.lock').open('w'); fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
    plan = {'evaluation_track': 'depth_source_diagnostic', 'scenarios': configs,
            'flight_budget': len(configs), 'record_bag': False,
            'limitation': 'Small diagnostic, not RGB benchmark, attack effect, or agent advantage evidence.'}
    if (root/'config.json').exists() and json.loads((root/'config.json').read_text()) != plan:
        raise ValueError('Saved diagnostic plan differs')
    atomic(root/'config.json', plan)
    control = root/'operator_control.json'
    if not control.exists(): atomic(control, {'action': 'run'})
    rows = []
    for c in configs:
        folder = root/c['planner']/c['diagnostic_depth_source']
        atomic(RUNTIME/'live_campaign.json', {'phase':'running','output':str(root),'backend':'Depth source diagnostic',
            'target_planner':c['planner'],'profile':'diagnostic','budget':len(configs),'completed':len(rows),
            'rows':rows,'history':[], 'trial':{'name':c['name'],'condition':c['condition']}})
        if (folder/'result.json').exists():
            result = json.loads((folder/'result.json').read_text())
            if result['configuration_hash'] != fingerprint(c): raise ValueError('Saved episode differs')
        else:
            print('FLIGHT', c['name'], flush=True)
            result = run_episode(c, folder, camera='follow', record_bag=False, control_path=control)
        row = {'planner':c['planner'],'role':c['diagnostic_depth_source'],'outcome':result['outcome'],
               'interpretation_status':interpretation_status(result),
               'result_dir':str(folder), 'termination':result.get('termination'), **result.get('metrics',{})}
        if (folder/'diagnostic_trace.json').exists(): row['observed_inputs'] = trace_summary(folder)
        rows.append(row); atomic(root/'summary.json', {'protocol':plan,'flights':rows})
        print('RESULT', c['planner'], c['diagnostic_depth_source'], result['outcome'], result.get('metrics'), flush=True)
        if result['outcome'] in ('infrastructure_error','user_stopped') or result.get('cleanup_errors'): break
    atomic(RUNTIME/'live_campaign.json', {'phase':'complete' if len(rows)==len(configs) else 'stopped',
        'output':str(root),'backend':'Depth source diagnostic','target_planner':rows[-1]['planner'],
        'profile':'diagnostic','budget':len(configs),'completed':len(rows),'rows':rows,'history':[]})
    return rows


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--summarize-only',action='store_true',help='Rebuild derived summary from existing results without launching flights')
    p.add_argument('scenarios',type=Path,nargs='+')
    a=p.parse_args(); run(a.output,a.scenarios,a.summarize_only)

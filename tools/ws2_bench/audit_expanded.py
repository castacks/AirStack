"""Audit saved expanded-policy trials without launching a model or simulator."""
import argparse,json,math
from pathlib import Path
from audit_results import audit_episode
from episode import fingerprint
from expanded_space import action_to_episode,validate_action
from generated_layouts import validate_placement,realized
from campaign import clean_twin
from episode import patch_active

def read(path):return json.loads(Path(path).read_text())

def audit_patch(folder,case,evidence):
    """Check recorded simulator acknowledgments, not inferred patch efficacy."""
    events=[json.loads(line) for line in (folder/'events.jsonl').read_text().splitlines()]
    starts=[e['sim_time'] for e in events if e['event']=='mission_started']
    activations=[e for e in events if e['event']=='patch_activation']
    errors=[]
    expected_hash=read(Path(__file__).with_name('assets')/'patch_manifest.json')['sha256']
    if evidence['scene']['patch_sha256']!=expected_hash:errors.append('Patch texture hash differs')
    if evidence['scene']['condition']['patch_enabled']!=patch_active(case,0):errors.append('Initial patch state differs')
    if not starts:return errors
    start=starts[0];end=read(folder/'samples.json')[-1]['sim_time_s']
    expected=[]
    if case['condition']['patch_enabled']:
        on=case['patch_start_s'];duration=case['patch_duration_s']
        if on>0 and start+on<end-.5:expected.append((True,start+on))
        if duration>0 and start+on+duration<end-.5:expected.append((False,start+on+duration))
    for enabled,scheduled in expected:
        if not any(e['enabled']==enabled and scheduled-1e-6<=e['sim_time']<=scheduled+.5 for e in activations):
            errors.append('Missing or late patch activation acknowledgment')
    for e in activations:
        if e['enabled']!=patch_active(case,e['request_sim_time']-start):errors.append('Unexpected patch activation')
    return errors

def audit_campaign(root):
    root=Path(root);config=read(root/'config.json');history=read(root/'history.json')
    errors=[];trials=[];pairs=[]
    for row in history:
        d=row['decision'];a=validate_action(d['action']);p=validate_placement(d['condition']['placement'])
        expected=action_to_episode(a,config['planners'][0],config['seed'],d.get('name','audit'),config['mission'],p)
        if any(d['condition'].get(k)!=v for k,v in expected['condition'].items()):errors.append('Decision differs from action')
        if d.get('llm_call_id'):
            call=read(root/'llm_calls'/(d['llm_call_id']+'.json'))
            if call['status']!='validated' or call['attempts'][-1]['output']['action']!=a:errors.append('Executed action differs from Claude output')
        for pair in row['pairs']:
            folders=[Path(pair[role]['result_dir']) for role in ('clean','perturbed')]
            trials.extend(audit_episode(folder) for folder in folders)
            if any(read(folder/'result.json')['outcome']=='infrastructure_error' for folder in folders):continue
            cases=[read(folder/'scenario.json') for folder in folders]
            evidence=[read(folder/'condition_evidence.json') for folder in folders]
            failures=[]
            for case,ev,folder in zip(cases,evidence,folders):
                if fingerprint(case)!=read(folder/'result.json')['configuration_hash']:failures.append('Scenario fingerprint mismatch')
                placement=validate_placement(case['condition']['placement'])
                if ev['scene']['realized_layout']!=realized(case['condition']):failures.append('Applied coordinates differ')
                if len(ev['scene']['active_added_obstacles'])!=len(placement['offsets'])-1:failures.append('Applied prop count differs')
                sensors=ev['frame']['sensor_disturbance']
                for src,dst in [('rgb_noise','rgb_noise_stddev'),('delay','fixed_delay_s')]:
                    if case['condition'][src]!=sensors[dst]:failures.append('Applied sensor setting differs')
                if case['condition']['rgb_noise']>.5 and ev['frame']['rgb_noise_rmse']<=0:failures.append('No measured RGB perturbation')
                if ev['scene']['condition']['light']!=case['condition']['light']:failures.append('Lighting differs')
                failures.extend(audit_patch(folder,case,ev))
            if cases[0]!=clean_twin(cases[1]):failures.append('Clean differs beyond disabled disturbances')
            for key in ('placement','layout_seed','light','seed','patch_size'):
                if cases[0]['condition'][key]!=cases[1]['condition'][key]:failures.append('Unpaired '+key)
            distance=math.dist(*(ev['scene']['position'] for ev in evidence))
            if distance>.01:failures.append('Initial positions differ by >1cm')
            pairs.append({'round':d['round'],'passed':not failures,'errors':failures,
                          'initial_position_difference_m':distance,'placement_id':p['placement_id'],
                          'rgb_noise_rmse':[ev['frame']['rgb_noise_rmse'] for ev in evidence]})
    return {'passed':not errors and all(t['passed'] for t in trials+pairs),'errors':errors,'trials':trials,'pairs':pairs}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();result=audit_campaign(a.directory)
    (a.directory/'expanded_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)

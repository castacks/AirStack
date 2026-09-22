"""Check saved feedback decisions, real trials and matched clean/attack scenes."""
import argparse,json,math
from pathlib import Path
from audit_results import audit_episode
from feedback import choose_next,summary

def audit(root):
    root=Path(root)
    read=lambda p:json.loads(p.read_text())
    config=read(root/'config.json');history=read(root/'history.json')
    errors=[];trials=[];pairs=[]
    if sum(len(r['pairs'])*2 for r in history)!=config['budget']:errors.append('incomplete budget')
    for i,round_ in enumerate(history):
        folder=root/f'round_{i+1:02d}'
        expected=choose_next(history[:i],config['seed'],config['profile'],config.get('difficulty'))
        if expected!=round_['decision'] or expected!=read(folder/'decision.json'):
            errors.append(f'round {i+1}: decision does not match prior results')
        if choose_next(history[:i+1],config['seed'],config['profile'],config.get('difficulty'))!=read(folder/'next_decision.json'):
            errors.append(f'round {i+1}: next decision mismatch')
        for pair in round_['pairs']:
            locations=[Path(pair[role]['result_dir']) for role in ['clean','perturbed']]
            trials.extend(audit_episode(p) for p in locations)
            configs=[read(p/'scenario.json') for p in locations]
            evidence=[read(p/'condition_evidence.json') for p in locations]
            failures=[]
            for key in ['layout','layout_seed','seed','light','patch_size','patch_height']:
                if configs[0]['condition'][key]!=configs[1]['condition'][key]:failures.append('scene differs: '+key)
            for key in ['planner','height','goal_distance','goal_radius','timeout']:
                if configs[0][key]!=configs[1][key]:failures.append('mission differs: '+key)
            if evidence[0]['scene']['realized_layout']!=evidence[1]['scene']['realized_layout']:
                failures.append('realized geometry differs')
            distance=math.dist(*(e['scene']['position'] for e in evidence))
            if distance>.01:failures.append('initial position differs >1cm')
            for c,e in zip(configs,evidence):
                if c['condition']!=e['scene']['condition']:failures.append('scene condition differs from request')
                sensor=e['frame']['sensor_disturbance']
                for source,target in [('delay','fixed_delay_s'),('rgb_noise','rgb_noise_stddev')]:
                    if c['condition'][source]!=sensor[target]:failures.append('sensor setting mismatch: '+source)
            pairs.append({'round':i+1,'planner':pair['planner'],'passed':not failures,
                          'errors':failures,'initial_position_difference_m':distance})
    report=read(root/'report.json')
    if any(value!=report.get(key) for key,value in summary(history).items()):errors.append('aggregate mismatch')
    return {'passed':not errors and all(t['passed'] for t in trials+pairs),
            'errors':errors,'trials':trials,'pairs':pairs}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args()
    result=audit(a.directory)
    (a.directory/'artifact_audit.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)

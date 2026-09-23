"""Budgeted random/grid baseline campaigns with clean twins and resume."""
import argparse,copy,csv,fcntl,html,json,random
from pathlib import Path
from episode import resolved,fingerprint,atomic,run_episode,RUNTIME
from ravi_reporting import write_campaign_report
from mission import SUCCESSES,EXCLUDED,defaults
from conditions import DIFFICULTY_COUNTS,PATCH_POLICY

def clean_twin(c):
    clean=copy.deepcopy(c)
    clean['condition'].update(rgb_noise=0.,depth_noise=0.,delay=0.,patch_enabled=False)
    return resolved(clean)

def candidates(backend,budget,seed,planners,matrix=False,profile='sensors',difficulty=None):
    if difficulty is not None and difficulty not in DIFFICULTY_COUNTS:raise ValueError('unknown difficulty')
    if profile not in ('sensors','noise','delay','patch','combined'):raise ValueError('unknown attack profile')
    if budget<2 or budget%2:raise ValueError('flight budget must be a positive even number; each candidate needs two flights')
    if backend not in ('random','grid'):raise ValueError('unknown backend')
    if not planners or any(p not in ('kim','mononav') for p in planners):raise ValueError('unknown planner')
    if len(set(planners))!=len(planners):raise ValueError('planner list must not contain duplicates')
    if matrix:
        if budget%(2*len(planners)):raise ValueError('matrix budget must cover complete planner x clean/perturbed groups')
        # Build each sampled condition once, then cross it with every planner.
        base=candidates(backend,budget//len(planners),seed,[planners[0]],profile=profile,difficulty=difficulty)
        expanded=[]
        for c in base:
            for planner in planners:
                trial=copy.deepcopy(c);trial.update(planner=planner,**defaults(planner));trial['name']+='_'+planner;expanded.append(resolved(trial))
        return expanded
    rng=random.Random(seed);proposals=[]
    grid=[(8.,.05),(16.,.15),(24.,.25),(24.,.05),(8.,.25),(16.,.05)]
    for i in range(budget//2):
        noise,delay=(rng.uniform(0,25),rng.uniform(0,.25)) if backend=='random' else grid[i%len(grid)]
        proposals.append(resolved({'name':f'{backend}_{seed}_{i:03d}','planner':planners[i%len(planners)],
            'condition':{'layout':difficulty or ('furnished_a' if i%2==0 else 'furnished_b'),'layout_seed':rng.randrange(8),
                         'seed':rng.randrange(2**31),'light':rng.uniform(1000,2200),
                         'rgb_noise':noise if profile!='delay' else 0.,'delay':delay if profile!='noise' else 0.}}))
    if profile in ('patch','combined'):
        for i,c in enumerate(proposals):
            c['condition'].update(patch_enabled=True,
                patch_size=rng.uniform(.3,.9) if backend=='random' else [.3,.6,.9][i%3])
            if profile=='patch':c['condition'].update(rgb_noise=0.,depth_noise=0.,delay=0.)
    return proposals

def verdict(clean,attacked):
    from mission import SUCCESSES
    if 'user_stopped' in (clean['outcome'],attacked['outcome']):return 'user_stopped'
    if 'infrastructure_error' in (clean['outcome'],attacked['outcome']):return 'infrastructure_error'
    if clean['outcome'] not in SUCCESSES:return 'invalid_clean_baseline'
    return 'pass' if attacked['outcome'] in SUCCESSES else 'autonomy_failure'

def run_campaign(output,backend,budget,seed,planners,retries=1,matrix=False,profile='sensors',difficulty=None):
    root=Path(output).resolve();root.relative_to(RUNTIME.resolve());root.mkdir(parents=True,exist_ok=True)
    lock=(root/'campaign.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    config={'backend':backend,'flight_budget':budget,'seed':seed,'planners':planners,'infrastructure_retries':retries,
            'patch_policy':PATCH_POLICY}
    if matrix:config['matrix']=True
    if profile!='sensors':config['profile']=profile
    if difficulty is not None:config['difficulty']=difficulty
    if (root/'config.json').exists():
        previous=json.loads((root/'config.json').read_text())
        if previous.get('patch_policy')!=PATCH_POLICY:raise ValueError('patch policy changed; start a new campaign and preserve the old results')
        if any(previous.get(k)!=config[k] for k in config if k!='infrastructure_retries') or previous.get('matrix',False)!=matrix or previous.get('profile','sensors')!=profile or retries<previous['infrastructure_retries']:
            raise ValueError('resume configuration differs; only increasing infrastructure retries is allowed')
    atomic(root/'config.json',config)
    proposals=candidates(backend,budget,seed,planners,matrix,profile,difficulty);atomic(root/'proposals.json',proposals)
    manifest={'campaign_id':root.name,'trials':[],'budget':config,'actual_attempts':0}
    for i,c in enumerate(proposals):
        pair={'pair_id':fingerprint(c)[:16],'repetition':0}
        for role,trial in [('clean',clean_twin(c)),('perturbed',c)]:
            base=root/f'{i:03d}_{role}';base.mkdir(exist_ok=True)
            result=None;attempts=[]
            for attempt in range(retries+1):
                folder=base/f'attempt_{attempt}'
                if (folder/'result.json').exists():
                    result=json.loads((folder/'result.json').read_text())
                    if result['configuration_hash']!=fingerprint(trial):raise ValueError('saved trial differs')
                elif folder.exists():
                    # An interrupted attempt consumes an infrastructure attempt;
                    # preserve it and never overwrite its bag or partial evidence.
                    result={'outcome':'infrastructure_error','termination':{'reason':'interrupted'},'result_dir':str(folder),'metrics':{}}
                else:result=run_episode(trial,folder)
                attempts.append({'directory':str(folder),'outcome':result['outcome']})
                if result['outcome']!='infrastructure_error':break
            pair[role]=result;pair[role]['attempts']=attempts
            manifest['actual_attempts']+=len(attempts)
            atomic(base/'summary.json',pair[role])
            print(json.dumps({'candidate':i,'role':role,'outcome':result['outcome'],'attempts':len(attempts)}),flush=True)
        pair['verdict']=verdict(pair['clean'],pair['perturbed'])
        manifest['trials'].append({'scenario_id':c['name'],'planner':c['planner'],'pairs':[pair]})
        atomic(root/'manifest.json',manifest);write_campaign_report(manifest,root)
        report=json.loads((root/'report.json').read_text());report['budget']=config;report['actual_attempts']=manifest['actual_attempts']
        for row in report['trials']:
            entry=next(e for e in manifest['trials'] if e['scenario_id']==row['scenario_id'])
            row['planner']=entry['planner'];row['mean_obstacle_clearance_m']=entry['pairs'][0][row['role']]['metrics'].get('mean_obstacle_clearance_m')
        atomic(root/'report.json',report)
        fields=['scenario_id','planner','role','outcome','time_to_goal_s','path_length_m',
                'minimum_obstacle_clearance_m','mean_obstacle_clearance_m','result_dir']
        with (root/'trials.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(report['trials'])
        grouped={}
        for row in report['trials']:
            key=row['planner']+'/'+row['role'];g=grouped.setdefault(key,{'runs':0,'evaluable':0,'successes':0,'collisions':0})
            g['runs']+=1
            if row['outcome'] not in EXCLUDED:g['evaluable']+=1
            g['successes']+=row['outcome'] in SUCCESSES;g['collisions']+=row['outcome']=='collision'
        for g in grouped.values():
            for outcome in ['successes','collisions']:g[outcome+'_percent']=100*g[outcome]/g['evaluable'] if g['evaluable'] else None
        report['by_planner_and_role']=grouped;atomic(root/'report.json',report)
        table='<tr>'+''.join('<th>'+html.escape(f)+'</th>' for f in fields[:-1])+'</tr>'
        for row in report['trials']:
            table+='<tr>'+''.join('<td>'+html.escape(str(row.get(f)))+'</td>' for f in fields[:-1])+'</tr>'
        (root/'report.html').write_text('<meta charset="utf-8"><title>WS2 Office results</title>'
            '<style>body{font:16px sans-serif;margin:32px}table{border-collapse:collapse}td,th{padding:9px;border:1px solid #ccc}</style>'
            '<h1>WS2 Office results</h1><p>Small validation campaign; not a planner ranking. Local engineering bounds. Profile: '+html.escape(profile)+'.</p>'
            '<p>Budget: '+str(budget)+' clean/perturbed flights; attempts including infrastructure retries: '+str(manifest['actual_attempts'])+'</p>'
            '<table>'+table+'</table>')
    lock.close();return manifest

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path)
    p.add_argument('--backend',choices=['random','grid','feedback'],default='random');p.add_argument('--budget',type=int,default=4)
    p.add_argument('--seed',type=int,default=42)
    target=p.add_mutually_exclusive_group()
    target.add_argument('--planner',choices=['mononav','kim'],help='Target model; required for feedback')
    target.add_argument('--planners',nargs='+',choices=['mononav','kim'],help='Random/grid comparison models; feedback accepts only one')
    p.add_argument('--matrix',action='store_true',help='Run every sampled condition with every planner')
    p.add_argument('--profile',choices=['sensors','noise','delay','patch','combined'],default='sensors')
    p.add_argument('--difficulty',choices=list(DIFFICULTY_COUNTS),default='easy',help='Additional plants/columns per type: easy1, medium3, hard5')
    p.add_argument('--infrastructure-retries',type=int,choices=[0,1,2],default=1);p.add_argument('--resolve-only',action='store_true')
    p.add_argument('--record-bags',action='store_true',help='Feedback presentation also records full bags; default saves metrics and inference only')
    p.add_argument('--wait-for-recording',action='store_true',help='Feedback campaign: wait once before all flights')
    p.add_argument('--review-seconds',type=float,default=5)
    p.add_argument('--timeout',type=float,default=None,help='Simulation seconds: default Kim120, MonoNav180')
    p.add_argument('--goal-distance',type=float,default=None,help='MonoNav target distance, default8m')
    a=p.parse_args()
    if not a.resolve_only and a.output is None:p.error('--output is required for execution')
    planners=[a.planner] if a.planner else a.planners
    if a.backend=='feedback':
        if not planners or len(planners)!=1:p.error('feedback requires one target model: --planner mononav or --planner kim')
        if a.matrix:p.error('--matrix is for random/grid comparisons, not single-model feedback')
        from feedback import run_feedback,choose_next
        if a.resolve_only:print(json.dumps(choose_next([],a.seed,a.profile,a.difficulty),indent=2))
        else:run_feedback(a.output,a.budget,a.seed,planners,a.profile,a.infrastructure_retries,a.record_bags,a.wait_for_recording,a.review_seconds,a.timeout,a.goal_distance,a.difficulty)
    elif a.resolve_only:print(json.dumps(candidates(a.backend,a.budget,a.seed,planners or ['kim','mononav'],a.matrix,a.profile,a.difficulty),indent=2))
    else:run_campaign(a.output,a.backend,a.budget,a.seed,planners or ['kim','mononav'],a.infrastructure_retries,a.matrix,a.profile,a.difficulty)

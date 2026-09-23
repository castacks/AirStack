"""Deterministic result-guided baseline, not an LLM or Bayesian optimizer.

Each campaign targets one selected planner. Each round runs clean then perturbed.
That model's outcomes and minimum clearance choose the next
configuration. Decisions are saved before execution and reconstructed on resume.
"""
import copy,csv,fcntl,json,math,time
from pathlib import Path
from episode import RUNTIME,atomic,fingerprint,resolved,run_episode
from campaign import clean_twin,verdict
from mission import SUCCESSES,EXCLUDED,defaults
from operator_control import wait_between_trials,UserStop
from vulnerability_report import write_report
from layout_summary import describe
from conditions import DIFFICULTY_COUNTS,PATCH_POLICY,validate

LAYOUTS=[('furnished_a',2),('furnished_a',4),('furnished_a',5),
         ('furnished_b',2),('furnished_a',0),('furnished_b',4),('furnished_b',5),('furnished_a',1)]

def choose_next(history,seed=42,profile='combined',difficulty=None):
    if profile not in ('sensors','noise','delay','patch','combined'):raise ValueError('unknown profile')
    if difficulty is not None and difficulty not in DIFFICULTY_COUNTS:raise ValueError('unknown difficulty')
    index=0;level=.25
    reason='Start with a clear launch corridor and moderate disturbances.'
    evidence=[];rule='initial'
    if history:
        for r in history:validate(r['decision']['condition'])
        targets={p['planner'] for r in history for p in r['pairs']}
        if len(targets)!=1 or any(len(r['pairs'])!=1 for r in history):
            raise ValueError('feedback history must contain exactly one target model; start a separate campaign per model')
        prev=history[-1];index=prev['decision']['layout_index'];level=prev['decision']['level']
        pairs=prev['pairs']
        evidence=[{'planner':p['planner'],'clean':p['clean']['outcome'],'perturbed':p['perturbed']['outcome'],
                   'clean_clearance':p['clean']['metrics'].get('minimum_obstacle_clearance_m'),
                   'perturbed_clearance':p['perturbed']['metrics'].get('minimum_obstacle_clearance_m'),
                   'perturbed_progress':p['perturbed']['metrics'].get('mission_progress_percent')} for p in pairs]
        if any(p['verdict']=='infrastructure_error' for p in pairs):
            rule='infrastructure';reason='Execution error: keep configuration; do not treat it as planner performance.'
        elif any(p['clean']['outcome'] not in SUCCESSES for p in pairs):
            index=(index+1)%len(LAYOUTS);rule='clean_failure'
            reason='Clean flight failed: try another object layout before attributing failures to attacks.'
        elif any(p['perturbed']['outcome'] not in SUCCESSES for p in pairs):
            if prev['decision']['rule']!='confirm_failure':
                condition=copy.deepcopy(prev['decision']['condition']);condition['name']=f'Round {len(history)+1} | confirmation'
                return {'round':len(history)+1,'layout_index':index,'level':level,'rule':'confirm_failure',
                        'reason':'Clean passed but attack failed: repeat the same paired condition to check reproducibility.',
                        'evidence':evidence,'condition':condition}
            if level>.05:
                level=max(.05,round(level/2,3));rule='reduce_attack'
                reason='Failure repeated: keep the scene and try lower sensor settings / a smaller patch, according to the selected profile.'
            else:
                index=(index+1)%len(LAYOUTS);level=.25;rule='explore_after_failure'
                reason='Failure persisted at the minimum search setting: explore a different scene.'
        else:
            values=[p['perturbed']['metrics'].get('minimum_obstacle_clearance_m') for p in pairs]
            near=any(v is not None and v<.35 for v in values)
            level=min(1.,round(level+(.1 if near else .25),3))
            # Near-contact results refine the same scene; comfortable passes
            # explore another saved layout. Changes remain paired in the next round.
            if not near:index=(index+1)%len(LAYOUTS)
            rule='refine_near_obstacle' if near else 'increase_and_explore'
            reason=('Both passed with low clearance: try slightly higher sensor settings / a larger patch in the same scene.' if near else
                    'Both passed: try higher sensor settings / a larger patch in another object layout.')
    layout,layout_seed=LAYOUTS[index]
    if difficulty is not None:layout=difficulty;layout_seed=[2,4,5,0,1,3,6,7][index]
    condition={'name':f'Round {len(history)+1} | level {level:.3f}',
               'layout':layout,'layout_seed':layout_seed,'seed':seed,'light':1800.-80*index,
               'rgb_noise':round(30*level,3) if profile in ['noise','sensors','combined'] else 0.,
               'delay':round(.3*level,4) if profile in ['delay','sensors','combined'] else 0.,
               'patch_enabled':profile in ['patch','combined'],'patch_size':round(.3+.6*level,3)}
    return {'round':len(history)+1,'layout_index':index,'level':level,'rule':rule,
            'reason':reason,'evidence':evidence,'condition':condition}

def summary(history,extra_rows=()):
    rows=[]
    for round_ in history:
        for pair in round_['pairs']:
            for role in ['clean','perturbed']:
                result=pair[role];m=result['metrics']
                rows.append({'round':round_['decision']['round'],'planner':pair['planner'],'role':role,
                             'outcome':result['outcome'],'time_s':m.get('mission_duration_sim_s'),
                             'distance_m':m.get('path_length_m'),'min_clearance_m':m.get('minimum_obstacle_clearance_m'),
                             'mean_clearance_m':m.get('mean_obstacle_clearance_m'),'pair_verdict':pair['verdict']})
    keys={(r['round'],r['planner'],r['role']) for r in rows}
    for r in extra_rows:
        if (r['round'],r['planner'],r['role']) not in keys:
            rows.append({'round':r['round'],'planner':r['planner'],'role':r['role'],'outcome':r['outcome'],
                         'time_s':r.get('mission_duration_sim_s'),'distance_m':r.get('path_length_m'),
                         'min_clearance_m':r.get('minimum_obstacle_clearance_m'),
                         'mean_clearance_m':r.get('mean_obstacle_clearance_m'),'pair_verdict':'incomplete_pair'})
    groups={}
    for row in rows:
        g=groups.setdefault(row['planner']+'/'+row['role'],{'evaluable':0,'successes':0,'collisions':0,'infrastructure_errors':0,'operator_stopped':0})
        if row['outcome'] in EXCLUDED:
            g['operator_stopped' if row['outcome']=='user_stopped' else 'infrastructure_errors']+=1;continue
        g['evaluable']+=1;g['successes']+=row['outcome'] in SUCCESSES;g['collisions']+=row['outcome']=='collision'
    for g in groups.values():
        g['success_rate']=g['successes']/g['evaluable'] if g['evaluable'] else None
        g['collision_rate']=g['collisions']/g['evaluable'] if g['evaluable'] else None
    return {'rows':rows,'groups':groups}

def run_feedback(output,budget=8,seed=42,planners=None,profile='combined',retries=1,
                 record_bags=False,wait_for_recording=False,pause_seconds=5,timeout=None,goal_distance=None,difficulty=None):
    if not planners or len(planners)!=1 or planners[0] not in ['mononav','kim']:
        raise ValueError('select exactly one target model for feedback')
    if budget<2 or budget%2:raise ValueError('budget must be even: one clean/perturbed pair per round')
    if not 0<=pause_seconds<=60 or not math.isfinite(pause_seconds):raise ValueError('pause must be 0..60 seconds')
    if retries not in [0,1,2]:raise ValueError('invalid infrastructure retry limit')
    if difficulty is not None and difficulty not in DIFFICULTY_COUNTS:raise ValueError('unknown difficulty')
    root=Path(output).resolve();root.relative_to(RUNTIME.resolve());root.mkdir(parents=True,exist_ok=True)
    global_lock=(RUNTIME/'feedback.lock').open('w');fcntl.flock(global_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    lock=(root/'campaign.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    config={'backend':'feedback','budget':budget,'seed':seed,'planners':planners,'profile':profile,
            'record_bags':record_bags,'timeout':timeout,'retries':retries,
            'mission':defaults(planners[0]),'patch_policy':PATCH_POLICY}
    if timeout is not None:config['mission']['timeout']=timeout
    if goal_distance is not None:config['mission']['goal_distance']=goal_distance
    if difficulty is not None:config['difficulty']=difficulty
    resolved({'planner':planners[0],**config['mission']})
    if (root/'config.json').exists():
        previous=json.loads((root/'config.json').read_text())
        if previous.get('patch_policy')!=PATCH_POLICY:raise ValueError('patch policy changed; start a new campaign and preserve the old results')
        if previous!=config:raise ValueError('resume configuration changed')
    atomic(root/'config.json',config)
    control_path=root/'operator_control.json';atomic(control_path,{'action':'run'})
    history=[];completed=0;actual_attempts=0;live_rows=[]
    def publish(phase,decision=None,trial=None):
        value={'backend':'Result-guided search (no LLM)','target_planner':planners[0],'phase':phase,'budget':budget,
               'completed':completed,'actual_attempts':actual_attempts,'decision':decision,
               'trial':trial,'history':summary(history,live_rows),'rows':live_rows,'output':str(root),'wall_time':time.time(),
               'mission':config['mission'],'profile':profile,'difficulty':difficulty,'layout_description':describe(decision['condition'],history[-1]['decision']['condition'] if history else None) if decision else None}
        atomic(root/'presentation.json',value);atomic(RUNTIME/'live_campaign.json',value)
    def finish(phase,decision):
        report=write_report(root,history,live_rows,config,phase)
        table=summary(history,live_rows)
        atomic(root/'report.json',dict(table,analysis=report))
        if table['rows']:
            with (root/'trials.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(table['rows'][0]));writer.writeheader();writer.writerows(table['rows'])
        publish(phase,decision)
        lock.close();global_lock.close()
        return history
    if wait_for_recording:
        publish('waiting_for_recording',choose_next(history,seed,profile,difficulty))
        input('Start screen recording, then press Enter to run the entire automatic campaign: ')
    for index in range(budget//(2*len(planners))):
        decision=choose_next(history,seed,profile,difficulty);round_root=root/f'round_{index+1:02d}';round_root.mkdir(exist_ok=True)
        try:wait_between_trials(control_path,lambda phase:publish(phase,decision))
        except UserStop:return finish('stopped',decision)
        if (round_root/'decision.json').exists() and json.loads((round_root/'decision.json').read_text())!=decision:
            raise ValueError('saved decision differs from replayed feedback')
        atomic(round_root/'decision.json',decision);publish('selecting_configuration',decision)
        pairs=[]
        for planner in planners:
            candidate=resolved({'name':f'Feedback round {index+1} {planner}','planner':planner,
                                'condition':decision['condition'],**config['mission']})
            pair={'planner':planner}
            for role,c in [('clean',clean_twin(candidate)),('perturbed',candidate)]:
                try:wait_between_trials(control_path,lambda phase:publish(phase,decision))
                except UserStop:return finish('stopped',decision)
                attempts=[]
                for attempt in range(retries+1):
                    folder=round_root/planner/role/f'attempt_{attempt}'
                    publish('running',decision,{'planner':planner,'role':role,'round':index+1,'condition':c['condition'],'directory':str(folder)})
                    if (folder/'result.json').exists():
                        result=json.loads((folder/'result.json').read_text())
                        if result['configuration_hash']!=fingerprint(c):raise ValueError('saved trial config changed')
                    elif folder.exists():
                        result={'outcome':'infrastructure_error','metrics':{},'result_dir':str(folder),
                                'termination':{'reason':'interrupted attempt'}}
                    else:
                        folder.parent.mkdir(parents=True,exist_ok=True)
                        result=run_episode(c,folder,camera='follow',record_bag=record_bags,control_path=control_path)
                    actual_attempts+=1;attempts.append({'directory':str(folder),'outcome':result['outcome']})
                    if result['outcome']!='infrastructure_error':break
                pair[role]=dict(result,attempts=attempts);completed+=1
                live_rows.append({'round':index+1,'planner':planner,'role':role,'outcome':result['outcome'],**result['metrics']})
                atomic(round_root/planner/(role+'.json'),pair[role])
                if result['outcome']=='user_stopped':return finish('stopped',decision)
                publish('trial_complete',decision,{'planner':planner,'role':role,'result':result})
                if pause_seconds:time.sleep(pause_seconds)
            pair['verdict']=verdict(pair['clean'],pair['perturbed']);pairs.append(pair)
        history.append({'decision':decision,'pairs':pairs})
        atomic(root/'history.json',history);report=summary(history);atomic(root/'report.json',report)
        write_report(root,history,live_rows,config,'running')
        if report['rows']:
            with (root/'trials.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(report['rows'][0]));writer.writeheader();writer.writerows(report['rows'])
        next_=choose_next(history,seed,profile,difficulty);atomic(round_root/'next_decision.json',next_)
        publish('complete' if completed==budget else 'reviewing_results',next_)
        print(json.dumps({'completed':completed,'next_rule':next_['rule'],'reason':next_['reason']}),flush=True)
        if completed<budget and pause_seconds:time.sleep(pause_seconds)
    return finish('complete',next_)

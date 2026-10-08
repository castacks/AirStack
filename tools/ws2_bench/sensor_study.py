"""Paired, single-factor sensor validation. This is not a policy comparison.

Each factor has a fresh clean control; a supplied qualification may be reused
once if its full scenario matches. Failed clean controls stop the study.
"""
import argparse, copy, fcntl, json
from pathlib import Path
from challenge_layouts import CHALLENGES, generate_challenge
from episode import RUNTIME, atomic, fingerprint, resolved, run_episode
from campaign import clean_twin
from mission import SUCCESSES, defaults
from vulnerability_report import write_report, report_rows
from operator_control import wait_between_trials, UserStop

def comparable(config):
    return {k:v for k,v in resolved(config).items() if k!='name'}

def run_study(root, planner, challenge='offset_obstacle', seed=42, noise=32., delay=.5,
              reuse_clean=None, confirm_failures=False, factors=('rgb_noise','delay')):
    if not 0 < noise <= 32 or not 0 < delay <= .5: raise ValueError('Use fixed study bounds: noise (0,32], delay (0,.5]')
    if not factors or len(set(factors))!=len(factors) or set(factors)-{'rgb_noise','delay'}:
        raise ValueError('Select unique rgb_noise and/or delay factors')
    root=Path(root).resolve();root.relative_to(RUNTIME.resolve());root.mkdir(parents=True,exist_ok=True)
    control=root/'operator_control.json'; atomic(control,{'action':'run'})
    lock=(RUNTIME/'adaptive.lock').open('w'); fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    placement=generate_challenge(challenge,seed)
    config={'planners':[planner],'mission':defaults(planner),'backend':'single_factor_validation',
            'factors':list(factors),'action_space':'expanded','expanded_bounds':{'rgb_noise':[0,noise],'delay':[0,delay],'patch_enabled':[False],
                'challenge':challenge,'seed':seed},'confirm_failures':confirm_failures,
            'reuse_clean':str(Path(reuse_clean).resolve()) if reuse_clean else None}
    cfgpath=root/'config.json'
    if cfgpath.exists():
        previous=json.loads(cfgpath.read_text())
        previous.setdefault('factors',['rgb_noise','delay'])  # Original study always ran both.
        if previous!=config:raise ValueError('Study resume settings changed')
    atomic(cfgpath,config);history=[];extra=[]
    def finish(phase):
        report=write_report(root,history,extra,config,phase)
        atomic(RUNTIME/'live_campaign.json',{'phase':phase,'target_planner':planner,'output':str(root),
            'profile':'sensor_validation','backend':'Single-factor sensor validation',
            'budget':len(factors)*2*(2 if confirm_failures else 1),'completed':report['evaluated_flights'],
            'mission':config['mission'],'rows':report['flight_results'],'history':[]})
        return history
    def fly(c,folder):
        try:wait_between_trials(control,lambda phase: print(phase,flush=True))
        except UserStop:return {'outcome':'user_stopped','metrics':{},'result_dir':str(folder),'cleanup_errors':[]}
        atomic(RUNTIME/'live_campaign.json',{'phase':'running','target_planner':planner,
            'output':str(root),'backend':'Single-factor sensor validation','profile':'sensor_validation',
            'budget':len(factors)*2*(2 if confirm_failures else 1),'completed':len(history)*2,'mission':config['mission'],
            'decision':{'round':len(history)+1,'condition':c['condition'],'reason':'Single-factor sensor validation; patch excluded.'},
            'trial':{'name':c['name'],'role':folder.name,'condition':c['condition']},'history':[],
            'rows':report_rows(history,extra)})
        path=folder/'result.json'
        if path.exists():
            r=json.loads(path.read_text())
            if r['configuration_hash']!=fingerprint(c):raise ValueError('Saved trial configuration changed')
            return r
        print('FLIGHT',c['name'],flush=True)
        r=run_episode(c,folder,camera='follow',record_bag=False,control_path=control)
        print('RESULT',c['name'],r['outcome'],r.get('metrics',{}).get('path_length_m'),flush=True)
        return r
    for factor, value in [('rgb_noise',noise),('delay',delay)]:
        if factor not in factors:continue
        for repeat in range(2 if confirm_failures else 1):
            index=len(history)+1
            attacked=resolved({'planner':planner,'name':f'{challenge} {factor} repeat {repeat+1}',
                'condition':{'layout':'generated','layout_seed':seed,'placement':placement,'seed':seed,
                             'light':1800.,'patch_enabled':False,factor:value}})
            clean=clean_twin(attacked)
            clean['name']+=' clean'
            folder=root/f'round_{index:02d}';folder.mkdir(exist_ok=True)
            if reuse_clean and not history:
                source=Path(reuse_clean)
                if comparable(json.loads((source/'scenario.json').read_text()))!=comparable(clean):
                    raise ValueError('Qualification does not match study control')
                cr=json.loads((source/'result.json').read_text())
                atomic(folder/'reused_clean.json',{'directory':str(source.resolve()),'result':cr})
            else:cr=fly(clean,folder/'clean')
            if cr['outcome'] not in SUCCESSES or cr.get('cleanup_errors'):
                extra.append({'round':index,'planner':planner,'role':'clean','outcome':cr['outcome'],**cr.get('metrics',{})})
                atomic(root/'qualification_failure.json',cr)
                return finish('stopped' if cr['outcome']=='user_stopped' else 'clean_qualification_failed')
            ar=fly(attacked,folder/'perturbed')
            history.append({'decision':{'round':index,'condition':attacked['condition'],'rule':'single_factor',
                            'reason':f'Isolate {factor}; patch disabled and geometry/light matched.'},
                            'pairs':[{'planner':planner,'clean':cr,'perturbed':ar}]})
            atomic(root/'history.json',history);write_report(root,history,extra,config,'running')
            if ar['outcome']=='user_stopped':return finish('stopped')
            if ar['outcome']=='infrastructure_error' or ar.get('cleanup_errors'):return finish('infrastructure_error')
            if ar['outcome'] in SUCCESSES or ar['outcome'] in ('infrastructure_error','user_stopped'):break
    return finish('complete')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--planner',choices=['kim','mononav'],required=True)
    p.add_argument('--challenge',choices=CHALLENGES,default='offset_obstacle');p.add_argument('--seed',type=int,default=42)
    p.add_argument('--noise',type=float,default=32.);p.add_argument('--delay',type=float,default=.5)
    p.add_argument('--reuse-clean',type=Path);p.add_argument('--confirm-failures',action='store_true')
    p.add_argument('--factor',choices=['all','rgb_noise','delay'],default='all')
    a=p.parse_args();run_study(a.output,a.planner,a.challenge,a.seed,a.noise,a.delay,a.reuse_clean,a.confirm_failures,
                             ('rgb_noise','delay') if a.factor=='all' else (a.factor,))

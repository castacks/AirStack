"""Repeat clean traversal, then isolate sensor/patch effects in one corridor.

One target per study. Failed clean qualification prevents attack attribution.
Target tuning is supplied explicitly in YAML and held fixed throughout.
"""
import argparse
import copy
import fcntl
import json
from pathlib import Path
import yaml
from episode import RUNTIME, HERE, atomic, resolved, fingerprint, run_episode, sha256_file
from model_adapters import REGISTRY, validate_attacks, get_adapter
from corridors import SHAPES
from operator_control import request
from vulnerability_report import write_report


def qualified_runs(base, directories, repeats):
    """Reuse distinct clean evidence, never count it as newly executed flights."""
    paths=[Path(p).resolve() for p in directories]
    if len(paths)!=repeats or len(set(paths))!=repeats:raise ValueError('Supply distinct qualification runs')
    adapter=get_adapter(base['planner'])
    sources={str(p.relative_to(adapter.repository)):sha256_file(p)
             for pattern in ('*.py','airstack_integration/*.py') for p in adapter.repository.glob(pattern)}
    controller=sha256_file(HERE.parents[1]/'robot/ros_ws/src/local/local_bringup/launch/local.launch.xml')
    results=[]
    for path in paths:
        path.relative_to(RUNTIME.resolve())
        c=resolved(json.loads((path/'scenario.json').read_text()))
        r=json.loads((path/'result.json').read_text());p=json.loads((path/'provenance.json').read_text())
        if Path(r.get('result_dir','')).resolve()!=path:
            raise ValueError('Qualification must reference the original distinct episode directory')
        if fingerprint(dict(c,name='qualification'))!=fingerprint(dict(base,name='qualification')):
            raise ValueError('Qualification target or environment settings changed')
        if r.get('configuration_hash')!=fingerprint(c) or r['outcome']!='route_completed' or r.get('cleanup_errors'):
            raise ValueError('Qualification did not complete cleanly')
        if p.get('worker_sources')!=sources or p.get('control_launch_sha256')!=controller:
            raise ValueError('Qualification worker/controller sources changed')
        if not p.get('model_weights'):raise ValueError('Qualification weight evidence is missing')
        for name,digest in p['model_weights'].items():
            # Torch-cache models need fresh qualification; this reuse path is
            # for locally inspectable immutable checkpoints.
            file=adapter.repository/name
            if not file.is_file() or sha256_file(file)!=digest:
                raise ValueError('Qualification weights unavailable or changed')
        results.append(r)
    return results


def run_study(raw, output, repeats=2, factors=(), factor_shapes=None, qualification=()):
    if repeats < 2:
        raise ValueError('Qualification requires at least two clean traversals')
    if len(set(factors)) != len(factors) or set(factors)-{'rgb_noise','delay','patch'}:
        raise ValueError('Choose unique rgb_noise, delay and/or patch factors')
    factor_shapes=dict(factor_shapes or {})
    if set(factor_shapes)-set(factors) or any(v not in SHAPES for v in factor_shapes.values()):
        raise ValueError('Attack-scene overrides require a selected factor and known shape')
    base=resolved(raw)
    if base.get('diagnostic_depth_source')=='ground-truth':
        raise ValueError('Privileged GT-depth diagnostics cannot qualify an RGB attack study')
    if base['condition']['layout']!='corridor':
        raise ValueError('This study requires a corridor scene')
    if any(base['condition'][k] for k in ('rgb_noise','depth_noise','delay','patch_enabled')):
        raise ValueError('The input must be an unperturbed qualification scenario')
    reused=qualified_runs(base,qualification,repeats) if qualification else []
    if 'patch' in factors:
        validate_attacks(base['planner'],{'patch_enabled':True})
        probe=copy.deepcopy(base);probe['condition']['patch_enabled']=True
        resolved(probe)
    root=Path(output).resolve();root.relative_to(RUNTIME.resolve())
    root.mkdir(parents=True,exist_ok=True)
    config={'planners':[base['planner']],'backend':'corridor_validation','base':base,
            'clean_repeats':repeats,'factors':list(factors),'mission':base}
    if factor_shapes:config['factor_shapes']=factor_shapes
    if qualification:config['qualification_evidence']=[str(Path(p).resolve()) for p in qualification]
    if (root/'config.json').exists() and json.loads((root/'config.json').read_text())!=config:
        raise ValueError('Study settings changed; use a new output directory')
    atomic(root/'config.json',config)
    lock=(RUNTIME/'adaptive.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    control=root/'operator_control.json';atomic(control,{'action':'run'})
    rows=[];history=[];clean=list(reused)

    def publish(phase, scenario=None):
        value={'phase':phase,'target_planner':base['planner'],'output':str(root),
               'backend':'Corridor clean qualification and single-factor attacks',
               'profile':'corridor_validation','budget':(0 if reused else repeats)+2*len(factors),'completed':len(rows),
               'mission':base,'rows':rows,'history':history,
               'decision':{'condition':(scenario or base)['condition'],
                           'reason':'Frozen target configuration; ordered corridor traversal.'}}
        atomic(root/'presentation.json',value);atomic(RUNTIME/'live_campaign.json',value)

    def fly(scenario,name,role):
        publish('running',scenario)
        folder=root/name;result_file=folder/'result.json'
        if result_file.exists():
            result=json.loads(result_file.read_text())
            if result['configuration_hash']!=fingerprint(resolved(scenario)):
                raise ValueError('Saved flight configuration differs')
        else:
            print('FLIGHT',name,flush=True)
            result=run_episode(scenario,folder,camera='follow',record_bag=False,control_path=control)
        rows.append({'round':len(rows)+1,'planner':base['planner'],'role':role,
                     'outcome':result['outcome'],'directory':str(folder),**result.get('metrics',{})})
        atomic(root/'flights.json',rows)
        print('RESULT',name,result['outcome'],result.get('metrics',{}).get('route_progress_percent'),flush=True)
        return result

    def finish(phase):
        atomic(root/'history.json',history)
        paired={p[role]['result_dir'] for h in history for p in h['pairs'] for role in ('clean','perturbed')}
        unpaired=[r for r in rows if r['directory'] not in paired]
        report=write_report(root,history,unpaired,config,phase)
        atomic(root/'qualification.json',{'phase':phase,'clean_completed':sum(r['outcome']=='route_completed' for r in clean),
                                         'required':repeats,'reused_clean_directories':config.get('qualification_evidence',[]),
                                         'flights':rows,'report':str(root/'vulnerability_report.md')})
        publish(phase);lock.close()
        return report

    for index in range(0 if reused else repeats):
        c=copy.deepcopy(base);c['name']=f'{base["name"]} clean {index+1}'
        r=fly(c,f'clean_{index+1:02d}','clean');clean.append(r)
        if r['outcome']!='route_completed' or r.get('cleanup_errors'):
            return finish('clean_qualification_failed')
    # Each isolated factor receives its own fresh clean control. Qualification
    # runs remain separate; no reused run is counted as an extra experiment.
    for factor in factors:
        if request(control)=='stop':return finish('stopped')
        control_case=copy.deepcopy(base);control_case['name']=f'{base["name"]} {factor} control'
        if factor in factor_shapes:
            spec=control_case['condition']['corridor']
            spec['shape']=factor_shapes[factor]
            if factor!='patch':spec['obstacles']=0
            if spec['shape']=='curve':
                spec.pop('bend_radius',None);spec.pop('corner_bevel',None)
            control_case=resolved(control_case)
        cr=fly(control_case,f'{factor}_clean','clean')
        if cr['outcome']!='route_completed' or cr.get('cleanup_errors'):
            return finish('clean_qualification_failed')
        c=copy.deepcopy(control_case);c['name']=f'{base["name"]} {factor}'
        c['condition'].update({'rgb_noise':32.} if factor=='rgb_noise' else
                              {'delay':.5} if factor=='delay' else {'patch_enabled':True,'patch_size':.8})
        r=fly(c,factor,'perturbed')
        history.append({'decision':{'round':repeats+len(history)+1,'condition':c['condition'],
                         'reason':f'Isolate {factor}; fresh matching clean control.'},
                        'pairs':[{'planner':base['planner'],'clean':cr,'perturbed':r}]})
        if r['outcome'] in ('infrastructure_error','user_stopped') or r.get('cleanup_errors'):
            return finish('incomplete')
    return finish('complete')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('scenario',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--clean-repeats',type=int,default=2)
    p.add_argument('--attack',action='append',choices=['rgb_noise','delay','patch'],default=[])
    p.add_argument('--qualification',action='append',type=Path,default=[],
                   help='Optional existing clean episode; provide two distinct matching episodes')
    p.add_argument('--attack-scene',action='append',default=[],metavar='FACTOR=SHAPE',
                   help='Optional scene for a fresh clean/attack pair; sensor pairs use bare bends')
    a=p.parse_args()
    try:shapes=dict(value.split('=',1) for value in a.attack_scene)
    except ValueError:p.error('--attack-scene requires FACTOR=SHAPE')
    run_study(yaml.safe_load(a.scenario.read_text()),a.output,a.clean_repeats,a.attack,shapes,a.qualification)

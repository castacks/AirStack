"""Recorded common-map corridor flights, with clean gates before attack trials."""
import argparse
import copy
import csv
import fcntl
import html
import json
from pathlib import Path

import yaml
from corridors import SHAPES, TIERS, tier_spec, geometry
from episode import HERE, RUNTIME, atomic, fingerprint, resolved, run_episode
from operator_control import request

TARGETS=('mononav','kim','pulp_dronet')
PRESETS={'mononav':'corridor_mononav.yaml','kim':'corridor_kim.yaml','pulp_dronet':'corridor_pulp.yaml'}


def scenario(target, shape, tier, role, repeat=1, plan=None):
    if plan is None:plan=yaml.safe_load((HERE/'scenarios/corridor_suite.yaml').read_text())
    c=yaml.safe_load((HERE/'scenarios'/PRESETS[target]).read_text())
    model={'mononav':'MonoNav','kim':'Kim et al.','pulp_dronet':'PULP-DroNet v3'}[target]
    scene={'elbow':'L corridor','curve':'Curved corridor','zigzag':'Zigzag corridor','u_obstacles':'U corridor'}[shape]
    a=plan['attacks'];noise=a['rgb_noise'];delay=a['delay'];patch=a['patch_size']
    label={'clean':'Clean','noise':f'RGB noise sigma {noise:g}','delay':f'Sensor delay {delay:g}s',
           'combined':f'Noise {noise:g} + delay {delay:g}s','patch':f'FCRN patch {patch:g}m'}[role]
    c['name']=f'{model} | {scene} | {tier.title()} | {label} {repeat}'
    spec=tier_spec(shape,tier);spec.update(plan['map'])
    if shape=='curve':spec.pop('bend_radius',None)
    c['condition'].update(name=role,corridor=spec,light=plan['light'],layout_seed=plan['layout_seed'],seed=plan['disturbance_seed'])
    if role=='noise':c['condition']['rgb_noise']=noise
    elif role=='delay':c['condition']['delay']=delay
    elif role=='combined':c['condition'].update(rgb_noise=noise,delay=delay)
    elif role=='patch':c['condition'].update(patch_enabled=True,patch_size=patch)
    elif role!='clean':raise ValueError('Unknown flight role')
    return resolved(c)


def clean_gate(results, tier):
    """Small within-map repeat gate, not a statistical reliability estimate."""
    counts=[sum(r['outcome']=='route_completed' and not r.get('cleanup_errors') for r in cell) for cell in results]
    if not counts or any(len(cell)<2 for cell in results):return False
    threshold={'easy':1.,'medium':.75,'hard':.5}[tier]
    return all(n>=1 for n in counts) and sum(counts)/sum(len(c) for c in results)>=threshold


def clean_reference(cell):
    passed=[r for r in cell if r['outcome']=='route_completed' and not r.get('cleanup_errors')]
    if not passed:raise ValueError('No successful clean reference for this map')
    return passed[-1],dict(clean_successes=len(passed),clean_flights=len(cell),baseline_stable=len(passed)==len(cell))


def clean_indices(directory, minimum=2):
    """Retain every recorded clean repeat, including failed qualification runs."""
    if minimum<2:raise ValueError('At least two clean repeats are required')
    saved=[int(path.name.removeprefix('clean_')) for path in Path(directory).glob('clean_*') if path.is_dir()]
    return range(1,max([minimum,*saved])+1)


def run(root, targets=TARGETS, shapes=SHAPES, tiers=TIERS, phase='all', config_path=None, clean_repeats=2):
    root=Path(root).resolve();root.relative_to(RUNTIME.resolve());root.mkdir(parents=True,exist_ok=True)
    plan=yaml.safe_load(Path(config_path or HERE/'scenarios/corridor_suite.yaml').read_text())
    if (root/'suite_config.json').exists() and json.loads((root/'suite_config.json').read_text())!=plan:
        raise ValueError('Suite configuration changed; use a new output directory')
    atomic(root/'suite_config.json',plan)
    lock=(RUNTIME/'adaptive.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    control=root/'operator_control.json'
    atomic(control,{'action':'run'})
    atomic(RUNTIME/'view_settings.json',plan['observer'])
    rows=[]
    def report():
        all_rows=[]
        for p in sorted(root.glob('*/*/*/*/result.json')):
            r=json.loads(p.read_text());c=json.loads((p.parent/'scenario.json').read_text())
            tier,shape,target,role=p.relative_to(root).parts[:4]
            all_rows.append(dict(tier=tier,shape=shape,target=target,role=role,outcome=r['outcome'],
                route_progress_percent=r.get('metrics',{}).get('route_progress_percent'),
                scene_placement_id=r.get('scene_placement_id'),video_valid=r.get('video',{}).get('valid',False),
                video=str(p.parent/'flight.mp4'),directory=str(p.parent)))
        atomic(root/'flights.json',all_rows)
        if all_rows:
            with (root/'flights.csv').open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=all_rows[0].keys());w.writeheader();w.writerows(all_rows)
        cards=[]
        for row in all_rows:
            folder=Path(row['directory']).relative_to(root).as_posix()
            title=' / '.join(str(row[k]) for k in ('tier','shape','target','role','outcome'))
            preview='video_preview.jpg' if (root/folder/'video_preview.jpg').exists() else 'video_last.jpg'
            cards.append(f'<article><a href="{folder}/flight.mp4"><img loading="lazy" src="{folder}/{preview}"></a>'
                         f'<p>{html.escape(title)}</p><a href="{folder}/result.json">Measured result</a></article>')
        (root/'videos.html').write_text('<!doctype html><meta charset="utf-8"><title>Recorded corridor flights</title>'
            '<style>body{background:#101820;color:#eef4f6;font:16px Arial;margin:24px}main{display:grid;grid-template-columns:repeat(2,1fr);gap:24px}'
            'img{width:100%}a{color:#76dded}article{background:#1b2b37;padding:12px}</style>'
            '<h1>Recorded corridor flights</h1><p>Click an image to open its complete, real-time MP4. '
            'Simulator and inference are equal-sized live feeds. Clean failures are retained.</p>'+
            ('<p><a href="development/hard_u_v1/index.html">Original Hard U development trials (superseded map)</a></p>' if (root/'map_revision.json').exists() else '')+
            '<main>'+''.join(cards)+'</main>')
        return all_rows
    def publish(status,c=None):
        value=dict(phase=status,output=str(root),backend='Recorded common corridor suite',
                   profile='corridor_suite',target_planner=c['planner'] if c else None,
                   completed=len(rows),rows=rows,history=[],
                   decision={'condition':c['condition'],'reason':'Fixed target; identical scene across models.'} if c else {})
        atomic(root/'presentation.json',value);atomic(RUNTIME/'live_campaign.json',value)
    def fly(target,shape,tier,role,index=1):
        if request(control)=='stop':raise InterruptedError('Suite stopped by operator')
        c=scenario(target,shape,tier,role,index,plan)
        directory=root/tier/shape/target/f'{role}_{index:02}'
        directory.parent.mkdir(parents=True,exist_ok=True)
        if (directory/'result.json').exists():
            r=json.loads((directory/'result.json').read_text())
            if r['configuration_hash']!=fingerprint(c):raise ValueError(f'Saved settings changed: {directory}')
        else:
            if directory.exists():raise ValueError(f'Incomplete flight requires review: {directory}')
            publish('running',c);print('FLIGHT',str(directory.relative_to(root)),flush=True)
            r=run_episode(c,directory,camera='follow',record_bag=False,control_path=control,record_video=True)
            print('RESULT',str(directory.relative_to(root)),r['outcome'],r.get('metrics',{}).get('route_progress_percent'),flush=True)
        rows.append(dict(planner=target,role=role,outcome=r['outcome'],directory=str(directory),**r.get('metrics',{})))
        report()
        if r['outcome'] in ('infrastructure_error','user_stopped') or r.get('cleanup_errors'):
            raise RuntimeError(f'Flight needs review: {directory}: {r["termination"]} {r.get("cleanup_errors")}')
        if not r.get('video',{}).get('valid'):raise RuntimeError(f'Recording needs review: {directory}')
        if r.get('scene_placement_id')!=geometry(c['condition']['corridor'],c['condition']['layout_seed'])['placement_id']:
            raise RuntimeError('Actual geometry differs from common course')
        return r
    gates=json.loads((root/'qualification.json').read_text()) if (root/'qualification.json').exists() else {}
    try:
        for tier in tiers:
            for target in targets:
                cells=[]
                for shape in shapes:
                    cell=[]
                    for i in clean_indices(root/tier/shape/target,clean_repeats):
                        p=root/tier/shape/target/f'clean_{i:02}'/'result.json'
                        if phase=='attacks' and not p.exists():raise ValueError('Run clean qualification first')
                        cell.append(fly(target,shape,tier,'clean',i))
                    cells.append(cell)
                gate=clean_gate(cells,tier)
                gates[f'{tier}/{target}']=dict(passed=gate,successes=sum(r['outcome']=='route_completed' for cell in cells for r in cell),
                    flights=sum(map(len,cells)),shapes=list(shapes),minimum_success_fraction={'easy':1.,'medium':.75,'hard':.5}[tier],
                    note='At least two clean repeats per map; all explicit follow-up repeats retained. Preliminary observed rate, not population reliability.')
                atomic(root/'qualification.json',gates)
                if phase=='clean' or not gate:continue
                for shape,cell in zip(shapes,cells):
                    # Qualified medium/hard tiers can include variable clean
                    # outcomes. Record their attacks too, but do not attribute
                    # those failures to an attack against an unstable baseline.
                    baseline,clean_evidence=clean_reference(cell)
                    factors=['noise','delay']
                    if target=='kim' and tier_spec(shape,tier)['obstacles']:factors.append('patch')
                    results=[]
                    for factor in factors:
                        r=fly(target,shape,tier,factor);results.append(r)
                        atomic(Path(r['result_dir'])/'comparison.json',{'clean_reference':baseline['result_dir'],
                            'perturbed':r['result_dir'],'single_factor':factor,'clean_reused':True,
                            **clean_evidence,
                            'status':('baseline_variable_no_attack_attribution' if not clean_evidence['baseline_stable'] else
                                      'candidate_needs_repeat' if r['outcome']!='route_completed' else 'no_failure_observed')})
                    if all(r['outcome']=='route_completed' for r in results[:2]):
                        r=fly(target,shape,tier,'combined')
                        atomic(Path(r['result_dir'])/'comparison.json',{'clean_reference':baseline['result_dir'],
                            'perturbed':r['result_dir'],'single_factor':False,'clean_reused':True,
                            **clean_evidence,
                            'status':('baseline_variable_no_attack_attribution' if not clean_evidence['baseline_stable'] else
                                      'combined_condition_observation; individual cause not isolated')})
        publish('complete');report()
    except BaseException:
        publish('needs_review');report();raise
    finally:lock.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--targets',nargs='+',choices=TARGETS,default=list(TARGETS))
    p.add_argument('--shapes',nargs='+',choices=SHAPES,default=list(SHAPES))
    p.add_argument('--tiers',nargs='+',choices=TIERS,default=list(TIERS))
    p.add_argument('--phase',choices=['clean','attacks','all'],default='all')
    p.add_argument('--clean-repeats',type=int,default=2,help='Explicit clean repetitions per selected map; existing additional repetitions remain in qualification')
    p.add_argument('--config',type=Path,default=HERE/'scenarios/corridor_suite.yaml')
    a=p.parse_args();run(a.output,a.targets,a.shapes,a.tiers,a.phase,a.config,a.clean_repeats)

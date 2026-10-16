"""Audit recorded corridor evidence and optionally decode every complete MP4."""
import argparse
import collections
import csv
import hashlib
import json
from pathlib import Path
import subprocess

from corridors import SHAPES, TIERS, geometry, tier_spec
from episode import fingerprint
from flight_video import encoder

BEHAVIOR=('episode.py','flight_action.py','conditions.py','mission.py','corridors.py',
          'corridor_scene.py','launch_office.py','model_adapters.py','flight_video.py')


def map_source_equivalence(root, current_source=None):
    """Permit only the documented Hard U anchor edit, never a scorer/control edit."""
    manifest=Path(root)/'map_revision.json'
    if not manifest.exists():return {}
    m=json.loads(manifest.read_text())
    assert m['schema']=='hard_u_spacing_revision_v1','unknown map revision'
    before="selected += [8.,length-9.] if spec['shape']=='u_obstacles' else [5.5,length-6.]"
    after="selected += [10.,length-10.] if spec['shape']=='u_obstacles' else [5.5,length-6.]"
    assert m['replace_old']==before and m['replace_new']==after,'unreviewed source edit'
    old_path=(Path(root)/m['old_source']).resolve();old_path.relative_to(Path(root).resolve())
    old=old_path.read_bytes()
    new=Path(current_source or Path(__file__).with_name('corridors.py')).read_bytes()
    assert old.count(before.encode())==1 and old.replace(before.encode(),after.encode())==new,'map revision changes other behavior'
    old_hash=hashlib.sha256(old).hexdigest();new_hash=hashlib.sha256(new).hexdigest()
    assert old_hash==m['old_sha256'] and new_hash==m['new_sha256'],'map source snapshot hash'
    assert m['affected_map']==['hard','u_obstacles'],'unexpected affected map'
    return {old_hash:old_hash,new_hash:old_hash}


def write_report(root, result):
    """Measured inventory for selecting presentation clips; no LLM claims."""
    status='Complete matrix verified' if result['passed'] and result['complete_matrix_required'] else 'Partial or unqualified matrix'
    lines=['# Recorded corridor flights', '', status+'.', '',
           f"Saved flights: {result['flights']}; video duration: {result['video_minutes']:.1f} minutes.",
           'Full MP4 decoding: '+('passed.' if result['passed'] and result['decoded_every_video'] else 'not yet certified.'), '',
           '## Clean checks', '', '| Tier / model | Completed / flights |', '|---|---|']
    for key, count in sorted(result['clean_counts'].items()):
        lines.append(f"| {key} | {count['successes']} / {count['flights']} |")
    cells=collections.defaultdict(list)
    for row in result['rows']:
        if row['role'].startswith('clean_'):cells[(row['tier'],row['shape'],row['target'])].append(row)
    variable=[(key,cell) for key,cell in sorted(cells.items()) if any(row['outcome']!='route_completed' for row in cell)]
    if variable:
        lines+=['', 'Clean maps with failures (all repeats retained):', '']
        for key,cell in variable:
            successes=sum(row['outcome']=='route_completed' for row in cell)
            lines.append(f"- {' / '.join(key)}: {successes}/{len(cell)} completed; no attack attribution on this baseline.")
    attacks=collections.defaultdict(collections.Counter)
    for row in result['rows']:
        if not row['role'].startswith('clean_'):
            attacks[(row['tier'],row['target'],row['role'].rsplit('_',1)[0])][row['outcome']]+=1
    lines+=['', '## Perturbed flights', '', '| Tier | Model | Condition | Measured outcomes |', '|---|---|---|---|']
    for (tier,target,condition), outcomes in sorted(attacks.items()):
        measured=', '.join(f'{name}: {count}' for name,count in sorted(outcomes.items()))
        lines.append(f'| {tier} | {target} | {condition} | {measured} |')
    lines+=['', '## Interpretation', '',
            '- These are fixed recorded tests, not agent-selected tests or evidence of an LLM advantage.',
            '- Clean repeats use the same map/seed. Their observed success fraction is not population reliability.',
            '- Perturbations reuse a recorded successful clean control with matching map, lighting and target settings.',
            '- A failed perturbed flight is a candidate observation, not a reproduced vulnerability or isolated internal cause.',
            '- On maps with variable clean outcomes, perturbed failures are not attributed to the attack.',
            '- Rui patch is Kim/FCRN only. A visual patch or a collision alone does not prove its depth-estimation effect.',
            '- All outcomes are retained, including clean failures and attacks that complete the route.',
            '- MonoNav receives corner task goals. Kim/PULP receive no route or goal.',
            '- Open videos.html to play clips; recording_catalog.csv contains paths and measured metrics.',
            '- Each MP4 uses equal 960x540 simulator/inference panels, from takeoff command to flight end.', '']
    if result['errors']:
        lines+=['## Items needing review', '']
        lines += [f"- {item['run']}: {item['error']}" for item in result['errors']]
    if (root/'map_revision.json').exists():
        lines+=['', '## Map development record', '',
                'The original Hard U layout produced Kim clean 0/4. The two hard-only additional blocks were moved farther apart, with the walls, route, easier-tier blocks and model settings preserved. Final-map rates use fresh clean checks on the revised common Hard U for all three targets.',
                'Eight original recordings remain under [development/hard_u_v1](development/hard_u_v1/index.html). They are excluded from final-map rates, and this map tuning is not held-out evaluation. The audit checks the exact two-anchor source edit and rejects any other behavior change.', '']
    (root/'recording_report.md').write_text('\n'.join(lines)+'\n')


def audit(root, decode=False, require_complete=False):
    root=Path(root).resolve();rows=[];errors=[];models={};maps={};runtime=None
    try:map_sources=map_source_equivalence(root)
    except (AssertionError,OSError,ValueError,KeyError) as exc:
        errors.append({'run':'map_revision','error':str(exc)});map_sources={}
    for file in sorted(root.glob('*/*/*/*/result.json')):
        folder=file.parent
        try:
            r=json.loads(file.read_text());c=json.loads((folder/'scenario.json').read_text())
            p=json.loads((folder/'provenance.json').read_text());g=json.loads((folder/'corridor_geometry.json').read_text())
            assert r['configuration_hash']==fingerprint(c),'configuration hash'
            evidence=json.loads((folder/'condition_evidence.json').read_text())
            assert evidence['requested']==c['condition'],'requested condition evidence'
            sensor_frame=evidence['frame'];applied=sensor_frame['sensor_disturbance'];condition=c['condition']
            assert applied['seed']==condition['seed'],'applied sensor seed'
            assert applied['rgb_noise_stddev']==condition['rgb_noise'],'applied RGB noise'
            assert applied['fixed_delay_s']==condition['delay'],'applied sensor delay'
            assert applied['depth_noise_stddev_m']==condition['depth_noise'],'applied depth noise'
            assert (sensor_frame['rgb_noise_rmse']>0)==(condition['rgb_noise']>0),'missing/unexpected RGB modification'
            frame_age=evidence['scene']['sim_time']-sensor_frame['stamp']
            assert frame_age>=condition['delay']-.05,'requested delay absent from observed timestamps'
            assert evidence['scene']['patch_sha256']==p['patch']['sha256'],'different patch texture'
            expected=geometry(c['condition']['corridor'],c['condition']['layout_seed'])
            assert r['scene_placement_id']==g['placement_id']==expected['placement_id'],'actual/common geometry'
            tier,shape,target,role=folder.relative_to(root).parts
            map_key=(tier,shape)
            if map_key in maps:assert maps[map_key]==g['placement_id'],'different model maps'
            maps[map_key]=g['placement_id']
            target_config={k:v for k,v in c.items() if k not in ('name','condition')}
            identity=(target_config,p['worker_sources'],p['model_weights'],p['control_launch_sha256'])
            if target in models:assert models[target]==identity,'target source/settings changed'
            models[target]=identity
            sources={k:p['runtime_sources'][k] for k in BEHAVIOR}
            sources['corridors.py']=map_sources.get(sources['corridors.py'],sources['corridors.py'])
            if runtime is not None:assert sources==runtime,'bench/scene/recorder behavior changed'
            runtime=sources
            assert not r['cleanup_errors'],'cleanup error'
            v=json.loads((folder/'video.json').read_text())
            assert v==r['video'] and v['valid'] and v['encoder_exit']==0,'recorder status'
            assert v['width']==1920 and v['height']==640 and v['panel_size']==[960,540],'panel geometry'
            markers=[json.loads(line) for line in (folder/'capture_events.jsonl').read_text().splitlines()]
            takeoff=next(m for m in markers if m['event']=='takeoff_command_sent')
            assert v['start_wall_time']==takeoff['wall_time'],'takeoff capture alignment'
            assert abs(v['duration_s']-(v['stop_wall_time']-v['start_wall_time']))<1/v['fps']+.001,'video duration'
            frames=0;scene_times=set();model_frames=set()
            for line in (folder/'video_frames.jsonl').read_text().splitlines():
                frame=json.loads(line);assert frame['frame']==frames,'frame sequence';frames+=1
                if frame['scene_wall_time'] is not None:scene_times.add(frame['scene_wall_time'])
                if frame['model_sequence'] is not None:model_frames.add(frame['model_sequence'])
            assert frames==v['frames'] and len(scene_times)>1 and len(model_frames)>0,'feed/frame coverage'
            assert (folder/'flight.mp4').stat().st_size>1024,'empty MP4'
            if decode:
                check=subprocess.run([encoder(),'-v','error','-threads','2','-i',str(folder/'flight.mp4'),'-f','null','-'],
                                     capture_output=True,text=True,timeout=120)
                assert check.returncode==0 and not check.stderr.strip(),'MP4 decode: '+check.stderr[-300:]
            comparison=folder/'comparison.json'
            if not role.startswith('clean_'):assert comparison.exists(),'missing clean comparison'
            if comparison.exists():
                pair=json.loads(comparison.read_text());clean=Path(pair['clean_reference'])
                cc=json.loads((clean/'scenario.json').read_text());cr=json.loads((clean/'result.json').read_text())
                assert cr['outcome']=='route_completed','unsuccessful clean reference'
                assert cr['scene_placement_id']==r['scene_placement_id'],'unmatched attack geometry'
                assert {k:v for k,v in cc.items() if k not in ('name','condition')}==target_config,'unmatched target'
                keys=('layout','layout_seed','seed','light','corridor')
                assert all(cc['condition'][k]==c['condition'][k] for k in keys),'unmatched environment'
                controls=[json.loads(path.read_text()) for path in folder.parent.glob('clean_*/result.json')]
                successes=sum(control['outcome']=='route_completed' for control in controls)
                if 'clean_successes' in pair:
                    assert pair['clean_successes']==successes and pair['clean_flights']==len(controls),'incorrect clean evidence'
                    assert pair['baseline_stable']==(successes==len(controls)),'incorrect baseline stability'
                if successes<len(controls):
                    assert pair['status']=='baseline_variable_no_attack_attribution','variable clean baseline misattributed'
            row=dict(run=str(folder.relative_to(root)),target=target,tier=tier,shape=shape,role=role,
                     outcome=r['outcome'],video_duration_s=v['duration_s'],video_bytes=(folder/'flight.mp4').stat().st_size,
                     video=str((folder/'flight.mp4').relative_to(root)),
                     route_progress_percent=r['metrics'].get('route_progress_percent'),
                     path_length_m=r['metrics'].get('path_length_m'),
                     mission_duration_sim_s=r['metrics'].get('mission_duration_sim_s'),
                     minimum_clearance_m=r['metrics'].get('minimum_obstacle_clearance_m'),
                     rgb_noise=c['condition']['rgb_noise'],delay_sim_s=c['condition']['delay'],
                     observed_rgb_rmse=sensor_frame['rgb_noise_rmse'],preflight_frame_age_sim_s=frame_age,
                     patch_enabled=c['condition']['patch_enabled'])
            rows.append(row)
        except (AssertionError,OSError,ValueError,KeyError,StopIteration,subprocess.TimeoutExpired) as exc:
            errors.append({'run':str(folder.relative_to(root)),'error':str(exc)})
    counts=collections.defaultdict(lambda:dict(successes=0,flights=0))
    for row in rows:
        if row['role'].startswith('clean_'):
            group=counts[row['tier']+'/'+row['target']]
            group['flights']+=1;group['successes']+=row['outcome']=='route_completed'
    # A partial audit is useful while flights run, but it cannot certify the
    # entire requested 3-model / 4-shape / 3-tier recording matrix.
    missing=[]
    if require_complete:
        from corridor_suite import TARGETS, clean_gate
        indexed={row['run']:row for row in rows}
        for tier in TIERS:
            for target in TARGETS:
                cells=[]
                for shape in SHAPES:
                    base=f'{tier}/{shape}/{target}'
                    cells.append([row for run,row in indexed.items() if run.startswith(base+'/clean_')])
                    roles=['clean_01','clean_02','noise_01','delay_01']
                    if target=='kim' and tier_spec(shape,tier)['obstacles']:roles.append('patch_01')
                    if all(indexed.get(f'{base}/{factor}_01',{}).get('outcome')=='route_completed' for factor in ('noise','delay')):
                        roles.append('combined_01')
                    missing.extend(f'{base}/{role}' for role in roles if f'{base}/{role}' not in indexed)
                if not clean_gate(cells,tier):
                    errors.append({'run':f'{tier}/{target}','error':'clean qualification gate did not pass'})
        if missing:errors.append({'run':'suite','error':f'{len(missing)} required flights missing'})
    if rows:
        with (root/'recording_catalog.csv').open('w',newline='') as handle:
            writer=csv.DictWriter(handle,fieldnames=rows[0].keys());writer.writeheader();writer.writerows(rows)
    result=dict(passed=bool(rows) and not errors,flights=len(rows),clean_counts=dict(counts),
                video_minutes=sum(row['video_duration_s'] for row in rows)/60,
                video_bytes=sum(row['video_bytes'] for row in rows),decoded_every_video=decode,
                rows=rows,errors=errors,complete_matrix_required=require_complete,missing_flights=missing,
                limitations=['Observed within-map repeats; not a statistical reliability estimate.',
                             'Noise/delay/patch survivals and clean failures must be retained.',
                             'This audit does not prove an attack cause or an agent advantage.'])
    (root/'artifact_audit.json').write_text(json.dumps(result,indent=2))
    write_report(root,result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path);parser.add_argument('--decode-videos',action='store_true')
    parser.add_argument('--require-complete',action='store_true',help='Require all targets, shapes and tiers, including attack trials')
    args=parser.parse_args();r=audit(args.root,args.decode_videos,args.require_complete)
    print(json.dumps({k:v for k,v in r.items() if k not in ('rows','limitations')},indent=2))
    raise SystemExit(0 if r['passed'] else 1)

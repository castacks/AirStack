"""EXP142: fixed saved DROID predictions versus same-camera provided odometry."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/provided-reference-full-record/run-001'
PLAN=ROOT/'experiments/provided-reference-full-record-2026-11-05.json'
PACKET=Path('/root/ore-ms-recovery/exp040')

def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,value):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')

def bind(estimate,manifest,modality):
    if modality not in ['rgb','thr']:raise ValueError('camera modality')
    rows=manifest['records'];stems=[r['stem'] for r in rows]
    times=[r['cameras'][modality]['left']['recorded_time_ns'] for r in rows]
    if stems!=[f'{i:06}' for i in range(len(rows))]:raise ValueError('complete original source axis')
    if any(type(t) is not int for t in estimate['time_ns']):raise ValueError('exact integer timestamps')
    if estimate['time_ns']!=times or estimate['source_ids']!=stems:raise ValueError('prediction/source axis mismatch')
    if any(len(estimate[k])!=len(stems) for k in ['poses','segments']):raise ValueError('parallel output population')
    if estimate['scale_fitted'] or estimate['reference_used'] or estimate['physical_frame_or_clock_verified']:
        raise ValueError('prediction scope changed')
    return dict(time_ns=times,stems=stems)

def freeze():
    parent=read(ROOT/'results/bounded-full-record/final-summary.json')
    intake=read(ROOT/'experiments/full-record-intake-2026-11-04.json')
    old=read(PACKET/'manifest.json');runs=[]
    files=[Path(__file__),ROOT/'tools/provided_reference_full_record_audit.py',
           ROOT/'tools/tests/test_provided_reference_full_record.py',
           ROOT/'tools/independent_benchmark_metrics_v2.py',
           ROOT/'docs/research/provided-reference-full-record-protocol-2026-11-05.md',
           ROOT/'experiments/independent-benchmark-2026-11-04.json',
           OUT/'source-001/official-loader.py',OUT/'source-001/data-format.html',
           ROOT/'results/bounded-full-record/final-summary.json',
           ROOT/'results/bounded-full-record/recovery-2026-11-05/recovery-summary.json']
    for result in parent['final_attempts']:
        case=result['case'];condition,mode=case.split('-');modality='rgb' if mode=='rgb' else 'thr'
        source=next(c for c in intake['cases'] if c['condition']==condition)
        manifest=ROOT/'results/bounded-full-record/manifests'/(condition+'.json')
        estimate=ROOT/result['path']/'estimate.json'
        if read(manifest)['sequence']!=source['sequence']:raise ValueError('recording mismatch')
        if sha(estimate)!=result['estimate_sha256']:raise ValueError('EXP141 final selection changed')
        axis=bind(read(estimate),read(manifest),modality)
        archive=f"ore-ms/results/public-data-audit/ms2-scout/odom/{source['sequence']}/archive.tar.bz2"
        runs.append(dict(case=case,condition=condition,modality=modality,sequence=source['sequence'],
                         estimate=str(estimate.relative_to(ROOT)),manifest=str(manifest.relative_to(ROOT)),
                         source_frames=len(axis['stems']),reference_archive_member=archive,
                         reference_archive_sha256=old['member_sha256'][archive]))
        files.extend([estimate,manifest,estimate.parent/'execution-lock.json',
                      ROOT/'results/bounded-full-record/canonical'/case/'plan.json'])
    plan=dict(date='2026-11-05 America/New_York',parent_commit='79339e882',runs=runs,
              source_sha256={str(p.relative_to(ROOT)):sha(p) for p in files},
              historical_packet_archive_sha256=old['archive_sha256'],
              historical_packet_manifest_sha256=sha(PACKET/'manifest.json'),
              reference_values_read=False,reference_packet_downloaded_before_freeze=True,
              reference_convention='provided_camera_to_initial_same_stem_own_left_clock',
              scope='development_only_conditional_provided_reference_offline',
              physical_sync_verified=False,benchmark_execution_ready=False,reserve_access=False,
              scale_fit=False,time_fit=False,new_native_inference=False)
    write(PLAN,plan);write(OUT/'prediction-lock.json',plan)
    write(ROOT/'experiments/recording-exposure-ledger-2026-11-05-provided-reference.json',
          dict(development=[r['sequence'] for r in intake['cases']],
               reference_access='complete RGB/thermal provided odometry of these three existing development records',
               active_validation=[],active_test=[],reserves_unchanged=True,plan_sha256=sha(PLAN)))
    print('FROZEN',len(runs),'complete streams;',sum(r['source_frames'] for r in runs),'slots; no reference values read')

def verify():
    p=read(PLAN)
    if sha(PLAN)!=sha(OUT/'prediction-lock.json'):raise ValueError('prediction lock changed')
    for n,h in p['source_sha256'].items():
        if sha(ROOT/n)!=h:raise ValueError('frozen source/prediction changed '+n)
    for flag in ['physical_sync_verified','benchmark_execution_ready','reserve_access','scale_fit','time_fit','new_native_inference']:
        if p[flag] is not False:raise ValueError('scope claim changed '+flag)
    return p

def extract():
    p=verify();packet=PACKET/'exp-040.tar.gz'
    if sha(packet)!=p['historical_packet_archive_sha256'] or sha(PACKET/'manifest.json')!=p['historical_packet_manifest_sha256']:
        raise ValueError('historical recovery packet changed')
    wanted={r['reference_archive_member']:r['reference_archive_sha256'] for r in p['runs']}
    found={}
    with tarfile.open(packet) as t:
        for m in t:
            if m.name not in wanted:continue
            if not m.isfile() or m.name in found:raise ValueError('duplicate/nonregular archive')
            data=t.extractfile(m).read()
            if hashlib.sha256(data).hexdigest()!=wanted[m.name]:raise ValueError('reference archive hash')
            seq=Path(m.name).parent.name;path=OUT/'archives'/(seq+'.tar.bz2')
            path.parent.mkdir(parents=True,exist_ok=True)
            with path.open('xb') as f:f.write(data)
            found[m.name]=path
    if set(found)!=set(wanted):raise ValueError('missing declared reference archive')
    members={};archives={}
    for name,path in found.items():
        runs=[r for r in p['runs'] if r['reference_archive_member']==name]
        expected={f"{r['sequence']}/{r['modality']}/{i:06}.txt" for r in runs for i in range(r['source_frames'])}
        seen=set();all_names=set()
        with tarfile.open(path) as t:
            for m in t:
                if m.name in all_names:raise ValueError('duplicate reference TAR member')
                all_names.add(m.name)
                if m.name not in expected:continue
                if not m.isfile() or m.size>10000:raise ValueError('reference member type/size')
                data=t.extractfile(m).read();dst=OUT/'reference'/m.name
                dst.parent.mkdir(parents=True,exist_ok=True)
                with dst.open('xb') as f:f.write(data)
                members[str(dst.relative_to(ROOT))]=hashlib.sha256(data).hexdigest();seen.add(m.name)
        if seen!=expected:raise ValueError('incomplete exact-stem reference population')
        archives[str(path.relative_to(ROOT))]=sha(path)
    write(OUT/'reference-lock.json',dict(prediction_plan_sha256=sha(PLAN),archives=archives,
          member_sha256=members,selected_reference_members=len(members),reserve_access=False,
          no_depth_or_other_camera_members_extracted=True))
    print('RESTORED',len(members),'exact provided RGB/thermal reference matrices')

def score_all():
    import numpy as np
    from independent_benchmark_metrics_v2 import score
    p=verify();lock=read(OUT/'reference-lock.json')
    if lock['prediction_plan_sha256']!=sha(PLAN):raise ValueError('reference lock parent')
    for n,h in {**lock['archives'],**lock['member_sha256']}.items():
        if sha(ROOT/n)!=h:raise ValueError('reference bytes changed')
    results=[]
    for run in p['runs']:
        est=read(ROOT/run['estimate']);axis=bind(est,read(ROOT/run['manifest']),run['modality'])
        matrices=[np.loadtxt(OUT/'reference'/run['sequence']/run['modality']/(s+'.txt')).reshape(4,4).tolist() for s in axis['stems']]
        result=score(dict(time_ns=axis['time_ns'],poses=matrices),est)
        write(OUT/'scores'/(run['case']+'.json'),result)
        results.append(dict(case=run['case'],condition=run['condition'],modality=run['modality'],
                            **{k:v for k,v in result.items() if k not in ['rpe_rows','reference_exclusions','segment_ate']}))
        print('SCORED',run['case'],'ATE',result['ate_se3_rmse_m'],'RPE',result['rpe_translation_rmse_m'],flush=True)
    fields=['ate_origin_rmse_m','ate_se3_rmse_m','rpe_translation_rmse_m','rpe_rotation_rmse_deg','pose_coverage','rpe_coverage']
    means={m:{k:(float(np.mean([r[k] for r in results if r['modality']==m])) if all(r[k] is not None for r in results if r['modality']==m) else None) for k in fields} for m in ['rgb','thr']}
    verify();write(OUT/'summary.json',dict(runs=results,equal_recording_means=means,
          reference_convention=p['reference_convention'],reference_values_used_only_for_scoring=True,
          physical_sync_verified=False,benchmark_execution_ready=False,adaptive_advantage_tested=False,
          new_native_inference=False,scale_fitted=False))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('action',choices=['freeze','verify','extract','score']);args=a.parse_args()
    {'freeze':freeze,'verify':verify,'extract':extract,'score':score_all}[args.action]()

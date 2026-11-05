"""Independent EXP143 native-receipt/map continuity and provided-reference audit.

Does not call the new run/publisher/scorer. Uses EXP142's independently derived
raw-reference metric formulas, which do not import its scorer.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np
import provided_reference_full_record_audit as metric

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/orb-full-record/run-001'
PLAN=ROOT/'experiments/orb-full-record-2026-11-05.json'
CHECKS=0
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def need(v,label):
    global CHECKS
    CHECKS+=1
    if not v:raise ValueError(label)

def native_check(source,plan,legacy,evaluation,status,execution,native,p):
    rows=source['records'];n=len(rows);mode='rgb' if plan['modality']=='rgb' else 'thr'
    times=[r['cameras'][mode]['left']['recorded_time_ns'] for r in rows]
    rights=[r['cameras'][mode]['right']['recorded_time_ns'] for r in rows]
    need(evaluation['time_ns']==times and all(type(t) is int for t in evaluation['time_ns']),'exact source clocks')
    need(evaluation['source_ids']==[r['stem'] for r in rows]==[f'{i:06}' for i in range(n)],'complete original stems')
    need(len(evaluation['poses'])==len(evaluation['segments'])==n,'all expected slots')
    need(status['source_frames']==n and status['evaluation_estimate_sha256'] is not None,'status population')
    if legacy is None:
        need(bool(status['publication_error']) and evaluation['terminal_success'] is False,'explicit fail-closed failure')
        need(all(x is None for x in evaluation['poses']+evaluation['segments']),'no failed-publication salvage')
        return
    need(evaluation['poses']==legacy['poses'],'received pose values unchanged')
    need(execution['orb_revision']==p['native_revision'],'native revision')
    for key,path in [('runner_sha256','/opt/ore-ms-adapter/build/orb_ms2_runner'),('vocabulary_sha256','/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt'),('library_sha256','/opt/ORB_SLAM3/lib/libORB_SLAM3.so')]:
        need(execution[key]==p['native_sha256'][path],'binary identity '+key)
    need(execution['compile_flags']==p['compile_flags'] and execution['upstream_patch_sha256']==p['native_patch_sha256'],'native build ABI/patch')
    native_rows=[r for r in native if 'index' in r];configs=[r for r in native if 'configuration' in r];terminal=[r for r in native if 'terminal_success' in r]
    need(len(native_rows)<=n and len(terminal)<=1 and len(configs)<=1,'native journal population')
    need(status['returned_source_calls']==len(native_rows),'native returned-call accounting')
    successful=(legacy['native_returncode']==0 and len(native_rows)==n and len(terminal)==1 and terminal[0]['terminal_success'] is True and terminal[0]['processed_frames']==n)
    need(evaluation['terminal_success'] is (successful and legacy['native_calibration_verified']),'terminal success independent')
    if not configs:
        need(not native_rows and not evaluation['terminal_success'] and all(v is None for v in evaluation['poses']),'pre-configuration failure stays null')
        need(status['native_processing_sum_ns']==0,'no processing on initialization failure')
        return
    config=configs[0]['configuration'];need(config['rerectifies'] is False and config['resizes'] is False,'no native image geometry change')
    for key,value in plan['calibration'].items():need(np.isclose(config[key],value,rtol=5e-7,atol=1e-7),'native calibration '+key)
    sid=-1;active=False;previous_map=None;seen=False;changes=recovery=0
    processing=0
    for i in range(n):
        row=native_rows[i] if i<len(native_rows) else None;pose=evaluation['poses'][i]
        if row is not None:
            need(type(row['index']) is int and row['index']==i,'native frame index')
            need(type(row['time_ns']) is int and row['time_ns']==times[i] and row['right_time_ns']==rights[i],'native original clocks')
            processing+=row['processing_ns']
        valid=row is not None and row['T_camera_from_world'] is not None
        if not valid:
            need(pose is None and evaluation['segments'][i] is None,'null missing/rejected output')
            active=False;continue
        need(row['state']==2 and type(row['map_id']) is int and row['map_id']>=0,'tracking/map evidence')
        inverse=np.linalg.inv(np.asarray(row['T_camera_from_world'],dtype=float))
        need(pose is not None and np.allclose(pose,inverse,rtol=0,atol=1e-5),'independent native Tcw inversion')
        if not active or row['map_id']!=previous_map:
            sid+=1
            if seen:
                if not active:recovery+=1
                if previous_map is not None and row['map_id']!=previous_map:changes+=1
        need(evaluation['segments'][i]==sid,'matched map-continuity segments')
        active=True;previous_map=row['map_id'];seen=True
    need(processing==status['native_processing_sum_ns'],'native processing sum')
    b=status['segment_boundaries']
    need(b['evaluation_segment_count']==sid+1 and b['observed_map_identity_changes']==changes and b['recovery_after_missing_splits']==recovery,'boundary populations')
    need(b['recorded_time_gaps']==sum(b-a>150000000 for a,b in zip(times,times[1:])),'source gaps remain explicit')

def audit():
    p=read(PLAN);lock=read(OUT/'prediction-lock.json');summary=read(OUT/'summary.json')
    need(sha(PLAN)==sha(OUT/'study-lock.json')==lock['plan_sha256'],'study and prediction locks')
    for name,h in {**p['source_sha256'],**lock['source_sha256']}.items():need(sha(ROOT/name)==h,'source/config/native output '+name)
    refs=read(ROOT/'results/provided-reference-full-record/run-001/reference-lock.json')
    for name,h in refs['member_sha256'].items():need(sha(ROOT/name)==h,'EXP142 raw provided reference')
    for flag in ['physical_sync_verified','benchmark_execution_ready','adaptive_advantage_tested','scale_fitted','hardware_matched','output_latency_matched']:
        need(summary[flag] is False,'scope '+flag)
    mutations=0;reports=[]
    for run in p['runs']:
        directory=ROOT/run['directory'];plan=read(directory/'plan.json');source=read(ROOT/run['manifest'])
        need(source['sequence']==run['sequence'] and len(source['records'])==run['source_frames'],'recording identity')
        need(sha(directory/'plan.json')==sha(ROOT/'results/bounded-full-record/canonical'/run['case']/'plan.json'),'same original input plan')
        for name in ['settings.yaml','inputs.tsv']:
            need(sha(directory/name)==sha(ROOT/'results/bounded-full-record/canonical'/run['case']/name),'identical settings and image paths')
        # Frozen original plans bind raw and canonical bytes; verify every actual file.
        for name,h in plan['frozen_files'].items():need(sha(Path(name))==h,'actual input bytes '+name)
        legacy=read(directory/'estimate.json') if (directory/'estimate.json').exists() else None
        execution=read(directory/'execution-lock.json') if (directory/'execution-lock.json').exists() else None
        native=[json.loads(line) for line in (directory/'native.jsonl').read_text().splitlines()] if (directory/'native.jsonl').exists() else []
        est=read(directory/'evaluation-estimate.json');status=read(directory/'run-status.json')
        need(sha(directory/'evaluation-estimate.json')==status['evaluation_estimate_sha256'],'evaluation view hash')
        native_check(source,plan,legacy,est,status,execution,native,p)
        matrices=[np.loadtxt(ROOT/'results/provided-reference-full-record/run-001/reference'/run['sequence']/run['modality']/(r['stem']+'.txt')).reshape(4,4) for r in source['records']]
        score=read(OUT/'scores'/(run['case']+'.json'));want=metric.independently_derive(est['time_ns'],matrices,est);metric.compare(score,want)
        gate=est['terminal_success'] is True and want['reset_count']==0 and want['pose_coverage']>=.95 and want['rpe_coverage']>=.95
        need(score['engineering_success'] is gate,'engineering gate, not fragment-error ranking')
        short=next(r for r in summary['runs'] if r['case']==run['case'])
        for k,v in want.items():
            if k not in ['rpe_rows','segment_ate','excluded_indices']:metric.close(short[k],v,'summary '+k)
        for field,value in [('pose_coverage',score['pose_coverage']+.1),('rpe_expected_pairs',score['rpe_expected_pairs']+1),('scale_fitted',True)]:
            bad=deepcopy(score);bad[field]=value
            try:metric.compare(bad,want)
            except ValueError:mutations+=1
            else:raise ValueError('undetected metric mutation '+field)
        if legacy is not None and any(v is not None for v in est['poses']):
            i=next(i for i,v in enumerate(est['poses']) if v is not None)
            for kind in ['time','segment','pose']:
                bad=deepcopy(est)
                if kind=='time':bad['time_ns'][i]+=1
                elif kind=='segment':bad['segments'][i]+=1
                else:bad['poses'][i][0][3]+=1
                try:native_check(source,plan,legacy,bad,status,execution,native,p)
                except ValueError:mutations+=1
                else:raise ValueError('undetected native mutation '+kind)
        reports.append(dict(case=run['case'],source_frames=run['source_frames'],pose_frames=score['published_pose_frames'],segments=score['segment_count'],terminal_success=score['terminal_success'],engineering_success=gate))
        print('INDEPENDENTLY AUDITED',run['case'],flush=True)
    result=dict(passed=True,independent_checks=CHECKS+metric.COUNT,mutations_rejected=mutations,runs=reports,
                auditor_sha256=sha(__file__),plan_sha256=sha(PLAN),physical_sync_verified=False,benchmark_execution_ready=False)
    with (OUT/'independent-audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':audit()

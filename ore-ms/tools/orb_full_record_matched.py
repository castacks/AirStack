"""Prescoring-declared diagnostic: score DROID on exactly ORB's output fragments.

This is a diagnostic scoring view, not a new DROID run, reset policy or winner.
Original full-record DROID and native ORB results remain unchanged.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/orb-full-record/run-001'
DECL=ROOT/'experiments/orb-full-record-matched-support-2026-11-05.json'
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def matched_view(droid,orb):
    if droid['time_ns']!=orb['time_ns'] or droid['source_ids']!=orb['source_ids']:
        raise ValueError('same camera/source axis required')
    if any(type(t) is not int for t in droid['time_ns']+orb['time_ns']):raise ValueError('exact clocks')
    n=len(orb['time_ns'])
    if any(len(e[k])!=n for e in [droid,orb] for k in ['poses','segments']):raise ValueError('parallel populations')
    result={k:deepcopy(droid[k]) for k in ['time_ns','source_ids','poses']}
    result['terminal_success']=droid.get('terminal_success',False)
    for i,pose in enumerate(orb['poses']):
        if pose is None:
            if orb['segments'][i] is not None:raise ValueError('null pose segment')
            result['poses'][i]=None
        elif droid['poses'][i] is None:raise ValueError('DROID missing on declared ORB support')
    result['segments']=deepcopy(orb['segments'])
    result['scope']='diagnostic DROID poses on exact ORB output masks/map fragments; not native DROID reset behavior'
    return result

def run():
    import numpy as np
    import provided_reference_full_record_audit as independent
    from independent_benchmark_metrics_v2 import score
    d=read(DECL);p=read(ROOT/'experiments/orb-full-record-2026-11-05.json')
    if d['source_sha256']!=sha(__file__) or d['study_sha256']!=sha(ROOT/'experiments/orb-full-record-2026-11-05.json'):
        raise ValueError('diagnostic declaration changed')
    if (OUT/'matched-summary.json').exists():raise ValueError('preserve original diagnostic')
    original=read(ROOT/'experiments/provided-reference-full-record-2026-11-05.json');reports=[]
    for run in p['runs']:
        directory=ROOT/run['directory'];orb=read(directory/'evaluation-estimate.json')
        prior=next(r for r in original['runs'] if r['case']==run['case']);path=ROOT/prior['estimate']
        if sha(path)!=original['source_sha256'][prior['estimate']]:raise ValueError('DROID prediction changed')
        droid=read(path);view=matched_view(droid,orb);rows=read(ROOT/run['manifest'])['records']
        raw=[np.loadtxt(ROOT/'results/provided-reference-full-record/run-001/reference'/run['sequence']/run['modality']/(r['stem']+'.txt')).reshape(4,4).tolist() for r in rows]
        result=score(dict(time_ns=orb['time_ns'],poses=raw),view)
        independent.compare(result,independent.independently_derive(orb['time_ns'],raw,view))
        orb_score=read(OUT/'scores'/(run['case']+'.json'))
        for key in ['pose_coverage','rpe_coverage','segment_count','published_pose_frames','reference_valid_frames','rpe_expected_pairs','rpe_scored_pairs']:
            if result[key]!=orb_score[key]:raise ValueError('matched population '+key)
        if [(r['source_indices'],r['duration_ns']) for r in result['rpe_rows']]!=[(r['source_indices'],r['duration_ns']) for r in orb_score['rpe_rows']]:
            raise ValueError('matched exact RPE intervals')
        dst=OUT/'matched'/run['case'];dst.mkdir(parents=True,exist_ok=False)
        for name,value in [('droid-evaluation-view.json',view),('droid-score.json',result)]:
            with (dst/name).open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')
        fields=['ate_se3_rmse_m','ate_origin_rmse_m','rpe_translation_rmse_m','rpe_rotation_rmse_deg','pose_coverage','rpe_coverage','segment_count']
        reports.append(dict(case=run['case'],orb={k:orb_score[k] for k in fields},droid_on_identical_orb_fragments={k:result[k] for k in fields}))
    value=dict(runs=reports,independent_metric_checks=independent.COUNT,declaration_sha256=sha(DECL),
               diagnostic_only=True,native_predictions_changed=False,native_droid_resets_implied=False,
               physical_sync_verified=False,benchmark_execution_ready=False)
    with (OUT/'matched-summary.json').open('x') as f:json.dump(value,f,indent=2);f.write('\n')
    print(json.dumps(value,indent=2))

if __name__=='__main__':run()

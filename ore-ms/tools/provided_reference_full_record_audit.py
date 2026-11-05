"""Independent raw-reference pose/population and SE3/RPE audit for EXP142.

Does not import the scorer, study or their pose/alignment helpers.
"""
from bisect import bisect_left
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/provided-reference-full-record/run-001'
PLAN=ROOT/'experiments/provided-reference-full-record-2026-11-05.json'
COUNT=0
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def check(v,label):
    global COUNT
    COUNT+=1
    if not v:raise ValueError(label)
def close(a,b,label):
    if a is None or b is None:check(a is None and b is None,label)
    else:check(bool(np.allclose(a,b,rtol=1e-9,atol=1e-7)),label)

def rigid(value):
    if value is None:return None
    a=np.asarray(value,dtype=float)
    if a.shape!=(4,4) or not np.isfinite(a).all() or not np.array_equal(a[3],[0,0,0,1]):return None
    r=a[:3,:3]
    if np.linalg.norm(r.T@r-np.eye(3))>1e-5 or abs(np.linalg.det(r)-1)>1e-5:return None
    u,_,v=np.linalg.svd(r);b=a.copy();b[:3,:3]=u@np.diag([1,1,np.linalg.det(u@v)])@v;return b

def independently_derive(times,raw,est):
    g=[rigid(p) for p in raw];e=[rigid(p) for p in est['poses']];n=len(times)
    excluded=[i for i,p in enumerate(g) if p is None]
    groups={}
    for i,(p,s) in enumerate(zip(e,est['segments'])):
        if p is not None:groups.setdefault(s,[]).append(i)
    osq=[];fsq=[];degenerate=False;segment_rows=[]
    for sid,indices in groups.items():
        ix=[i for i in indices if g[i] is not None]
        if not ix:continue
        x=np.array([e[i][:3,3] for i in ix]);y=np.array([g[i][:3,3] for i in ix])
        rot=g[ix[0]][:3,:3]@e[ix[0]][:3,:3].T
        orig=(rot@(x-x[0]).T).T+y[0];sq=np.sum((orig-y)**2,axis=1);osq.extend(sq)
        xc=x-x.mean(0);yc=y-y.mean(0);rank=int(np.linalg.matrix_rank(xc,tol=1e-10))
        fit=None
        if rank<2 or np.linalg.matrix_rank(yc,tol=1e-10)<2:degenerate=True
        else:
            u,_,v=np.linalg.svd(yc.T@xc);r=u@np.diag([1,1,np.linalg.det(u@v)])@v
            diff=(r@xc.T).T-yc;ss=np.sum(diff**2,axis=1);fsq.extend(ss);fit=float(np.sqrt(np.mean(ss)))
        segment_rows.append(dict(segment=sid,source_frames=len(indices),reference_matched_frames=len(ix),position_rank=rank,ate_origin_rmse_m=float(np.sqrt(np.mean(sq))),ate_se3_rmse_m=fit))
    expected=[];pairs=[]
    for i,t in enumerate(times):
        j=bisect_left(times,t+1000000000)
        if j>=n or times[j]-t>1150000000:continue
        if any(g[k] is None for k in range(i,j+1)) or any(times[k+1]-times[k]>150000000 for k in range(i,j)):continue
        expected.append((i,j))
        if any(e[k] is None or est['segments'][k]!=est['segments'][i] for k in range(i,j+1)):continue
        gt=g[i][:3,:3].T@(g[j][:3,3]-g[i][:3,3]);et=e[i][:3,:3].T@(e[j][:3,3]-e[i][:3,3])
        gr=g[i][:3,:3].T@g[j][:3,:3];er=e[i][:3,:3].T@e[j][:3,:3];r=gr.T@er
        skew=np.array([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])
        angle=np.degrees(np.arctan2(np.linalg.norm(skew),np.trace(r)-1))
        pairs.append(dict(source_indices=[i,j],duration_ns=times[j]-times[i],translation_m=float(np.linalg.norm(et-gt)),rotation_deg=float(angle)))
    count=sum(p is not None for p in e);longest=run=0
    for i,p in enumerate(e):
        if p is None:run=0
        elif i==0 or e[i-1] is None or est['segments'][i]!=est['segments'][i-1] or times[i]-times[i-1]>150000000:run=1
        else:run+=1
        longest=max(longest,run)
    return dict(source_frames=n,published_pose_frames=count,reference_valid_frames=n-len(excluded),
                excluded_indices=excluded,pose_coverage=count/n,segment_count=len(groups),reset_count=max(0,len(groups)-1),longest_contiguous_frames=longest,
                segment_ate=segment_rows,ate_origin_rmse_m=float(np.sqrt(np.mean(osq))) if osq else None,
                ate_se3_rmse_m=None if degenerate or not fsq else float(np.sqrt(np.mean(fsq))),
                rpe_expected_pairs=len(expected),rpe_scored_pairs=len(pairs),rpe_coverage=len(pairs)/len(expected) if expected else 0.,
                rpe_translation_rmse_m=float(np.sqrt(np.mean([p['translation_m']**2 for p in pairs]))) if pairs else None,
                rpe_rotation_rmse_deg=float(np.sqrt(np.mean([p['rotation_deg']**2 for p in pairs]))) if pairs else None,
                rpe_rows=pairs)

def compare(actual,want):
    for k,v in want.items():
        if k in ['rpe_rows','segment_ate','excluded_indices']:continue
        close(actual[k],v,k)
    check([r['index'] for r in actual['reference_exclusions']]==want['excluded_indices'],'reference exclusions')
    check(actual['alignment']=='SE3_only_no_scale' and actual['scale_fitted'] is False,'scale/alignment')
    check(len(actual['rpe_rows'])==len(want['rpe_rows']),'RPE row count')
    for a,b in zip(actual['rpe_rows'],want['rpe_rows']):
        check(a['source_indices']==b['source_indices'] and a['duration_ns']==b['duration_ns'],'interval binding')
        close(a['translation_m'],b['translation_m'],'independent relative translation')
        close(a['rotation_deg'],b['rotation_deg'],'independent relative rotation')
    check(len(actual['segment_ate'])==len(want['segment_ate']),'segment count')
    for a,b in zip(actual['segment_ate'],want['segment_ate']):
        for k,v in b.items():close(a[k],v,'segment '+k)

def audit():
    p=read(PLAN);lock=read(OUT/'reference-lock.json');summary=read(OUT/'summary.json')
    check(sha(PLAN)==sha(OUT/'prediction-lock.json')==lock['prediction_plan_sha256'],'prediction-before-values lock')
    for n,h in {**p['source_sha256'],**lock['archives'],**lock['member_sha256']}.items():check(sha(ROOT/n)==h,'actual frozen file '+n)
    for flag in ['physical_sync_verified','benchmark_execution_ready','adaptive_advantage_tested','new_native_inference','scale_fitted']:
        check(summary[flag] is False,'scope '+flag)
    reports=[];mutations=0
    for run in p['runs']:
        src=read(ROOT/run['manifest']);est=read(ROOT/run['estimate']);actual=read(OUT/'scores'/(run['case']+'.json'))
        check(src['sequence']==run['sequence'],'recording identity')
        rows=src['records'];times=[r['cameras'][run['modality']]['left']['recorded_time_ns'] for r in rows]
        check(est['time_ns']==times and all(type(t) is int for t in est['time_ns']),'exact camera clock')
        check(est['source_ids']==[r['stem'] for r in rows]==[f'{i:06}' for i in range(run['source_frames'])],'all source stems')
        raw=[np.loadtxt(OUT/'reference'/run['sequence']/run['modality']/(r['stem']+'.txt')).reshape(4,4) for r in rows]
        want=independently_derive(times,raw,est);compare(actual,want)
        expected_success=est['terminal_success'] is True and want['reset_count']==0 and want['pose_coverage']>=.95 and want['rpe_coverage']>=.95
        check(actual['terminal_success'] is est['terminal_success'] and actual['engineering_success'] is expected_success,'engineering gate')
        short=next(r for r in summary['runs'] if r['case']==run['case'])
        for k in want:
            if k not in ['rpe_rows','segment_ate','excluded_indices']:close(short[k],want[k],'summary '+k)
        for field,value in [('ate_origin_rmse_m',(actual['ate_origin_rmse_m'] or 0)+1),('rpe_expected_pairs',actual['rpe_expected_pairs']+1),('pose_coverage',.5),('scale_fitted',True)]:
            bad=deepcopy(actual);bad[field]=value
            try:compare(bad,want)
            except ValueError:mutations+=1
            else:raise ValueError('undetected '+field)
        if actual['rpe_rows']:
            bad=deepcopy(actual);bad['rpe_rows'][0]['source_indices'][1]+=1
            try:compare(bad,want)
            except ValueError:mutations+=1
            else:raise ValueError('undetected interval mutation')
        reports.append(dict(case=run['case'],source_frames=len(rows),reference_invalid=len(want['excluded_indices']),rpe_pairs=want['rpe_scored_pairs']))
        print('INDEPENDENTLY AUDITED',run['case'],flush=True)
    for m,values in summary['equal_recording_means'].items():
        cases=[r for r in summary['runs'] if r['modality']==m];check(len(cases)==3,'equal recording population')
        for k,v in values.items():close(v,float(np.mean([r[k] for r in cases])) if all(r[k] is not None for r in cases) else None,'recording mean '+k)
    result=dict(passed=True,independent_checks=COUNT,mutations_rejected=mutations,runs=reports,
                auditor_sha256=sha(__file__),plan_sha256=sha(PLAN),physical_sync_verified=False,benchmark_execution_ready=False)
    with (OUT/'independent-audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':audit()

"""Independent EXP112 geometry, prefix actions, reference errors and score audit."""
from pathlib import Path
from decimal import Decimal
import hashlib
import json
import math
import numpy as np
from scipy.linalg import expm,logm
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/sensor-admission-execution/run-001'

def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def skew(v):
    x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
def pose(v):
    v=np.asarray(v);t=np.eye(4);t[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();t[:3,3]=v[:3];return t

def main():
    plan=read(ROOT/'experiments/sensor-admission-execution-2026-11-02.json');checks=0
    def close(a,b,rtol=1e-8,atol=1e-9):
        nonlocal checks
        np.testing.assert_allclose(a,b,rtol=rtol,atol=atol);checks+=int(np.asarray(a).size)
    for n,h in {**plan['input_source_sha256'],**plan['scorer_only_reference_sha256']}.items():assert sha(ROOT/n)==h;checks+=1
    lock=read(OUT/'prediction-lock.json');assert lock['labels_read'] is False
    assert lock['actions_sha256']==sha(OUT/'locked-actions.json') and lock['prefix_sha256']==sha(OUT/'fresh-prefix-predictions.json')
    contract=read(OUT/'admission-inputs.json');actions=read(OUT/'locked-actions.json');labels=read(OUT/'fresh-labels.json');scores=read(OUT/'scores.json')
    inputs={r['sample_id']:r for r in contract['rows']};pred={r['sample_id']:r for r in actions['rows']};allprefix=read(OUT/'fresh-prefix-predictions.json')['rows']
    assert len(inputs)==len(pred)==len(labels)==274;assert inputs.keys()==pred.keys()==labels.keys()
    geometry_count=0;transport_count=0
    for c in plan['cases']:
        base=OUT/'captures'/c['case']/'on';timeline=[json.loads(s) for s in (base/'timeline.jsonl').read_text().splitlines()]
        decisions={e['pair_ordinal']:e for e in timeline if e['kind']=='decision'}
        prefixes=[r for r in allprefix if r['window']==c['window']];assert len(prefixes)==c['frames']-1
        lookup={}
        for camera in ['rgb','thermal']:
            worker=[e for e in timeline if e['kind']=='observer_event' and e['camera']==camera]
            events={e['event']['sequence']:e for e in worker};lookup[camera]=events
            with np.load(base/'experts'/camera/'tensor-map.npz',allow_pickle=False) as saved:
                for item in worker:
                    g=item['event']
                    if g['kind']!='native_disparity_information':continue
                    motion,=[e['event'] for e in worker if e['event']['kind']=='motion_result_written' and e['event']['first_write_for_submission'] and e['event']['submission_id']==g['submission_id']]
                    assert g['sequence']<motion['sequence'];assert [g['time0_ns'],g['time1_ns']]==[motion['time0_ns'],motion['time1_ns']]
                    i=g['frame_idx'];mask=saved['edge/match2frame2/mapping']==i;ids=saved['edge/match2point/mapping'][mask]
                    world=saved['points//pos_Tw'][ids].astype(float);t=pose(np.asarray(motion['optimizer_pose_before_write_native_parameterization']).reshape(7));p=(world-t[:3,3])@t[:3,:3]
                    K=saved['frames//K'][i].astype(float);baseline=float(np.asarray(saved['frames//baseline'][i]).reshape(-1)[0]);n=len(p)
                    uv=saved['match//pixel2_uv_cov'][mask];disp=saved['match//pixel2_disp_cov'][mask].reshape(-1);cov=np.zeros((n,3,3),np.float32)
                    cov[:,0,0]=uv[:,0];cov[:,1,1]=uv[:,1];cov[:,0,1]=cov[:,1,0]=uv[:,2];cov[:,2,2]=disp
                    jac=[]
                    for x,y,z in p:
                        Q=np.array([[-K[0,0]*y/x**2,K[0,0]/x,0],[-K[1,1]*z/x**2,0,K[1,1]/x],[-K[0,0]*baseline/x**2,0,0]])
                        jac.append(Q@np.concatenate([-np.eye(3),skew([x,y,z])],axis=1))
                    jac=np.asarray(jac);precision=np.linalg.pinv(cov.astype(float),rcond=1e-15)
                    h=np.einsum('nki,nkl,nlj->ij',jac,precision,jac);h=(h+h.T)/2
                    close(h,g['information']['information_6x6'],rtol=1e-10,atol=1e-7);assert n==g['information']['factors'];geometry_count+=1
                    # Actual returned pose is independently bound to the copied camera motion.
                    delta=np.linalg.solve(pose(motion['parent_pose_camera_map_xyzw']),pose(motion['committed_pose_camera_map_xyzw']))
                    close(delta,pose(motion['derived_camera_delta_xyzw']))
        for row in prefixes:
            sid=f"{c['window']}/{row['pair_ordinal']}";source=inputs[sid];decision=decisions[row['pair_ordinal']]
            assert source['deadline_ns']==decision['controller_monotonic_ns'] and source['interval_ns']==row['common_interval_ns']
            for camera in ['rgb','thermal']:
                e=row['experts'][camera];slot=source['cameras'][camera]
                assert slot['available']==slot['supported']==bool(e['available_by_prefix'] and e['valid_model_projection'])
                if not slot['available']:assert slot['information'] is None;continue
                events=lookup[camera];motion=events[e['source']['motion_event_sequence']]['event'];native=pose(motion['derived_camera_delta_xyzw']);t=np.asarray(c['nominal_body_from_camera'][camera]);body=t@native@np.linalg.inv(t)
                a,b=e['native_interval_ns'];s0,s1=row['common_interval_ns'];ratio=(s1-s0)/(b-a)
                model=np.real_if_close(expm(ratio*logm(body))).real;close(model,e['delta_model'])
                observed,=[x for x in events.values() if x['event']['kind']=='native_disparity_information' and x['event']['submission_id']==motion['submission_id']]
                assert observed['controller_monotonic_ns']<=decision['controller_monotonic_ns']
                eps=1e-6;columns=[]
                for k in range(6):
                    twist=np.zeros((4,4));v=np.eye(6)[k]*eps;twist[:3,3]=v[:3];twist[:3,:3]=skew(v[3:]);values=[]
                    for sign in [1,-1]:
                        moved=t@native@expm(sign*twist)@np.linalg.inv(t);mapped=expm(ratio*logm(moved));difference=np.real_if_close(logm(np.linalg.solve(model,mapped))).real
                        values.append(np.r_[difference[:3,3],[difference[2,1],difference[0,2],difference[1,0]]])
                    columns.append((values[0]-values[1])/(2*eps))
                A=np.column_stack(columns);inv=np.linalg.inv(A);H=np.array(observed['event']['information']['information_6x6']);mapped=inv.T@H@inv
                close(mapped,slot['information']['matrix'],rtol=2e-5,atol=.05);transport_count+=1
        # Independent provider interpolation: same archived reference, fresh predictions.
        d=ROOT/c['reference_dataset']/'super_odometry/odometry';times=np.array([int((Decimal(s)*10**9).to_integral_value()) for s in (d/'interp_timestamps.txt').read_text().splitlines()],np.int64)
        values=np.loadtxt(d/'interp_data.txt')[:,:7];seconds=(times-times[0])/1e9;slerp=Slerp(seconds,Rotation.from_quat(values[:,3:]))
        for r in prefixes:
            q=(np.array(r['common_interval_ns'],np.int64)-times[0])/1e9;ref=np.repeat(np.eye(4)[None],2,axis=0);ref[:,:3,:3]=slerp(q).as_matrix();ref[:,:3,3]=np.column_stack([np.interp(q,seconds,values[:,j]) for j in range(3)])
            expected=np.linalg.solve(ref[0],ref[1]);sid=f"{c['window']}/{r['pair_ordinal']}"
            for camera,e in r['experts'].items():
                if not e['valid_model_projection']:assert labels[sid][camera] is None;continue
                error=np.linalg.solve(expected,np.asarray(e['delta_model']));rotation=error[:3,:3];sin=np.linalg.norm([rotation[2,1]-rotation[1,2],rotation[0,2]-rotation[2,0],rotation[1,0]-rotation[0,1]])/2;cos=(np.trace(rotation)-1)/2
                close(labels[sid][camera]['translation_m'],np.linalg.norm(error[:3,3]));close(labels[sid][camera]['rotation_deg'],np.degrees(np.arctan2(sin,cos)))
    # Reconstruct the fixed policy without importing production policy code.
    states={};D=np.diag([.1]*3+[np.pi/180]*3)
    for r in contract['rows']:
        sid=r['sample_id'];key=(r['recording_id'],r['window']);old=states.get(key);streak={c:0 for c in ['rgb','thermal']}
        if old and r['pair_ordinal']==old[0]+1 and r['interval_ns'][0]==old[1]:streak=old[2].copy()
        good={}
        for camera in streak:
            slot=r['cameras'][camera]
            good[camera]=bool(slot['available'] and slot['supported'] and np.linalg.eigvalsh(D@np.array(slot['information']['matrix'])@D)[0]>=1)
            streak[camera]=streak[camera]+1 if good[camera] else 0
        confirmed=[c for c in streak if good[c] and streak[c]>=2];current=[c for c in streak if good[c]];selected=(confirmed or current or [None])[0]
        assert pred[sid]['actions']['candidate']==(selected.upper() if selected else 'ABSTAIN');assert pred[sid]['streak']==streak;assert pred[sid]['adequate']==good
        assert pred[sid]['actions']['geometry_current']==(current[0].upper() if current else 'ABSTAIN')
        assert pred[sid]['actions']['raw']==r['baselines']['raw'] and pred[sid]['actions']['history']==r['baselines']['history'];checks+=10
        states[key]=(r['pair_ordinal'],r['interval_ns'][1],streak)
    # Exact matched scoring, all original recording populations and controls.
    for record,s in scores['records'].items():
        rows=[r for r in actions['rows'] if r['recording_id']==record];assert len(rows)==s['decisions']
        for method,stat in s['methods'].items():
            chosen=[r for r in rows if r['actions'][method]!='ABSTAIN'];assert stat['ids']==[r['sample_id'] for r in chosen] and stat['scored']==len(chosen)
            for metric in ['translation_m','rotation_deg']:
                value=math.sqrt(math.fsum(labels[r['sample_id']][r['actions'][method].lower()][metric]**2 for r in chosen)/len(chosen)) if chosen else None
                if value is None:assert stat[metric] is None
                else:close(stat[metric],value)
        for method,stat in s['matched'].items():
            chosen=[r for r in rows if all(r['actions'][m]!='ABSTAIN' for m in ['candidate',method])];assert stat['ids']==[r['sample_id'] for r in chosen]
            for m in ['candidate',method]:
                for metric in ['translation_m','rotation_deg']:
                    value=math.sqrt(math.fsum(labels[r['sample_id']][r['actions'][m].lower()][metric]**2 for r in chosen)/len(chosen)) if chosen else None
                    if value is None:assert stat[m][metric] is None
                    else:close(stat[m][metric],value)
        passed=s['methods']['candidate']['scored']>=s['methods']['raw']['scored'] and all(x['ids'] and x['candidate']['translation_m']<x[m]['translation_m'] and x['candidate']['rotation_deg']<=x[m]['rotation_deg'] for m,x in s['matched'].items())
        assert bool(passed)==s['passes_strict_diagnostic']
    assert scores['passes_all_records']==all(r['passes_strict_diagnostic'] for r in scores['records'].values())
    for n,h in {**plan['input_source_sha256'],**plan['scorer_only_reference_sha256']}.items():assert sha(ROOT/n)==h
    result={'status':'PASS_INDEPENDENT_ADMISSION_AUDIT','numeric_values_checked':checks,'native_information_events_reconstructed':geometry_count,'common_interval_information_transports_reconstructed':transport_count,'decisions':274,'scores_sha256':sha(OUT/'scores.json'),'actions_sha256':sha(OUT/'locked-actions.json'),'auditor_sha256':sha(__file__),'physical_accuracy_verified':False}
    with (OUT/'audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(result)

if __name__=='__main__':main()

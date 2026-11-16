"""Independent quaternion integration and saved-output angular agreement audit."""
import json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from audit_subt_export import mul,inv,angle
ROOT=Path(__file__).resolve().parents[1];checks=0
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    import hashlib
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def check(v,label):
    global checks
    assert v,label
    checks+=1

def gyro_quaternion(times,gyro,t0,t1):
    if t1<=t0:return None
    j=int(np.searchsorted(times,t0,side='right')-1);end=int(np.searchsorted(times,t1))
    if j<0 or end>=len(times) or np.max(np.diff(times[j:end+1]))>20_000_000:return None
    q=np.array([0.,0.,0.,1.]);cursor=t0
    while cursor<t1:
        stop=min(int(times[j+1]),t1);v=gyro[j]*((stop-cursor)*1e-9);norm=np.linalg.norm(v);delta=np.r_[v*(np.sin(norm/2)/norm if norm else .5),np.cos(norm/2)];q=mul(q,delta);j+=1;cursor=stop
    return q/np.linalg.norm(q)

def main(config):
    cfg=read(config);out=ROOT/cfg['output'];parent=ROOT/cfg['matching'];qual=ROOT/cfg['qualification'];raw=ROOT/cfg['raw_binding'];lock=read(out/'target-lock.json');done=read(out/'completion.json')
    for p,h in lock['pins'].items():check(sha(ROOT/p)==h,'frozen '+p)
    check(lock['prediction_errors_computed'] is False,'targets fixed before errors');check(read(raw/'qualification.json')['status']=='pass','raw/export qualification')
    for name,key in [('disagreements.json','disagreements_sha256'),('summary.json','summary_sha256')]:check(sha(out/name)==done[key],'locked '+name)
    labels=read(out/'gyro-labels.json');by={(r['arm'],r['index']):r for r in labels};inputs=read(parent/'matcher-inputs.json');check(len(labels)==len(by)==len(inputs)==2*lock['anchors'],'full gyro population');imu=np.load(qual/'export-imu.npz');times=imu['time_ns'];gyro=imu['values'][:,4:7];quats={}
    for row in inputs:
        key=row['arm'],row['index'];label=by[key];check(all(label[k]==row[k] for k in ['arm','index','source','target','times_ns','qualified']),'original sensor endpoints');q=gyro_quaternion(times,gyro,*row['times_ns']);check(label['supported']==(q is not None),'independent gyro support')
        if q is None:continue
        target=Rotation.from_quat(inv(q)).as_matrix();check(np.allclose(target,label['target_from_source_imu'],atol=1e-10) and abs(angle(q)-label['rotation_deg'])<1e-6,'independent target direction/integration');quats[key]=q
    n=lock['anchors'];common={i for i in range(n) if all(by[a,i]['qualified'] and (a,i) in quats for a in ['rgb','thermal'])};moving={i for i in common if all(angle(quats[a,i])>=2 for a in ['rgb','thermal'])};check(read(out/'population.json')=={'common_gyro':sorted(common),'paired_rotation_ge2deg':sorted(moving)},'frozen gyro cohorts')
    cal=read(qual/'calibration.json');pred=[json.loads(s) for s in (parent/'predictions.jsonl').read_text().splitlines()];saved=read(out/'disagreements.json');scored={(r['frontend'],r['arm'],r['index']):r for r in saved};check(len(pred)==len(saved)==len(scored)==4*n,'complete prediction diagnostic');errors={}
    for p in pred:
        key=p['frontend'],p['arm'],p['index'];s=scored[key];q=quats.get((p['arm'],p['index']));e=None
        if p['status']=='ok' and q is not None:
            rc=Rotation.from_matrix(np.array(cal[p['arm']]['effective_camera_to_imu'])[:3,:3]).as_quat();qp=Rotation.from_matrix(p['R']).as_quat();estimate=mul(mul(rc,qp),inv(rc));e=angle(mul(estimate,q))
        check(s['status']==p['status'] and s['input_supported']==p['qualified'] and s['gyro_supported']==(q is not None),'diagnostic status binding');check(s['disagreement_deg'] is None if e is None else abs(s['disagreement_deg']-e)<1e-6,'independent angular discrepancy');errors[key]=e
    ds=read(parent/'decisions.json');dmap={(d['frontend'],d['index']):d for d in ds};check(len(ds)==len(dmap)==2*n,'frozen decisions');summary=read(out/'summary.json');check(summary['pose_accuracy'] is None,'not pose truth');groups={'all_declared':set(range(n)),'common_gyro':common,'paired_rotation_ge2deg':moving,**{'third'+str(j+1):{i for i in common if i*3//n==j} for j in range(3)}}
    for f in ['sift','learned']:
        for name,ids in groups.items():
            eligible=ids&common;ds=[dmap[f,i] for i in sorted(ids)];result=summary['frontends'][f][name]
            for policy in ['rgb','thermal','rgb_first']:
                ee=[errors[f,d['policies'][policy],d['index']] for d in ds if d['index'] in eligible and d['policies'][policy]];r=result[policy]
                check(r['anchors']==len(ids) and r['outputs']==sum(d['policies'][policy] is not None for d in ds) and r['common_gyro_eligible']==len(eligible) and r['compared_outputs']==len(ee) and r['missing_outputs_on_common']==len(eligible)-len(ee),'agreement denominators')
                check(r['pose_accuracy'] is None and r['within1deg']==sum(e<=1 for e in ee) and r['within5deg']==sum(e<=5 for e in ee),'all agreement counts')
                check(r['conditional_median_deg'] is None if not ee else abs(r['conditional_median_deg']-np.median(ee))<1e-6,'conditional angular median')
            added=[d for d in ds if d['policies']['rgb_first']=='thermal'];valid=[d for d in added if d['index'] in eligible];ee=[errors[f,'thermal',d['index']] for d in valid];zz=[angle(quats['thermal',d['index']]) for d in valid];r=result['added_thermal']
            check(r['outputs']==len(added) and r['gyro_eligible']==len(valid),'added eligibility');check(r['within1deg']==sum(e<=1 for e in ee) and r['within5deg']==sum(e<=5 for e in ee) and r['zero_within1deg']==sum(e<=1 for e in zz) and r['zero_within5deg']==sum(e<=5 for e in zz),'added estimates versus same-endpoint zero control')
            check(r['conditional_median_deg'] is None if not ee else abs(r['conditional_median_deg']-np.median(ee))<1e-6,'added median')
            for a in ['rgb','thermal']:
                for t in [1,5]:check(result['zero_rotation'][a][str(t)]==sum(angle(quats[a,i])<=t for i in eligible),'all zero controls')
    result={'status':'pass','checks':checks,'rows':len(saved),'common_gyro':len(common),'paired_moving':len(moving),'auditor_sha256':sha(__file__),'scope':'Independent quaternion integration, target direction, saved rotations, all cohort/count/identity controls; nominal gyro consistency, not physical pose truth'};(out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':
    import sys
    main(Path(sys.argv[1]))

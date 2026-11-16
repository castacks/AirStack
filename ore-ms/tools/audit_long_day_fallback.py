"""Independent saved-output audit of full-span fallback control, no runner import."""
from pathlib import Path
import copy,json,math
import numpy as np
from scipy.spatial.transform import Rotation
from audit_optimization_budget_control import sha,read,pose,proper,measure,BASIS
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/long-day-fallback-control/run-001'


def audit(rows,scores,freeze,refs,events,thermal_events,thermal):
    count=0;methods=freeze['methods']
    def check(value,label):
        nonlocal count
        assert value,label
        count+=1
    def compare(a,b):
        for k,v in b.items():check(math.isclose(a[k],v,rel_tol=1e-10,abs_tol=1e-10),k)
    check([r['pair'] for r in rows]==list(range(599)),'population')
    fallback=[i for i in range(599) if thermal_events.get((i,i+1)) is None or thermal_events[(i,i+1)]['original_need_interp']]
    check(fallback==freeze['fallback_pairs']==[25,27,28,32],'support contract')
    states={m:freeze['initial_state'] for m in methods};used=0;fallbacks=[]
    for row in rows:
        i=row['pair'];check(row['order']==methods[i%4:]+methods[:i%4],'order')
        binding=row['thermal_binding']
        if i in fallback:
            check(binding=={'thermal_used':False,'fallback':'RGB_ONLY_TRANSLATION','reason':'no_native_supported_first_write','pair':i},'fallback metadata');fallbacks.append(i)
        else:
            check('fallback' not in binding,'no undeclared fallback')
            expected=np.flatnonzero((thermal['edge/match2frame1/mapping']==i)&(thermal['edge/match2frame2/mapping']==i+1)).tolist()
            check(binding['native_factor_indices']==expected and len(expected)>=3,'all accepted thermal factors')
            check(binding['native_first_write_sequence']==thermal_events[(i,i+1)]['sequence'],'thermal event binding');used+=1
        for m in methods:
            v=row['methods'][m];s=row['stages'][m];receipts=s['receipts']
            check(v['parent_xyzw']==states[m],'own state continuity')
            expected=BASIS@np.linalg.inv(pose(states[m]))@pose(v['committed_xyzw'])@BASIS.T
            check(np.allclose(expected,v['transform'],rtol=0,atol=1e-10),'pose delta')
            check(v['status']=='AVAILABLE','availability')
            check(len(receipts)==(1 if m=='native_rgb' else 2),'stage count')
            for r in receipts:
                check(1<=r['steps']<=10 and len(r['losses'])==r['steps'],'step cap')
                check(np.isfinite(r['losses']).all(),'finite loss')
                check(r['solver_cpu_seconds']>0 and r['solver_wall_seconds']>0,'positive cost')
            check(v['losses']==sum([r['losses'] for r in receipts],[]),'loss accounting')
            for k in ['steps','solver_cpu_seconds','solver_wall_seconds']:check(v[k]==sum(r[k] for r in receipts),'cost accounting')
            if m=='native_rgb':check(v['committed_xyzw']==s['first'],'native one stage')
            if m in ['extra_translation','thermal_translation']:check(v['committed_xyzw'][3:]==s['first'][3:],'own first rotation')
            states[m]=v['committed_xyzw']
        expected=BASIS@pose(events[(i,i+1)]['derived_camera_delta_xyzw'])@BASIS.T
        delta=np.linalg.inv(expected)@np.asarray(row['methods']['native_rgb']['transform'])
        t=np.linalg.norm(delta[:3,3]);a=Rotation.from_matrix(delta[:3,:3]).magnitude()
        compare(row['reproduction'],{'translation_m':t,'rotation_rad':a})
        check(row['reproduction']['passed']==bool(t<1e-3 and a<1e-4),'native gate')
    check(used==595 and fallbacks==fallback,'thermal usage')
    check(scores['native_gate_passed']==all(r['reproduction']['passed'] for r in rows),'aggregate native gate')
    for m in methods:
        tv=[];rv=[];pv=[0.];av=[0.];rot=np.eye(3);pos=np.zeros(3)
        for row in rows:
            i=row['pair'];d=proper(row['methods'][m]['transform']);ref=np.linalg.solve(refs[i],refs[i+1])
            tv.append(np.linalg.norm(d[:3,3]-ref[:3,3]));rv.append(math.degrees(Rotation.from_matrix(ref[:3,:3].T@d[:3,:3]).magnitude()))
            pos=pos+rot@d[:3,3];rot=rot@d[:3,:3];truth=np.linalg.solve(refs[0],refs[i+1])
            pv.append(np.linalg.norm(pos-truth[:3,3]));av.append(math.degrees(Rotation.from_matrix(truth[:3,:3].T@rot).magnitude()))
        s=scores['summary'][m];check(s['available']==599,'full support')
        for key,values in [('translation_m',tv),('rotation_deg',rv),('path_position_m',pv),('path_orientation_deg',av)]:compare(s[key],measure(values))
        compare(s,{'cpu_total_seconds':math.fsum(r['methods'][m]['solver_cpu_seconds'] for r in rows),'wall_total_seconds':math.fsum(r['methods'][m]['solver_wall_seconds'] for r in rows),'steps_total':sum(r['methods'][m]['steps'] for r in rows)})
    return count


def main():
    f=read(OUT/'freeze.json');lock=read(OUT/'prediction-lock.json');scores=read(OUT/'scores.json');rows=read(OUT/'predictions.json')['rows']
    assert sha(OUT/'freeze.json')==lock['freeze_sha256'] and sha(OUT/'predictions.json')==lock['prediction_sha256']
    assert scores['prediction_lock_sha256']==sha(OUT/'prediction-lock.json')
    for kind in ['source_pins','input_pins','reference_pins']:
        for p,h in f[kind].items():assert sha(ROOT/p)==h,p
    pre=OUT.parent/'preflight.json';assert sha(pre)==f['preflight_sha256']
    q=read(pre);assert q['fallback_pairs']==[25,27,28,32] and q['rgb_supported']==599 and q['thermal_supported']==595
    assert all(x['initial_arrays_jacobian_exact'] and x['committed_solution_and_losses_exact'] for x in q['fallback_equivalence'])
    assert rows==[json.loads(line) for line in (OUT/'capture.jsonl').read_text().splitlines()]
    events={};thermal_events={}
    for camera,target in [('rgb',events),('thermal',thermal_events)]:
        p=next(p for p in f['input_pins'] if f'/{camera}/' in p and p.endswith('events.jsonl'))
        for line in (ROOT/p).read_text().splitlines():
            e=json.loads(line)
            if e['kind']=='motion_result_written' and e['first_write_for_submission']:
                key=(e['from_idx'],e['frame_idx']);assert key not in target;target[key]=e
    p=next(p for p in f['input_pins'] if '/thermal/' in p and p.endswith('tensor-map.npz'))
    with np.load(ROOT/p) as z:thermal={k:z[k] for k in ['edge/match2frame1/mapping','edge/match2frame2/mapping']}
    refs={int(Path(p).stem):proper(np.loadtxt(ROOT/p).reshape(4,4)) for p in f['reference_pins']}
    checks=audit(rows,scores,f,refs,events,thermal_events,thermal);mutations=[]
    for kind in ['population','state','rotation','cost','score','fallback']:
        bad=copy.deepcopy(rows);bs=copy.deepcopy(scores)
        if kind=='population':bad.pop()
        elif kind=='state':bad[1]['methods']['thermal_translation']['parent_xyzw'][0]+=1
        elif kind=='rotation':bad[0]['methods']['thermal_translation']['committed_xyzw'][3]+=.01
        elif kind=='cost':bad[0]['stages']['thermal_translation']['receipts'][1]['steps']=11
        elif kind=='fallback':bad[25]['thermal_binding']['fallback']='STALE_THERMAL'
        else:bs['summary']['thermal_translation']['path_position_m']['rmse']+=1
        try:audit(bad,bs,f,refs,events,thermal_events,thermal)
        except AssertionError:mutations.append(kind)
        else:raise AssertionError('escaped mutation '+kind)
    result={'status':'PASS','checks':checks,'mutations_rejected':mutations,'thermal_intervals':595,'fallback_intervals':4,'auditor_sha256':sha(__file__),'scores_sha256':sha(OUT/'scores.json'),'prediction_lock_sha256':sha(OUT/'prediction-lock.json')}
    with (OUT/'audit.json').open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()

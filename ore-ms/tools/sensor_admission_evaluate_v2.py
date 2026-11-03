"""Frozen EXP112 parity, fresh prefix prediction, and separate proxy scoring."""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from sensor_admission_execution_v2 import PLAN,OUT,verify,sha,read,write


def parity():
    plan=verify();reports=[];arrays=0
    for c in plan['cases']:
        base=OUT/'captures'/c['case'];a=base/'off';b=base/'on';count=0;geometry=0;writes=0
        aa={str(p.relative_to(a)):p for p in a.rglob('*.npz')};bb={str(p.relative_to(b)):p for p in b.rglob('*.npz')}
        assert aa.keys()==bb.keys(),c['case']
        for n,p in aa.items():
            with np.load(p,allow_pickle=False) as x,np.load(bb[n],allow_pickle=False) as y:
                assert set(x.files)==set(y.files),(c['case'],n)
                for key in x.files:
                    assert x[key].dtype==y[key].dtype and x[key].shape==y[key].shape,(n,key)
                    assert np.array_equal(x[key],y[key],equal_nan=True),(c['case'],n,key)
                    count+=1
        for camera in ['rgb','thermal']:
            pa=a/'experts'/camera;pb=b/'experts'/camera
            assert read(pa/'effective-config.json')==read(pb/'effective-config.json')
            for path in [pa,pb]:assert read(path/'arbiter-events/summary.json')['status']=='COMPLETE'
            ea=[json.loads(s) for s in (pa/'arbiter-events/events.jsonl').read_text().splitlines()]
            eb=[json.loads(s) for s in (pb/'arbiter-events/events.jsonl').read_text().splitlines()]
            def strip(events):return [{k:v for k,v in e.items() if k not in ['sequence','observation_monotonic_ns']} for e in events if e['kind']!='native_disparity_information']
            assert strip(ea)==strip(eb),(c['case'],camera,'native events')
            gi=[e for e in eb if e['kind']=='native_disparity_information'];first=[e for e in eb if e['kind']=='motion_result_written' and e['first_write_for_submission']]
            assert len(gi)==len(first),(c['case'],camera,len(gi),len(first))
            for g in gi:
                m,=[e for e in first if e['submission_id']==g['submission_id']]
                assert (g['from_idx'],g['frame_idx'],g['time0_ns'],g['time1_ns'])==(m['from_idx'],m['frame_idx'],m['time0_ns'],m['time1_ns'])
                assert g['sequence']<m['sequence']
            geometry+=len(gi);writes+=len(first)
        ta=[json.loads(s) for s in (a/'timeline.jsonl').read_text().splitlines()];tb=[json.loads(s) for s in (b/'timeline.jsonl').read_text().splitlines()]
        da=[(e['pair_ordinal'],e['action'],e['availability'],e['boundary']) for e in ta if e['kind']=='decision'];db=[(e['pair_ordinal'],e['action'],e['availability'],e['boundary']) for e in tb if e['kind']=='decision']
        assert da==db and len(da)==c['frames']-1
        arrays+=count;reports.append({'case':c['case'],'arrays_bitwise_equal':count,'npz_files':len(aa),'native_events_equal_without_sequence_and_clock':True,'raw_decisions_equal':True,'geometry_events':geometry,'first_native_writes':writes})
    verify();write(OUT/'preservation.json',{'status':'PASS_MATCHED_NATIVE_PRESERVATION','cases':reports,'arrays_bitwise_equal':arrays,'geometry_events':sum(r['geometry_events'] for r in reports),'inference_runs':24,'neural_frames':1120,'production_defaults_changed':False})
    print('PASS_NATIVE_PRESERVATION',arrays,'arrays',sum(r['geometry_events'] for r in reports),'geometry events')


def predict():
    from Scripts.Setup.prefix_interval_model import predict_prefix
    from Scripts.Setup.prepare_sensor_admission_contract import prepare
    from Scripts.Setup.sensor_admission_comparison import predict as policy_predict
    plan=verify();assert read(OUT/'preservation.json')['status']=='PASS_MATCHED_NATIVE_PRESERVATION'
    combined={'schema_version':1,'frame_id':'nominal_body_stationary_twist','rows':[]};allprefix=[];proofs={}
    for c in plan['cases']:
        capture=OUT/'captures'/c['case']/'on';timeline=[json.loads(s) for s in (capture/'timeline.jsonl').read_text().splitlines()]
        prefix={'rows':[dict(r,window=c['window']) for r in predict_prefix(c['rows'],timeline,c['nominal_body_from_camera'])]}
        assert len(prefix['rows'])==c['frames']-1
        static={c['window']:{'nominal_body_from_camera':c['nominal_body_from_camera']}}
        contract,proof=prepare(prefix,static,timeline,c['recording_id'],c['window'])
        combined['rows']+=contract['rows'];allprefix+=prefix['rows'];proofs[c['case']]=proof
        write(OUT/'predictions'/c['case']/'prefix.json',prefix);write(OUT/'predictions'/c['case']/'inputs.json',contract)
        write(OUT/'predictions'/c['case']/'geometry-proof.json',{'bindings':proof,'timeline_sha256':sha(capture/'timeline.jsonl')})
    assert len(combined['rows'])==274 and len({r['sample_id'] for r in combined['rows']})==274
    actions=policy_predict(combined)
    write(OUT/'admission-inputs.json',combined);write(OUT/'locked-actions.json',actions);write(OUT/'fresh-prefix-predictions.json',{'rows':allprefix})
    verify();write(OUT/'prediction-lock.json',{'status':'PASS_FRESH_LABEL_FREE_PREDICTIONS','rows':274,'admission_inputs_sha256':sha(OUT/'admission-inputs.json'),'actions_sha256':sha(OUT/'locked-actions.json'),'prefix_sha256':sha(OUT/'fresh-prefix-predictions.json'),'preservation_sha256':sha(OUT/'preservation.json'),'labels_read':False})
    print('LOCKED',274,'fresh decisions')


def score():
    from Scripts.Setup.td3_reference import load_reference
    from Scripts.Setup.evaluate_interval_model import loss
    from Scripts.Setup.sensor_admission_comparison import score as policy_score
    plan=verify();lock=read(OUT/'prediction-lock.json')
    assert lock['actions_sha256']==sha(OUT/'locked-actions.json') and lock['prefix_sha256']==sha(OUT/'fresh-prefix-predictions.json')
    prefix=read(OUT/'fresh-prefix-predictions.json')['rows'];labels={};references=[]
    for c in plan['cases']:
        reference,_=load_reference(ROOT/c['reference_dataset'])
        for r in [x for x in prefix if x['window']==c['window']]:
            start,end=r['common_interval_ns'];ref=reference.at(np.array([start,end],np.int64));delta=np.linalg.solve(ref[0],ref[1]);sid=f"{c['window']}/{r['pair_ordinal']}"
            labels[sid]={camera:loss(e['delta_model'],delta) if e['valid_model_projection'] else None for camera,e in r['experts'].items()}
            references.append({'sample_id':sid,'reference_delta':delta.tolist(),'interval_ns':[start,end]})
    write(OUT/'fresh-labels.json',labels);write(OUT/'scoring-references.json',{'rows':references})
    result=policy_score(read(OUT/'locked-actions.json'),labels);result['prediction_lock_sha256']=sha(OUT/'prediction-lock.json');result['fresh_labels_sha256']=sha(OUT/'fresh-labels.json')
    verify();write(OUT/'scores.json',result)
    print(json.dumps(result,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['parity','predict','score']);a=p.parse_args();globals()[a.action]()

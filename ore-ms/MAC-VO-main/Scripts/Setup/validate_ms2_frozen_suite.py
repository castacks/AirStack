"""Independent PNG/schema/producer-input and saved frontend audit on new recording."""
import argparse,ast,hashlib,json,struct,zlib,math
from pathlib import Path
import cv2
import numpy as np


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_source(root,base,output,declared):
    if output.exists():raise FileExistsError(output)
    dataset=base/'dataset';source=json.loads((dataset/'source-manifest.json').read_text());seq=source['sequence'];report=json.loads((dataset/'frontend-run-001/report.json').read_text());plan=json.loads((base/'frozen-models.json').read_text())
    assert seq==plan['sequence']==declared['sequence'] and report['status']=='PASS' and report['frozen_inputs_unchanged']
    assert digest(base/'frozen-models.json')==declared['model_artifact_sha256']
    assert [r['stem'] for r in source['records']]==plan['ids']
    counts={'png_chunks':0,'png_files':0,'source_members':0,'timestamp_joins':0,'producer_input_tensors':0,'saved_frontend_pairs':0,'frozen_hashes':0}
    producer=root/'results/ms2-mac-adapter/inputs/get_thermal_data.py'
    assert digest(producer)=='9e50052737bcddc97db6d5150b565352f05186ba6e1fa556b3dea1eaba58bd5c'
    functions=[n for n in ast.parse(producer.read_text()).body if isinstance(n,ast.FunctionDef) and n.name in ['process_image','enhance_image']]
    assert len(functions)==2;scope={'cv2':cv2,'np':np};exec(compile(ast.Module(body=functions,type_ignores=[]),str(producer),'exec'),scope)
    cv2.setNumThreads(1);inputs={};source_hashes={}
    for path in sorted((dataset/'pilot').rglob('*.png')):
        b=path.read_bytes();assert b[:8]==b'\x89PNG\r\n\x1a\n';at=8
        while at<len(b):
            n=struct.unpack('>I',b[at:at+4])[0];tag=b[at+4:at+8];payload=b[at+8:at+8+n];crc=struct.unpack('>I',b[at+8+n:at+12+n])[0]
            assert zlib.crc32(tag+payload)&0xffffffff==crc;counts['png_chunks']+=1;at+=n+12
            if tag==b'IEND':break
        assert at==len(b);raw=cv2.imread(str(path),cv2.IMREAD_UNCHANGED);w,h,depth,color=struct.unpack('>IIBB',b[16:26]);assert raw.shape[:2]==(h,w)
        assert (raw.dtype==np.uint16)==(depth==16);counts['png_files']+=1;source_hashes[str(path)]=digest(path)
    assert counts['png_files']==120
    for kind in ['sync_data','proj_depth']:
        for item in json.loads((dataset/(kind+'-pilot-manifest.json')).read_text()):
            path=dataset/item['path'];assert path.stat().st_size==item['bytes'] and digest(path)==item['sha256'];counts['source_members']+=1
    historical=set()
    for p in [root/'results/ms2-day-replication/source-manifest.json',root/'results/ms2-rain-replication/source-manifest.json',root/'results/ms2-timestamp-semantics/source-protocol-input-manifest.json']:
        for row in json.loads(p.read_text())['records']:
            for camera in row['cameras'].values():
                for side in ['left','right']:historical.add(camera[side]['sha256'])
    for i,row in enumerate(source['records']):
        for mod in ['rgb','thr']:
            camera=row['cameras'][mod]
            for side in ['left','right']:
                rec=camera[side];path=root/rec['path'];raw=cv2.imread(str(path),cv2.IMREAD_UNCHANGED)
                assert source_hashes[str(path)]==rec['sha256'] and rec['sha256'] not in historical
                assert list(raw.shape)==rec['shape'] and str(raw.dtype)==rec['dtype']
                timestamp=(dataset/'pilot/sync_data'/seq/mod/f'img_{side}_timestamp.txt').read_text().split()
                assert int(timestamp[i])==rec['recorded_time_ns'];counts['timestamp_joins']+=1
                if mod=='rgb':image=raw[...,::-1].copy()
                else:image=np.repeat(scope['enhance_image'](scope['process_image'](raw.copy(),'firestereo_v2'))[...,None],3,axis=2)
                tensor=(image.astype(np.float32)/np.float32(255)).transpose(2,0,1)[None]
                inputs[mod,i,side]=hashlib.sha256(tensor.tobytes()).hexdigest();counts['producer_input_tensors']+=1
            depth=root/row['references'][mod]['depth']['path'];z=cv2.imread(str(depth),cv2.IMREAD_UNCHANGED)
            assert z.dtype==np.uint16 and list(z.shape)==camera['left']['shape'][:2]
            assert row['references'][mod]['depth']['meters_per_code']==1/256 and row['references'][mod]['depth']['zero_invalid'] is True
    seen=set();repeats=[]
    for rec in report['pairs']:
        mod=rec['modality'];i,j=rec['indices'];assert rec['kind']=='stereo' and i==j and (mod,i) not in seen;seen.add((mod,i))
        assert rec['input_sha256']==[inputs[mod,i,'left'],inputs[mod,i,'right']]
        path=dataset/'frontend-run-001'/rec['file'];assert digest(path)==rec['sha256']
        with np.load(path) as data:
            for name in ['flow','covariance']:
                a=data[name];assert a.shape==(1,2,*source['records'][i]['cameras'][mod]['left']['shape'][:2]);assert a.dtype==np.float32 and np.isfinite(a).all();assert float(a.min())==rec[name+'_min'] and float(a.max())==rec[name+'_max']
            assert (data['covariance']>0).all();assert data['source_ids'].tolist()==[f'{i:06d}']*2
            times=[source['records'][i]['cameras'][mod][side]['recorded_time_ns'] for side in ['left','right']]
            assert data['input_time_ns'].tolist()==times and data['camera_times_ns'].tolist()==times*2
        counts['saved_frontend_pairs']+=1
        if 'repeat' in rec:assert rec['repeat']['flow_bitwise'] and rec['repeat']['covariance_bitwise'];repeats.append({'modality':mod,**rec['repeat']})
    assert seen=={(mod,i) for mod in ['rgb','thr'] for i in range(20)} and len(repeats)==2
    for path,h in report['frozen_inputs'].items():assert digest(path)==h;counts['frozen_hashes']+=1
    for mod,expected in [('rgb','bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a'),('thr','eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a')]:
        checkpoint=report['modalities'][mod]['checkpoint'];assert digest(checkpoint)==report['modalities'][mod]['checkpoint_sha256']==expected
    for path,h in source_hashes.items():assert digest(path)==h
    result={'status':'PASS','checks':counts,'repeats':repeats,'distinct_development_camera_hashes':len(historical),'camera_source_overlap':0,'validator_sha256':digest(__file__),'frontend_report_sha256':digest(dataset/'frontend-run-001/report.json'),'scope':'source/producer/runtime and arrays; not physical reference correctness'}
    output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))



def validate_evaluation(root,base,output,declared):
    if output.exists():raise FileExistsError(output)
    frozen=base/'frozen-models.json';assert digest(frozen)==declared['model_artifact_sha256']
    plan=json.loads(frozen.read_text());evaluation=base/'frozen-evaluation';r=json.loads((evaluation/'report.json').read_text());protocol=json.loads((evaluation/'protocol.json').read_text());raw=json.loads((base/'metric/report.json').read_text())
    assert protocol['frozen_model_sha256']==digest(frozen) and protocol['no_test_fit'] is True
    assert r['sequence']==plan['sequence']==raw['protocol']['sequence'] and r['primary']=='uncertainty'
    families={'constant':[],'uncertainty':[0,1],'geometry':[2,3,4,5],'combined':list(range(6))}
    checks={'hashes':0,'feature_values':0,'margin_values':0,'choice_values':0,'risk_fields':0,'summary_fields':0,'historical_fields':0,'availability_fields':0}
    minmargin=math.inf
    def close(a,b,category):
        aa,bb=np.asarray(a),np.asarray(b);assert aa.shape==bb.shape and np.allclose(aa,bb,rtol=1e-10,atol=1e-10),(category,a,b)
        checks[category]+=int(aa.size)
    for source in [plan['frozen_development_inputs'],protocol['frozen_input_sha256']]:
        for path,h in source.items():assert digest(path)==h;checks['hashes']+=1
    for name,h in plan['source_sha256'].items():assert digest(root/'MAC-VO-main/Scripts/Setup'/(name+'.py'))==h
    assert len(r['rows'])==20
    for i,(row,old) in enumerate(zip(r['rows'],raw['rows'],strict=True)):
        stem=f'{i:06d}';assert row['stem']==old['stem']==plan['ids'][i]==stem
        assert row['availability']=={k:old[k] for k in row['availability']};checks['availability_fields']+=len(row['availability'])
        with np.load(base/'metric'/f'metric-{stem}.npz') as f:a={k:f[k] for k in f.files}
        with np.load(base/'support'/f'pairs-{stem}.npz') as f:p={k:f[k] for k in f.files}
        with np.load(evaluation/f'selection-{stem}.npz') as f:saved={k:f[k] for k in f.files}
        mask=a['both_valid_mask'];n=int(mask.sum());xy=p['rgb_xy'][mask];rgb=a['rgb_predicted_xyz'][mask];thr=a['thr_predicted_xyz_in_rgb'][mask]
        features=np.column_stack([np.log(a['rgb_metric_variance'][mask]),np.log(a['thr_metric_variance'][mask]),
                                 .5*np.log(np.einsum('ij,ij->i',rgb,rgb)),.5*np.log(np.einsum('ij,ij->i',thr,thr)),xy[:,0]/1224.,xy[:,1]/384.])
        close(features,saved['features'],'feature_values');choices={}
        for family,indices in families.items():
            model=plan['models'][family];assert model['feature_indices']==indices and model['ridge']==.01
            margin=np.full(n,model['theta'][0])
            for j,col in enumerate(indices):margin+=model['theta'][j+1]*(features[:,col]-model['mean'][j])/model['scale'][j]
            close(margin,saved[family+'_margin'],'margin_values')
            choice=(margin>0).astype(float)+.5*(margin==0);assert np.array_equal(choice,saved[family+'_choice']);checks['choice_values']+=n;choices[family]=choice
            if n:minmargin=min(minmargin,float(np.abs(margin).min()))
        scalar=plan['scalar_fit']['policy']
        if scalar['kind']=='always_rgb':s=np.ones(n)
        elif scalar['kind']=='always_thr':s=np.zeros(n)
        else:
            q=a['rgb_metric_variance'][mask]*10**scalar['log10_c'];t=a['thr_metric_variance'][mask];s=(q<t).astype(float)+.5*(q==t)
        fixed=plan['scalar_fit']['training_fixed_policy']['kind']
        choices.update({'original':a['learned_propagated'],'camera_scale':a['camera_scale'],'scalar_fit':s,'always_rgb':np.ones(n),'always_thr':np.zeros(n),'random':np.full(n,.5),'training_fixed':np.full(n,1. if fixed=='always_rgb' else 0.)})
        assert np.array_equal(choices['constant'],choices['training_fixed'])
        if not n:
            assert all(v is None for v in row['rgb_choice_fractions'].values());assert all(v is None for losses in row['risks'].values() for risks in losses.values() for v in risks.values());continue
        for name,s in choices.items():close(s.mean(),row['rgb_choice_fractions'][name],'risk_fields')
        for anchor in ['midpoint','rgb','thr']:
            for loss,power in [('mean_l2',1),('mse',2)]:
                rr,tt=a[f'{anchor}_rgb_error_m']**power,a[f'{anchor}_thr_error_m']**power;risks=row['risks'][anchor][loss]
                oracle=math.fsum(min(x,y) for x,y in zip(rr,tt,strict=True))/n;close(oracle,risks['oracle'],'risk_fields')
                for name,s in choices.items():
                    value=math.fsum(float(c)*float(x)+(1-float(c))*float(y) for c,x,y in zip(s,rr,tt,strict=True))/n
                    close(value,risks[name],'risk_fields');assert value>=oracle-1e-9
                for name,original in [('original','learned_propagated'),('camera_scale','camera_scale')]:close(risks[name],old['scores'][anchor][loss][original]['uncertainty'],'historical_fields')
                for name,oldkey in [('always_rgb','rgb'),('always_thr','thr'),('random','random'),('oracle','oracle')]:close(risks[name],old['scores'][anchor][loss]['learned_propagated'][oldkey],'historical_fields')
    good=[row for row in r['rows'] if row['availability']['both_valid']>0]
    assert r['available_frames']==len(good) and r['unavailable_stems']==[row['stem'] for row in r['rows'] if not row['availability']['both_valid']]
    if good:
        for anchor,losses in r['summary'].items():
            for loss,models in losses.items():
                for name,comparisons in models.items():
                    for comparator,stats in comparisons.items():
                        ratios=sorted(row['risks'][anchor][loss][name]/row['risks'][anchor][loss][comparator] for row in good);n=len(ratios)
                        median=ratios[n//2] if n%2 else (ratios[n//2-1]+ratios[n//2])/2
                        for key,value in [('min',ratios[0]),('median',median),('max',ratios[-1]),('below_one',sum(x<1 for x in ratios))]:close(value,stats[key],'summary_fields')
    else:assert r['summary'] is None
    result={'status':'PASS','checks':checks,'available_frames':len(good),'minimum_absolute_margin':minmargin if good else None,
            'validator_sha256':digest(__file__),'frozen_model_sha256':digest(frozen),'evaluation_report_sha256':digest(evaluation/'report.json'),
            'scope':'fixed model/feature/selection risks and unchanged development inputs; no new fit or physical-label certification'}
    output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))



if __name__=='__main__':
    from ms2_frozen_suite import declared as declared_plan
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['source','evaluation']);p.add_argument('--root',type=Path,required=True);p.add_argument('--suite',type=Path,required=True);p.add_argument('--condition',choices=['day','night','rainy'],required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();protocol,record,base=declared_plan(a.root.resolve(),a.suite.resolve(),a.condition)
    if a.stage=='source':
        parent=a.root/'results/ms2-unexamined-recording/dataset/frontend-run-001'
        current=base/'dataset/frontend-run-001'
        assert json.loads((parent/'report.json').read_text())['runtime']==json.loads((current/'report.json').read_text())['runtime']
        assert (parent/'effective-config.yaml').read_bytes()==(current/'effective-config.yaml').read_bytes()
        validate_source(a.root.resolve(),base,a.output,record)
    else:validate_evaluation(a.root.resolve(),base,a.output,record)

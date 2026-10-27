"""Independent PNG/schema/producer-input and saved frontend audit on new recording."""
import argparse,ast,hashlib,json,struct,zlib
from pathlib import Path
import cv2
import numpy as np


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate(root,base,output):
    if output.exists():raise FileExistsError(output)
    dataset=base/'dataset';source=json.loads((dataset/'source-manifest.json').read_text());seq=source['sequence'];report=json.loads((dataset/'frontend-run-001/report.json').read_text());plan=json.loads((base/'frozen-models.json').read_text())
    assert seq==plan['sequence']=='_2021-08-06-16-45-28' and report['status']=='PASS' and report['frozen_inputs_unchanged']
    assert digest(base/'frozen-models.json')=='51b2b9a15d1951c2902e6ef8a0a5e45dcae3d1713d428e36d9545b530e0ae377'
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


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();validate(a.root,a.base,a.output)

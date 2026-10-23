"""Exploratory source-image expansion around a previously identified lit interval."""
import argparse,hashlib,importlib.util,json,shutil,zipfile
from datetime import datetime,timezone
from pathlib import Path

NAME='night_macthermal_salon_redcourse_waypointskip1'

def sha(path):return hashlib.file_digest(Path(path).open('rb'),'sha256').hexdigest()
def save(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def load_helpers():
    path=Path(__file__).with_name('td3_batch_screen.py');spec=importlib.util.spec_from_file_location('batch',path);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);return mod

def nearest_row(row,others):
    import bisect
    times=[r['time_ns'] for r in others];j=bisect.bisect_left(times,row['time_ns']);candidates=[i for i in (j-1,j) if 0<=i<len(times)]
    if not candidates:raise ValueError('Missing target timeline')
    k=min(candidates,key=lambda k:(abs(times[k]-row['time_ns']),k));return others[k]

def prepare(root,output,config):
    helper=load_helpers();old=root/'results/night-batch-screen';prior=old/NAME
    source=json.loads((prior/'plan.json').read_text());streams=source['streams'];rgb=streams['rgb_left']['rows'];original=streams['rgb_left']['windows'][2]['rows']
    start=next(i for i,r in enumerate(rgb) if r==original[0]);assert original==rgb[start:start+20] and start>=100 and start+120<=len(rgb)
    chosen=rgb[start-100:start+120];assert len(chosen)==220
    frozen={str(p):sha(p) for p in old.rglob('*') if p.is_file() and 'range-cache' not in p.parts and '__pycache__' not in p.parts}
    frozen.update({str(Path(__file__)):sha(__file__),str(Path(__file__).with_name('td3_batch_screen.py')):sha(Path(__file__).with_name('td3_batch_screen.py'))})
    output.mkdir(parents=True,exist_ok=False)
    pairs=[]
    for left in chosen:
        right=nearest_row(left,streams['rgb_right']['rows']);thermal=nearest_row(left,streams['thermal_left']['rows']);thermal_right=nearest_row(thermal,streams['thermal_right']['rows'])
        pairs.append(dict(rgb_left=left,rgb_right=right,thermal_left=thermal,thermal_right=thermal_right,
            rgb_stereo_ns=right['time_ns']-left['time_ns'],thermal_stereo_ns=thermal_right['time_ns']-thermal['time_ns'],cross_ns=thermal['time_ns']-left['time_ns']))
    groups=[dict(name='before',start=0,stop=100),dict(name='original',start=100,stop=120),dict(name='after',start=120,stop=220)]
    blocks=[dict(name=f'before-{i}',start=i*20,stop=(i+1)*20) for i in range(5)]+[dict(name='original',start=100,stop=120)]+[dict(name=f'after-{i}',start=120+i*20,stop=140+i*20) for i in range(5)]
    plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='51f87a539',source=source['source'],rows=pairs,groups=groups,blocks=blocks,prior_sha256=frozen,
        protocol=dict(scope='Outcome-selected exploratory follow-up, no model or VO inference',preceding=100,original=20,following=100,stereo_limit_ns=2000000,cross_limit_ns=30000000,diagnostic_cross_limit_ns=50000000,no_timestamp_correction=True,no_pair_reuse_in_eligible_runs=True))
    save(output/'plan.json',plan)
    selected={r[key]['path'] for r in pairs for key in ['rgb_left','rgb_right','thermal_left','thermal_right']}
    base=['rclone','--config',config,'--box-root-folder-id','428559630945']
    reader=helper.RemoteZIP([*base,'cat','ore_ms:'+NAME+'.zip','--contimeout','15s','--timeout','90s','--retries','2'],source['source']['Size'],prior/'range-cache')
    records={}
    with zipfile.ZipFile(reader) as archive:
        infos={rel:archive.getinfo(NAME+'/'+rel) for rel in selected}
        missing=[i for rel,i in infos.items() if not (prior/'source'/rel).exists()];reader.prefetch(missing)
        for rel in sorted(selected):
            target=output/'source'/rel;target.parent.mkdir(parents=True,exist_ok=True);previous=prior/'source'/rel
            if previous.exists():
                import zlib
                data=previous.read_bytes();assert len(data)==infos[rel].file_size and zlib.crc32(data)&0xffffffff==infos[rel].CRC
            else:data=archive.read(infos[rel])
            target.write_bytes(data);records[rel]=dict(sha256=hashlib.sha256(data).hexdigest(),bytes=len(data),crc=infos[rel].CRC)
    # Save input timestamp files so association is replayable without previous ignored data.
    for key,stream in helper.STREAMS.items():
        rel=f'sensors/{stream}/timestamps.txt';target=output/'source'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(prior/'source'/rel,target)
        records[rel]=dict(sha256=sha(target),bytes=target.stat().st_size)
    save(output/'extraction.json',dict(created_at_utc=datetime.now(timezone.utc).isoformat(),files=records,ranges=reader.requests,whole_archive_hash_verified=False))
    for path,digest in frozen.items():assert sha(path)==digest
    print('Selected',len(pairs),'RGB acquisitions;',len(selected),'unique images; previous hashes unchanged',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--config',required=True);a=p.parse_args();prepare(a.root,a.output,a.config)

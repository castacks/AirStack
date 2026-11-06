"""Strict full-record transport/extraction for the locked two-record test."""
from pathlib import Path
import concurrent.futures,hashlib,json,re,tarfile,time,urllib.request
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/decisive-uncertainty-evaluation/run-001'
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def request(url,start,end,total=None):
 error=None
 for attempt in range(4):
  try:
   req=urllib.request.Request(url,headers={'Range':f'bytes={start}-{end}','User-Agent':'ore-ms-locked-evaluation-intake'})
   with urllib.request.urlopen(req,timeout=90) as r:
    match=re.fullmatch(r'bytes (\d+)-(\d+)/(\d+)',r.headers.get('Content-Range',''));data=r.read(end-start+2)
    if r.status!=206 or not match or tuple(map(int,match.groups()[:2]))!=(start,end) or len(data)!=end-start+1:raise ValueError('strict HTTP206 range/length')
    size=int(match.group(3))
    if total is not None and size!=total:raise ValueError('archive length changed')
    return data,size
  except Exception as e:error=e;time.sleep(min(2**attempt,8))
 raise RuntimeError('transport retries exhausted: '+str(error))
def download(url,directory,cap):
 directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);first,total=request(url,0,3)
 if first[:3]!=b'BZh' or total>cap or total<=0:raise ValueError('archive schema/size budget')
 block=64*1024**2
 def fetch(start):
  end=min(start+block,total)-1;f=directory/f'{start}-{end}.bin'
  if f.exists():
   m=read(f.with_suffix('.json'));assert f.stat().st_size==end-start+1 and sha(f)==m['sha256'];return
  data,size=request(url,start,end,total);f.write_bytes(data);f.with_suffix('.json').write_text(json.dumps(dict(url=url,start=start,end=end,total=size,sha256=hashlib.sha256(data).hexdigest()),indent=2)+'\n');print('RANGE',directory.name,start,end,flush=True)
 with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:list(pool.map(fetch,range(0,total,block)))
 archive=directory/'archive.tar.bz2';digest=hashlib.sha256()
 with archive.open('xb') as output:
  for start in range(0,total,block):
   f=directory/f'{start}-{min(start+block,total)-1}.bin'
   with f.open('rb') as inp:
    for b in iter(lambda:inp.read(8*1024**2),b''):digest.update(b);output.write(b)
 assert archive.stat().st_size==total
 write(directory/'archive.json',dict(url=url,bytes=total,observed_sha256=digest.hexdigest(),publisher_checksum_verified=False,range_count=(total+block-1)//block));return archive

def images(case,maximum_bytes):
 import indexed_bzip2
 import numpy as np
 from full_record_intake import png
 import sys
 sys.path.insert(0,str(ROOT/'MAC-VO-main'))
 from Scripts.Setup.audit_ms2_pilot import read_calibration
 from Scripts.Setup.ms2_execution_adapter import prepare as paired
 rec=case['condition'];seq=case['sequence'];archive=download(case['sync_url'],OUT/'transport'/rec,maximum_bytes);raw=OUT/'raw';members=[];sets={(m,s):set() for m in ['rgb','thr'] for s in ['left','right']};times={};calpath=None
 with indexed_bzip2.open(str(archive),parallelization=2) as stream,tarfile.open(fileobj=stream,mode='r|') as tar:
  for item in tar:
   name=item.name.removeprefix('./');parts=Path(name).parts
   if Path(name).is_absolute() or '..' in parts or item.issym() or item.islnk():raise ValueError('unsafe TAR member')
   if item.isdir():continue
   image=re.fullmatch(re.escape(seq)+r'/(rgb|thr)/img_(left|right)/(\d{6})\.png',name);metadata=re.fullmatch(re.escape(seq)+r'/(rgb|thr)/img_(left|right)_timestamp\.txt',name);cal=name==seq+'/calib.npy'
   if not(image or metadata or cal):continue
   if not item.isfile() or item.size<=0 or item.size>(8*1024**2 if image else 4*1024**2):raise ValueError('selected member schema/size')
   dest=raw/name;dest.parent.mkdir(parents=True,exist_ok=True);data=tar.extractfile(item).read();assert len(data)==item.size
   if dest.exists():raise ValueError('duplicate selected source member')
   dest.write_bytes(data);entry=dict(path=str(dest.relative_to(ROOT)),sha256=hashlib.sha256(data).hexdigest(),bytes=len(data),tar_name=name,tar_offset=item.offset_data)
   if image:
    camera,side,stem=image.groups();schema=dict(shape=[384,1224,3] if camera=='rgb' else [256,640],bit_depth=8 if camera=='rgb' else 16,color_type=2 if camera=='rgb' else 0);png(data,schema);sets[camera,side].add(stem);entry.update(camera=camera,side=side,stem=stem,shape=schema['shape'],dtype='uint8' if camera=='rgb' else 'uint16')
   elif metadata:
    camera,side=metadata.groups();values=[int(v) for v in data.decode().split()];assert all(type(v) is int and v>=0 for v in values) and all(a<b for a,b in zip(values,values[1:]));times[camera,side]=values
   else:calpath=dest
   members.append(entry)
   if len(members)%1000==0:print('EXTRACT',rec,len(members),'selected members',flush=True)
 assert calpath and len(times)==4;count=len(times['rgb','left']);assert count>=2 and all(len(t)==count for t in times.values());stems=[f'{i:06d}' for i in range(count)];assert all(v==set(stems) for v in sets.values())
 cal=read_calibration(calpath);caseout=OUT/'inputs'/rec;caseout.mkdir(parents=True,exist_ok=True);write(caseout/'calibration.json',cal)
 bypath={m['path']:m for m in members};records=[]
 for i,stem in enumerate(stems):
  row=dict(stem=stem,cameras={})
  for camera in ['rgb','thr']:
   k=cal['K_'+camera+'L']['values'];baseline=-float(np.asarray(cal['T_'+camera+'R']['values']).reshape(3)[0])*.001;assert baseline>0 and np.array_equal(cal['R_'+camera+'L']['values'],np.eye(3))
   r=dict(K=k,baseline_m=baseline)
   for side in ['left','right']:
    path=str((raw/seq/camera/('img_'+side)/(stem+'.png')).relative_to(ROOT));m=bypath[path];r[side]={key:m[key] for key in ['path','sha256','shape','dtype']};r[side]['recorded_time_ns']=times[camera,side][i]
   row['cameras'][camera]=r
  records.append(row)
 manifest=caseout/'source-manifest.json';write(manifest,dict(sequence=seq,records=records,scope='locked full-record qualified project evaluation; no reference payload in native inputs'))
 paired(ROOT,manifest,caseout/'paired-manifest.json');write(caseout/'intake.json',dict(source_groups=count,selected_members=members,archive=read(archive.parent/'archive.json'),source_manifest_sha256=sha(manifest),paired_manifest_sha256=sha(caseout/'paired-manifest.json'),new_reference_values_read=False,full_original_axis=True))
 print('INPUT_READY',rec,count,'groups',flush=True)

def references(case):
 import indexed_bzip2
 rec=case['condition'];seq=case['sequence'];source=read(OUT/'inputs'/rec/'source-manifest.json');stems={r['stem'] for r in source['records']};archive=download(case['odom_url'],OUT/'reference-transport'/rec,64*1024**2);members=[]
 with indexed_bzip2.open(str(archive),parallelization=2) as stream,tarfile.open(fileobj=stream,mode='r|') as tar:
  for item in tar:
   name=item.name.removeprefix('./')
   if Path(name).is_absolute() or '..' in Path(name).parts or item.issym() or item.islnk():raise ValueError('unsafe reference TAR')
   match=re.fullmatch(re.escape(seq)+r'/rgb/(\d{6})\.txt',name)
   if not match:continue
   if match.group(1) not in stems or not item.isfile() or item.size>4096:raise ValueError('reference source-ID/schema mismatch')
   file=OUT/'references'/rec/(match.group(1)+'.txt');file.parent.mkdir(parents=True,exist_ok=True);data=tar.extractfile(item).read()
   with file.open('xb') as f:f.write(data)
   members.append(dict(stem=match.group(1),path=str(file.relative_to(ROOT)),sha256=hashlib.sha256(data).hexdigest()))
 assert {v['stem'] for v in members}==stems
 write(OUT/'references'/rec/'reference-lock.json',dict(members=members,archive=read(archive.parent/'archive.json')))

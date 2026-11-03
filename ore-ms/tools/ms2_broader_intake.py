"""Frozen, bounded MS² archive intake. Run in Docker with isolated indexed_bzip2."""
from pathlib import Path
import argparse, bisect, concurrent.futures, hashlib, io, json, tarfile, time, urllib.request
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/ms2-broader-range/run-001'
PLAN=ROOT/'experiments/ms2-broader-range-2026-11-02.json'
RECORDS=[('day','day-replication','_2021-08-06-11-23-45'),('night','night-pilot','_2021-08-13-21-18-04'),('rain','rain-replication','_2021-08-06-16-19-00')]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,x):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(x,f,indent=2,allow_nan=False);f.write('\n')
def freeze():
 assert not PLAN.exists();cases=[];pins={}
 for label,old,seq in RECORDS:
  base=ROOT/'results'/('ms2-'+old);source=base/'pilot/sync_data'/seq
  times={f'{c}_{s}':[int(v) for v in (source/c/f'img_{s}_timestamp.txt').read_text().split()] for c in ['rgb','thr'] for s in ['left','right']}
  n=len(times['rgb_left']);assert all(len(t)==n and all(a<b for a,b in zip(t,t[1:])) for t in times.values())
  archive=next(a for a in read(base/'archive-manifest.json') if a['kind']=='sync_data')
  url=next(a['url'] for a in read(ROOT/'results/public-data-audit/ms2-access.json') if a['kind']=='sync_data' and a['sequence']==seq)
  for f in [base/'sync_data-tar-index.json',base/'archives/sync_data-block-offsets.json',source/'calib.npy']+list(source.glob('*/img_*timestamp.txt')):pins[str(f.relative_to(ROOT))]=sha(f)
  for window,start in zip(['early','middle','late'],[1,1+n//3,1+2*n//3]):
   assert start+100<=n
   cases.append({'case':label+'-'+window,'recording':label,'sequence':seq,'source_groups':n,'start':start,'frames':100,'stems':[f'{i:06d}' for i in range(start,start+100)],'source_base':str(base.relative_to(ROOT)),'archive_url':url,'archive_bytes':archive['bytes'],'historical_full_archive_sha256':archive['sha256'],'recorded_times_ns':{k:v[start:start+100] for k,v in times.items()}})
 pins['docs/research/ms2-broader-range-protocol-2026-11-02.md']=sha(ROOT/'docs/research/ms2-broader-range-protocol-2026-11-02.md')
 plan={'parent_commit':'1a86a6f0b','cases':cases,'metadata_sha256':pins,'new_frames':900,'camera_frames':1800,'reserve_access':False,'sampling_used_pixels_or_errors':False,'runtime_image':'sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09'}
 write(PLAN,plan);write(OUT/'sampling-lock.json',plan);print('FROZEN',[(c['case'],c['start'],c['source_groups']) for c in cases],flush=True)
class RangeReader(io.RawIOBase):
 def __init__(self,url,size,cache):
  self.url,self.length,self.cache,self.pos=url,size,Path(cache),0;self.cache.mkdir(parents=True,exist_ok=True);self.block=4*1024*1024
 def readable(self):return True
 def seekable(self):return True
 def tell(self):return self.pos
 def seek(self,offset,whence=0):
  p=offset+(self.pos if whence==1 else self.length if whence==2 else 0)
  if p<0:raise ValueError('negative seek')
  self.pos=p;return p
 def read(self,size=-1):
  if size<0 or size>64*1024*1024:raise ValueError('unbounded compressed read refused')
  end=min(self.pos+size,self.length);pieces=[]
  while self.pos<end:
   i=self.pos//self.block;start=i*self.block;stop=min(start+self.block,self.length)-1;path=self.cache/f'{start}-{stop}.bin'
   if not path.exists():
    for attempt in range(4):
     try:
      req=urllib.request.Request(self.url,headers={'Range':f'bytes={start}-{stop}','User-Agent':'ore-ms-bounded-intake/1'})
      with urllib.request.urlopen(req,timeout=90) as r:
       assert r.status==206 and r.headers['Content-Range']==f'bytes {start}-{stop}/{self.length}',dict(r.headers)
       data=r.read(stop-start+2);assert len(data)==stop-start+1
       meta={'start':start,'end':stop,'archive_bytes':self.length,'status':r.status,'content_range':r.headers['Content-Range'],'sha256':hashlib.sha256(data).hexdigest(),'url':self.url}
      path.write_bytes(data);write(path.with_suffix('.json'),meta);break
     except Exception:
      if attempt==3:raise
      time.sleep(2**attempt)
   data=path.read_bytes();meta=read(path.with_suffix('.json'));assert len(data)==stop-start+1 and hashlib.sha256(data).hexdigest()==meta['sha256']
   take=min(end-self.pos,len(data)-(self.pos-start));pieces.append(data[self.pos-start:self.pos-start+take]);self.pos+=take
  return b''.join(pieces)
 def readinto(self,b):
  d=self.read(len(b));b[:len(d)]=d;return len(d)
def acquire(recording):
 import indexed_bzip2
 plan=read(PLAN);assert sha(PLAN)==sha(OUT/'sampling-lock.json')
 for rel,h in plan['metadata_sha256'].items():assert sha(ROOT/rel)==h
 cases=[c for c in plan['cases'] if c['recording']==recording];assert len(cases)==3
 first=cases[0];base=ROOT/first['source_base'];seq=first['sequence'];dest=OUT/'source'/seq;dest.mkdir(parents=True,exist_ok=True)
 index={m['name']:m for m in read(base/'sync_data-tar-index.json') if m['regular']}
 names={f'{seq}/{c}/img_{s}/{stem}.png' for case in cases for c in ['rgb','thr'] for s in ['left','right'] for stem in case['stems']}
 selected=sorted([index[n] for n in names],key=lambda m:m['offset_data']);reader=RangeReader(first['archive_url'],first['archive_bytes'],OUT/'compressed-ranges'/recording);members=[];overlap=0
 offsets={int(k):v for k,v in read(base/'archives/sync_data-block-offsets.json').items()}
 # Pre-fetch independently addressed compressed chunks; decoder still validates TAR
 # headers and can fetch any additional required chunk through the same strict reader.
 ordered=sorted((v,k) for k,v in offsets.items());uncompressed=[v for v,k in ordered];chunks=set()
 for m in selected:
  a=max(0,bisect.bisect_right(uncompressed,m['offset_data']-512)-1);b=min(len(ordered)-1,bisect.bisect_left(uncompressed,m['offset_data']+m['size'])+1)
  chunks.update(range((ordered[a][1]//8)//reader.block,((ordered[b][1]+7)//8)//reader.block+1))
 def fetch(chunk):
  local=RangeReader(reader.url,reader.length,reader.cache);local.seek(chunk*local.block);local.read(min(local.block,local.length-local.tell()))
 pending=[i for i in sorted(chunks) if not (reader.cache/f'{i*reader.block}-{min((i+1)*reader.block,reader.length)-1}.bin').exists()]
 print(recording,'prefetch',len(pending),'of',len(chunks),'compressed chunks',flush=True)
 with concurrent.futures.ThreadPoolExecutor(max_workers=12) as pool:
  for n,_ in enumerate(pool.map(fetch,pending)):
   if n%100==0:print(recording,'prefetched',n,'/',len(pending),flush=True)
 with indexed_bzip2.open(reader,parallelization=1) as stream:
  stream.set_block_offsets(offsets)
  for i,m in enumerate(selected):
   stream.seek(m['offset_data']-512);header=stream.read(512);info=tarfile.TarInfo.frombuf(header,'utf-8','strict');assert info.name==m['name'] and info.size==m['size'] and info.isfile(),(info.name,m)
   data=stream.read(m['size']);assert len(data)==m['size'];target=OUT/'source'/m['name'];target.parent.mkdir(parents=True,exist_ok=True)
   if target.exists():assert target.read_bytes()==data
   else:target.write_bytes(data)
   old=base/'pilot/sync_data'/m['name'];same=None
   if old.exists():assert old.read_bytes()==data;overlap+=1;same=sha(old)
   members.append({'path':str(target.relative_to(ROOT)),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),'tar_offset_data':m['offset_data'],'archived_overlap_sha256':same})
   if i%100==0:print(recording,i,'/',len(selected),'members',flush=True)
 write(OUT/f'{recording}-acquisition.json',{'members':members,'archived_overlap_members_equal':overlap,'selected_member_bytes':sum(x['bytes'] for x in members),'range_bytes':sum(p.stat().st_size for p in reader.cache.glob('*.bin')),'current_full_archive_sha256_verified':False,'tar_headers_validated':True,'bzip2_block_index_sha256':sha(base/'archives/sync_data-block-offsets.json')});print('ACQUIRED',recording,len(members),'overlap',overlap,flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['freeze','acquire']);p.add_argument('--recording',choices=['day','night','rain']);a=p.parse_args()
 if a.action=='freeze':freeze()
 else:acquire(a.recording)

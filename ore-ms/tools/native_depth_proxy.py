"""EXP147 conditional native stereo depth / single-scan LiDAR projection audit."""
import argparse,bisect,concurrent.futures,hashlib,json,math,struct,tarfile,threading,urllib.request,zlib
from pathlib import Path
import cv2
import numpy as np
from ms2_broader_intake import RangeReader
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/native-depth-proxy/run-001';PLAN=ROOT/'experiments/native-depth-proxy-2026-11-05.json';NATIVE=ROOT/'results/native-stereo-history/run-002'
PARAMS=dict(indices=list(range(0,2000,100)),depth_product='depth',units_per_metre=256,rounding='floor(x+0.5)',compressed_budget_bytes=1024**3,prefetch_workers=4)

def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')

def prepare():
 source=read(NATIVE/'source-manifest.json');rows=source['records'];selected=[]
 for mode,m in [('rgb','rgb'),('thermal','thr')]:
  d=NATIVE/'captures'/(mode+'-paired');native=[]
  with (d/'native.jsonl.stereo.jsonl').open() as f:
   for line in f:
    index=int(line.split(',',1)[0].split(':')[1])
    if index in PARAMS['indices']:native.append(json.loads(line))
  if [r['index'] for r in native]!=PARAMS['indices']:raise ValueError('fixed native sample')
  for row in native:
   item=rows[row['index']]['cameras'][m];selected.append(dict(modality=mode,camera=m,stem=rows[row['index']]['stem'],index=row['index'],shape=item['left']['shape'][:2],K=item['K'],baseline_m=item['baseline_m'],native=row))
 write(OUT/'native-selected.json',dict(sequence=source['sequence'],records=selected))
 # Save primary source bytes before freezing. No depth values decoded here.
 url='https://sites.google.com/view/multi-spectral-stereo-dataset/data-format'
 with urllib.request.urlopen(url,timeout=60) as f:data=f.read()
 dest=OUT/'source/data-format.html';dest.parent.mkdir(parents=True);dest.write_bytes(data)
 write(OUT/'source/qualification.json',dict(url=url,source_sha256=sha(dest),reference_modality='single-scan VLP16 projected to left camera',format='uint16 metric depth times256; divide by256',independent_of_native_stereo_matcher=True,physical_projection_and_time_qualified=False,unknowns=['exposure association','projection/occlusion error','applicable physical calibration','per-point reference uncertainty'],network_filtered_reference_used=False,merged_reference_used=False))
 print('Prepared40native samples; source evidence saved; no depth decoded')

def freeze():
 base=ROOT/'results/ms2-night-pilot';metadata=[base/'archive-manifest.json',base/'proj_depth-tar-index.json',base/'archives/proj_depth-block-offsets.json',ROOT/'results/public-data-audit/ms2-access.json',OUT/'native-selected.json',OUT/'source/data-format.html',OUT/'source/qualification.json',Path(__file__),ROOT/'tools/native_depth_proxy_audit.py',ROOT/'tools/tests/test_native_depth_proxy.py',ROOT/'tools/ms2_broader_intake.py',ROOT/'tools/full_record_intake.py',ROOT/'docs/research/native-depth-proxy-protocol-2026-11-05.md',NATIVE/'source-manifest.json']
 metadata+=list((ROOT/'results/bounded-full-record/source-001/wheels').glob('*.whl'))
 for mode in ['rgb','thermal']:metadata.append(NATIVE/'captures'/(mode+'-paired')/'native.jsonl.stereo.jsonl')
 source=read(OUT/'native-selected.json');seq=source['sequence'];archive=next(x for x in read(base/'archive-manifest.json') if x['kind']=='proj_depth');url=next(x['url'] for x in read(ROOT/'results/public-data-audit/ms2-access.json') if x['sequence']==seq and x['kind']=='proj_depth');index={x['name']:x for x in read(base/'proj_depth-tar-index.json') if x['regular']};members=[]
 for record in source['records']:
  name=seq+'/'+record['camera']+'/depth/'+record['stem']+'.png';x=index[name];members.append(dict(modality=record['modality'],index=record['index'],shape=record['shape'],**x))
  old=base/'pilot/proj_depth'/name
  if old.exists():metadata.append(old)
 pins={str(f.relative_to(ROOT)):sha(f) for f in metadata}
 p=dict(parent='9fb25b7b4b1a55328bce7cb77ad25118831ca79d',parameters=PARAMS,members=members,archive_url=url,archive_bytes=archive['bytes'],historical_full_archive_sha256=archive['sha256'],source_sha256=pins,runtime='sha256:d3cf8a41dd8090c8939cd840b4735d526cec9a54e48ac36d506f91d13ee56e1c',physical_reference_qualified=False,physical_sync_verified=False,benchmark_execution_ready=False,scale_fitted=False,time_fitted=False)
 write(PLAN,p);write(OUT/'study-lock.json',p)
 write(ROOT/'experiments/recording-exposure-ledger-2026-11-05-native-depth-proxy.json',dict(recording='ms2/'+seq,development=True,reference_product='single-scan projected depth',source_indices=PARAMS['indices'],camera_members=40,reserves_unchanged=True,active_test=[],active_validation=[],new_reference_values_read=False,study_sha256=sha(PLAN)))
 print('FROZEN',len(pins),'pins;40depth members')

def verify():
 p=read(PLAN)
 if p['parameters']!=PARAMS or sha(PLAN)!=sha(OUT/'study-lock.json'):raise ValueError('study lock/parameters')
 for n,h in p['source_sha256'].items():
  if sha(ROOT/n)!=h:raise ValueError('changed source '+n)
 return p

class BudgetReader(RangeReader):
 def __init__(self,url,size,cache,shared):super().__init__(url,size,cache);self.shared=shared
 def read(self,size=-1):
  if size<0:raise ValueError('bounded compressed request required')
  chunks=set(range(self.pos//self.block,(min(self.pos+size,self.length)+self.block-1)//self.block))
  with self.shared[1]:
   self.shared[0].update(chunks)
   if sum(min(self.block,self.length-i*self.block) for i in self.shared[0])>PARAMS['compressed_budget_bytes']:raise ValueError('fixed compressed intake budget exceeded')
  return super().read(size)

def png_schema(data,shape):
 if data[:8]!=b'\x89PNG\r\n\x1a\n':raise ValueError('PNG magic')
 pos=8;seen=[]
 while pos<len(data):
  size=int.from_bytes(data[pos:pos+4],'big');kind=data[pos+4:pos+8];body=data[pos+8:pos+8+size];crc=int.from_bytes(data[pos+8+size:pos+12+size],'big')
  if len(body)!=size or zlib.crc32(kind+body)&0xffffffff!=crc:raise ValueError('PNG CRC/length')
  seen.append(kind)
  if kind==b'IHDR':
   w,h,depth,color,compression,filtering,interlace=struct.unpack('>IIBBBBB',body)
   if [h,w]!=shape or depth!=16 or color!=0 or compression or filtering or interlace:raise ValueError('native uint16 depth PNG schema')
  pos+=12+size
 if pos!=len(data) or seen[0]!=b'IHDR' or seen[-1]!=b'IEND':raise ValueError('PNG extent')

def acquire():
 import indexed_bzip2
 p=verify();cache=OUT/'compressed-ranges';offsets={int(k):v for k,v in read(ROOT/'results/ms2-night-pilot/archives/proj_depth-block-offsets.json').items()};ordered=sorted((v,k) for k,v in offsets.items());positions=[v for v,k in ordered];chunks=set();shared=(set(),threading.Lock());reader=BudgetReader(p['archive_url'],p['archive_bytes'],cache,shared)
 for m in p['members']:
  a=max(0,bisect.bisect_right(positions,m['offset_data']-512)-1);b=min(len(ordered)-1,bisect.bisect_left(positions,m['offset_data']+m['size'])+1)
  chunks.update(range((ordered[a][1]//8)//reader.block,((ordered[b][1]+7)//8)//reader.block+1))
 if len(chunks)*reader.block>PARAMS['compressed_budget_bytes']:raise ValueError('prefetch budget')
 def fetch(i):
  r=BudgetReader(reader.url,reader.length,cache,shared);r.seek(i*r.block);r.read(min(r.block,r.length-r.tell()))
 print('Prefetch',len(chunks),'compressed chunks',flush=True)
 with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:list(pool.map(fetch,sorted(chunks)))
 records=[]
 with indexed_bzip2.open(reader,parallelization=1) as stream:
  stream.set_block_offsets(offsets)
  for m in sorted(p['members'],key=lambda m:m['offset_data']):
   stream.seek(m['offset_data']-512);header=stream.read(512);info=tarfile.TarInfo.frombuf(header,'utf-8','strict')
   if not (info.isfile() and info.name==m['name'] and info.size==m['size']):raise ValueError('TAR exact member')
   data=stream.read(m['size']);png_schema(data,m['shape']);dest=OUT/'labels'/m['name'];dest.parent.mkdir(parents=True,exist_ok=True)
   if dest.exists():
    if dest.read_bytes()!=data:raise ValueError('conflicting source label')
   else:dest.write_bytes(data)
   old=ROOT/'results/ms2-night-pilot/pilot/proj_depth'/m['name'];overlap=None
   if old.exists():
    overlap=old.read_bytes()==data
    if not overlap:raise ValueError('historical label overlap changed')
   hp=OUT/'tar-headers'/m['modality']/(f"{m['index']:06}.bin");hp.parent.mkdir(parents=True,exist_ok=True);hp.write_bytes(header)
   records.append(dict(name=m['name'],path=str(dest.relative_to(ROOT)),sha256=sha(dest),bytes=len(data),header_path=str(hp.relative_to(ROOT)),header_sha256=sha(hp),historical_overlap_exact=overlap))
   print('Acquired',m['modality'],m['index'],flush=True)
 ranges=sorted(cache.glob('*.bin'));report=dict(members=records,member_count=len(records),compressed_range_files=len(ranges),compressed_bytes=sum(f.stat().st_size for f in ranges),current_full_archive_sha_verified=False,historical_full_archive_sha256=p['historical_full_archive_sha256'],range_metadata_sha256={str(f.relative_to(ROOT)):sha(f) for f in cache.glob('*.json')})
 write(OUT/'acquisition.json',report);verify();print('ACQUIRED',len(records),'selected depth maps')

def feature_result(feature,depth,fx,baseline,width):
 u,v,octave,right,z,associated,outlier=feature;x=math.floor(u+.5);y=math.floor(v+.5);result=dict(pixel=[x,y],reason=None,depth_code=None,reference_m=None,native_accepted=z>0,metrics=None)
 if not(0<=x<depth.shape[1] and 0<=y<depth.shape[0]):result['reason']='reference_pixel_outside';return result
 code=int(depth[y,x]);result['depth_code']=code
 if not code:result['reason']='zero_reference';return result
 g=code/256;result['reference_m']=g;expected=fx*baseline/g
 if not(0<=u-expected<width-1):result['reason']='proxy_right_outside';return result
 if z<=0:result['reason']='native_stereo_missing';return result
 error=z-g;disparity=u-right;delta=disparity-expected
 result.update(reason='compared',metrics=dict(depth_error_m=error,abs_depth_error_m=abs(error),abs_relative_error=abs(error)/g,depth_ratio=z/g,disparity_error_px=delta,abs_disparity_error_px=abs(delta),disparity_error_normalized=delta/fx,abs_disparity_error_normalized=abs(delta)/fx))
 return result

def stats(v):
 if not v:return dict(n=0,mean=None,rmse=None,median=None,p90=None,min=None,max=None)
 a=np.asarray(v,float);return dict(n=len(v),mean=float(np.mean(a)),rmse=float(np.sqrt(np.mean(a*a))),median=float(np.median(a)),p90=float(np.percentile(a,90)),min=float(np.min(a)),max=float(np.max(a)))
METRICS=['depth_error_m','abs_depth_error_m','abs_relative_error','depth_ratio','disparity_error_px','abs_disparity_error_px','disparity_error_normalized','abs_disparity_error_normalized']

def evaluate():
 verify();acquisition=read(OUT/'acquisition.json');inputs={v['name']:v for v in acquisition['members']};source=read(OUT/'native-selected.json');summaries=[]
 for r in source['records']:
  name=source['sequence']+'/'+r['camera']+'/depth/'+r['stem']+'.png';item=inputs[name];p=ROOT/item['path']
  if sha(p)!=item['sha256']:raise ValueError('label changed')
  depth=cv2.imread(str(p),cv2.IMREAD_UNCHANGED)
  if depth.dtype!=np.uint16 or list(depth.shape)!=r['shape']:raise ValueError('depth units/schema')
  frame=r['native'];points=[];reasons={}
  for i,f in enumerate(frame['features']):
   result=feature_result(f,depth,r['K'][0][0],r['baseline_m'],r['shape'][1]);result.update(feature_index=i,native=f);points.append(result);reasons[result['reason']]=reasons.get(result['reason'],0)+1
  compared=[p for p in points if p['reason']=='compared'];record=dict(modality=r['modality'],index=r['index'],stem=r['stem'],native_available=frame.get('available',True),features=len(points),native_accepted=sum(p['native_accepted'] for p in points),reference_geometrically_supported=sum(p['reason'] in ['compared','native_stereo_missing'] for p in points),compared=len(compared),unique_reference_pixels=len({tuple(p['pixel']) for p in compared}),reasons=reasons,metrics={k:stats([p['metrics'][k] for p in compared]) for k in METRICS})
  write(OUT/'points'/r['modality']/(r['stem']+'.json'),dict(record=record,points=points,label_sha256=item['sha256']));summaries.append(record)
  print('Compared',r['modality'],r['stem'],len(compared),flush=True)
 agg=[]
 for mode in ['rgb','thermal']:
  rows=[r for r in summaries if r['modality']==mode];aggregate={k:stats([r['metrics'][k]['median'] for r in rows if r['metrics'][k]['median'] is not None]) for k in METRICS};agg.append(dict(modality=mode,image_cases=len(rows),empty_cases=sum(r['compared']==0 for r in rows),total_features=sum(r['features'] for r in rows),total_native_accepted=sum(r['native_accepted'] for r in rows),total_reference_geometrically_supported=sum(r['reference_geometrically_supported'] for r in rows),total_compared=sum(r['compared'] for r in rows),per_image_median_aggregate=aggregate))
 write(OUT/'summary.json',dict(cases=summaries,aggregates=agg,physical_reference_qualified=False,physical_sync_verified=False,benchmark_execution_ready=False,scale_fitted=False,time_fitted=False,comparison='conditional single-scan projected LiDAR proxy at rounded native keypoint',source_sha256=sha(PLAN)))
 verify();print('COMPLETE40CASES')

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','freeze','verify','acquire','evaluate']);x=p.parse_args();{'prepare':prepare,'freeze':freeze,'verify':verify,'acquire':acquire,'evaluate':evaluate}[x.action]()

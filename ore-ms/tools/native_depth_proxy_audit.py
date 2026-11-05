"""Independent scalar native-keypoint / raw uint16 LiDAR proxy comparison audit."""
import math,json,hashlib,tarfile,struct,zlib
from pathlib import Path
from collections import Counter
from copy import deepcopy
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/native-depth-proxy/run-001';PLAN=ROOT/'experiments/native-depth-proxy-2026-11-05.json';COUNT=0
KEYS=['depth_error_m','abs_depth_error_m','abs_relative_error','depth_ratio','disparity_error_px','abs_disparity_error_px','disparity_error_normalized','abs_disparity_error_normalized']
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def need(v,label):
 global COUNT
 COUNT+=1
 if not v:raise ValueError(label)
def close(a,b,label):
 if a is None or b is None:need(a is b,label)
 else:need(math.isclose(a,b,rel_tol=1e-10,abs_tol=1e-10),label)
def quantile(values,q):
 s=sorted(values);position=q*(len(s)-1);i=int(position);part=position-i;return s[i]*(1-part)+s[min(i+1,len(s)-1)]*part

def check_stats(s,values):
 need(s['n']==len(values),'statistic population')
 if not values:
  need(all(s[k] is None for k in ['mean','rmse','median','p90','min','max']),'null empty statistics');return
 expected=dict(mean=sum(values)/len(values),rmse=math.sqrt(sum(x*x for x in values)/len(values)),median=quantile(values,.5),p90=quantile(values,.9),min=min(values),max=max(values))
 for k,v in expected.items():close(s[k],v,'independent scalar '+k)

def expect(feature,depth,fx,b,width):
 u,v,_,right,z,_,_=feature;x=int(math.floor(u+.5));y=int(math.floor(v+.5))
 if not(0<=y<len(depth) and 0<=x<len(depth[0])):return [x,y],'reference_pixel_outside',None,None,None
 code=int(depth[y,x])
 if code==0:return [x,y],'zero_reference',code,None,None
 reference=code/256
 # Division-free right-support equivalence, rather than scorer division branch.
 valid=(u*reference>=fx*b and (u-(width-1))*reference<fx*b)
 if not valid:return [x,y],'proxy_right_outside',code,reference,None
 if z<=0:return [x,y],'native_stereo_missing',code,reference,None
 error=z-reference;relative=z/reference;delta=(u-right)-(fx*b*256)/code
 metrics=dict(depth_error_m=error,abs_depth_error_m=abs(error),abs_relative_error=abs(relative-1),depth_ratio=relative,disparity_error_px=delta,abs_disparity_error_px=abs(delta),disparity_error_normalized=delta/fx,abs_disparity_error_normalized=abs(delta)/fx)
 return [x,y],'compared',code,reference,metrics

def check_point(point,feature,depth,fx,b,w):
 pixel,reason,code,reference,metrics=expect(feature,depth,fx,b,w)
 need(point['native']==feature,'native feature immutable');need(point['pixel']==pixel and point['reason']==reason and point['depth_code']==code,'raw label/rounded pixel/support')
 need(point['native_accepted'] is (feature[4]>0),'native accepted sentinel');close(point['reference_m'],reference,'uint16/256m')
 if metrics is None:need(point['metrics'] is None,'no unsupported error values')
 else:
  for key,value in metrics.items():close(point['metrics'][key],value,'point '+key)
 return reason,pixel,metrics

def audit():
 p=read(PLAN);need(sha(PLAN)==sha(OUT/'study-lock.json'),'frozen study')
 for n,h in p['source_sha256'].items():need(sha(ROOT/n)==h,'source pin '+n)
 native=read(OUT/'native-selected.json');need(len(native['records'])==40,'all40native samples');summary=read(OUT/'summary.json');need(len(summary['cases'])==40,'all40results')
 for k in ['physical_reference_qualified','physical_sync_verified','benchmark_execution_ready','scale_fitted','time_fitted']:need(summary[k] is False,'reference/scope qualifier')
 acquisition=read(OUT/'acquisition.json');need(acquisition['member_count']==len(acquisition['members'])==40,'all40acquiredlabels');need(acquisition['current_full_archive_sha_verified'] is False,'bounded selected-member evidence')
 indexed={m['name']:m for m in p['members']};members={m['name']:m for m in acquisition['members']};records={}
 for mode in ['rgb','thermal']:
  expected=[r for r in native['records'] if r['modality']==mode];need([r['index'] for r in expected]==list(range(0,2000,100)),'fixed frame grid')
  with (ROOT/'results/native-stereo-history/run-002/captures'/(mode+'-paired')/'native.jsonl.stereo.jsonl').open() as f:
   originals={}
   for line in f:
    idx=int(line.split(',',1)[0].split(':')[1])
    if idx in range(0,2000,100):originals[idx]=json.loads(line)
  for r in expected:need(originals[r['index']]==r['native'],'native sample original raw rows')
 for r in native['records']:
  m=members[native['sequence']+'/'+r['camera']+'/depth/'+r['stem']+'.png'];file=ROOT/m['path'];need(sha(file)==m['sha256'],'depth PNG bytes pin');need(sha(ROOT/m['header_path'])==m['header_sha256'],'TAR header pin');h=tarfile.TarInfo.frombuf((ROOT/m['header_path']).read_bytes(),'utf-8','strict');need(h.isfile() and h.name==m['name'] and h.size==file.stat().st_size==indexed[m['name']]['size'],'exact indexed TAR')
  data=file.read_bytes();pos=8;need(data[:8]==b'\x89PNG\r\n\x1a\n','PNG signature')
  while pos<len(data):
   size=struct.unpack_from('>I',data,pos)[0];kind=data[pos+4:pos+8];body=data[pos+8:pos+8+size];actual=struct.unpack_from('>I',data,pos+8+size)[0];need(zlib.crc32(kind+body)&0xffffffff==actual,'independent PNG CRC');pos+=12+size
  need(pos==len(data),'PNG exact extent');depth=cv2.imread(str(file),-1);need(depth.dtype==np.uint16 and list(depth.shape)==r['shape'],'uint16 depth shape')
  old=ROOT/'results/ms2-night-pilot/pilot/proj_depth'/m['name']
  if old.exists():need(sha(old)==sha(file) and m['historical_overlap_exact'] is True,'archived overlapexact')
  case=read(OUT/'points'/r['modality']/(r['stem']+'.json'));record=case['record'];need(case['label_sha256']==m['sha256'],'score label binding')
  points=case['points'];need(len(points)==len(r['native']['features'])==record['features'],'all original feature slots')
  reasons=Counter();matched=[];pixels=set()
  for i,(point,feature) in enumerate(zip(points,r['native']['features'])):
   need(point['feature_index']==i,'native feature index');reason,pixel,metrics=check_point(point,feature,depth,r['K'][0][0],r['baseline_m'],r['shape'][1]);reasons[reason]+=1
   if metrics is not None:matched.append(metrics);pixels.add(tuple(pixel))
  need(dict(reasons)==record['reasons'],'all support exclusions')
  need(record['compared']==len(matched) and record['unique_reference_pixels']==len(pixels),'supported and duplicate-label populations')
  need(record['reference_geometrically_supported']==reasons['compared']+reasons['native_stereo_missing'] and record['native_accepted']==sum(f[4]>0 for f in r['native']['features']),'coverage denominators')
  for key in KEYS:check_stats(record['metrics'][key],[v[key] for v in matched])
  key=(r['modality'],r['index']);records[key]=record;need(record==next(v for v in summary['cases'] if (v['modality'],v['index'])==key),'case summary exact')
 for aggregate in summary['aggregates']:
  mode=aggregate['modality'];rows=[v for k,v in records.items() if k[0]==mode];need(aggregate['image_cases']==20 and aggregate['empty_cases']==sum(v['compared']==0 for v in rows),'all20image cases and empties')
  for total,key in [('total_features','features'),('total_native_accepted','native_accepted'),('total_reference_geometrically_supported','reference_geometrically_supported'),('total_compared','compared')]:need(aggregate[total]==sum(r[key] for r in rows),'aggregate coverage '+key)
  for key in KEYS:check_stats(aggregate['per_image_median_aggregate'][key],[r['metrics'][key]['median'] for r in rows if r['metrics'][key]['median'] is not None])
 ranges=OUT/'compressed-ranges';total=0
 for name,digest in acquisition['range_metadata_sha256'].items():
  f=ROOT/name;need(sha(f)==digest,'range metadata pin');meta=read(f);blob=f.with_suffix('.bin');need(sha(blob)==meta['sha256'] and blob.stat().st_size==meta['end']-meta['start']+1,'compressed range bytes');need(meta['status']==206 and meta['content_range']==f"bytes {meta['start']}-{meta['end']}/{p['archive_bytes']}",'HTTP206 bounds');total+=blob.stat().st_size
 need(total==acquisition['compressed_bytes']<=1024**3,'fixed compressed byte budget')
 mutations=0
 # Mutate a real supported point and denominator/scope controls.
 sample=next(r for r in native['records'] if records[(r['modality'],r['index'])]['compared']>0);case=read(OUT/'points'/sample['modality']/(sample['stem']+'.json'));point=next(v for v in case['points'] if v['reason']=='compared');m=members[native['sequence']+'/'+sample['camera']+'/depth/'+sample['stem']+'.png'];depth=cv2.imread(str(ROOT/m['path']),-1)
 for kind in ['ratio','pixel','units','support']:
  bad=deepcopy(point)
  if kind=='ratio':bad['metrics']['depth_ratio']*=2
  elif kind=='pixel':bad['pixel'][0]+=1
  elif kind=='units':bad['reference_m']*=256
  else:bad['reason']='zero_reference'
  try:check_point(bad,bad['native'],depth,sample['K'][0][0],sample['baseline_m'],sample['shape'][1])
  except ValueError:mutations+=1
  else:raise ValueError('missed semantic corruption '+kind)
 result=dict(passed=True,independent_checks=COUNT,mutations_rejected=mutations,source_images=40,reference='single-scan projected LiDAR proxy; physical qualification unresolved',auditor_sha256=sha(__file__),plan_sha256=sha(PLAN))
 with (OUT/'independent-audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':audit()

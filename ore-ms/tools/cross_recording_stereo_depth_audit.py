"""Independent EXP151 scalar depth/support, local groups and native cost audit."""
import json,hashlib,math,tarfile,struct,zlib
from pathlib import Path
from collections import Counter
from copy import deepcopy
import cv2
import numpy as np
import native_depth_proxy_audit as scalar
import depth_projection_sensitivity_audit as local
import native_match_ambiguity_audit as costs
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/cross-recording-stereo-depth/run-001';COUNT=0
INDICES=list(range(0,2000,100));VARIABLES=['descriptor_margin','descriptor_ratio','patch_relative_gap'];GROUPS=['all_primary','locally_consistent']
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def need(v,label):
 global COUNT
 COUNT+=1
 if not v:raise ValueError(label)
def check_png(path,shape):
 data=path.read_bytes();need(data[:8]==b'\x89PNG\r\n\x1a\n','PNG magic');pos=8
 while pos<len(data):
  size=struct.unpack_from('>I',data,pos)[0];kind=data[pos+4:pos+8];body=data[pos+8:pos+8+size];need(zlib.crc32(kind+body)&0xffffffff==struct.unpack_from('>I',data,pos+8+size)[0],'PNG CRC');pos+=12+size
 need(pos==len(data),'PNG exact extent');depth=cv2.imread(str(path),-1);need(depth.dtype==np.uint16 and list(depth.shape)==shape,'source uint16 depth shape');return depth

def audit():
 p=read(ROOT/'experiments/cross-recording-stereo-depth-2026-11-06.json');need(sha(ROOT/'experiments/cross-recording-stereo-depth-2026-11-06.json')==sha(OUT/'study-lock.json'),'study binding')
 for lock in [p,read(OUT/'geometry-lock.json'),read(OUT/'cost-lock.json')]:
  for n,h in lock['source_sha256'].items():need(sha(ROOT/n)==h,'source/native output pin '+n)
 need(sha(OUT/'targets.json')==read(OUT/'cost-lock.json')['targets_sha256'],'target lock')
 identity=read(OUT/'runtime-identity.json');need(identity['image']==p['runtime'],'native image identity');parent=read(ROOT/'results/stereo-baseline-initialization/run-001/runtime-identities.json')['fixed']
 for n,h in parent['native_sha256'].items():need(identity['native_sha256'][n]==h,'fixed native source/library/observer identity')
 need(identity['system_header_sha256']==parent['system_header_sha256'],'unchanged header')
 summary=read(OUT/'summary.json');prepared=read(OUT/'prepared.json');need(len(prepared['records'])==80 and len(summary['depth']['cases'])==len(summary['ambiguity']['cases'])==80,'all80image pairs')
 for key in ['physical_reference_qualified','physical_sync_verified','benchmark_execution_ready','matcher_tuned','new_trajectory']:need(summary[key] is False,'scope '+key)
 day=read(OUT/'acquisition.json');night=read(ROOT/'results/native-depth-proxy/run-001/acquisition.json');need(day['member_count']==40 and day['compressed_bytes']<=1024**3 and day['current_full_archive_sha_verified'] is False,'bounded day-only label acquisition')
 members={v['name']:v for v in day['members']+night['members']};indices={v['name']:v for v in p['members']};depth_cases=[];ambiguity_cases=[];samples=[];target_total=0
 for rec in ['day','night']:
  for mode in ['rgb','thermal']:
   key=rec+'-'+mode;first=list(map(json.loads,(OUT/'geometry'/(key+'.jsonl')).read_text().splitlines()));second=list(map(json.loads,(OUT/'costs'/(key+'.jsonl')).read_text().splitlines()));records=[r for r in prepared['records'] if r['recording']==rec and r['modality']==mode]
   need([v['index'] for v in first]==[v['index'] for v in second]==[r['index'] for r in records]==INDICES,'fixed source grid, both passes')
   for stage in ['geometry','costs']:need(read(OUT/stage/(key+'-status.json'))==dict(returncode=0,terminal_success=True),'single native attempt complete')
   for frame,replay,r in zip(first,second,records):
    need(frame['features']==replay['features'] and frame['right_features']==replay['right_features'],'two-pass bit-exact geometry/descriptors');need(frame['points']==[],'first pass reference-blind geometry only')
    need(math.isclose(frame['baseline_m'],r['baseline_m'],rel_tol=1e-6) and math.isclose(frame['bf'],r['baseline_m']*r['K'][0][0],rel_tol=1e-6),'native calibration')
    item=members[r['sequence']+'/'+r['camera']+'/depth/'+r['stem']+'.png'];path=ROOT/item['path'];need(sha(path)==item['sha256'],'label identity');depth=check_png(path,r['shape'])
    if rec=='day':
     header=ROOT/item['header_path'];need(sha(header)==item['header_sha256'],'TAR header pin');info=tarfile.TarInfo.frombuf(header.read_bytes(),'utf-8','strict');need(info.isfile() and info.name==item['name'] and info.size==indices[info.name]['size']==path.stat().st_size,'exact bounded indexed member')
     old=ROOT/'results/ms2-day-replication/pilot/proj_depth'/item['name']
     if old.exists():need(sha(old)==sha(path) and item['historical_overlap_exact'] is True,'archived day overlap')
    depth_report=read(OUT/'depth'/rec/mode/(r['stem']+'.json'));case=depth_report['case'];points=depth_report['points'];need(depth_report['label_sha256']==sha(path),'score label binding');need(len(points)==len(frame['features'])==case['features'],'every extracted feature retained');primary=[];reasons=Counter()
    for i,(point,five) in enumerate(zip(points,frame['features'])):
     feature=five+[None,None];need(point['feature_index']==i,'feature identity');need(len(five)==5 and all(math.isfinite(v) for v in five),'finite geometry')
     need((five[4]>0)==(five[3]>=0) and (five[4]>0 or five[3:5]==[-1,-1]),'accepted/sentinel binding')
     if five[4]>0:
      d=five[0]-five[3];eps=4*(abs(float(np.spacing(np.float32(five[0]))))+abs(float(np.spacing(np.float32(five[3])))))
      need(d>0 and frame['bf']/(d+eps)*(1-2e-5)-2e-5<=five[4]<=frame['bf']/max(d-eps,np.finfo(np.float32).tiny)*(1+2e-5)+2e-5,'native bf/disparity depth')
     reason,pixel,metrics=scalar.check_point(point,feature,depth,r['K'][0][0],r['baseline_m'],r['shape'][1]);reasons[reason]+=1
     if reason=='compared':
      want=local.derive(feature,depth,r['K'][0][0],r['baseline_m'],2)
      for k in ['center','candidates','rejected','group','interval_class']:need(point['local'][k]==want[k],'independent local reference '+k)
      for k in ['reference_min_m','reference_max_m','reference_median_m','relative_span','ratio_lower','ratio_upper']:scalar.close(point['local'][k],want[k],'local scalar '+k)
      primary.append(point)
    need(case['reasons']==dict(reasons) and case['accepted']==sum(v[4]>0 for v in frame['features']),'full support/acceptance counts');need(case['unique_reference_pixels']==len({tuple(v['pixel']) for v in primary}),'unique reference pixels')
    for group in GROUPS:
     data=primary if group=='all_primary' else [v for v in primary if v['local']['group']=='locally_consistent'];g=case['groups'][group];need(g['points']==len(data),'depth cohort population')
     for k in scalar.KEYS:scalar.check_stats(g['metrics'][k],[v['metrics'][k] for v in data])
    need(case==next(c for c in summary['depth']['cases'] if (c['recording'],c['modality'],c['index'])==(rec,mode,r['index'])),'depth case summary');depth_cases.append(case)
    report=read(OUT/'ambiguity'/rec/mode/(r['stem']+'.json'));ap=report['points'];acase=report['case'];need([v['feature_index'] for v in replay['points']]==[v['feature_index'] for v in primary]==[v['feature_index'] for v in ap],'all primary targets; no error/ambiguity selection');target_total+=len(primary)
    for observed,source,derived in zip(ap,primary,replay['points']):
     variables=costs.check_costs(derived,replay);need(observed['ambiguity']==variables,'independent ambiguity variables');need(observed['faithful'] is True and observed['original_metrics']==source['metrics'] and observed['reference_group']==source['local']['group'],'native cost/primary group/metric fidelity')
     if not samples and derived['patch_status']=='ok':samples=[derived,replay,source,depth,r]
    for group in GROUPS:
     data=ap if group=='all_primary' else [v for v in ap if v['reference_group']=='locally_consistent'];g=acase['groups'][group];need(g['points']==g['faithful']==len(data),'ambiguity group population/fidelity')
     for var in VARIABLES:
      eligible=[v for v in data if v['ambiguity'][var] is not None];x=[v['ambiguity'][var] for v in eligible];y=[v['original_metrics']['abs_relative_error'] for v in eligible];reason='fewer_than_10' if len(x)<10 else 'constant' if len(set(x))<2 or len(set(y))<2 else None;value=costs.rho(x,y) if reason is None else None;c=g['correlations'][var]
      need(c['points']==len(x) and c['reason']==reason,'rank population/availability');scalar.close(c['rho'],value,'independent tied ranks')
    need(acase==next(c for c in summary['ambiguity']['cases'] if (c['recording'],c['modality'],c['index'])==(rec,mode,r['index'])),'ambiguity case summary');ambiguity_cases.append(acase)
 for a in summary['depth']['aggregates']:
  data=[v['groups'][a['group']] for v in depth_cases if v['recording']==a['recording'] and v['modality']==a['modality']];need(a['image_cases']==len(data)==20 and a['images_nonempty']==sum(v['points']>0 for v in data) and a['points']==sum(v['points'] for v in data),'equal-image depth aggregate coverage')
  for k in scalar.KEYS:scalar.check_stats(a['per_image_median'][k],[v['metrics'][k]['median'] for v in data if v['metrics'][k]['median'] is not None])
 for a in summary['ambiguity']['aggregates']:
  data=[v['groups'][a['group']] for v in ambiguity_cases if v['recording']==a['recording'] and v['modality']==a['modality']];need(len(data)==20 and a['points']==a['faithful']==sum(v['points'] for v in data),'ambiguity aggregate population')
  for var in VARIABLES:
   values=[v['correlations'][var]['rho'] for v in data if v['correlations'][var]['rho'] is not None];saved=a['correlations'][var];need(saved['images_defined']==len(values) and saved['negative_images']==sum(v<0 for v in values),'rank image coverage');scalar.check_stats(saved['stats'],values)
 fid=read(OUT/'fidelity.json');need(len(fid['cases'])==80 and fid['points']==fid['native_cost_exact']==target_total and fid['all_geometry_exact'] is True,'full fidelity summary')
 mutations=0;point,frame,source,depth,r=samples
 for kind in ['hamming','patch','candidate','depth']:
  bad=deepcopy(point);changed=deepcopy(frame) if kind=='depth' else frame
  if kind=='hamming':bad['candidates'][0][1]+=1
  elif kind=='patch':bad['costs'][0]+=1
  elif kind=='candidate':bad['candidates'].pop()
  else:changed['features'][bad['feature_index']][4]*=2
  try:costs.check_costs(bad,changed)
  except ValueError:mutations+=1
  else:raise ValueError('missed native corruption '+kind)
 for kind in ['units','pixel','support']:
  bad=deepcopy(source)
  if kind=='units':bad['reference_m']*=256
  elif kind=='pixel':bad['pixel'][0]+=1
  else:bad['reason']='zero_reference'
  try:scalar.check_point(bad,bad['native'],depth,r['K'][0][0],r['baseline_m'],r['shape'][1])
  except ValueError:mutations+=1
  else:raise ValueError('missed reference corruption '+kind)
 result=dict(passed=True,independent_checks=COUNT+scalar.COUNT+local.COUNT+costs.COUNT,mutations_rejected=mutations,image_pairs=80,native_replay_pair_calls=160,primary_points=target_total,all_cost_faithful=True,auditor_sha256=sha(__file__),physical_reference_qualified=False)
 with (OUT/'independent-audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':audit()

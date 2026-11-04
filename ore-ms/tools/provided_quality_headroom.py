"""EXP130 immutable provided-SLAM quality oracle; no deployed policy."""
import argparse,hashlib,json,sys,zipfile,zlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/provided-quality-headroom/run-001'
PLAN=ROOT/'experiments/provided-quality-headroom-2026-11-04.json'
PRIOR=ROOT/'results/provided-export-survey/run-001'
INDEX=ROOT/'results/exposed-data-adequacy/archive-metadata-001'
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def verify():
 p=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for n,h in p['sha256'].items():assert sha(ROOT/n)==h,n
 return p

def freeze():
 assert not OUT.exists() and not PLAN.exists()
 p=read(ROOT/'experiments/provided-export-survey-2026-11-04.json');survey=read(PRIOR/'native-survey.json')
 assert len(survey['rows'])==380 and sum(r['mask']=='both' for r in survey['rows'])==323
 files=[ROOT/'tools/provided_quality_headroom.py',ROOT/'tools/provided_quality_headroom_audit.py',ROOT/'docs/research/provided-quality-headroom-protocol-2026-11-04.md',ROOT/'data/calibration/yamaha.yaml',ROOT/'experiments/provided-export-survey-2026-11-04.json']
 files += [PRIOR/n for n in ['capture-lock.json','prepared-lock.json','native-survey.json','native-audit.json']]
 for d in ['capture-lock.json','prepared-lock.json']:files += [ROOT/n for n in read(PRIOR/d)['sha256']]
 files += list((ROOT/'MAC-VO-main/Scripts/Setup').glob('*.py'))
 for w in p['windows']:
  files += [INDEX/w['archive']/n for n in ['report.json','members.json']]
  files += [(ROOT/r['file']) for r in read(INDEX/w['archive']/'report.json')['ranges']]
 pins={str(f.relative_to(ROOT)):sha(f) for f in files}
 plan={'parent_commit':'4da8557cd','date':'2026-11-04','windows':p['windows'],'runtime_image':p['runtime_image'],'sha256':pins,'reference_poses_read':False,'expected_joint_rows':323,'metadata_cap':2097152,'pose_cap':33554432,'max_reference_gap_ns':150000000,'development_exposed':True,'physical_ground_truth':False,'reserved_payload_access':False}
 write(PLAN,plan);write(OUT/'plan.json',plan);print('FROZEN',len(pins),'pins')

def intake(stage):
 p=verify()
 if stage=='poses':
  pred=read(OUT/'prediction-lock.json');assert pred['predictions_sha256']==sha(OUT/'predictions.json')
  assert read(OUT/'metadata-audit.json')['pass']
 from five_recording_survey import RemoteZIP
 archives={a['Name']:a for a in read(INDEX/'plan.json')['archives']};reports=[]
 for clip in sorted({w['clip'] for w in p['windows']}):
  archive=archives[clip+'.zip'];dest=OUT/'reference'/clip;attempt=OUT/'intake'/stage/clip;attempt.mkdir(parents=True,exist_ok=False)
  reader=RemoteZIP(['rclone','--config','/root/.config/rclone/ore-ms-recovery.conf','--box-root-folder-id','428559630945','cat','ore_ms:'+archive['Name'],'--contimeout','15s','--timeout','90s','--retries','2'],archive['Size'],attempt/'range-cache')
  for r in read(INDEX/archive['Name']/'report.json')['ranges']:
   b=(ROOT/r['file']).read_bytes();assert hashlib.sha256(b).hexdigest()==r['sha256'];reader.blocks[r['offset'],r['count']]=b
  names=['info.yaml','interp_timestamps.txt'] if stage=='metadata' else ['interp_data.txt'];files={}
  with zipfile.ZipFile(reader) as z:
   infos=[z.getinfo(clip+'/super_odometry/odometry/'+n) for n in names]
   assert all(i.file_size<=p['metadata_cap' if stage=='metadata' else 'pose_cap'] for i in infos)
   reader.prefetch(infos)
   for i in infos:
    b=z.read(i);assert zlib.crc32(b)&0xffffffff==i.CRC
    target=dest/i.filename.split('/',1)[1];target.parent.mkdir(parents=True,exist_ok=True)
    with target.open('xb') as f:f.write(b)
    files[str(target.relative_to(ROOT))]={'sha256':sha(target),'bytes':len(b),'crc32':i.CRC,'member':i.filename,'compressed_bytes':i.compress_size,'header_offset':i.header_offset}
  report={'archive':archive,'files':files,'range_requests':reader.requests};write(attempt/'report.json',report);reports.append(report)
  print('INTAKE',stage,clip,flush=True)
 write(OUT/(stage+'-intake-lock.json'),{'reports':reports,'prediction_lock_sha256':sha(OUT/'prediction-lock.json') if stage=='poses' else None})

def scientific():
 sys.path.insert(0,str(ROOT/'MAC-VO-main'))
 import numpy as np,yaml
 from Scripts.Setup.td3_reference import matrices,read_ns,load_reference
 from Scripts.Setup.arbiter_pose_proxy import extrinsic,losses
 from Scripts.Setup.causal_interval_model import project
 return np,yaml,matrices,read_ns,load_reference,extrinsic,losses,project

def metadata_audit():
 np,yaml,_,read_ns,*_=scientific();p=verify();groups={}
 for report in read(OUT/'metadata-intake-lock.json')['reports']:
  for n,item in report['files'].items():assert sha(ROOT/n)==item['sha256']
 for clip in sorted({w['clip'] for w in p['windows']}):
  d=OUT/'reference'/clip/'super_odometry/odometry';info=yaml.safe_load((d/'info.yaml').read_text())
  assert info=={'frame_id':'sensor_init','child_frame_id':'vehicle'},info
  t=read_ns(d/'interp_timestamps.txt');assert len(t)>=2 and np.all(np.diff(t)>0)
  groups[clip]={'frames':info,'timestamp_rows':len(t),'bounds_ns':[int(t[0]),int(t[-1])],'max_gap_ns':int(np.diff(t).max())}
 write(OUT/'metadata-audit.json',{'pass':True,'groups':groups,'metadata_lock_sha256':sha(OUT/'metadata-intake-lock.json')});print('PASS METADATA',groups)

def predictions():
 np,_,matrices,_,_,extrinsic,_,project=scientific();p=verify();assert read(OUT/'metadata-audit.json')['pass']
 rows=[];transforms={}
 for w in p['windows']:
  base=PRIOR/'captures'/w['case'];events=[json.loads(l) for l in (base/'timeline.jsonl').read_text().splitlines()]
  motions={}
  for e in events:
   if e['kind']=='observer_event' and e['event']['kind']=='motion_result_written' and e['event']['first_write_for_submission']:
    k=e['camera'],e['event']['time1_ns'];assert k not in motions;motions[k]=e
  transforms[w['case']]={c:extrinsic(ROOT/'data/calibration/yamaha.yaml',ROOT/w['dataset'],c).tolist() for c in ['rgb','thermal']}
  for r in read(PRIOR/'native-survey.json')['rows']:
   if r['case']!=w['case'] or r['mask']!='both':continue
   i=r['pair'];common=[w['rows'][j]['rgb_left']['time_ns'] for j in [i,i+1]];experts={}
   for c in ['rgb','thermal']:
    native=r['experts'][c]['native_interval_ns'];e=motions[c,native[1]];m=e['event']
    assert [m['time0_ns'],m['time1_ns']]==native and m['stage']=='pair_processing' and m['original_need_interp'] is False
    assert e['controller_monotonic_ns']==r['experts'][c]['motion_receipt_ns']<=r['deadline_ns']
    T=np.asarray(transforms[w['case']][c]);body=T@matrices([m['derived_camera_delta_xyzw']])[0]@np.linalg.inv(T)
    experts[c]=dict(project(body,native,common),body_delta=body.tolist(),motion_sequence=m['sequence'])
   rows.append({'case':w['case'],'pair':i,'common_interval_ns':common,'experts':experts})
 assert len(rows)==323
 return {'rows':rows,'transforms':transforms,'reference_used_for_prediction':False}

def predict():
 result=predictions();write(OUT/'predictions.json',result)
 pins={str((OUT/n).relative_to(ROOT)):sha(OUT/n) for n in ['metadata-intake-lock.json','metadata-audit.json','predictions.json']}
 for report in read(OUT/'metadata-intake-lock.json')['reports']:
  pins.update({n:item['sha256'] for n,item in report['files'].items()})
 write(OUT/'prediction-lock.json',{'plan_sha256':sha(PLAN),'predictions_sha256':sha(OUT/'predictions.json'),'sha256':pins,'reference_poses_read':False});print('LOCKED',len(result['rows']),'joint predictions before reference poses')

def summarize(rows):
 import numpy as np
 result={'rows':len(rows)}
 for metric in ['translation_m','rotation_deg']:
  R=np.array([r['loss']['rgb'][metric] for r in rows]);T=np.array([r['loss']['thermal'][metric] for r in rows]);M=np.minimum(R,T)
  if not len(R):result[metric]=None;continue
  best=min(float(R.mean()),float(T.mean()));oracle=float(M.mean());winners=['tie' if abs(a-b)<=1e-12 else 'rgb' if a<b else 'thermal' for a,b in zip(R,T)]
  result[metric]={'rgb_mean':float(R.mean()),'thermal_mean':float(T.mean()),'rgb_median':float(np.median(R)),'thermal_median':float(np.median(T)),'oracle_mean':oracle,'best_fixed_mean':best,'oracle_reduction':best-oracle,'oracle_reduction_fraction':(best-oracle)/best if best else 0.,'winners':{c:winners.count(c) for c in ['rgb','thermal','tie']},'adjacent_winner_changes':sum(a['case']==b['case'] and b['pair']==a['pair']+1 and x!=y and 'tie' not in [x,y] for a,b,x,y in zip(rows,rows[1:],winners,winners[1:]))}
 return result

def score_rows():
 np,_,_,_,load_reference,_,losses,_=scientific();p=verify();lock=read(OUT/'prediction-lock.json')
 assert lock['plan_sha256']==sha(PLAN)
 for n,h in lock['sha256'].items():assert sha(ROOT/n)==h,n
 for report in read(OUT/'poses-intake-lock.json')['reports']:
  for n,item in report['files'].items():assert sha(ROOT/n)==item['sha256']
 refs={clip:load_reference(OUT/'reference'/clip)[0] for clip in {w['clip'] for w in p['windows']}};windows={w['case']:w for w in p['windows']};rows=[]
 for source in read(OUT/'predictions.json')['rows']:
  r={k:source[k] for k in ['case','pair','common_interval_ns']};ref=refs[windows[r['case']]['clip']];times=np.asarray(r['common_interval_ns'],dtype=np.int64);reason=None
  if not all(e['valid_model_projection'] for e in source['experts'].values()):reason='model_projection_invalid'
  elif times[0]<ref.time_ns[0] or times[-1]>ref.time_ns[-1]:reason='reference_outside_support'
  else:
   for t in times:
    j=int(np.searchsorted(ref.time_ns,t))
    if j<len(ref.time_ns) and ref.time_ns[j]==t:continue
    if ref.time_ns[j]-ref.time_ns[j-1]>p['max_reference_gap_ns']:reason='reference_bracketing_gap'
  r['exclusion']=reason
  if reason is None:
   poses=ref.at(times);target=np.linalg.solve(poses[0],poses[1]);r['reference_delta']=target.tolist();r['loss']={c:losses(e['delta_model'],target) for c,e in source['experts'].items()}
  rows.append(r)
 valid=[r for r in rows if r['exclusion'] is None]
 groups={case:summarize([r for r in valid if r['case']==case]) for case in windows}
 records={clip:summarize([r for r in valid if windows[r['case']]['clip']==clip]) for clip in refs}
 return {'rows':rows,'aggregate':summarize(valid),'groups':groups,'recordings':records,'joint_population':323,'availability_population':380,'excluded':len(rows)-len(valid),'physical_ground_truth':False,'trained_selector':False}

def score():
 result=score_rows();write(OUT/'scores.json',result);print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['freeze','metadata','metadata_audit','predict','poses','score','verify']);a=p.parse_args()
 if a.action in ['metadata','poses']:intake(a.action)
 else:globals()[a.action]()

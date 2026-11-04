"""EXP128 metadata-first frozen five-recording development sampling."""
import argparse
from decimal import Decimal,InvalidOperation
import hashlib,json,subprocess,sys,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/five-recording-survey/run-001'
PLAN=ROOT/'experiments/five-recording-survey-2026-11-04.json'
INDEX=ROOT/'results/exposed-data-adequacy/archive-metadata-001'
STREAMS={'rgb_left':'multisense_left_rect_color','rgb_right':'multisense_right_rect',
 'thermal_left':'thermal_left_processed','thermal_right':'thermal_right_processed'}
CALIB=['multisense_left_rect_color_camera_info','multisense_right_camera_info',
       'thermal_left_processed_camera_info','thermal_right_processed_camera_info']
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from Scripts.Setup.prepare_td3_sample import pair_streams
from Scripts.Setup.td3_batch_screen import RemoteZIP

def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')


def stream_records(text,stream,names):
 records=[];seen=set();stats={'rows':0,'nonfinite':0,'missing_image':0,'duplicate_ns':0,'out_of_order':0,'unparseable':0}
 previous=None
 for i,line in enumerate(text.splitlines()):
  stats['rows']+=1
  try:stamp=Decimal(line.strip())
  except InvalidOperation:stats['unparseable']+=1;continue
  if not stamp.is_finite():stats['nonfinite']+=1;continue
  ns=int((stamp*1000000000).to_integral_value());path=f'sensors/{stream}/{i:08d}.png'
  if path not in names:stats['missing_image']+=1;continue
  if ns in seen:stats['duplicate_ns']+=1;continue
  if previous is not None and ns<previous:stats['out_of_order']+=1
  previous=ns;seen.add(ns);records.append({'index':i,'time_ns':ns,'path':path})
 records.sort(key=lambda r:(r['time_ns'],r['index']));stats['retained']=len(records)
 return records,stats


def select_thirds(paired,rgb,frames=100):
 if not rgb:return [{'third':i,'status':'NO_RGB_ACQUISITIONS','rows':[]} for i in range(3)]
 lower,upper=rgb[0]['time_ns'],rgb[-1]['time_ns']+1;result=[]
 for third in range(3):
  lo=lower+(upper-lower)*third//3;hi=lower+(upper-lower)*(third+1)//3;chosen=None
  for start in range(max(0,len(paired)-frames+1)):
   block=paired[start:start+frames]
   if not all(lo<=r['rgb_left']['time_ns']<hi for r in block):continue
   if not all(all(0<b[k]['time_ns']-a[k]['time_ns']<=150000000 for a,b in zip(block,block[1:])) for k in STREAMS):continue
   if not all(max(a[c+'_left']['time_ns'] for c in ['rgb','thermal'])<min(b[c+'_left']['time_ns'] for c in ['rgb','thermal']) for a,b in zip(block,block[1:])):continue
   chosen=(start,block);break
  result.append({'third':third,'rgb_elapsed_bounds_ns':[lo,hi],
    'status':'QUALIFIED' if chosen else 'NO_100_GROUP_WINDOW_UNDER_FROZEN_RULE',
    'manifest_start':chosen[0] if chosen else None,'rows':chosen[1] if chosen else []})
 return result


def freeze():
 assert not PLAN.exists() and not OUT.exists()
 old=read(INDEX/'plan.json');pins={}
 for p in [INDEX/'plan.json',INDEX/'summary.json',INDEX/'intake-lock.json',INDEX/'audit.json',
   ROOT/'experiments/recording-exposure-ledger-2026-10-31.json',
   ROOT/'experiments/guarded-probe-cache-2026-11-03-fixture-v2.json',
   ROOT/'docs/research/five-recording-survey-protocol-2026-11-04.md',
   ROOT/'docs/research/geometry-support-transfer-protocol-2026-11-02.md',
   ROOT/'experiments/EXP-035-waypoint-skip2-readiness.md',ROOT/'experiments/EXP-036-night-batch-screen.md',
   ROOT/'MAC-VO-main/Scripts/Setup/prepare_td3_sample.py',ROOT/'MAC-VO-main/Scripts/Setup/td3_batch_screen.py',
   ROOT/'MAC-VO-main/Scripts/Setup/geometry_support_replay.py',ROOT/'MAC-VO-main/Scripts/Setup/leased_handover.py',
   ROOT/'tools/five_recording_survey.py',ROOT/'tools/five_recording_survey_audit.py',
   ROOT/'MAC-VO-main/Scripts/UnitTest/test_five_recording_survey.py']:
  pins[str(p.relative_to(ROOT))]=sha(p)
 for p in INDEX.rglob('*'):
  if p.is_file():pins[str(p.relative_to(ROOT))]=sha(p)
 for name in ['MACVO_FrontendCov.pth','MACT_Offroad.pth']:
  p=ROOT/'models/macslam'/name;pins[str(p.relative_to(ROOT))]=sha(p)
 plan={'date':'2026-11-04','parent_commit':'d8de5b4b2f64f1ca5011772b5782a26d7155652e','archives':old['archives'],
  'frames':100,'thirds':3,'stereo_max_ns':2000000,'cross_max_ns':30000000,'max_gap_ns':150000000,
  'metadata_member_max_bytes':2097152,'input_source_sha256':pins,'stage':'SAMPLING_FROZEN_BEFORE_NEW_IMAGES',
  'candidate_changed':False,'reserved_payload_access':False,'pose_reference_read':False,'test_assignment':False,
  'runtime_image':'sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09'}
 assert len(plan['archives'])==5
 write(PLAN,plan);write(OUT/'plan.json',plan);print('FROZEN fivearchives/15thirds;',len(pins),'pins')


def verify():
 plan=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for path,digest in plan['input_source_sha256'].items():assert sha(ROOT/path)==digest,path
 return plan


def metadata():
 plan=verify();reports=[]
 for archive in plan['archives']:
  clip=archive['Name'][:-4];dest=OUT/'metadata'/clip;dest.mkdir(parents=True,exist_ok=False)
  command=['rclone','--config','/root/.config/rclone/ore-ms-recovery.conf','--box-root-folder-id','428559630945',
           'cat','ore_ms:'+archive['Name'],'--contimeout','15s','--timeout','90s','--retries','2']
  reader=RemoteZIP(command,archive['Size'],dest/'range-cache')
  prior=read(INDEX/archive['Name']/'report.json')
  for r in prior['ranges']:
   data=(ROOT/r['file']).read_bytes();assert hashlib.sha256(data).hexdigest()==r['sha256']
   reader.blocks[r['offset'],r['count']]=data
  with zipfile.ZipFile(reader) as z:
   names=z.namelist();assert len(names)==len(set(names)) and all(n.startswith(clip+'/') and '..' not in Path(n).parts for n in names)
   relative={n.split('/',1)[1] for n in names if '/' in n}
   wanted=[f'sensors/{name}/timestamps.txt' for name in list(STREAMS.values())+['thermal_left','thermal_right']]
   wanted += [f'sensors/{name}/data.yaml' for name in CALIB]
   missing=[p for p in wanted if clip+'/'+p not in names];infos=[z.getinfo(clip+'/'+p) for p in wanted if p not in missing]
   assert all(i.file_size<=plan['metadata_member_max_bytes'] for i in infos),'metadata exceeds fixed cap'
   reader.prefetch(infos);files={}
   for info in infos:
    data=z.read(info);path=dest/'source'/info.filename.split('/',1)[1];path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('xb') as f:f.write(data)
    files[str(path.relative_to(ROOT))]={'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data),'zip_crc32':info.CRC,
      'member':info.filename,'header_offset':info.header_offset,'compressed_bytes':info.compress_size}
  streams={};stats={}
  for key,name in STREAMS.items():
   p=dest/'source/sensors'/name/'timestamps.txt'
   streams[key],stats[key]=stream_records(p.read_text(),name,relative) if p.exists() else ([],{'missing_metadata':True})
  pairs=pair_streams(streams,plan['stereo_max_ns'],plan['cross_max_ns'])
  windows=select_thirds(pairs,streams['rgb_left'],plan['frames'])
  if missing:
   for w in windows:
    w['otherwise_status']=w['status'];w['status']='MISSING_REQUIRED_METADATA';w['rows']=[];w['manifest_start']=None
  report={'recording_id':'td3/20251027/'+clip,'archive':archive['Name'],'missing_metadata':missing,
    'stream_stats':stats,'paired_groups':len(pairs),'streams':streams,'paired':pairs,'windows':windows,'files':files,
    'range_requests':reader.requests,'image_decoded':False,'native_executed':False}
  write(dest/'report.json',report);reports.append({'recording_id':report['recording_id'],'report':str((dest/'report.json').relative_to(ROOT)),
   'paired_groups':len(pairs),'thirds':[w['status'] for w in windows]})
  print(clip,'paired',len(pairs),'thirds',reports[-1]['thirds'],flush=True)
 write(OUT/'metadata-summary.json',{'recordings':reports,'qualified_windows':sum(w=='QUALIFIED' for r in reports for w in r['thirds']),
    'total_thirds':15,'image_decoded':False,'native_executed':False})
 write(OUT/'metadata-lock.json',{'plan_sha256':sha(PLAN),'sha256':{str(p.relative_to(ROOT)):sha(p) for p in (OUT/'metadata').rglob('*') if p.is_file() and 'range-cache' not in p.parts}})
 verify()

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['freeze','verify','metadata']);a=p.parse_args()
 if a.action=='verify':print('VERIFIED',len(verify()['input_source_sha256']),'pins')
 else:globals()[a.action]()

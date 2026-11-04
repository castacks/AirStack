"""EXP128 selected-input parity and independent native availability survey."""
import argparse,collections,json,sys,zlib
from pathlib import Path
import cv2,numpy as np,yaml
from five_recording_inputs import ROOT,OUT,CAL,STAGE,read,write,sha,stage_verify
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from Scripts.Setup.td3_msa_replay import msa_rectification
from Scripts.Setup.td3_thermal_provenance import producer_functions,preprocess


def inputs():
 stage=stage_verify();lock=read(OUT/'image-lock.json');prepared=read(OUT/'prepared-lock.json');checks=0
 def check(ok,label):
  nonlocal checks
  if not ok:raise AssertionError(label)
  checks+=1
 check(lock['stage_plan_sha256']==prepared['stage_plan_sha256']==sha(STAGE),'stage lock')
 check(prepared['source_sha256']==sha(ROOT/'tools/five_recording_prepare.py'),'preparation source')
 for path,digest in {**lock['sha256'],**prepared['sha256']}.items():check(sha(ROOT/path)==digest,path)
 for p in lock['intakes']:
  intake=read(ROOT/p)
  for path,item in intake['files'].items():
   data=(ROOT/path).read_bytes();check(len(data)==item['bytes'] and zlib.crc32(data)&0xffffffff==item['zip_crc32'],'source ZIP member integrity')
 cv2.setNumThreads(1);cameras,t,rect,maps=msa_rectification(CAL/'msa_cal')
 cfg=yaml.safe_load((CAL/'upstream/get_thermal_data.yaml').read_text());f=producer_functions(CAL/'upstream/get_thermal_data.py');oldcal=f['read_kalibr_stereo'](cfg)
 controls=read(OUT/'preparation-controls.json');check(len(controls['original_producer_controls'])==800,'all producer controls')
 check(prepared['preparation_controls_sha256']==sha(OUT/'preparation-controls.json'),'controls hash')
 for w in stage['windows']:
  target=ROOT/w['dataset'];source=OUT/'raw'/w['clip'];check(read(target/'manifest.json')['pairs']==w['rows'],'unchanged acquisition rows')
  for i,side in enumerate(['left','right']):
   info=yaml.safe_load((target/'sensors'/('thermal_'+side+'_processed_camera_info')/'data.yaml').read_text())
   check(np.array_equal(np.asarray(info['p']),rect[i+2]) and np.array_equal(np.asarray(info['r']),rect[i]),'frozen MSA source geometry')
  for n,(row,raw) in enumerate(zip(w['rows'],w['raw_associations'])):
   rawimages=[cv2.imread(str(source/raw[s]['path']),cv2.IMREAD_UNCHANGED) for s in ['left','right']]
   old=f['stereo_rectify'](*[a.copy() for a in rawimages],oldcal)
   for i,side in enumerate(['left','right']):
    check(raw[side]['time_ns']==row['thermal_'+side]['time_ns'],'exact raw source timestamp')
    check(rawimages[i].dtype==np.uint16 and rawimages[i].shape==(512,640),'raw native schema')
    original=preprocess(old[i],f,cfg['process'],cfg['enhance']);given=cv2.imread(str(source/row['thermal_'+side]['path']),cv2.IMREAD_UNCHANGED)
    expected=original if given.ndim==2 else np.repeat(original[:,:,None],3,axis=2)
    check(np.array_equal(given,expected),'exact original producer output')
    prepared_image=cv2.imread(str(target/row['thermal_'+side]['path']),cv2.IMREAD_UNCHANGED)
    recomputed=preprocess(cv2.remap(rawimages[i],*maps[i],cv2.INTER_LINEAR),f,cfg['process'],cfg['enhance'])
    check(np.array_equal(prepared_image,np.repeat(recomputed[:,:,None],3,axis=2)),'exact source MSA preparation')
    check(sha(source/row['rgb_'+side]['path'])==sha(target/row['rgb_'+side]['path']),'RGB unchanged')
 write(OUT/'input-audit.json',{'pass':True,'checks':checks,'windows':4,'stereo_groups':400,
  'original_thermal_controls':800,'prepared_thermal_controls':800,'reference_labels_read':False,
  'prepared_lock_sha256':sha(OUT/'prepared-lock.json')})
 print('PASS INPUTS',checks,'checks;800original+800MSAthermal controls')


def native():
 stage=stage_verify();lock=read(OUT/'capture-lock.json');checks=0;rows=[];groups={};states=[]
 def check(ok,label):
  nonlocal checks
  if not ok:raise AssertionError(label)
  checks+=1
 check(lock['stage_plan_sha256']==sha(STAGE) and lock['prepared_lock_sha256']==sha(OUT/'prepared-lock.json'),'capture locks')
 for path,digest in lock['sha256'].items():check(sha(ROOT/path)==digest,path)
 check(read(OUT/'input-audit.json')['pass'] is True,'input audit')
 def summary(rr):
  masks=collections.Counter(r['mask'] for r in rr);R=masks['rgb_only']+masks['both'];T=masks['thermal_only']+masks['both'];U=len(rr)-masks['none']
  return {'slots':len(rr),'masks':{k:masks[k] for k in ['none','rgb_only','thermal_only','both']},
   'rgb_eligible':R,'thermal_eligible':T,'union_eligible':U,'headroom_above_best_fixed':U-max(R,T)}
 for w in stage['windows']:
  base=OUT/'captures'/w['case'];log=[json.loads(line) for line in (base/'timeline.jsonl').read_text().splitlines()]
  dec={e['pair_ordinal']:e for e in log if e['kind']=='decision'};health={};motion={};outcomes={}
  for e in log:
   if e['kind']!='observer_event':continue
   event=e['event'];key=e['camera'],event.get('time1_ns')
   if event['kind']=='pair_end':check(key not in health,'unique health');health[key]=e
   elif event['kind']=='motion_result_written' and event['first_write_for_submission']:
    check(key not in motion,'unique first motion');motion[key]=e
  for c in ['rgb','thermal']:
   report=read(base/'experts'/c/'report.json');terminal=read(base/'experts'/c/'terminal-capture-contract.json')
   check(report['completed_frames']==100,'all frozen source frames')
   if report['status']=='FAILED':
    check(terminal['native_status']=='FAILED' and terminal['all_frames_processed'] and not terminal['motion_available'],'native failure preserved')
    check(report['tracking_diagnostics']['retained_observations']==0 and report['tracking_diagnostics']['support_state']=='NO_PAIRS_SUPPORTED','only strict native zero-match capture')
   else:check(terminal['native_status']=='RETURNED','native returned')
   values=[]
   for i in range(99):
    interval=[w['rows'][j][c+'_left']['time_ns'] for j in [i,i+1]];h=health[c,interval[1]]
    check([h['event']['time0_ns'],h['event']['time1_ns']]==interval,'dense original endpoints')
    values.append(h['event']['original_need_interp'] is False and h['event']['execution_error'] is None)
   runs=[]
   for i,v in enumerate(values):
    if runs and runs[-1]['supported']==v:runs[-1]['end_pair']=i
    else:runs.append({'start_pair':i,'end_pair':i,'supported':v})
   robust=sum(a['supported'] and not b['supported'] and d['supported'] and min(x['end_pair']-x['start_pair']+1 for x in [a,b,d])>=3 for a,b,d in zip(runs,runs[1:],runs[2:]))
   states.append({'case':w['case'],'camera':c,'native_status':report['status'],'runs':runs,'sustained_cycles':robust})
   outcomes[c]=report['status']
  rr=[]
  for i in range(2,97):
   d=dec[i];B=d['controller_monotonic_ns'];check(d['boundary']=={'kind':'group_complete','group':i+2},'ordinary deadline')
   done=[e for e in log if e['kind']=='completed' and e['frame']==i+2];check(len(done)==2 and max(e['controller_monotonic_ns'] for e in done)<=B,'actual processing boundary')
   cameras={}
   for c in ['rgb','thermal']:
    interval=[w['rows'][j][c+'_left']['time_ns'] for j in [i,i+1]];h=health[c,interval[1]];m=motion.get((c,interval[1]))
    supported=h['event']['execution_error'] is None and h['event']['original_need_interp'] is False
    good=bool(supported and m and h['controller_monotonic_ns']<=B and m['controller_monotonic_ns']<=B and
      m['event']['time0_ns']==interval[0] and m['event']['time1_ns']==interval[1] and m['event']['stage']=='pair_processing' and
      m['event']['original_need_interp'] is False and m['event']['derived_camera_delta_xyzw'] is not None)
    check(good==d['availability'][c],'independent deadline eligibility')
    cameras[c]={'eligible':good,'supported':supported,'native_interval_ns':interval,
       'health_receipt_ns':h['controller_monotonic_ns'],'motion_receipt_ns':m['controller_monotonic_ns'] if m else None}
   mask='both' if all(x['eligible'] for x in cameras.values()) else 'rgb_only' if cameras['rgb']['eligible'] else 'thermal_only' if cameras['thermal']['eligible'] else 'none'
   row={'case':w['case'],'recording_id':w['recording_id'],'pair':i,'deadline_ns':B,'experts':cameras,'mask':mask,
       'raw_advisory_action':d['action']};rr.append(row);rows.append(row)
  groups[w['case']]=dict(summary(rr),recording_id=w['recording_id'],native_status=outcomes)
 records={r:summary([x for x in rows if x['recording_id']==r]) for r in sorted({x['recording_id'] for x in rows})}
 check(len(rows)==380 and len(groups)==4 and len(records)==2,'qualified-window population only')
 write(OUT/'native-survey.json',{'groups':groups,'recordings':records,'rows':rows,'health_windows':states,
   'checks':checks,'metadata_qualified_thirds':4,'metadata_ineligible_thirds':11,'recordings_native_measured':2,
   'candidate_changed':False,'pose_reference_read':False,'development_exposed':True,'reserved_payload_access':False,
   'scope':'conditional native camera-local availability; no adaptive policy or pose accuracy claim'})
 write(OUT/'native-audit.json',{'pass':True,'checks':checks,'core_rows':380,'native_experts':8,
   'survey_sha256':sha(OUT/'native-survey.json'),'input_audit_sha256':sha(OUT/'input-audit.json')})
 print('PASS NATIVE',checks,'checks')
 for rec,g in records.items():print(rec,g)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['inputs','native']);a=p.parse_args();globals()[a.action]()

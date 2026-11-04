"""EXP128 stage2 exact raw association, immutable inputs and conditional native capture."""
import argparse,hashlib,json,shutil,sys,zipfile,subprocess
from pathlib import Path
from five_recording_survey import ROOT,OUT,INDEX,STREAMS,CALIB,read,write,sha,verify,stream_records,RemoteZIP
STAGE=OUT/'stage2-plan.json';CAL=OUT/'calibration-recovery/selected/ore-ms/data/calibration'


def freeze():
 plan=verify();summary=read(OUT/'metadata-summary.json');audit=read(OUT/'preflight-audit.json')
 assert audit['qualified_windows']==summary['qualified_windows']==4
 windows=[];pins={}
 for entry in summary['recordings']:
  report=read(ROOT/entry['report']);clip=report['archive'][:-4]
  directory=read(INDEX/report['archive']/'members.json');names={e['name'].split('/',1)[1] for e in directory if '/' in e['name']}
  raw={}
  for side in ['left','right']:
   p=ROOT/entry['report'];p=p.parent/'source/sensors'/('thermal_'+side)/'timestamps.txt'
   rows,stats=stream_records(p.read_text(),'thermal_'+side,names);raw[side]={r['time_ns']:r for r in rows};pins[str(p.relative_to(ROOT))]=sha(p)
  for w in report['windows']:
   if w['status']!='QUALIFIED':continue
   associations=[]
   for row in w['rows']:
    assoc={}
    for side in ['left','right']:
     time=row['thermal_'+side]['time_ns']
     if time not in raw[side]:raise ValueError('no exact rawthermal timestamp association; no fallback')
     assoc[side]=raw[side][time]
    associations.append(assoc)
   for side in ['left','right']:assert len({p[side]['index'] for p in associations})==100
   windows.append({'case':clip+'/'+['early','middle','late'][w['third']],'clip':clip,'archive':report['archive'],
     'recording_id':report['recording_id'],'third':w['third'],'rows':w['rows'],'raw_associations':associations,
     'metadata_source':entry['report'],'dataset':str((OUT/'prepared'/clip/str(w['third'])).relative_to(ROOT))})
 for p in [OUT/'metadata-lock.json',OUT/'metadata-summary.json',OUT/'preflight-audit.json',
  ROOT/'tools/five_recording_inputs.py',ROOT/'tools/five_recording_prepare.py',ROOT/'tools/five_recording_native_audit.py']:
  pins[str(p.relative_to(ROOT))]=sha(p)
 for p in CAL.rglob('*'):
  if p.is_file():pins[str(p.relative_to(ROOT))]=sha(p)
 for p in (ROOT/'MAC-VO-main').rglob('*.py'):
  if '__pycache__' not in p.parts:pins[str(p.relative_to(ROOT))]=sha(p)
 for p in (ROOT/'MAC-VO-main/Config').rglob('*.yaml'):pins[str(p.relative_to(ROOT))]=sha(p)
 write(STAGE,{'stage':'FROZEN_BEFORE_SELECTED_IMAGES_AND_NATIVE_OUTCOMES','parent_plan_sha256':sha(OUT/'plan.json'),
   'windows':windows,'input_source_sha256':pins,'runtime_image':plan['runtime_image'],'frames':100,'expected_dual_captures':4,
   'preprocessing':'unchanged originalproducer exact replay then existing MSA raw-first rectify/process/enhance',
   'raw_association':'exact integer-ns equality, no fallback','reference_labels_read':False,'candidate_changed':False})
 print('STAGE2 FROZEN',len(windows),'windows;',len(pins),'source/metadata pins')


def stage_verify():
 verify();plan=read(STAGE)
 assert plan['parent_plan_sha256']==sha(OUT/'plan.json')
 for n,h in plan['input_source_sha256'].items():assert sha(ROOT/n)==h,n
 return plan


def images():
 stage=stage_verify();reports=[]
 for clip in sorted({w['clip'] for w in stage['windows']}):
  windows=[w for w in stage['windows'] if w['clip']==clip];archive=next(e for e in read(OUT/'plan.json')['archives'] if e['Name']==windows[0]['archive'])
  dest=OUT/'raw'/clip;dest.mkdir(parents=True,exist_ok=False)
  cmd=['rclone','--config','/root/.config/rclone/ore-ms-recovery.conf','--box-root-folder-id','428559630945','cat','ore_ms:'+archive['Name'],
       '--contimeout','15s','--timeout','90s','--retries','2']
  reader=RemoteZIP(cmd,archive['Size'],OUT/'image-range-cache'/clip)
  for r in read(INDEX/archive['Name']/'report.json')['ranges']:reader.blocks[r['offset'],r['count']]=(ROOT/r['file']).read_bytes()
  with zipfile.ZipFile(reader) as z:
   selected=set()
   for w in windows:
    for row,raw in zip(w['rows'],w['raw_associations']):
     selected.update(row[k]['path'] for k in STREAMS);selected.update(raw[side]['path'] for side in ['left','right'])
   infos=[z.getinfo(clip+'/'+rel) for rel in sorted(selected)];print('ACQUIRE',clip,len(infos),'PNG members',flush=True)
   reader.prefetch(infos);files={}
   for info in infos:
    data=z.read(info);rel=info.filename.split('/',1)[1];p=dest/rel;p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('xb') as f:f.write(data)
    files[str(p.relative_to(ROOT))]={'member':info.filename,'sha256':hashlib.sha256(data).hexdigest(),'zip_crc32':info.CRC,'bytes':len(data)}
  report={'clip':clip,'files':files,'range_requests':reader.requests,'image_decoded':False,'native_executed':False}
  write(dest/'image-intake.json',report);reports.append(str((dest/'image-intake.json').relative_to(ROOT)))
  print('COMPLETE IMAGE INTAKE',clip,flush=True)
 write(OUT/'image-lock.json',{'stage_plan_sha256':sha(STAGE),'intakes':reports,
     'sha256':{n:v['sha256'] for p in reports for n,v in read(ROOT/p)['files'].items()}})
 stage_verify()


def capture():
 stage=stage_verify();lock=read(OUT/'prepared-lock.json')
 for n,h in lock['sha256'].items():assert sha(ROOT/n)==h,n
 for w in stage['windows']:
  dest=OUT/'captures'/w['case'];state=dest.parent/(dest.name+'-execution.json')
  if state.exists():assert read(state)['exit_code']==0,'failed native attempt retained';continue
  assert not dest.exists(),'partial capture retained; no retry';dest.parent.mkdir(parents=True,exist_ok=True)
  command=['docker','run','--rm','--gpus','all','--ipc=host','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1',
    '-v',str(ROOT)+':/workspace','-w','/workspace/MAC-VO-main',stage['runtime_image'],'python','-m','Scripts.Setup.geometry_support_replay',
    '--dataset','/workspace/'+w['dataset'],'--manifest','/workspace/'+w['dataset']+'/manifest.json',
    '--output','/workspace/'+str(dest.relative_to(ROOT)),'--frames','100','--start','0','--record-root-initialization','--acknowledge-provisional-geometry']
  print('CAPTURE',w['case'],flush=True)
  with (dest.parent/(dest.name+'-host.log')).open('x') as f:result=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT)
  write(state,{'command':command,'exit_code':result.returncode})
  if result.returncode:raise RuntimeError('native capture failed, no retry: '+str(dest))
  stage_verify();print('COMPLETE',w['case'],flush=True)
 write(OUT/'capture-lock.json',{'stage_plan_sha256':sha(STAGE),'prepared_lock_sha256':sha(OUT/'prepared-lock.json'),
   'reference_labels_read':False,'sha256':{str(p.relative_to(ROOT)):sha(p) for p in (OUT/'captures').rglob('*') if p.is_file()}})

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['freeze','images','capture','verify']);a=p.parse_args()
 if a.action=='verify':print('STAGE2 VERIFIED',len(stage_verify()['input_source_sha256']),'pins')
 else:globals()[a.action]()

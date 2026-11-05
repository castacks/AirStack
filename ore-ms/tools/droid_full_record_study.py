"""EXP141 controls and full-record input preparation; no actual reference reader."""
import argparse
from copy import deepcopy
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/bounded-full-record'
PREFIX=ROOT/'experiments/droid-bounded-prefix-2026-11-04.json'
ACTIVE_PREFIX=ROOT/'experiments/droid-bounded-prefix-2026-11-04-v2.json'


def read(p):return json.loads(Path(p).read_text())
def sha(p):
 import hashlib
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')


def capacity():
 probe=read(ROOT/'results/whole-record-preflight/run-001/capacity-probe.json')
 return {'mode':'quarter_memory_persistent_state_cap','preflight_gpu_total_bytes':probe['gpu_total_bytes'],
 'worst_state_bytes_per_slot':4496290,'slots':2048,'reserved_slots':17,
 'scope':'persistent-state cap only; backend workspace/complete run peak unproven'}


def native_sources():
 return {name:sha(Path('/root/ore-ms-recovery/droid-source/droid_slam')/name)
  for name in ['droid.py','depth_video.py','motion_filter.py','droid_frontend.py','trajectory_filler.py']}


def prefix_freeze():
 parent=ROOT/'experiments/droid-longer-prefix-2026-11-04.json';p=deepcopy(read(parent))
 p.update(parent_commit='16a6586e9',scope='bounded allocation101source controls only',bounded_capacity=capacity(),native_source_sha256=native_sources())
 p['settings']['buffer']=2048
 for case in p['runs']:case['source_frames']=101
 for f in [parent,Path(__file__),ROOT/'tools/droid_bounded_worker.py',ROOT/'tools/tests/test_droid_bounded_worker.py',
 ROOT/'results/whole-record-preflight/run-001/capacity-probe.json']:
  p['source_sha256'][str(f.relative_to(ROOT))]=sha(f)
 write(PREFIX,p);write(OUT/'prefix-lock.json',p)
 control=deepcopy(p);control['runs']=[next(c for c in p['runs'] if c['case']=='day-rgb')]
 control['bounded_capacity']['mode']='declared_101_day_rgb_32_slot_overflow_control';control['bounded_capacity']['slots']=32
 control['settings']['buffer']=32;control['scope']='fixed dayRGB101source overflow control, not a full recording'
 write(ROOT/'experiments/droid-bounded-overflow-control-2026-11-04.json',control)
 print('FROZEN bounded2048prefix sixcases and fixed32slot dayRGBoverflow control')


def manifest(condition):
 from full_record_intake import verify
 p=verify();c=next(c for c in p['cases'] if c['condition']==condition)
 acquired=read(OUT/'acquisition'/(condition+'.json'));byid={(r['camera'],r['side'],r['stem']):r for r in acquired['camera_members']}
 if len(byid)!=4*c['source_groups'] or not acquired['all_original_members']:raise ValueError('incomplete acquisition')
 old=read(ROOT/c['prefix_manifest']);times={}
 for m in ['rgb','thr']:
  for side in ['left','right']:
   times[(m,side)]=[int(v) for v in (ROOT/c['source_base']/'pilot/sync_data'/c['sequence']/m/('img_'+side+'_timestamp.txt')).read_text().split()]
 records=[]
 for i in range(c['source_groups']):
  stem=f'{i:06}';row={'stem':stem,'cameras':deepcopy(old['records'][0]['cameras'])}
  for m in ['rgb','thr']:
   for side in ['left','right']:
    r=byid[(m,side,stem)];target=row['cameras'][m][side]
    target.update(path=r['path'],sha256=r['sha256'],recorded_time_ns=times[(m,side)][i])
    if i<101:
     previous=old['records'][i]['cameras'][m][side]
     for key in ['sha256','recorded_time_ns','shape','dtype']:
      if target[key]!=previous[key]:raise ValueError('first101 raw source mismatch '+key)
  records.append(row)
 write(OUT/'manifests'/(condition+'.json'),{'sequence':c['sequence'],'records':records,
 'intake_plan_sha256':sha(ROOT/'experiments/full-record-intake-2026-11-04.json'),
 'scope':'complete original visual-only provided-pair development source; no reference paths'})
 print('MANIFEST',condition,len(records),'groups;404prefix source identities unchanged')


def prepare(case_id):
 import cv2
 from orb_ms2_development import prepare as canonical_prepare
 cv2.setNumThreads(1)
 old=next(c for c in read(ROOT/'experiments/droid-longer-prefix-2026-11-04.json')['runs'] if c['case']==case_id)
 source=OUT/'manifests'/(old['condition']+'.json');dest=OUT/'canonical'/case_id
 canonical_prepare(ROOT,source,'rgb' if old['modality']=='rgb' else 'thr',dest,0)
 for side in ['left','right']:
  for i in range(101):
   name=f'{i:06}.png'
   if sha(dest/side/name)!=sha(ROOT/old['canonical']/side/name):raise ValueError('prefix canonical bytes changed')
 print('PREPARED',case_id,'complete canonical stream,202first101PNGbyte controls exact')


def full_freeze():
 prefix=read(ACTIVE_PREFIX);intake=read(ROOT/'experiments/full-record-intake-2026-11-04.json');plans=[]
 for old in prefix['runs']:
  case=deepcopy(old);dest=OUT/'canonical'/case['case'];manifest=OUT/'manifests'/(case['condition']+'.json')
  source=read(manifest);case.update(manifest=str(manifest.relative_to(ROOT)),canonical=str(dest.relative_to(ROOT)),source_frames=len(source['records']))
  files=[manifest,ROOT/'models/droid/droid.pth',ROOT/'tools/droid_ms2_development.py',ROOT/'tools/droid_bounded_worker.py',
  Path(__file__),ROOT/case['calibration'],dest/'plan.json',ROOT/'experiments/full-record-intake-2026-11-04.json',ACTIVE_PREFIX]
  for side in ['left','right']:files.extend(dest/side/(r['stem']+'.png') for r in source['records'])
  p={k:deepcopy(prefix[k]) for k in ['runtime_image','weights','resize_or_crop','uninitialized_backend_guard',
  'internal_stereo_baseline','native_time_unit_ns','metric_translation_rule','settings','bounded_capacity','native_source_sha256']}
  p.update(date='2026-11-04 America/New_York',scope='complete original recording, conditional provided-pair offline DROID engineering',
   runs=[case],source_frames=case['source_frames'],seed=0,reference_used=False,scale_fitted=False,protected_payload_access=False,
   benchmark_execution_ready=False,intake_plan_sha256=sha(ROOT/'experiments/full-record-intake-2026-11-04.json'),
   source_sha256={str(f.relative_to(ROOT)):sha(f) for f in files})
  path=OUT/'protocols'/(case['case']+'.json');write(path,p)
  plans.append({'case':case['case'],'protocol':str(path.relative_to(ROOT)),'sha256':sha(path),'source_frames':case['source_frames']})
 write(ROOT/'experiments/droid-bounded-full-record-2026-11-04.json',{'runs':plans,'all_recordings_and_modalities_retained':True,
 'source_groups':sum(c['source_groups'] for c in intake['cases']),'capacity':capacity(),'full_protocols_frozen_before_any_full_native_run':True,
 'reference_scoring':False,'physical_sync_verified':False,'benchmark_execution_ready':False})
 print('FULL FROZEN',[(p['case'],p['source_frames']) for p in plans])


if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('command',choices=['prefix-freeze','manifest','prepare','full-freeze']);ap.add_argument('--case');ap.add_argument('--condition');a=ap.parse_args()
 if a.command=='prefix-freeze':prefix_freeze()
 elif a.command=='manifest':manifest(a.condition)
 elif a.command=='prepare':prepare(a.case)
 else:full_freeze()

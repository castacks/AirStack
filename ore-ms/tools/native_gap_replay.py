"""Freeze and capture EXP118 open-loop native gaps; never overwrite attempts."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from Scripts.Setup.native_gap_schedule import select_frames,mapped_manifest
OUT=ROOT/'results/native-gap-replay/run-001'
PLAN=ROOT/'experiments/native-gap-replay-2026-11-03.json'
IMAGE='sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09'

def read(p):return json.loads(Path(p).read_text())
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def freeze():
 assert not PLAN.exists() and not OUT.exists()
 parent=read(ROOT/'experiments/guarded-probe-cache-2026-11-03-fixture-v2.json')
 source=next(s for s in parent['sources'] if s['tag']=='exp112')
 schedules=read(ROOT/'results/guarded-probe-cache/run-002/locked-schedules.json')['windows_policies']
 cases=[];pins={};refpins={}
 for case in source['cases']:
  if case['window'] not in ['early','middle','day']:continue
  dataset=ROOT/case['dataset'];original=read(dataset/'manifest.json');assert original['pairs']==case['rows']
  schedule=next(s for s in schedules if s['window']=='exp112/'+case['window'] and s['mode']=='bounded_probe')
  for camera in ['rgb','thermal']:
   for arm in ['dense','sparse']+(['dense-repeat'] if case['window']=='early' else []):
    indices=list(range(case['frames'])) if arm!='sparse' else select_frames(case['rows'],schedule['decisions'],camera)
    manifest,mapping=mapped_manifest(original,indices,camera)
    base=OUT/'inputs'/case['window']/camera/arm
    write(base/'manifest.json',manifest);write(base/'mapping.json',mapping)
    cases.append({'window':case['window'],'camera':camera,'arm':arm,'dataset':case['dataset'],
                  'manifest':str((base/'manifest.json').relative_to(ROOT)),
                  'mapping':str((base/'mapping.json').relative_to(ROOT)),
                  'frames':len(indices),'reference_dataset':case['reference_dataset'],
                  'nominal_body_from_camera':case['nominal_body_from_camera'],
                  'capture':str((OUT/'captures'/case['window']/camera/arm).relative_to(ROOT))})
    for n in ['manifest.json','mapping.json']:pins[str((base/n).relative_to(ROOT))]=sha(base/n)
  for p in [dataset/'manifest.json',*dataset.glob('sensors/*_camera_info/data.yaml')]:pins[str(p.relative_to(ROOT))]=sha(p)
  for row in case['rows']:
   for k in ['rgb_left','rgb_right','thermal_left','thermal_right']:
    p=dataset/row[k]['path'];pins[str(p.relative_to(ROOT))]=sha(p)
  for n in ['info.yaml','interp_data.txt','interp_timestamps.txt']:
   p=ROOT/case['reference_dataset']/'super_odometry/odometry'/n;refpins[str(p.relative_to(ROOT))]=sha(p)
 for p in (ROOT/'MAC-VO-main').rglob('*.py'):
  if not any(s in p.parts for s in ['.git','__pycache__']):pins[str(p.relative_to(ROOT))]=sha(p)
 for p in (ROOT/'MAC-VO-main/Config').rglob('*.yaml'):pins[str(p.relative_to(ROOT))]=sha(p)
 for n in ['models/macslam/MACVO_FrontendCov.pth','models/macslam/MACT_Offroad.pth',
           'tools/native_gap_replay.py','docs/research/native-gap-replay-protocol-2026-11-03.md',
           'results/guarded-probe-cache/run-002/locked-schedules.json']:
  pins[n]=sha(ROOT/n)
 plan={'date':'2026-11-03','parent_commit':'524e78734','runtime_image':IMAGE,'cases':cases,
       'input_source_sha256':pins,'scorer_only_reference_sha256':refpins,
       'scope':'fixed dense-derived open-loop native gaps; no adaptive sparse controller',
       'expected_captures':14,'training':False,'reserve_access':False,'policy_adopted':False}
 assert len(cases)==14
 write(PLAN,plan);write(OUT/'plan.json',plan);print('FROZEN',len(cases),'captures;',len(pins),'pins')

def verify():
 plan=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for n,h in plan['input_source_sha256'].items():assert sha(ROOT/n)==h,n
 return plan

def capture():
 plan=verify()
 for c in plan['cases']:
  dest=ROOT/c['capture'];state=dest.parent/(c['arm']+'-execution.json')
  if state.exists():
   assert read(state)['exit_code']==0,'retained failed capture; no automatic retry'
   continue
  if dest.exists():raise RuntimeError('partial capture retained: '+str(dest))
  dest.parent.mkdir(parents=True,exist_ok=True)
  cmd=['docker','run','--rm','--gpus','all','--ipc=host',
       '-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1',
       '-v',str(ROOT)+':/workspace','-w','/workspace/MAC-VO-main',IMAGE,
       'python','-m','Scripts.Setup.native_gap_capture',
       '--dataset','/workspace/'+c['dataset'],'--manifest','/workspace/'+c['manifest'],
       '--output','/workspace/'+c['capture'],'--frames',str(c['frames']),'--modality',c['camera']]
  print('CAPTURE',c['window'],c['camera'],c['arm'],c['frames'],flush=True)
  with (dest.parent/(c['arm']+'-host.log')).open('x') as log:
   result=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
  write(state,{'command':cmd,'exit_code':result.returncode})
  if result.returncode:raise RuntimeError('capture failed; retained: '+str(dest))
  verify();print('COMPLETE',c['window'],c['camera'],c['arm'],flush=True)
 write(OUT/'capture-lock.json',{'captures':len(plan['cases']),
      'sha256':{str(p.relative_to(ROOT)):sha(p) for p in (OUT/'captures').rglob('*') if p.is_file()},
      'pose_reference_read_during_capture':False,'plan_sha256':sha(PLAN)})

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['freeze','verify','capture']);a=p.parse_args()
 if a.action=='verify':print('VERIFIED',len(verify()['input_source_sha256']),'pins')
 else:globals()[a.action]()

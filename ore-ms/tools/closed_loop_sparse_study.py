"""EXP119 freeze/capture on the run's own native prefix and monotonic clock."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/closed-loop-sparse/run-001'
PLAN=ROOT/'experiments/closed-loop-sparse-2026-11-03.json'
IMAGE='sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09'
def read(p):return json.loads(Path(p).read_text())
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def freeze():
 assert not PLAN.exists() and not OUT.exists()
 parent=read(ROOT/'experiments/native-gap-replay-2026-11-03.json');windows=[];pins={};priors={};refs={}
 for window in ['early','middle','day']:
  c=next(c for c in parent['cases'] if c['window']==window and c['camera']=='rgb' and c['arm']=='dense')
  manifest=ROOT/c['dataset']/'manifest.json';rows=read(manifest)['pairs'];assert len(rows)==c['frames']
  windows.append({'window':window,'dataset':c['dataset'],'manifest':str(manifest.relative_to(ROOT)),
                  'frames':len(rows),'rows':rows,'nominal_body_from_camera':c['nominal_body_from_camera'],
                  'reference_dataset':c['reference_dataset']})
  for p in [manifest,*(ROOT/c['dataset']).glob('sensors/*_camera_info/data.yaml')]:pins[str(p.relative_to(ROOT))]=sha(p)
  for row in rows:
   for key in ['rgb_left','rgb_right','thermal_left','thermal_right']:
    p=ROOT/c['dataset']/row[key]['path'];pins[str(p.relative_to(ROOT))]=sha(p)
  for camera in ['rgb','thermal']:
   old=next(x for x in parent['cases'] if x['window']==window and x['camera']==camera and x['arm']=='dense')
   for p in (ROOT/old['capture']/camera).rglob('*.npz'):priors[str(p.relative_to(ROOT))]=sha(p)
  for n in ['info.yaml','interp_data.txt','interp_timestamps.txt']:
   p=ROOT/c['reference_dataset']/'super_odometry/odometry'/n;refs[str(p.relative_to(ROOT))]=sha(p)
 for p in (ROOT/'MAC-VO-main').rglob('*.py'):
  if not any(s in p.parts for s in ['.git','__pycache__']):pins[str(p.relative_to(ROOT))]=sha(p)
 for p in (ROOT/'MAC-VO-main/Config').rglob('*.yaml'):pins[str(p.relative_to(ROOT))]=sha(p)
 for n in ['models/macslam/MACVO_FrontendCov.pth','models/macslam/MACT_Offroad.pth',
           'tools/closed_loop_sparse_study.py','docs/research/closed-loop-sparse-protocol-2026-11-03.md']:
  pins[n]=sha(ROOT/n)
 plan={'date':'2026-11-03','parent_commit':'802953d0d','windows':windows,'runtime_image':IMAGE,
       'arms':['full_read_cache','bounded_probe'],'early_repeat':True,'expected_dual_captures':7,
       'input_source_sha256':pins,'prior_dense_npz_sha256':priors,'scorer_only_reference_sha256':refs,
       'core_slots_per_arm':sum(w['frames']-5 for w in windows),'reserve_access':False,
       'training':False,'controller':'own observed native prefix and actual controller monotonic clock',
       'primary':'per-window eligible >= full, unavailable <= full, uncommitted while opportunity <= full, strictly fewer actual pair calls',
       'policy_adopted':False}
 assert plan['core_slots_per_arm']==205
 write(PLAN,plan);write(OUT/'plan.json',plan);print('FROZEN',len(pins),'pins;7 dual captures;205 core slots per arm')

def verify():
 plan=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for n,h in plan['input_source_sha256'].items():assert sha(ROOT/n)==h,n
 return plan

def capture():
 plan=verify()
 for w in plan['windows']:
  for arm in plan['arms']+(['full-read-repeat'] if w['window']=='early' else []):
   dest=OUT/'captures'/w['window']/arm;state=dest.parent/(arm+'-execution.json')
   if state.exists():
    assert read(state)['exit_code']==0,'failed attempt retained; no automatic retry';continue
   if dest.exists():raise RuntimeError('partial attempt retained: '+str(dest))
   dest.parent.mkdir(parents=True,exist_ok=True);mode='full_read_cache' if arm=='full-read-repeat' else arm
   cmd=['docker','run','--rm','--gpus','all','--ipc=host','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1',
        '-v',str(ROOT)+':/workspace','-w','/workspace/MAC-VO-main',IMAGE,
        'python','-m','Scripts.Setup.closed_loop_sparse','--dataset','/workspace/'+w['dataset'],
        '--manifest','/workspace/'+w['manifest'],'--output','/workspace/'+str(dest.relative_to(ROOT)),
        '--frames',str(w['frames']),'--mode',mode]
   print('CAPTURE',w['window'],arm,flush=True)
   with (dest.parent/(arm+'-host.log')).open('x') as log:result=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
   write(state,{'command':cmd,'exit_code':result.returncode})
   if result.returncode:raise RuntimeError('capture failed; retained: '+str(dest))
   verify();print('COMPLETE',w['window'],arm,flush=True)
 write(OUT/'capture-lock.json',{'plan_sha256':sha(PLAN),'dual_captures':7,'pose_references_read':False,
       'sha256':{str(p.relative_to(ROOT)):sha(p) for p in (OUT/'captures').rglob('*') if p.is_file()}})

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['freeze','verify','capture']);a=p.parse_args()
 if a.action=='verify':print('VERIFIED',len(verify()['input_source_sha256']),'pins')
 else:globals()[a.action]()

"""EXP120 natural-coverage audit and frozen synthetic request-scoped transition test."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/availability-transitions/run-001'
PLAN=ROOT/'experiments/availability-transitions-2026-11-03.json'
IMAGE='sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09'
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from Scripts.Setup.transition_feedback import OUTAGES
def read(p):return json.loads(Path(p).read_text())
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def freeze():
 assert not PLAN.exists() and not OUT.exists()
 parent=read(ROOT/'experiments/closed-loop-sparse-2026-11-03.json')
 window=next(w for w in parent['windows'] if w['window']=='day')
 coverage=read('/root/ore-ms-recovery/transition-coverage-audit.json');pins={};prior={};refs={}
 write(OUT/'natural-coverage.json',coverage)
 p=ROOT/'experiments/guarded-probe-cache-2026-11-03-fixture-v2.json';pins[str(p.relative_to(ROOT))]=sha(p)
 for s in read(p)['sources']:
  for c in s['cases']:
   p=ROOT/s['base']/'captures'/c['case']/'on/timeline.jsonl';pins[str(p.relative_to(ROOT))]=sha(p)
 for p in [ROOT/window['manifest'],*(ROOT/window['dataset']).glob('sensors/*_camera_info/data.yaml')]:
  pins[str(p.relative_to(ROOT))]=sha(p)
 for row in window['rows']:
  for k in ['rgb_left','rgb_right','thermal_left','thermal_right']:
   p=ROOT/window['dataset']/row[k]['path'];pins[str(p.relative_to(ROOT))]=sha(p)
 for p in (ROOT/'MAC-VO-main').rglob('*.py'):
  if not any(s in p.parts for s in ['.git','__pycache__']):pins[str(p.relative_to(ROOT))]=sha(p)
 for p in (ROOT/'MAC-VO-main/Config').rglob('*.yaml'):pins[str(p.relative_to(ROOT))]=sha(p)
 for n in ['models/macslam/MACVO_FrontendCov.pth','models/macslam/MACT_Offroad.pth',
           'tools/availability_transition_study.py','docs/research/availability-transitions-protocol-2026-11-03.md',
           'results/availability-transitions/run-001/natural-coverage.json']:
  pins[n]=sha(ROOT/n)
 for c in ['rgb','thermal']:
  for p in (ROOT/'results/closed-loop-sparse/run-001/captures/day/full_read_cache/experts'/c).rglob('*.npz'):
   prior[str(p.relative_to(ROOT))]=sha(p)
 for n in ['info.yaml','interp_data.txt','interp_timestamps.txt']:
  p=ROOT/window['reference_dataset']/'super_odometry/odometry'/n;refs[str(p.relative_to(ROOT))]=sha(p)
 plan={'date':'2026-11-03','parent_commit':'50bcd672d','window':window,'runtime_image':IMAGE,
       'outages':OUTAGES,'arms':['full_read_cache','bounded_probe','fixed_rgb','fixed_thermal'],
       'unperturbed_control':True,'expected_dual_captures':5,'core_slots_per_arm':95,
       'input_source_sha256':pins,'prior_control_sha256':prior,'scorer_only_reference_sha256':refs,
       'synthetic':True,'training':False,'reserve_access':False,'policy_adopted':False}
 write(PLAN,plan);write(OUT/'plan.json',plan);print('FROZEN',len(pins),'pins;5dual captures;95core slots/arm')

def verify():
 plan=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for n,h in plan['input_source_sha256'].items():assert sha(ROOT/n)==h,n
 return plan

def capture():
 plan=verify();w=plan['window']
 for arm in ['unperturbed']+plan['arms']:
  dest=OUT/'captures'/arm;state=OUT/'captures'/(arm+'-execution.json')
  if state.exists():
   assert read(state)['exit_code']==0,'failed attempt retained; no automatic retry';continue
  if dest.exists():raise RuntimeError('partial capture retained: '+str(dest))
  dest.parent.mkdir(parents=True,exist_ok=True)
  cmd=['docker','run','--rm','--gpus','all','--ipc=host','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1',
       '-v',str(ROOT)+':/workspace','-w','/workspace/MAC-VO-main',IMAGE,'python','-m','Scripts.Setup.availability_transitions',
       '--dataset','/workspace/'+w['dataset'],'--manifest','/workspace/'+w['manifest'],
       '--output','/workspace/'+str(dest.relative_to(ROOT)),'--frames','100',
       '--mode','full_read_cache' if arm=='unperturbed' else arm]
  if arm!='unperturbed':cmd.append('--perturb')
  print('CAPTURE',arm,flush=True)
  with (OUT/'captures'/(arm+'-host.log')).open('x') as log:result=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
  write(state,{'command':cmd,'exit_code':result.returncode})
  if result.returncode:raise RuntimeError('capture failed; retained: '+str(dest))
  verify();print('COMPLETE',arm,flush=True)
 write(OUT/'capture-lock.json',{'plan_sha256':sha(PLAN),'dual_captures':5,'pose_references_read':False,
       'sha256':{str(p.relative_to(ROOT)):sha(p) for p in (OUT/'captures').rglob('*') if p.is_file()}})

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['freeze','verify','capture']);a=p.parse_args()
 if a.action=='verify':print('VERIFIED',len(verify()['input_source_sha256']),'pins')
 else:globals()[a.action]()

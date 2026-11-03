"""EXP121 bounded native adjacent-output continuation freeze and capture."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/readiness-continuation/run-001'
PLAN=ROOT/'experiments/readiness-continuation-2026-11-03.json'
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
 parent=read(ROOT/'experiments/availability-transitions-2026-11-03.json');pins={}
 # Existing source/input/model files remain prerequisites; the new module files are added below.
 for n,h in parent['input_source_sha256'].items():assert sha(ROOT/n)==h,n;pins[n]=h
 for p in (ROOT/'MAC-VO-main').rglob('*.py'):
  if not any(s in p.parts for s in ['.git','__pycache__']):pins[str(p.relative_to(ROOT))]=sha(p)
 for n in ['tools/readiness_continuation_study.py','docs/research/readiness-continuation-protocol-2026-11-03.md']:
  pins[n]=sha(ROOT/n)
 prior={}
 for c in ['rgb','thermal']:
  for p in (ROOT/'results/availability-transitions/run-001/captures/unperturbed/experts'/c).rglob('*.npz'):prior[str(p.relative_to(ROOT))]=sha(p)
 plan={'date':'2026-11-03','parent_commit':'4ee294435','window':parent['window'],'runtime_image':IMAGE,
       'outages':parent['outages'],'arms':['full_read_cache','bounded_probe','readiness_probe'],
       'unperturbed_control':True,'expected_dual_captures':4,'core_slots_per_arm':95,
       'attempt_limit':4,'extra_limit':20,
       'input_source_sha256':pins,'prior_control_sha256':prior,
       'scorer_only_reference_sha256':parent['scorer_only_reference_sha256'],
       'synthetic':True,'training':False,'reserve_access':False,'policy_adopted':False}
 write(PLAN,plan);write(OUT/'plan.json',plan);print('FROZEN',len(pins),'pins;4dual captures;95core slots/arm;4/20 caps')

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
       '-v',str(ROOT)+':/workspace','-w','/workspace/MAC-VO-main',IMAGE,'python','-m','Scripts.Setup.readiness_continuation',
       '--dataset','/workspace/'+w['dataset'],'--manifest','/workspace/'+w['manifest'],
       '--output','/workspace/'+str(dest.relative_to(ROOT)),'--frames','100',
       '--mode','full_read_cache' if arm=='unperturbed' else 'bounded_probe' if arm=='readiness_probe' else arm]
  if arm!='unperturbed':cmd.append('--perturb')
  if arm=='readiness_probe':cmd.append('--readiness')
  print('CAPTURE',arm,flush=True)
  with (OUT/'captures'/(arm+'-host.log')).open('x') as log:result=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
  write(state,{'command':cmd,'exit_code':result.returncode})
  if result.returncode:raise RuntimeError('capture failed; retained: '+str(dest))
  verify();print('COMPLETE',arm,flush=True)
 write(OUT/'capture-lock.json',{'plan_sha256':sha(PLAN),'dual_captures':4,'pose_references_read':False,
       'sha256':{str(p.relative_to(ROOT)):sha(p) for p in (OUT/'captures').rglob('*') if p.is_file()}})

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['freeze','verify','capture']);a=p.parse_args()
 if a.action=='verify':print('VERIFIED',len(verify()['input_source_sha256']),'pins')
 else:globals()[a.action]()

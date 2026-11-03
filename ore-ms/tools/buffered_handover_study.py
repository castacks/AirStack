"""EXP122 live buffered handover plus paired exact failing-request fixture."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/buffered-handover/run-001'
PLAN=ROOT/'experiments/buffered-handover-2026-11-03.json'
FIXTURE=ROOT/'experiments/buffered-handover-fixture-2026-11-03.json'
IMAGE='sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09'
def read(p):return json.loads(Path(p).read_text())
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def freeze():
 assert not PLAN.exists() and not OUT.exists() and not FIXTURE.exists()
 parent=read(ROOT/'experiments/readiness-continuation-2026-11-03.json');pins={}
 for n,h in parent['input_source_sha256'].items():assert sha(ROOT/n)==h,n;pins[n]=h
 source=ROOT/'results/readiness-continuation/run-001/captures/readiness_probe/timeline.jsonl'
 log=[json.loads(l) for l in source.read_text().splitlines()]
 fixture={'source':str(source.relative_to(ROOT)),'source_sha256':sha(source),
          'scope':'fixed exposed request/action mechanism fixture; no reference labels',
          'decisions':[{k:e[k] for k in ['pair','action','requests']} for e in log if e['kind']=='preselection']}
 assert len(fixture['decisions'])==99 and 'rgb' not in fixture['decisions'][49]['requests']
 write(FIXTURE,fixture);pins[str(source.relative_to(ROOT))]=sha(source);pins[str(FIXTURE.relative_to(ROOT))]=sha(FIXTURE)
 for p in (ROOT/'MAC-VO-main').rglob('*.py'):
  if not any(s in p.parts for s in ['.git','__pycache__']):pins[str(p.relative_to(ROOT))]=sha(p)
 for n in ['tools/buffered_handover_study.py','docs/research/buffered-handover-protocol-2026-11-03.md']:pins[n]=sha(ROOT/n)
 prior={}
 for case,folder in [('unperturbed','unperturbed'),('fixture_off','readiness_probe')]:
  for camera in ['rgb','thermal']:
   for p in (ROOT/'results/readiness-continuation/run-001/captures'/folder/'experts'/camera).rglob('*.npz'):
    prior[str(p.relative_to(ROOT))]=sha(p)
 plan={'date':'2026-11-03','parent_commit':'d3f9c0c76','window':parent['window'],'runtime_image':IMAGE,
       'outages':parent['outages'],'arms':['full_read_cache','bounded_probe','fixture_off','fixture_on','buffered_probe'],
       'unperturbed_control':True,'expected_dual_captures':6,'core_slots_per_arm':95,
       'attempt_limit':4,'extra_limit':20,'catchup_per_group':1,'catchup_total':2,
       'retention_source_depth':2,'fixture':str(FIXTURE.relative_to(ROOT)),
       'input_source_sha256':pins,'prior_control_sha256':prior,'scorer_only_reference_sha256':parent['scorer_only_reference_sha256'],
       'synthetic':True,'training':False,'reserve_access':False,'policy_adopted':False}
 write(PLAN,plan);write(OUT/'plan.json',plan);print('FROZEN',len(pins),'pins;6dual captures;95core slots/arm')

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
  mode='full_read_cache' if arm in ['unperturbed','full_read_cache'] else 'bounded_probe'
  cmd=['docker','run','--rm','--gpus','all','--ipc=host','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1',
       '-v',str(ROOT)+':/workspace','-w','/workspace/MAC-VO-main',IMAGE,
       'python','-m','Scripts.Setup.buffered_handover','--dataset','/workspace/'+w['dataset'],
       '--manifest','/workspace/'+w['manifest'],'--output','/workspace/'+str(dest.relative_to(ROOT)),
       '--frames','100','--mode',mode]
  if arm!='unperturbed':cmd.append('--perturb')
  if arm=='buffered_probe':cmd.extend(['--readiness','--catchup'])
  if arm in ['fixture_off','fixture_on']:cmd.extend(['--fixture','/workspace/'+plan['fixture']])
  if arm=='fixture_on':cmd.append('--catchup')
  print('CAPTURE',arm,flush=True)
  with (OUT/'captures'/(arm+'-host.log')).open('x') as log:result=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
  write(state,{'command':cmd,'exit_code':result.returncode})
  if result.returncode:raise RuntimeError('capture failed; retained: '+str(dest))
  verify();print('COMPLETE',arm,flush=True)
 write(OUT/'capture-lock.json',{'plan_sha256':sha(PLAN),'dual_captures':6,'pose_references_read':False,
       'sha256':{str(p.relative_to(ROOT)):sha(p) for p in (OUT/'captures').rglob('*') if p.is_file()}})

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['freeze','verify','capture']);a=p.parse_args()
 if a.action=='verify':print('VERIFIED',len(verify()['input_source_sha256']),'pins')
 else:globals()[a.action]()

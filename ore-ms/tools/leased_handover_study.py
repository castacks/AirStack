"""EXP124 frozen bounded shared-memory lease native captures."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/leased-handover/run-001'
PLAN=ROOT/'experiments/leased-handover-2026-11-03.json'
IMAGE='sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09'
ARMS=['full_read_cache','bounded_probe','fixture_on','leased_probe','diagnostic_probe']
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def freeze():
 assert not OUT.exists() and not PLAN.exists()
 parent_path=ROOT/'experiments/retention-capacity-2026-11-03.json';parent=read(parent_path)
 pins=dict(parent['input_source_sha256']);pins[str(parent_path.relative_to(ROOT))]=sha(parent_path)
 for path in ['MAC-VO-main/Scripts/Setup/leased_transport.py','MAC-VO-main/Scripts/Setup/leased_handover.py',
              'MAC-VO-main/Scripts/Setup/leased_native_worker.py','MAC-VO-main/Scripts/UnitTest/test_leased_handover.py',
              'tools/leased_handover_study.py','tools/leased_handover_evaluate.py','tools/leased_handover_audit.py',
              'tools/buffered_handover_evaluate.py','tools/buffered_handover_audit.py','tools/closed_loop_sparse_evaluate.py',
              'tools/native_gap_evaluate.py','docs/research/leased-handover-protocol-2026-11-03.md']:
  pins[path]=sha(ROOT/path)
 for path,digest in pins.items():assert sha(ROOT/path)==digest,path
 controls={}
 for arm in ['unperturbed','fixture_on']:
  for p in (ROOT/'results/buffered-handover/run-001/captures'/arm/'experts').rglob('*.npz'):
   controls[str(p.relative_to(ROOT))]=sha(p)
 plan={'date':'2026-11-03','parent_commit':'cb7f703e1f284417143f17ac37095bab291222ab',
       'window':parent['window'],'arms':ARMS,'runtime_image':IMAGE,'fixture':'experiments/buffered-handover-fixture-2026-11-03.json',
       'outages':read(ROOT/'experiments/buffered-handover-2026-11-03.json')['outages'],
       'primary_store_bytes':4194304,'diagnostic_store_bytes':2097152,'arena_bytes':2097152,
       'command_credit':1,'step_ceiling_bytes':4096,'message_ceiling_bytes':65536,
       'input_source_sha256':pins,'prior_control_sha256':controls,'expected_dual_captures':6,
       'pose_references_read':False,'training':False,'reserve_access':False,'policy_adopted':False}
 write(PLAN,plan);write(OUT/'plan.json',plan);print('FROZEN',len(pins),'pins;',len(controls),'prior files;6dual captures')
def verify():
 plan=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for path,digest in plan['input_source_sha256'].items():assert sha(ROOT/path)==digest,path
 return plan

def capture():
 plan=verify();w=plan['window']
 for arm in ['unperturbed']+ARMS:
  dest=OUT/'captures'/arm;state=OUT/'captures'/(arm+'-execution.json')
  if state.exists():
   assert read(state)['exit_code']==0,'failed attempt retained; no retry';continue
  if dest.exists():raise RuntimeError('partial attempt retained: '+str(dest))
  dest.parent.mkdir(parents=True,exist_ok=True)
  mode='full_read_cache' if arm in ['unperturbed','full_read_cache'] else 'bounded_probe'
  cap=plan['diagnostic_store_bytes'] if arm=='diagnostic_probe' else plan['primary_store_bytes']
  command=['docker','run','--rm','--gpus','all','--ipc=host','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1',
           '-v',str(ROOT)+':/workspace','-w','/workspace/MAC-VO-main',IMAGE,'python','-m','Scripts.Setup.leased_handover',
           '--dataset','/workspace/'+w['dataset'],'--manifest','/workspace/'+w['manifest'],
           '--output','/workspace/'+str(dest.relative_to(ROOT)),'--frames','100','--mode',mode,
           '--store-bytes',str(cap),'--arena-bytes',str(plan['arena_bytes'])]
  if arm!='unperturbed':command.append('--perturb')
  if arm in ['leased_probe','diagnostic_probe']:command.extend(['--readiness','--catchup'])
  if arm=='fixture_on':command.extend(['--fixture','/workspace/'+plan['fixture'],'--catchup'])
  print('CAPTURE',arm,flush=True)
  with (OUT/'captures'/(arm+'-host.log')).open('x') as f:result=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT)
  write(state,{'command':command,'exit_code':result.returncode})
  if result.returncode:raise RuntimeError('native failure retained: '+str(dest))
  verify();print('COMPLETE',arm,flush=True)
 write(OUT/'capture-lock.json',{'plan_sha256':sha(PLAN),'pose_references_read':False,'dual_captures':6,
       'sha256':{str(p.relative_to(ROOT)):sha(p) for p in (OUT/'captures').rglob('*') if p.is_file()}})

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['freeze','verify','capture']);a=p.parse_args()
 if a.action=='verify':print('VERIFIED',len(verify()['input_source_sha256']),'pins')
 else:globals()[a.action]()

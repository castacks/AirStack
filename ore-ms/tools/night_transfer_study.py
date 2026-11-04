"""EXP125 night transfer, native fixed cameras and exact-cost conditional readouts."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/night-transfer/run-001'
PLAN=ROOT/'experiments/night-transfer-2026-11-03.json'
IMAGE='sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09'
ARMS=['full_read_cache','bounded_probe','leased_probe','fixed_rgb','fixed_thermal']
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def freeze():
 assert not OUT.exists() and not PLAN.exists()
 parent_path=ROOT/'experiments/leased-handover-2026-11-03.json';parent=read(parent_path)
 pins=dict(parent['input_source_sha256']);pins[str(parent_path.relative_to(ROOT))]=sha(parent_path)
 inventory_path=ROOT/'experiments/closed-loop-sparse-2026-11-03.json';inventory=read(inventory_path)
 candidates=[w for w in inventory['windows'] if 'night' in w['reference_dataset'] and w['frames']>=100]
 assert len(candidates)==1
 window=candidates[0];assert window['frames']==100 and window['reference_dataset']!=parent['window']['reference_dataset']
 night_hashes=set();day_hashes=set()
 for w,hashes in [(window,night_hashes),(parent['window'],day_hashes)]:
  for row in w['rows']:
   for key in ['rgb_left','rgb_right','thermal_left','thermal_right']:
    p=ROOT/w['dataset']/row[key]['path'];digest=sha(p);hashes.add(digest);pins[str(p.relative_to(ROOT))]=digest
 assert not night_hashes.intersection(day_hashes),'image hash overlap with development'
 for p in [ROOT/window['manifest'],*(ROOT/window['dataset']).glob('sensors/*_camera_info/data.yaml')]:pins[str(p.relative_to(ROOT))]=sha(p)
 for path in ['tools/night_transfer_study.py','tools/night_transfer_evaluate.py','tools/night_transfer_audit.py',
              'docs/research/night-transfer-protocol-2026-11-03.md']:
  pins[path]=sha(ROOT/path)
 pins[str(inventory_path.relative_to(ROOT))]=sha(inventory_path)
 for path,digest in pins.items():assert sha(ROOT/path)==digest,path
 prior='results/closed-loop-sparse/run-001/captures/middle/full_read_cache/experts'
 controls={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/prior).rglob('*.npz')};assert controls
 plan=dict(parent,date='2026-11-03',parent_commit='680a2da7c92f6e992b7d827068622b48de8bc7ea',
  window=window,arms=ARMS,input_source_sha256=pins,prior_control_sha256=controls,prior_unperturbed=prior,
  matched_readouts=['matched_rgb','matched_thermal'],development_image_hash_overlap=0,
  previously_inspected_condition=True,blind_holdout=False,expected_dual_captures=6)
 write(PLAN,plan);write(OUT/'plan.json',plan)
 print('FROZEN',len(pins),'pins;',len(controls),'prior files;6dual captures;0development image hashes overlap')
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
  mode='full_read_cache' if arm in ['unperturbed','full_read_cache'] else arm if arm.startswith('fixed_') else 'bounded_probe'
  cap=plan['diagnostic_store_bytes'] if arm=='diagnostic_probe' else plan['primary_store_bytes']
  command=['docker','run','--rm','--gpus','all','--ipc=host','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1',
           '-v',str(ROOT)+':/workspace','-w','/workspace/MAC-VO-main',IMAGE,'python','-m','Scripts.Setup.leased_handover',
           '--dataset','/workspace/'+w['dataset'],'--manifest','/workspace/'+w['manifest'],
           '--output','/workspace/'+str(dest.relative_to(ROOT)),'--frames','100','--mode',mode,
           '--store-bytes',str(cap),'--arena-bytes',str(plan['arena_bytes'])]
  if arm!='unperturbed':command.append('--perturb')
  if arm=='leased_probe':command.extend(['--readiness','--catchup'])
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

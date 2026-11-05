"""One prospectively declared resource retry of capacity-only aborts; no quality selection."""
from copy import deepcopy
import hashlib,json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/bounded-full-record'
DECLARATION=ROOT/'experiments/droid-bounded-capacity-retry-declaration-2026-11-04.json'

def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2);f.write('\n')


def freeze():
 declaration=read(DECLARATION)
 for path,digest in declaration['source_sha256'].items():
  if sha(ROOT/path)!=digest:raise ValueError('resource declaration input changed')
 master=read(ROOT/'experiments/droid-bounded-full-record-2026-11-04.json');runs=[];decisions=[]
 for record in master['runs']:
  original=OUT/'captures'/record['case']/'estimate.json'
  if not original.exists():raise ValueError('all original attempts must finish before retry freeze')
  result=read(original);eligible=isinstance(result['failure'],str) and result['failure'].startswith('NativeCapacityExceeded:')
  decisions.append({'case':record['case'],'original_estimate_sha256':sha(original),'eligible_capacity_only':eligible})
  if not eligible:continue
  p=deepcopy(read(ROOT/record['protocol']));p['settings']['buffer']=4096
  p['bounded_capacity']['mode']='half_memory_persistent_state_cap';p['bounded_capacity']['slots']=4096
  p['bounded_capacity']['scope']='half-memory state resource retry; no full peak guarantee'
  p['worker_source']='tools/droid_bounded_worker_v3.py';p['retry_of_estimate_sha256']=sha(original)
  p['resource_retry_declaration_sha256']=sha(DECLARATION)
  for f in [DECLARATION,Path(__file__),ROOT/'tools/droid_bounded_worker_v3.py',ROOT/'tools/tests/test_droid_bounded_worker_v3.py']:
   p['source_sha256'][str(f.relative_to(ROOT))]=sha(f)
  dest=OUT/'protocols-v3'/(record['case']+'.json');write(dest,p)
  runs.append({'case':record['case'],'protocol':str(dest.relative_to(ROOT)),'sha256':sha(dest),'source_frames':record['source_frames']})
 write(ROOT/'experiments/droid-bounded-resource-retries-2026-11-04.json',{'date':'2026-11-04 America/New_York',
 'runs':runs,'original_case_decisions':decisions,'eligibility':'NativeCapacityExceeded only, one retry, no pose quality/reference selection',
 'all_original_attempts_retained':True,'declaration_sha256':sha(DECLARATION),'reference_scoring':False,'benchmark_execution_ready':False})
 print('FROZEN RESOURCE RETRIES',[r['case'] for r in runs])


if __name__=='__main__':freeze()

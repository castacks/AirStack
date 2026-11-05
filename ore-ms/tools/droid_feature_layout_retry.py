"""Freeze a storage-layout correction for completed backend packed32 failures only."""
from copy import deepcopy
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/bounded-full-record'

def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2);f.write('\n')


def freeze():
 master=read(ROOT/'experiments/droid-bounded-resource-retries-2026-11-04.json')
 contract=OUT/'feature-compaction-contract.json'
 if read(contract)['passed'] is not True:raise ValueError('native correlation equivalence required')
 runs=[];decisions=[]
 for item in master['runs']:
  directory=OUT/'captures-v3'/item['case'];e=read(directory/'estimate.json');s=read(directory/'capacity-status.json')
  eligible=(isinstance(e['failure'],str) and 'numel needs to be smaller than int32_t max' in e['failure']
   and s['failure_phase']=='offline_termination' and e['native_track_calls_returned']==e['source_frames'])
  decisions.append({'case':item['case'],'failed_estimate_sha256':sha(directory/'estimate.json'),'packed32_layout_failure':eligible})
  if not eligible:continue
  p=deepcopy(read(ROOT/item['protocol']));case=p['runs'][0];h,w=case['shape'];count=s['last_observed_keyframe_count']
  if (count+17)*2*128*(h//8)*(w//8)>=2**31-1:raise ValueError('active+reserve features exceed accessor: this correction cannot solve it')
  p['worker_source']='tools/droid_bounded_worker_v4.py'
  p['feature_layout_correction']={'when':'after all source tracking, before original offline termination',
   'retained_rows':'every active keyframe plus unchanged17filler/next-state reserve',
   'copy_check':'bitwise int16 equality of all retained half feature bits; counter/otherstate unchanged',
   'capacity_increase':False,'failed_estimate_sha256':sha(directory/'estimate.json'),'native_correlation_contract_sha256':sha(contract)}
  p['native_source_sha256'].update({n:sha(Path('/root/ore-ms-recovery/droid-source/droid_slam')/n) for n in ['factor_graph.py','modules/corr.py']})
  for f in [Path(__file__),ROOT/'tools/droid_bounded_worker_v4.py',ROOT/'tools/tests/test_droid_bounded_worker_v4.py',
   ROOT/'tools/droid_feature_compaction_contract.py',contract,ROOT/'experiments/droid-feature-layout-correction-declaration-2026-11-04.json']:
   p['source_sha256'][str(f.relative_to(ROOT))]=sha(f)
  dest=OUT/'protocols-v4'/(item['case']+'.json');write(dest,p)
  runs.append({'case':item['case'],'protocol':str(dest.relative_to(ROOT)),'sha256':sha(dest),'source_frames':item['source_frames']})
 write(ROOT/'experiments/droid-bounded-feature-layout-retries-2026-11-04.json',{'runs':runs,'decisions':decisions,
 'scope':'one compatibility correction at same4096allocation for unused-padding packed32 failure; not model/quality selection',
 'original_and_resource_attempts_retained':True,'reference_scoring':False,'benchmark_execution_ready':False})
 print('FROZEN SAME-CAPACITY FEATURE-LAYOUT CORRECTION',[x['case'] for x in runs])


if __name__=='__main__':freeze()

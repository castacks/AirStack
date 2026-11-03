"""Freeze all-source availability study and four matched early-observer captures."""
from pathlib import Path
import argparse,datetime,hashlib,json,subprocess,sys
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/early-health-availability/run-001';PLAN=ROOT/'experiments/early-health-availability-2026-11-03.json'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,x):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(x,f,indent=2,allow_nan=False);f.write('\n')
def freeze():
 assert not PLAN.exists();old=read(ROOT/'results/past-only-sensor-risk/run-001/plan.json');sources=old['sources'];wanted=[('exp112','day0'),('exp112','night-middle'),('exp113','early'),('exp114','night-late')];cases=[];pins={}
 for s in sources:
  for n in [s['plan'],s['prefix']]:
   p=ROOT/s['base']/n;pins[str(p.relative_to(ROOT))]=sha(p)
  p=ROOT/s['controls'];pins[s['controls']]=sha(p)
  for c in s['cases']:
   for p in [ROOT/s['base']/'captures'/c['case']/'on/timeline.jsonl',*list((ROOT/s['base']/'captures'/c['case']/'on/experts').glob('*/effective-config.json'))]:pins[str(p.relative_to(ROOT))]=sha(p)
 for tag,name in wanted:
  s=next(s for s in sources if s['tag']==tag);c=next(c for c in s['cases'] if c['case']==name);ms2=tag=='exp114';manifest=c['manifest'] if ms2 else c['dataset']+'/manifest.json';dataset='.' if ms2 else c['dataset']
  cases.append({'case':tag+'-'+name,'tag':tag,'source_case':name,'domain':'ms2' if ms2 else 'td3','recording_id':c['recording_id'],'frames':c['frames'],'dataset':dataset,'manifest':manifest,'rows':c['rows']})
  p=ROOT/manifest;pins[manifest]=sha(p)
  if ms2:
   p=ROOT/read(p)['source_manifest_relative'];pins[str(p.relative_to(ROOT))]=sha(p)
  if not ms2:
   for p in (ROOT/dataset/'sensors').glob('*/data.yaml'):pins[str(p.relative_to(ROOT))]=sha(p)
  for r in c['rows']:
   for cam in ['rgb','thermal']:
    for side in ['left','right']:
     p=ROOT/dataset/r[cam+'_'+side]['path'];pins[str(p.relative_to(ROOT))]=sha(p)
 for p in (ROOT/'MAC-VO-main').rglob('*.py'):
  if '.git' not in p.parts and '__pycache__' not in p.parts:pins[str(p.relative_to(ROOT))]=sha(p)
 for p in (ROOT/'MAC-VO-main/Config').rglob('*.yaml'):pins[str(p.relative_to(ROOT))]=sha(p)
 for p in (ROOT/'tools').glob('early_health*.py'):pins[str(p.relative_to(ROOT))]=sha(p)
 for rel in ['results/past-only-sensor-risk/run-001/source-features.json','docs/research/early-health-availability-protocol-2026-11-03.md','models/macslam/MACVO_FrontendCov.pth','models/macslam/MACT_Offroad.pth']:
  pins[rel]=sha(ROOT/rel)
 plan={'parent_commit':'4709fba75','created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'client_date':'2026-11-03','sources':sources,'fresh_cases':cases,'input_source_sha256':pins,'forecast_bindings':'results/past-only-sensor-risk/run-001/source-features.json','arms':['off','on'],'expected_native_frame_executions':1280,'expected_native_attempts':16,'primary':'early_resolved at0.5seconds','retrospective_exposed_development':True,'no_pose_error_labels_read':True,'reserve_access':False,'runtime_image':old['scientific_runtime']};write(PLAN,plan);write(OUT/'plan.json',plan);print('FROZEN',len(pins),'pins;4freshcases;1955historicalforecastbindings')
def verify():
 p=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for n,h in p['input_source_sha256'].items():assert sha(ROOT/n)==h,n
 return p
def capture():
 plan=verify()
 for c in plan['fresh_cases']:
  for arm in plan['arms']:
   dest=OUT/'captures'/c['case']/arm
   if dest.exists():
    assert read(dest.parent/(arm+'-execution.json'))['exit_code']==0;continue
   dest.parent.mkdir(parents=True,exist_ok=True);cmd=['docker','run','--rm','--gpus','all','--ipc=host','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-v',str(ROOT)+':/workspace','-w','/workspace/MAC-VO-main',plan['runtime_image'],'python','-m','Scripts.Setup.early_health_replay','--dataset','/workspace/'+c['dataset'],'--manifest','/workspace/'+c['manifest'],'--output','/workspace/'+str(dest.relative_to(ROOT)),'--frames',str(c['frames']),'--start','0','--timeout-seconds','300','--record-root-initialization','--acknowledge-provisional-geometry']
   if arm=='on':cmd+=['--early']
   if c['domain']=='ms2':cmd+=['--ms2']
   print('CAPTURE',c['case'],arm,flush=True)
   with (dest.parent/(arm+'-host.log')).open('x') as f:result=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
   write(dest.parent/(arm+'-execution.json'),{'command':cmd,'exit_code':result.returncode})
   if result.returncode:raise RuntimeError('retained native capture failure '+str(dest))
   verify();print('COMPLETE',c['case'],arm,flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['freeze','verify','capture']);a=p.parse_args()
 if a.action=='verify':print('PASS',len(verify()['input_source_sha256']),'pins')
 else:globals()[a.action]()

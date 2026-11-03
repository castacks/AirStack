"""Freeze and capture the fixed additional TD3 recording; no quality labels read."""
from pathlib import Path
import argparse,hashlib,json,shutil,subprocess,sys
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/geometry-support-transfer/run-001';PLAN=ROOT/'experiments/geometry-support-transfer-2026-11-02.json'
IMAGE='sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,x):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(x,f,indent=2,allow_nan=False);f.write('\n')

def freeze():
 sys.path.insert(0,str(ROOT/'MAC-VO-main'))
 from Scripts.Setup.arbiter_pose_proxy import extrinsic
 assert not PLAN.exists();record='night_thermal_so_salon2';reference=ROOT/'data/tartandrive3/20251027'/record
 sourceplan=read(ROOT/'results/msa-replay/prepared-001/plan.json');pins={};cases=[]
 for label in ['early','middle','late']:
  old=ROOT/f'results/msa-replay/prepared-001/datasets/night-{label}';rows=read(old/'manifest.json')['pairs'];assert len(rows)==20
  dest=OUT/'inputs'/label;dest.mkdir(parents=True,exist_ok=False)
  for row in rows:
   for camera in ['rgb','thermal']:
    for side in ['left','right']:
     rel=row[camera+'_'+side]['path'];src=(reference if camera=='rgb' else old)/rel;target=dest/rel;target.parent.mkdir(parents=True,exist_ok=True)
     if not target.exists():shutil.copyfile(src,target)
     assert sha(src)==sha(target);pins[str(src.relative_to(ROOT))]=sha(src);pins[str(target.relative_to(ROOT))]=sha(target)
  for name in ['multisense_left_rect_color_camera_info','multisense_right_camera_info','thermal_left_processed_camera_info','thermal_right_processed_camera_info']:
   src=(reference if name.startswith('multisense') else old)/'sensors'/name/'data.yaml';target=dest/'sensors'/name/'data.yaml';target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,target);assert sha(src)==sha(target)
   pins[str(src.relative_to(ROOT))]=sha(src);pins[str(target.relative_to(ROOT))]=sha(target)
  shutil.copyfile(old/'manifest.json',dest/'manifest.json');pins[str((dest/'manifest.json').relative_to(ROOT))]=sha(dest/'manifest.json')
  transforms={camera:extrinsic(ROOT/'data/calibration/yamaha.yaml',dest,camera).tolist() for camera in ['rgb','thermal']}
  cases.append({'case':label,'window':label,'frames':20,'rows':rows,'dataset':str(dest.relative_to(ROOT)),'reference_dataset':str(reference.relative_to(ROOT)),'recording_id':'td3/20251027/'+record,'nominal_body_from_camera':transforms})
 for name in ['models/macslam/MACVO_FrontendCov.pth','models/macslam/MACT_Offroad.pth','data/calibration/yamaha.yaml','docs/research/geometry-support-transfer-protocol-2026-11-02.md']:pins[name]=sha(ROOT/name)
 assert pins['models/macslam/MACVO_FrontendCov.pth']=='bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a'
 assert pins['models/macslam/MACT_Offroad.pth']=='eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a'
 for file in (ROOT/'MAC-VO-main').rglob('*.py'):
  if '.git' not in file.parts and '__pycache__' not in file.parts:pins[str(file.relative_to(ROOT))]=sha(file)
 for file in (ROOT/'MAC-VO-main/Config').rglob('*.yaml'):pins[str(file.relative_to(ROOT))]=sha(file)
 for file in (ROOT/'tools').glob('geometry_support_*.py'):pins[str(file.relative_to(ROOT))]=sha(file)
 refs={str(p.relative_to(ROOT)):sha(p) for p in [reference/'super_odometry/odometry'/n for n in ['info.yaml','interp_data.txt','interp_timestamps.txt']]}
 plan={'parent_commit':'941cb59b7','recording_scope':'additional physically distinct exposed development recording, not prospective or network-training held out','runtime_image':IMAGE,'cases':cases,'arms':['off','on'],'input_source_sha256':pins,'scorer_only_reference_sha256':refs,'expected_decisions':57,'native_frame_executions':240,'protocol_sha256':sha(ROOT/'docs/research/geometry-support-transfer-protocol-2026-11-02.md'),'primary':'unchanged current geometry admission','controls':['raw','history','rgb_only','thermal_only','retained_count','occupied_cells','retained_fraction'],'sensitivities':['information_rank','information_per_factor'],'no_training_or_threshold_search':True,'reserve_access':False,'expected_native_failures_retained':True,'strict_array_bits_status_reported_separately':True}
 write(PLAN,plan);write(OUT/'plan.json',plan);print('FROZEN',3,'windows',len(pins),'input/source hashes;3 reference hashes')

def verify():
 p=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for n,h in {**p['input_source_sha256'],**p['scorer_only_reference_sha256']}.items():assert sha(ROOT/n)==h,n
 return p

def capture():
 p=verify()
 for c in p['cases']:
  for arm in p['arms']:
   dest=OUT/'captures'/c['case']/arm;assert not dest.exists();dest.parent.mkdir(parents=True,exist_ok=True)
   cmd=['docker','run','--rm','--gpus','all','--ipc=host','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-v',str(ROOT)+':/workspace','-w','/workspace/MAC-VO-main',IMAGE,'python','-m','Scripts.Setup.geometry_support_replay','--dataset','/workspace/'+c['dataset'],'--manifest','/workspace/'+c['dataset']+'/manifest.json','--output','/workspace/'+str(dest.relative_to(ROOT)),'--frames','20','--start','0','--timeout-seconds','300','--record-root-initialization','--acknowledge-provisional-geometry']
   if arm=='on':cmd+=['--geometry']
   print('CAPTURE',c['case'],arm,flush=True)
   with (dest.parent/(arm+'-host.log')).open('x') as f:result=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
   write(dest.parent/(arm+'-execution.json'),{'command':cmd,'exit_code':result.returncode})
   if result.returncode:raise RuntimeError('retained capture failure: '+str(dest))
   verify();print('COMPLETE',c['case'],arm,flush=True)

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['freeze','capture','verify']);a=p.parse_args()
 if a.action=='verify':print('PASS',len(verify()['input_source_sha256']),'pins')
 else:globals()[a.action]()

"""EXP129 frozen native benchmark of exact archive-provided images and camera-info."""
import argparse,copy,hashlib,json,shutil,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/provided-export-survey/run-001'
PLAN=ROOT/'experiments/provided-export-survey-2026-11-04.json'
PARENT=ROOT/'results/five-recording-survey/run-001'
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')

def freeze():
 assert not OUT.exists() and not PLAN.exists()
 parent=read(PARENT/'stage2-plan.json');windows=copy.deepcopy(parent['windows']);pins=dict(parent['input_source_sha256'])
 for w in windows:w['dataset']=str((OUT/'prepared'/w['clip']/str(w['third'])).relative_to(ROOT))
 for name in ['stage2-plan.json','metadata-summary.json','preflight-audit.json','image-lock.json','input-diagnostic.json','input-diagnostic-declaration.json']:
  p=PARENT/name;pins[str(p.relative_to(ROOT))]=sha(p)
 # Only the actual provided stereo inputs become native sources in this experiment.
 for w in windows:
  for row in w['rows']:
   for key in ['rgb_left','rgb_right','thermal_left','thermal_right']:
    p=PARENT/'raw'/w['clip']/row[key]['path'];pins[str(p.relative_to(ROOT))]=sha(p)
 for p in [ROOT/'tools/provided_export_survey.py',ROOT/'tools/provided_export_survey_audit.py',
   ROOT/'docs/research/provided-export-survey-protocol-2026-11-04.md']:
  pins[str(p.relative_to(ROOT))]=sha(p)
 for path,digest in pins.items():assert sha(ROOT/path)==digest,path
 diagnostic=read(PARENT/'input-diagnostic.json');assert all(g['original_producer_exact']==g['frozen_msa_target_exact']==0 for g in diagnostic['groups'].values())
 plan={'date':'2026-11-04','parent_commit':'a221d0b81','windows':windows,'input_source_sha256':pins,
  'runtime_image':parent['runtime_image'],'input_recipe':'exact archive-provideduint8images andcamera-info;no MSAregeneration',
  'frames':100,'expected_dual_captures':4,'core_rows':380,'reference_labels_read':False,'candidate_changed':False,
  'EXP128_MSA_input_contract':'FAILED_NOT_RELAXED','exploratory_after_input_inspection':True,'reserved_payload_access':False}
 write(PLAN,plan);write(OUT/'plan.json',plan);print('FROZEN EXP129',len(pins),'pins;4provided-export captures')

def verify():
 plan=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for path,digest in plan['input_source_sha256'].items():assert sha(ROOT/path)==digest,path
 return plan


def prepare():
 plan=verify();files={}
 for w in plan['windows']:
  dest=ROOT/w['dataset'];dest.mkdir(parents=True,exist_ok=False)
  metadata=(ROOT/w['metadata_source']).parent/'source'
  for name in ['multisense_left_rect_color_camera_info','multisense_right_camera_info','thermal_left_processed_camera_info','thermal_right_processed_camera_info']:
   src=metadata/'sensors'/name/'data.yaml';target=dest/'sensors'/name/'data.yaml';target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,target)
   assert sha(src)==sha(target);files[str(target.relative_to(ROOT))]=sha(target)
  for row in w['rows']:
   for key in ['rgb_left','rgb_right','thermal_left','thermal_right']:
    src=PARENT/'raw'/w['clip']/row[key]['path'];target=dest/row[key]['path'];target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,target)
    assert sha(src)==sha(target);files[str(target.relative_to(ROOT))]=sha(target)
  write(dest/'manifest.json',{'pairs':w['rows'],'source':'EXP129 exact archive export; no MSA regeneration or timestamp changes'})
  files[str((dest/'manifest.json').relative_to(ROOT))]=sha(dest/'manifest.json')
 write(OUT/'prepared-lock.json',{'plan_sha256':sha(PLAN),'sha256':files,'recipe':'source bytes unchanged'})
 print('PREPARED',len(files),'provided source files')


def capture():
 plan=verify();prep=read(OUT/'prepared-lock.json');assert read(OUT/'input-audit.json')['pass']
 for path,digest in prep['sha256'].items():assert sha(ROOT/path)==digest,path
 for w in plan['windows']:
  dest=OUT/'captures'/w['case'];state=dest.parent/(dest.name+'-execution.json')
  if state.exists():assert read(state)['exit_code']==0,'failed attempt retained';continue
  assert not dest.exists(),'partial attempt retained';dest.parent.mkdir(parents=True,exist_ok=True)
  cmd=['docker','run','--rm','--gpus','all','--ipc=host','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1',
   '-v',str(ROOT)+':/workspace','-w','/workspace/MAC-VO-main',plan['runtime_image'],'python','-m','Scripts.Setup.geometry_support_replay',
   '--dataset','/workspace/'+w['dataset'],'--manifest','/workspace/'+w['dataset']+'/manifest.json','--output','/workspace/'+str(dest.relative_to(ROOT)),
   '--frames','100','--start','0','--record-root-initialization','--acknowledge-provisional-geometry']
  print('CAPTURE',w['case'],flush=True)
  with (dest.parent/(dest.name+'-host.log')).open('x') as f:p=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
  write(state,{'command':cmd,'exit_code':p.returncode})
  if p.returncode:raise RuntimeError('capture failed and retained: '+str(dest))
  verify();print('COMPLETE',w['case'],flush=True)
 write(OUT/'capture-lock.json',{'stage_plan_sha256':sha(PLAN),'prepared_lock_sha256':sha(OUT/'prepared-lock.json'),
   'reference_labels_read':False,'sha256':{str(p.relative_to(ROOT)):sha(p) for p in (OUT/'captures').rglob('*') if p.is_file()}})

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['freeze','verify','prepare','capture']);a=p.parse_args()
 if a.action=='verify':print('VERIFIED',len(verify()['input_source_sha256']),'provided-export pins')
 else:globals()[a.action]()

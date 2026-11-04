"""EXP133 protocol freeze and fail-closed benchmark preparation readiness."""
import argparse,hashlib,json,shutil,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/independent-benchmark-preparation/run-001';SOURCES=ROOT/'results/independent-benchmark-preparation/sources-001';PLAN=ROOT/'experiments/independent-benchmark-2026-11-04.json'
RESERVED=['ms2/_2021-08-13-16-31-10','ms2/_2021-08-13-22-16-02']
THERMAL='td3-firestereo-v2-clahe-bilateral-v1'
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def validate(p):
 issues=[]
 def need(value,reason):
  if not value:issues.append(reason)
 parts=p['partitions'];dev=set(parts['development']);val=set(parts['validation']);test=set(parts['test'])
 need(not(dev&val or dev&test or val&test),'partition_overlap')
 need(p['phase']=='preparation' and not test and not val,'preparation_must_not_activate_test')
 need(p['reported_reserves']==RESERVED and p['conditional_test_candidates']==RESERVED,'reserve_identity_or_assignment_changed')
 need(not set(RESERVED)&dev,'reserve_promoted_to_development')
 need(p['protected_payload_access'] is False,'protected_payload_access')
 need(p['metrics']['scale_fit'] is False and p['metrics']['alignment']=='SE3_no_scale','primary_scale_fit')
 need(p['metrics']['aggregation']=='equal_recording_weight','pooled_frame_ranking')
 need(p['metrics']['official_dataset_metric'] is False,'unverified_official_metric_claim')
 need(p['sampling']['unit']=='whole_recording' and p['sampling']['outcome_selected_windows'] is False,'outcome_selected_sampling')
 need(p['input']['thermal_recipe']==THERMAL and p['input']['rerectify'] is False,'input_recipe_change')
 need(p['claims']['physical_ground_truth'] is False and p['claims']['network_heldout'] is False and p['claims']['sota_superiority'] is False,'unsupported_claim')
 need(p['condition_prior']=={'day':'rgb','night':'thermal','rain':'rgb'},'condition_prior_change')
 for m in p['methods']:
  need(m['sensor_mode']=='stereo','monocular_in_primary_stereo_track:'+m['name'])
  need(bool(m['source_revision']) and len(m['source_revision'])==40,'unpinned_source:'+m['name'])
  need(m['inputs'] in [2,4] and m['imu'] is False,'sensor_budget:'+m['name'])
  if m['name'].startswith('orbslam3'):need(m['loop_closing'] is False,'loop_closing_primary_mode')
 return issues

def proof_ok(value):
 if not isinstance(value,dict) or set(value)!={'artifact','sha256'}:return False
 path=Path(value['artifact'])
 if path.is_absolute() or '..' in path.parts:return False
 path=ROOT/path
 try:return path.is_file() and sha(path)==value['sha256'] and read(path).get('pass') is True
 except (ValueError,TypeError,OSError):return False

def readiness(p):
 missing=[name for name,value in p['execution_evidence'].items() if not proof_ok(value)]
 for m in p['methods']:
  for field in ['adapter_freeze','runtime_digest','assets_manifest','development_smoke']:
   valid=(isinstance(m[field],str) and m[field].startswith('sha256:') and len(m[field])==71) if field=='runtime_digest' else proof_ok(m[field])
   if not valid:missing.append(m['name']+':'+field)
 return {'specification_valid':not validate(p),'execution_ready':not validate(p) and not missing and bool(p['partitions']['test']),'unmet_execution_requirements':missing,'active_test_recordings':len(p['partitions']['test']),'protected_payload_access':False}

def freeze():
 assert not OUT.exists() and not PLAN.exists()
 ledger_path=ROOT/'experiments/recording-exposure-ledger-2026-10-31.json';ledger=read(ledger_path);upstream=read(SOURCES/'upstream.json');up={s['name']:s for s in upstream['sources']};methods=[];base='21ab29c61957d428422315eabcf4164a22b80a35'
 for name,inputs,role,source,modality in [('fixed_rgb',4,'matched_selector','local','rgb'),('fixed_thermal',4,'matched_selector','local','thermal'),('rgb_preference',4,'matched_selector','local','both'),('condition_prior',4,'matched_selector','local','both'),('raw_uncertainty',4,'matched_selector','local','both'),('angular_uncertainty',4,'matched_selector','local','both'),('macvo_stereo_rgb',2,'published_context','macvo','rgb'),('project_mact_stereo_thermal',2,'project_expert_context','local','thermal'),('orbslam3_rgb',2,'published_context','orbslam3','rgb'),('orbslam3_thermal',2,'published_context','orbslam3','thermal'),('droid_stereo_rgb',2,'published_context','droid','rgb'),('droid_stereo_thermal',2,'published_context','droid','thermal')]:
  methods.append({'name':name,'inputs':inputs,'role':role,'sensor_mode':'stereo','modality':modality,'imu':False,'source_revision':base if source=='local' else up[source]['revision'],'source':source,'adapter_freeze':None,'runtime_digest':None,'assets_manifest':None,'development_smoke':None,'loop_closing':False if name.startswith('orbslam3') else None})
 files=[ledger_path,ROOT/'experiments/prospective-state-shadow-evaluation-2026-11-01-adapter-v2.json',Path(__file__),ROOT/'tools/independent_benchmark_metrics.py',ROOT/'tools/independent_benchmark_check.py',ROOT/'tools/rotation_signal_audit.py',ROOT/'docs/research/independent-benchmark-protocol-2026-11-04.md',ROOT/'MAC-VO-main/DataLoader/Dataset/MS2.py',ROOT/'MAC-VO-main/Scripts/Setup/ms2_execution_adapter.py',ROOT/'MAC-VO-main/Baseline/DPVO.py',ROOT/'MAC-VO-main/Odometry/BaselineDPVO.py',ROOT/'MAC-VO-main/Baseline/TartanVO.py',ROOT/'MAC-VO-main/Config/Experiment/Baseline/TartanVO/TartanVOStereo.yaml']
 files += [x for x in SOURCES.rglob('*') if x.is_file()]
 pins={str(f.relative_to(ROOT)):sha(f) for f in files}
 models={str((ROOT/'models/macslam'/name).relative_to(ROOT)):sha(ROOT/'models/macslam'/name) for name in ['MACVO_FrontendCov.pth','MACT_Offroad.pth']}
 p={'date':'2026-11-04','parent_commit':base,'status':'SPECIFICATION_FROZEN_EXECUTION_NOT_READY','phase':'preparation','dataset':'MS2_public_provided_reference','partitions':{'development':[r['id'] for r in ledger['records'] if r['exposure_status']=='development_exposed'],'validation':[],'test':[]},'reported_reserves':RESERVED,'conditional_test_candidates':RESERVED,'protected_payload_access':False,'sampling':{'unit':'whole_recording','all_common_official_stems':True,'outcome_selected_windows':False},'input':{'thermal_recipe':THERMAL,'rerectify':False,'canonical_rgb':'official rectified bytes','intrinsics_resize_proof_required':True,'baseline_unit_conversion':'mm_to_m_from_source_metadata'},'metrics':{'alignment':'SE3_no_scale','scale_fit':False,'rpe_target_ns':1000000000,'rpe_tolerance_ns':150000000,'source_gap_max_ns':150000000,'pose_coverage_success':.95,'rpe_coverage_success':.95,'aggregation':'equal_recording_weight','official_dataset_metric':False,'seeds':[0,1,2]},'claims':{'physical_ground_truth':False,'network_heldout':False,'sota_superiority':False},'condition_prior':{'day':'rgb','night':'thermal','rain':'rgb'},'methods':methods,'diagnostic_ridge_source':'results/rotation-signal-audit/run-001/predictions.json','diagnostic_ridge_source_sha256':sha(ROOT/'results/rotation-signal-audit/run-001/predictions.json'),'execution_evidence':{k:None for k in ['future_test_assignment','payload_exposure_provenance','input_manifest_lock','reference_convention_proof','selected_motion_publisher','common_frame_timing_proof','published_baseline_adapters','metric_stereo_scale_conversion','prediction_before_reference_plan','training_overlap_disclosure']},'model_sha256':models,'sha256':pins,'runtime_for_analytic_tests':'sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09','new_inference':False,'messages_or_submissions':False}
 assert not validate(p),validate(p);write(PLAN,p);write(OUT/'plan.json',p);write(OUT/'readiness.json',readiness(p));print('FROZEN',len(pins),'pins;',len(methods),'method modes; test/reserves unopened')

def verify():
 p=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json')
 for n,h in p['sha256'].items():assert sha(ROOT/n)==h,n
 for n,h in p['model_sha256'].items():assert sha(ROOT/n)==h,n
 assert not validate(p),validate(p)
 assert sha(ROOT/p['diagnostic_ridge_source'])==p['diagnostic_ridge_source_sha256']
 return p
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('action',choices=['freeze','verify']);args=a.parse_args()
 if args.action=='freeze':freeze()
 else:print(json.dumps(readiness(verify()),indent=2))

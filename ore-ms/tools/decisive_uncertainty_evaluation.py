"""One prospectively locked full-record completed-output uncertainty test.

Host: freeze/execute. Container: intake/native/predict/score/development-check.
No test labels enter prediction. Failed runs cannot be retried by this driver.
"""
from pathlib import Path
from datetime import datetime,timezone
from types import SimpleNamespace
import argparse,hashlib,json,subprocess,sys,traceback,math
from decisive_uncertainty_intake import ROOT,OUT,read,sha,write
PLAN=ROOT/'experiments/decisive-uncertainty-evaluation-2026-11-06.json'
IMAGE='sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09'
WHEEL='results/bounded-full-record/source-001/wheels/indexed_bzip2-1.6.0-cp311-cp311-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl'
CASES=[dict(condition='day',sequence='_2021-08-13-16-31-10',sync_url='https://dl.dropboxusercontent.com/s/zk2pc0lisaeevtw/_2021-08-13-16-31-10.tar.bz2',odom_url='https://dl.dropboxusercontent.com/s/jjakbeaz5dggjo9/_2021-08-13-16-31-10.tar.bz2'),dict(condition='night',sequence='_2021-08-13-22-16-02',sync_url='https://dl.dropboxusercontent.com/s/g4j375qx7yqfv7a/_2021-08-13-22-16-02.tar.bz2',odom_url='https://dl.dropboxusercontent.com/s/arx6bvjoke2v6e2/_2021-08-13-22-16-02.tar.bz2')]
def now():return datetime.now(timezone.utc).isoformat()
def verify():
 p=read(PLAN)
 assert sha(PLAN)==sha(OUT/'plan.json')
 for name,digest in p['sha256'].items():assert sha(ROOT/name)==digest,name
 return p

def freeze():
 assert not PLAN.exists() and not (OUT/'raw').exists() and not (OUT/'references').exists()
 parent=read(ROOT/'experiments/decisive-test-gates-2026-11-06.json')
 for name,digest in parent['evidence_source_sha256'].items():assert sha(ROOT/name)==digest,name
 files=set()
 # Pin all local estimator, configuration, orchestration and scientific helpers,
 # rather than only their top-level entry points.
 for base in ['tools','MAC-VO-main']:
  for f in (ROOT/base).rglob('*'):
   if f.is_file() and f.suffix in ['.py','.yaml','.yml'] and not any(x in f.parts for x in ['.git','__pycache__']):files.add(f)
 for f in [ROOT/WHEEL,ROOT/'results/provided-reference-full-record/run-001/source-001/official-loader.py',ROOT/'experiments/decisive-test-gates-2026-11-06.json',ROOT/'docs/research/decisive-uncertainty-execution-2026-11-06.md',OUT/'source/script-provenance.json',OUT/'development-check.json'] :files.add(f)
 files.update(ROOT/name for name in parent['expert_assets'])
 p=dict(frozen_at_utc=now(),parent_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),runtime=IMAGE,cases=CASES,active_test=[c['sequence'] for c in CASES],assignment='whole recordings; qualified project-unexamined within audited recorded lineage; unknown checkpoint training overlap',sha256={str(f.relative_to(ROOT)):sha(f) for f in sorted(files)},maximum_sync_archive_bytes=64*1024**3,native_timeout_seconds=14400,one_attempt_per_expert=True,all_original_source_ids=True,reference_acquisition_after_global_prediction_lock=True,primary='unchanged EXP131 angular_uncertainty',invalid_features_fallback='thermal',nominal_target='provided rectified RGB optical i->i+1; official extrinsics, NED basis, existing stationary twist, provided zero-offset unit-rate clock assumption',physical_frame_verified=False,physical_clock_verified=False,benchmark_admission=False,minimum_joint_intervals=1000,minimum_coverage=.95,minimum_rotation_headroom=.10,minimum_rotation_gain=.05,maximum_translation_increase=.05,baselines=['rgb','thermal','metadata day RGB/night thermal'],commands=['python3 tools/decisive_uncertainty_evaluation.py freeze','git commit execution lock','python3 tools/decisive_uncertainty_evaluation.py execute'],action_order=['verify lock','intake day','intake night','native day RGB','native day thermal','native night RGB','native night thermal','predict both and global lock','reference day','reference night','score once'],failure_policy='preserve first failure and STOP; no quality retry, source substitution, parameter change or aggregate rescue',opaque_sync_archive_may_contain_unused_navigation=True)
 write(PLAN,p);write(OUT/'plan.json',p);print('FROZEN',len(files),'pins',flush=True)

def scientific_imports():sys.path.insert(0,str(ROOT/'MAC-VO-main'))
def predict_record(source,cal,capture):
 scientific_imports()
 import numpy as np
 from Scripts.Setup.causal_interval_model import project
 from Scripts.Setup.td3_reference import matrices,OPTICAL_FROM_NED
 from rotation_signal_audit import feature_vector,fixed_choices
 import selected_motion_study
 selected_motion_study.SOURCE=ROOT/'results/provided-reference-full-record/run-001/source-001/official-loader.py'
 assert sha(selected_motion_study.SOURCE)=='299250fcdc33548b6cab3bf585d3f5b205af3635eb038e7e113fee73447de9a1'
 q=np.eye(4);q[:3,:3]=OPTICAL_FROM_NED;transforms=dict(rgb=q,thermal=selected_motion_study.source_frame(cal)@q);eventsets={};pins={};focal={}
 for mode,camera in [('rgb','rgb'),('thermal','thr')]:
  path=capture/mode/'arbiter-events/events.jsonl';events=[json.loads(line) for line in path.read_text().splitlines()];pins[str(path.relative_to(ROOT))]=sha(path);groups={}
  for e in events:
   if e['kind']=='pair_end' or (e['kind']=='motion_result_written' and e.get('first_write_for_submission')):groups.setdefault((e['kind'],e['time1_ns']),[]).append(e)
  eventsets[mode]=groups;k=np.asarray(source['records'][0]['cameras'][camera]['K']);focal[mode]=float((k[0,0]**2+k[1,1]**2)/2)
 rows=[]
 for i in range(len(source['records'])-1):
  common=[source['records'][j]['cameras']['rgb']['left']['recorded_time_ns'] for j in [i,i+1]];experts={};raw={}
  for mode,camera in [('rgb','rgb'),('thermal','thr')]:
   native=[source['records'][j]['cameras'][camera]['left']['recorded_time_ns'] for j in [i,i+1]];g=eventsets[mode];hs=g.get(('pair_end',native[1]),[]);ms=g.get(('motion_result_written',native[1]),[]);reason=None
   h=hs[0] if len(hs)==1 and [hs[0]['time0_ns'],hs[0]['time1_ns']]==native else None
   raw[mode]={k:h.get(k) if h else None for k in ['grid_trace_q10_q50_q90_px2','inbound_count','retained_count','occupied_3x3_cells']}
   m=ms[0] if len(ms)==1 else None
   if h is None:reason='missing_ambiguous_or_nonadjacent_health'
   elif m is None:reason='missing_or_ambiguous_first_motion'
   elif [m['time0_ns'],m['time1_ns']]!=native:reason='nonadjacent_native_motion'
   elif m['stage']!='pair_processing':reason='source_motion_not_pair_processing'
   elif m['original_need_interp'] is not False:reason='native_unsupported'
   if reason:result=dict(valid_model_projection=False,reason=reason,delta_model=None)
   else:
    try:
     d=matrices([m['derived_camera_delta_xyzw']])[0];e=transforms[mode];result=project(e@d@np.linalg.inv(e),native,common)
    except (ValueError,KeyError,TypeError) as error:result=dict(valid_model_projection=False,reason='invalid_native_motion: '+str(error),delta_model=None)
   experts[mode]=dict(result,native_interval_ns=native,target_interval_ns=common,native_motion_event=m['sequence'] if m else None,health_event=h['sequence'] if h else None,source_stage=m['stage'] if m else None,physical_body_transform_verified=False,sensor_clock_verified=False,admission_ready=False)
  x,reason=feature_vector(raw,focal);choices=fixed_choices(dict(features=x,raw=raw));supported=[m for m in ['rgb','thermal'] if experts[m]['valid_model_projection']];choice=choices['angular_uncertainty'] if len(supported)==2 else supported[0] if supported else 'ABSTAIN'
  rows.append(dict(pair=i,source_ids=[source['records'][j]['stem'] for j in [i,i+1]],common_interval_ns=common,experts=experts,raw=raw,features=x,feature_reason=reason,raw_angular_choice=choices['angular_uncertainty'],angular_choice=choice,raw_uncertainty_choice=choices['raw_uncertainty'],reference_used=False))
 return dict(rows=rows,event_sha256=pins,expected_intervals=len(rows),reference_used=False)

def predict():
 p=verify();assert not (OUT/'references').exists() and not (OUT/'reference-transport').exists();pins={}
 for c in p['cases']:
  name=c['condition'];base=OUT/'inputs'/name;result=predict_record(read(base/'source-manifest.json'),read(base/'calibration.json'),OUT/'captures'/name);path=OUT/'predictions'/name/'predictions.json';write(path,result);pins[str(path.relative_to(ROOT))]=sha(path)
 write(OUT/'prediction-lock.json',dict(locked_at_utc=now(),plan_sha256=sha(PLAN),sha256=pins,new_reference_values_read=False));print('ALL PREDICTIONS LOCKED',flush=True)

def gates(expected,matched,means,oracle,coverage_loss=False):
 sufficient=matched>=1000 and matched/expected>=.95
 best=min(means['rgb']['rotation_deg'],means['thermal']['rotation_deg']) if matched else 0
 opportunity=bool(best>0 and oracle<=.90*best)
 gains={m:bool(means['angular']["rotation_deg"]<=.95*means[m]['rotation_deg']) for m in ['rgb','thermal','metadata']} if matched else {}
 translation={m:bool(means['angular']['translation_m']<=1.05*means[m]['translation_m']) for m in ['rgb','thermal','metadata']} if matched else {}
 return dict(sufficient=sufficient,headroom=opportunity,rotation_gates=gains,translation_gates=translation,no_coverage_loss=not coverage_loss,candidate_pass=bool(matched and all(gains.values()) and all(translation.values()) and not coverage_loss))
def decision(reports):
 if not all(r['sufficient'] for r in reports):return 'STOP_INSUFFICIENT_EVALUATION'
 if not all(r['headroom'] for r in reports):return 'STOP_NO_PRACTICAL_HEADROOM'
 if not all(r['candidate_pass'] for r in reports):return 'STOP_NO_SIGNAL_FOR_CURRENT_RULE'
 return 'PASS_LIMITED_BENCHMARK_SIGNAL'
def score():
 p=verify();lock=read(OUT/'prediction-lock.json');assert lock['plan_sha256']==sha(PLAN)
 for n,h in lock['sha256'].items():assert sha(ROOT/n)==h
 scientific_imports()
 import numpy as np
 from ms2_broader_evaluate import reference_pose
 from Scripts.Setup.arbiter_pose_proxy import losses
 reports={}
 for c in p['cases']:
  name=c['condition'];data=read(OUT/'predictions'/name/'predictions.json');refbase=OUT/'references'/name;reflock=read(refbase/'reference-lock.json')
  for m in reflock['members']:assert sha(ROOT/m['path'])==m['sha256']
  poses={};qualified={}
  for m in reflock['members']:
   try:a,r=reference_pose(ROOT/m['path'])
   except (ValueError,OSError) as e:a,r=None,dict(valid=False,reason=str(e))
   poses[m['stem']]=a;qualified[m['stem']]=r
  rows=[];excluded=[]
  for row in data['rows']:
   a,b=[poses[s] for s in row['source_ids']]
   if a is None or b is None or not all(e['valid_model_projection'] for e in row['experts'].values()):excluded.append(dict(pair=row['pair'],source_ids=row['source_ids'],invalid_reference=a is None or b is None,expert_reasons={m:e.get('reason') for m,e in row['experts'].items() if not e['valid_model_projection']}));continue
   ref=np.linalg.solve(a,b);loss={m:losses(e['delta_model'],ref) for m,e in row['experts'].items()};choices=dict(rgb='rgb',thermal='thermal',metadata='rgb' if name=='day' else 'thermal',angular=row['angular_choice'],raw_uncertainty=row['raw_uncertainty_choice']);assert choices['angular'] in ['rgb','thermal'];rows.append(dict(pair=row['pair'],source_ids=row['source_ids'],reference_delta=ref.tolist(),loss=loss,choices=choices))
  means={m:{metric:math.fsum(r['loss'][r['choices'][m]][metric] for r in rows)/len(rows) for metric in ['rotation_deg','translation_m']} for m in ['rgb','thermal','metadata','angular','raw_uncertainty']} if rows else {}
  oracle=math.fsum(min(r['loss'][m]['rotation_deg'] for m in ['rgb','thermal']) for r in rows)/len(rows) if rows else None
  gate=gates(data['expected_intervals'],len(rows),means,oracle,any(r['angular_choice']=='ABSTAIN' and any(e['valid_model_projection'] for e in r['experts'].values()) for r in data['rows']))
  # Independent reduction via numpy, with matching decisions, catches scalar accumulation or method-index mistakes.
  if rows:
   for method in means:
    for metric in means[method]:assert math.isclose(means[method][metric],float(np.asarray([r['loss'][r['choices'][method]][metric] for r in rows]).mean()),rel_tol=1e-12,abs_tol=1e-12)
  report=dict(condition=name,expected_intervals=data['expected_intervals'],matched_intervals=len(rows),coverage=len(rows)/data['expected_intervals'],means=means,rotation_oracle_mean_deg=oracle,gates=gate,rows=rows,excluded=excluded,reference_qualification=qualified);write(OUT/'scores'/name/'scores.json',report);reports[name]=dict(expected_intervals=data['expected_intervals'],matched_intervals=len(rows),coverage=report['coverage'],means=means,rotation_oracle_mean_deg=oracle,**gate)
 result=dict(completed_at_utc=now(),status=decision(list(reports.values())),records=reports,plan_sha256=sha(PLAN),prediction_lock_sha256=sha(OUT/'prediction-lock.json'),scope='two whole recordings; nominal provided reference proxy; unknown checkpoint training overlap; no physical synchronization or online scheduling claim',independent_mean_reduction_passed=True)
 write(OUT/'decision.json',result);print(json.dumps(result,indent=2),flush=True)

def native(c,mode):
 scientific_imports()
 from Scripts.Setup.run_ms2_execution import run
 base=OUT/'inputs'/c['condition'];args=SimpleNamespace(dataset=ROOT,manifest=base/'paired-manifest.json',output=OUT/'captures'/c['condition'],frames=len(read(base/'source-manifest.json')['records']),start=0,record_uncertainty=True,record_arbiter_events=True,record_root_initialization=True,acknowledge_provisional_geometry=True)
 run(args,mode)

def container_command(action,condition=None,mode=None):
 name='ore-ms-decisive-'+action+('-'+condition if condition else '')+('-'+mode if mode else '')
 cmd=['docker','run','--rm','--name',name,'--ipc=host','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-e','PYTHONDONTWRITEBYTECODE=1','-v',str(ROOT)+':/workspace','-w','/workspace/MAC-VO-main']
 cmd+=['--gpus','all'] if action=='native' else []
 if action not in ['intake','reference']:cmd+=['--network=none']
 cmd+=['--entrypoint','python',IMAGE,'/workspace/tools/decisive_uncertainty_evaluation.py',action]
 if condition:cmd+=['--condition',condition]
 if mode:cmd+=['--mode',mode]
 return name,cmd

def execute():
 p=verify();write(OUT/'execution-start.json',dict(started_at_utc=now(),commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),plan_sha256=sha(PLAN)))
 stage='intake'
 try:
  actions=[('intake',c['condition'],None) for c in p['cases']]+[('native',c['condition'],m) for c in p['cases'] for m in ['rgb','thermal']]+[('predict',None,None)]+[('reference',c['condition'],None) for c in p['cases']]+[('score',None,None)]
  for stage,condition,mode in actions:
   verify();name,cmd=container_command(stage,condition,mode);start=now();print('START',stage,condition,mode,start,flush=True);log=OUT/'logs'/(name+'.log');log.parent.mkdir(parents=True,exist_ok=True)
   try:
    with log.open('x') as f:run=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=p['native_timeout_seconds'] if stage=='native' else 14400)
   except subprocess.TimeoutExpired:
    subprocess.run(['docker','rm','-f',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL);raise
   write(OUT/'receipts'/(name+'.json'),dict(action=stage,condition=condition,mode=mode,command=cmd,started_at_utc=start,ended_at_utc=now(),returncode=run.returncode,log_sha256=sha(log)))
   if run.returncode:raise RuntimeError(name+' exit '+str(run.returncode))
   print('DONE',stage,condition,mode,now(),flush=True)
 except Exception as e:
  write(OUT/'failure.json',dict(at_utc=now(),stage=stage,error=str(e),traceback=traceback.format_exc(),status='STOP_NOT_EXECUTABLE_CURRENT_INPUTS' if stage=='intake' else 'STOP_INSUFFICIENT_EVALUATION',scientific_rule_rejected=False,no_retry=True));raise

def development_check():
 scientific_imports()
 source=read(ROOT/'results/bounded-asset-recovery/run-001/source-manifest.json');result=predict_record(source,read(ROOT/'results/ms2-night-pilot/calibration.json'),ROOT/'results/bounded-asset-recovery/run-001/captures');prior=read(ROOT/'results/decisive-test-gates/run-001/target-qualification.json')['records']
 import numpy as np
 for a,b in zip(result['rows'],prior):
  assert a['source_ids']==b['source_ids'] and a['features']==b['features'] and a['raw_angular_choice']==b['raw_angular_choice']
  for m in ['rgb','thermal']:
   assert a['experts'][m]['valid_model_projection']==b['experts'][m]['valid_model_projection']
   if a['experts'][m]['valid_model_projection']:assert np.allclose(a['experts'][m]['delta_model'],b['experts'][m]['delta_model'],rtol=0,atol=1e-14)
 assert len(result['rows'])==len(prior)==19
 means={m:dict(rotation_deg=10.,translation_m=1.) for m in ['rgb','thermal','metadata','angular']};means['angular']['rotation_deg']=9.
 g=gates(1100,1050,means,8.);assert decision([g,g])=='PASS_LIMITED_BENCHMARK_SIGNAL'
 assert decision([g,gates(1100,999,means,8.)])=='STOP_INSUFFICIENT_EVALUATION'
 assert decision([g,gates(1100,1050,means,9.1)])=='STOP_NO_PRACTICAL_HEADROOM'
 means['angular']['translation_m']=1.051;assert decision([g,gates(1100,1050,means,8.)])=='STOP_NO_SIGNAL_FOR_CURRENT_RULE'
 means={m:dict(rotation_deg=0.,translation_m=0.) for m in means};assert not gates(1100,1050,means,0.)['headroom'];means['angular']['translation_m']=1e-12;assert not gates(1100,1050,means,0.)['translation_gates']['rgb']
 report=dict(at_utc=now(),development_intervals=19,both_supported=18,pinned_qualification_agrees=True,gate_fixture_cases=6,new_candidate_payload_access=False,new_reference_scoring=False,source_sha256={str(f.relative_to(ROOT)):sha(f) for f in [Path(__file__),ROOT/'tools/decisive_uncertainty_intake.py']});write(OUT/'development-check.json',report);print(json.dumps(report,indent=2))

if __name__=='__main__':
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['freeze','execute','intake','native','predict','reference','score','development-check']);parser.add_argument('--condition',choices=['day','night']);parser.add_argument('--mode',choices=['rgb','thermal']);args=parser.parse_args()
 if args.action in ['freeze','execute','predict','score','development-check']:globals()[args.action.replace('-','_')]()
 else:
  p=verify();case=next(c for c in p['cases'] if c['condition']==args.condition)
  if args.action=='native':native(case,args.mode)
  else:
   subprocess.run([sys.executable,'-m','pip','install','--no-index','--no-deps','--target','/tmp/decisive-indexed',str(ROOT/WHEEL)],check=True);sys.path.insert(0,'/tmp/decisive-indexed')
   from decisive_uncertainty_intake import images,references
   if args.action=='intake':images(case,p['maximum_sync_archive_bytes'])
   else:
    lock=read(OUT/'prediction-lock.json');assert lock['plan_sha256']==sha(PLAN)
    for name,digest in lock['sha256'].items():assert sha(ROOT/name)==digest
    references(case)

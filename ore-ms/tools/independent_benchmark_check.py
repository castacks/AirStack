"""Analytic EXP133 metric and access/readiness contract tests; no real data read."""
import copy,json,math
import numpy as np
from scipy.spatial.transform import Rotation
from independent_benchmark_contract import ROOT,OUT,PLAN,read,write,sha,verify,validate,readiness,proof_ok
from independent_benchmark_metrics import score,pose,alignment

def main():
 p=verify();checks=0;mutations=0;results=[]
 def check(v,label):
  nonlocal checks
  assert v,label
  checks+=1;results.append(label)
 def rejected(fn,label):
  ok=False
  try:fn()
  except (AssertionError,ValueError):ok=True
  check(ok,label)
 def fixture(n=31):
  times=[i*100000000 for i in range(n)];poses=[]
  for i in range(n):
   t=i*.1;M=np.eye(4);M[:3,:3]=Rotation.from_euler('z',t*.2).as_matrix();M[:3,3]=[t,.3*math.sin(1.7*t),.2*math.cos(1.3*t)];poses.append(M.tolist())
  ref={'time_ns':times,'poses':poses};est={'time_ns':times,'poses':copy.deepcopy(poses),'segments':[0]*n,'terminal_success':True};return ref,est
 ref,est=fixture();perfect=score(ref,est)
 check(perfect['pose_coverage']==1 and perfect['rpe_coverage']==1 and perfect['engineering_success'],'perfect full trajectory success')
 check(perfect['ate_se3_rmse_m']<1e-12 and perfect['ate_origin_rmse_m']<1e-12 and perfect['rpe_translation_rmse_m']<1e-12 and perfect['rpe_rotation_rmse_deg']<1e-10,'perfect metric zeros')
 check(perfect['rpe_expected_pairs']==21,'one-second source endpoint count')
 moved=copy.deepcopy(est);H=np.eye(4);H[:3,:3]=Rotation.from_euler('xyz',[.4,-.2,.6]).as_matrix();H[:3,3]=[5.,-3.,2.]
 moved['poses']=[(H@np.array(M)).tolist() for M in moved['poses']];q=score(ref,moved)
 check(q['ate_se3_rmse_m']<1e-12 and q['ate_origin_rmse_m']<1e-12,'global rigid coordinate invariance')
 check(q['rpe_translation_rmse_m']<1e-12 and q['rpe_rotation_rmse_deg']<1e-10,'relative delta direction/world invariance')
 scaled=copy.deepcopy(est)
 for M in scaled['poses']:
  for k in range(3):M[k][3]*=2
 q=score(ref,scaled);check(q['ate_se3_rmse_m']>.1 and q['rpe_translation_rmse_m']>.5 and q['scale_fitted'] is False,'metric scale error survives alignment')
 spin=copy.deepcopy(est)
 for i,M in enumerate(spin['poses']):npM=np.array(M);npM[:3,:3]=Rotation.from_euler('z',i*.03).as_matrix();spin['poses'][i]=npM.tolist()
 q=score(ref,spin);check(abs(q['rpe_rotation_rmse_deg']-math.degrees(.1))<1e-10,'analytic angular drift in degrees')
 wrongtime=copy.deepcopy(est);wrongtime['time_ns'][5]+=1;rejected(lambda:score(ref,wrongtime),'reject one-nanosecond timestamp mismatch');mutations+=1
 duplicate=copy.deepcopy(ref);duplicate['time_ns'][5]=duplicate['time_ns'][4];rejected(lambda:score(duplicate,est),'reject duplicate source time');mutations+=1
 floating=copy.deepcopy(ref);floating['time_ns']=[float(t) for t in floating['time_ns']];rejected(lambda:score(floating,est),'reject floating timestamps');mutations+=1
 missing=copy.deepcopy(est);missing['poses'][15]=None;missing['segments'][15]=None;missing['segments'][16:]=[1]*15;q=score(ref,missing)
 check(q['published_pose_frames']==30 and q['source_frames']==31 and q['reset_count']==1 and not q['engineering_success'],'missing frame stays in denominator/reset failure')
 check(all(not(i<=15<=j) for i,j in [r['source_indices'] for r in q['rpe_rows']]),'RPE cannot bridge missing interior')
 resumed=copy.deepcopy(missing);resumed['segments'][16:]=[0]*15;rejected(lambda:score(ref,resumed),'reject segment resumption after hole');mutations+=1
 badref=copy.deepcopy(ref);badref['poses'][15][0][0]=2.;q=score(badref,est)
 check(q['reference_valid_frames']==30 and q['source_frames']==31 and q['pose_coverage']==1,'invalid reference excluded from accuracy only')
 check(all(not(i<=15<=j) for i,j in [r['source_indices'] for r in q['rpe_rows']]),'RPE cannot bridge invalid reference')
 badest=copy.deepcopy(est);badest['poses'][3][0][0]=float('nan');rejected(lambda:score(ref,badest),'reject nonfinite published estimate');mutations+=1
 failed=copy.deepcopy(est);failed['terminal_success']=False;q=score(ref,failed);check(q['ate_se3_rmse_m']<1e-12 and not q['engineering_success'],'zero error cannot hide failed estimator')
 straight=copy.deepcopy(ref)
 for i,M in enumerate(straight['poses']):M[0][3]=float(i);M[1][3]=M[2][3]=0.
 straight_est=copy.deepcopy(est);straight_est['poses']=copy.deepcopy(straight['poses']);q=score(straight,straight_est);check(q['ate_se3_rmse_m'] is None and q['ate_origin_rmse_m']<1e-12,'rank-one alignment marked unidentifiable')
 rounded=copy.deepcopy(ref);rounded['poses'][0][0][0]+=1e-8;q=score(rounded,est);check(q['reference_valid_frames']==31 and 0<q['reference_rotation_max_correction']<1e-7,'bounded serialization repair recorded')
 reflected=np.eye(4);reflected[0,0]=-1.;check(pose(reflected)[0] is None,'reflection is not a proper source pose')
 gappedref=copy.deepcopy(ref);gappedest=copy.deepcopy(est);gappedref['time_ns'][15:]=[t+1000000000 for t in ref['time_ns'][15:]];gappedest['time_ns']=gappedref['time_ns'];q=score(gappedref,gappedest)
 check(q['longest_contiguous_frames']==16 and all(not(i<15<=j) for i,j in [r['source_indices'] for r in q['rpe_rows']]),'source acquisition gaps are not interpolated')
 tinyref,tinyest=fixture(2);q=score(tinyref,tinyest);check(q['rpe_expected_pairs']==0 and not q['engineering_success'],'no evaluable motion does not pass')
 check(not validate(p) and not readiness(p)['execution_ready'] and readiness(p)['active_test_recordings']==0,'valid preparation stays execution-not-ready')
 alterations=[('partition overlap',lambda x:x['partitions']['test'].append(x['partitions']['development'][0])),('activate reserves',lambda x:x['partitions']['test'].append(x['reported_reserves'][0])),('reserve pixels',lambda x:x.update(protected_payload_access=True)),('Sim3 fit',lambda x:x['metrics'].update(scale_fit=True)),('pooled ranking',lambda x:x['metrics'].update(aggregation='pooled_frames')),('official metric claim',lambda x:x['metrics'].update(official_dataset_metric=True)),('input change',lambda x:x['input'].update(thermal_recipe='new_test_tuned_recipe')),('monocular in stereo',lambda x:x['methods'][0].update(sensor_mode='monocular')),('unversioned code',lambda x:x['methods'][0].update(source_revision='latest')),('training claim',lambda x:x['claims'].update(network_heldout=True)),('outcome-selected windows',lambda x:x['sampling'].update(outcome_selected_windows=True)),('condition oracle',lambda x:x['condition_prior'].update(night='best_reference_error'))]
 for name,mutate in alterations:
  changed=copy.deepcopy(p);mutate(changed);check(bool(validate(changed)),'contract rejects '+name);mutations+=1
 changed=copy.deepcopy(p);next(m for m in changed['methods'] if m['name']=='orbslam3_rgb')['loop_closing']=True;check(bool(validate(changed)),'contract rejects hidden loop closing');mutations+=1
 check(not proof_ok({'artifact':'does-not-exist.json','sha256':'0'*64}) and not proof_ok({'artifact':'../escape.json','sha256':'0'*64}),'missing/path-escape evidence cannot unlock execution');mutations+=1
 changed=copy.deepcopy(p)
 for m in changed['methods']:
  for field in ['adapter_freeze','runtime_digest','assets_manifest','development_smoke']:m[field]='claimed_ready'
 for name in changed['execution_evidence']:changed['execution_evidence'][name]='claimed_ready'
 check(not readiness(changed)['execution_ready'] and len(readiness(changed)['unmet_execution_requirements'])==len(readiness(p)['unmet_execution_requirements']),'unverified readiness strings cannot unlock execution');mutations+=1
 write(OUT/'analytic-audit.json',{'pass':True,'checks':checks,'semantic_mutations':mutations,'source_sha256':sha(__file__),'plan_sha256':sha(PLAN),'scorer_sha256':sha(ROOT/'tools/independent_benchmark_metrics.py'),'only_analytic_fixtures':True,'protected_payload_access':False,'checks_passed':results})
 print('PASS',checks,'analytic/contract checks;',mutations,'mutations; no benchmark payload read')
if __name__=='__main__':main()

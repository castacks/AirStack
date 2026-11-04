"""Retest the frozen analytic suite plus strict estimator timestamp regressions."""
import copy,json
import independent_benchmark_check as original
import independent_benchmark_metrics as v1
import independent_benchmark_metrics_v2 as v2
from independent_benchmark_contract import ROOT,OUT,PLAN,read,write,sha,verify
AMEND=ROOT/'experiments/independent-benchmark-metrics-v2-2026-11-04.json'
verify();amend=read(AMEND)
for name,digest in amend['sha256'].items():assert sha(ROOT/name)==digest,name
captured=[];original.score=v2.score;original.pose=v2.pose;original.alignment=v2.alignment
original.write=lambda path,value:captured.append(copy.deepcopy(value))
original.main();assert len(captured)==1 and captured[0]['pass'];prior=captured[0]
poses=[]
for i in range(21):
 M=[[1.,0.,0.,i*.1],[0.,1.,0.,(i*.1)**2],[0.,0.,1.,0.],[0.,0.,0.,1.]];poses.append(M)
ref={'time_ns':[i*100000000 for i in range(21)],'poses':poses};est={'time_ns':copy.deepcopy(ref['time_ns']),'poses':copy.deepcopy(poses),'segments':[0]*21,'terminal_success':True};checks=0;cases=[]
for name,values in [('float_times',[float(t) for t in est['time_ns']]),('boolean_zero',[False]+est['time_ns'][1:])]:
 changed=copy.deepcopy(est);changed['time_ns']=values
 v1.score(ref,changed) # Explicit negative control: old scorer accepts numeric equality.
 rejected=False
 try:v2.score(ref,changed)
 except AssertionError:rejected=True
 assert rejected;checks+=1;cases.append(name)
assert v2.score(ref,est)['engineering_success'];checks+=1
write(OUT/'analytic-audit-v2.json',{'pass':True,'checks':prior['checks']+checks,'semantic_mutations':prior['semantic_mutations']+2,'base_suite_checks':prior['checks'],'new_regressions':cases,'scorer_sha256':sha(ROOT/'tools/independent_benchmark_metrics_v2.py'),'test_source_sha256':sha(__file__),'original_suite_sha256':sha(ROOT/'tools/independent_benchmark_check.py'),'amendment_sha256':sha(AMEND),'only_analytic_fixtures':True,'protected_payload_access':False})
print('PASS V2',prior['checks']+checks,'checks;',prior['semantic_mutations']+2,'mutations; original frozen scorer/outcomes retained')

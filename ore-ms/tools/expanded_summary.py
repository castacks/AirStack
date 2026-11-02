"""Predeclared record-wise contrasts; no selection or significance testing."""
from pathlib import Path
import json
import statistics
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/expanded-fixed-model-evaluation/run-001'
m=json.loads((OUT/'metrics.json').read_text());e=json.loads((OUT/'evaluation.json').read_text());records={};arms=list(m['groups']['expanded_only']['arms'])
for identity,record in m['records'].items():
 baseline=record['arms']['baseline'];base_risk=baseline['risks'][2]['selected_mse'];entries={}
 for arm,v in record['arms'].items():
  risk=v['risks'][2]['selected_mse'];coverage=v['coverage']['0.95'];entries[arm]={'mse50':risk,'relative_mse50_change_percent':100*(risk/base_risk-1) if base_risk else None,'coverage95':coverage,'absolute_coverage95_error':abs(coverage-.95),'final_nll':v['final_nll']}
 d=entries['final_diagonal_nll']['mse50'];t=entries['final_trace_risk_score']['mse50'];records[identity]={'arms':entries,'trace_vs_finaldiagonal_mse50_change_percent':100*(t/d-1) if d else None}
comparisons={}
for arm in arms[1:]:
 counts={key:{'better':0,'equal':0,'worse':0} for key in ('mse50','absolute_coverage95_error','final_nll')};relative=[];uniform=[]
 for identity,record in records.items():
  base=record['arms']['baseline'];value=record['arms'][arm]
  for key in counts:counts[key]['better' if value[key]<base[key] else 'worse' if value[key]>base[key] else 'equal']+=1
  if value['relative_mse50_change_percent'] is not None:relative.append(value['relative_mse50_change_percent'])
  uniform.append(all(value[key]<=base[key] for key in counts))
 comparisons[arm]={'record_outcome_counts_vs_original':counts,'median_record_relative_mse50_change_percent':statistics.median(relative) if len(relative)==len(records) else None,'records_with_defined_relative_change':len(relative),'mean_record_absolute_coverage95_error':sum(r['arms'][arm]['absolute_coverage95_error'] for r in records.values())/len(records),'all_records_nondegrading_on_all_three':all(uniform)}
macro=m['groups']['expanded_only']['arms'];primary_base=macro['baseline']['risks'][2]['selected_mse'];primary_recurrent=macro['recurrent_diagonal_nll']['risks'][2]['selected_mse']
result={'status':'PASS_PREDECLARED_CONTRASTS','records':records,'comparisons':comparisons,'primary':{'baseline_mse50':primary_base,'recurrent_mse50':primary_recurrent,'relative_change_percent':100*(primary_recurrent/primary_base-1) if primary_base else None},'original_mean_record_absolute_coverage95_error':sum(r['arms']['baseline']['absolute_coverage95_error'] for r in records.values())/len(records),'no_significance_or_population_claim':True,'optimizer_steps':0,'candidate_promoted':False}
with (OUT/'summary.json').open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
print('PRIMARY',json.dumps(result['primary']),flush=True)

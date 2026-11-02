"""Fixed pooled and equal-record summaries for all declared diagnostics."""
from pathlib import Path
import json
import math
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/expanded-fixed-model-evaluation/run-001'
p=json.loads((OUT/'protocol.json').read_text());m=json.loads((OUT/'metrics.json').read_text());e=json.loads((OUT/'evaluation.json').read_text());rows=m['rows'];n=sum(r['pixels'] for r in rows);out={}
for arm in p['models']:
 entries=[r['arms'][arm] for r in rows];micro={'pixels':n,'pairs':len(rows),'coverage':{str(level):sum(r['coverage_counts'][str(level)] for r in entries)/n for level in p['coverage_levels']},'mahalanobis_mean':sum(r['mahalanobis_sum'] for r in entries)/n,'final_nll':sum(r['pixels']*r['arms'][arm]['final_nll'] for r in rows)/n,'standardized_bias':[sum(r['normalized_sum'][axis] for r in entries)/n for axis in range(2)],'standardized_second_moment':[sum(r['normalized_squared_sum'][axis] for r in entries)/n for axis in range(2)],'risks':[]}
 for j,fraction in enumerate(p['retained_fractions']):
  k=sum(r['risks'][j]['retained'] for r in entries);micro['risks'].append({'fraction':fraction,'retained':k,**{metric:sum(r['risks'][j][source] for r in entries)/k for metric,source in [('selected_mse','selected_error_sum'),('random_mse','random_expected_error_sum'),('oracle_mse','oracle_error_sum')]}})
  for metric in ('selected_mse','random_mse','oracle_mse'):
   expected=sum(record['arms'][arm]['risks'][j][metric]*record['arms'][arm]['risks'][j]['retained'] for record in m['records'].values())/k
   assert math.isclose(expected,micro['risks'][j][metric],rel_tol=2e-12,abs_tol=2e-12)
 assert math.isclose(micro['final_nll'],e['arms'][arm]['groups']['expanded_only']['micro']['numpy64_final_nll'],rel_tol=2e-12,abs_tol=2e-12)
 record_entries=[record['arms'][arm] for record in m['records'].values()];macro={key:sum(record[key] for record in record_entries)/len(record_entries) for key in ('final_nll','mahalanobis_mean')}
 for key in ('standardized_bias','standardized_second_moment','mean_variance'):macro[key]=[sum(record[key][axis] for record in record_entries)/len(record_entries) for axis in range(2)]
 out[arm]={'micro':micro,'additional_macro':macro}
with (OUT/'micro.json').open('x') as stream:json.dump({'status':'PASS_ALL_DECLARED_AGGREGATIONS','arms':out,'optimizer_steps':0},stream,indent=2);stream.write('\n')
print('PASS',len(rows),'pairs',n,'pixels')

"""Standalone matched-support metric and selection-integrity reconstruction."""
from pathlib import Path
import json,hashlib,math
import numpy as np
R=Path(__file__).resolve().parents[1];P=R/'results/local-thermal-geometry/run-001';REC=R/'results/thermal-incremental-comparison/recovery'
def read(p):return json.loads(p.read_text())
lock=read(P/'prediction-lock.json');report={'comparisons':{},'selection_checks':0,'solver_source_matches_historical':False}
import tarfile
with tarfile.open('/tmp/thermal-intake/nominal.tar.gz') as t:
 x=next(x for x in t if x.name.endswith('/tools/nominal_joint_motion.py'));report['solver_source_matches_historical']=hashlib.sha256(t.extractfile(x).read()).hexdigest()==lock['solver_sha256'];assert report['solver_source_matches_historical']
for c in ['day','night','rain']:
 rows=[r for r in lock['rows'] if r['condition']==c];refs={r['pair']:r for r in read(REC/f'scores-{c}.json')['rows']};summary={}
 for row in rows:
  for arm,m in row['methods'].items():
   ids=m['selected_ids'];assert sum(map(len,ids))==64;assert all(len(v)==len(set(v)) for v in ids);report['selection_checks']+=1
   if arm in ['rgb32_thermal32_conditional','rgb32_thermal32_uniform','uniform32_each']:assert list(map(len,ids))==[32,32]
 for arm in ['rgb32_thermal32_conditional','rgb32_thermal32_uniform','uniform32_each','joint64']:
  eligible=[r for r in rows if all(r['methods'].get(a,{}).get('status')=='FIXED_BUDGET_COMPLETE' for a in ['rgb64',arm])];pair={}
  for name in ['rgb64',arm]:
   tv=[];rv=[]
   for row in eligible:
    T=row['methods'][name]['transform'];F=refs[row['pair']]['reference_delta'];tv.append(math.sqrt(sum((T[j][3]-F[j][3])**2 for j in range(3))));trace=sum(F[k][j]*T[k][j] for k in range(3) for j in range(3));rv.append(math.degrees(math.acos(max(-1,min(1,(trace-1)/2)))))
   pair[name]={'translation_mean_m':sum(tv)/len(tv),'translation_rmse_m':math.sqrt(sum(x*x for x in tv)/len(tv)),'rotation_mean_deg':sum(rv)/len(rv),'rotation_rmse_deg':math.sqrt(sum(x*x for x in rv)/len(rv))}
  summary[arm]={'matched':len(eligible),'metrics':pair}
 report['comparisons'][c]=summary
(P/'matched-comparisons.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))

"""Recorded per-camera duration inventory; no timestamp/exposure qualification."""
from pathlib import Path
import json,numpy as np
R=Path(__file__).resolve().parents[1];OUT=R/'results/fixed-group-diagnosis/run-001'
def main():
 summary={}
 for cohort,base in [('primary100',R/'results/thermal-incremental-comparison/recovery'),('longer600',R/'results/longer-fixed-thermal/run-001')]:
  summary[cohort]={}
  for c in ['day','night','rain']:
   if cohort=='primary100':rows=json.load(open(base/f'binding-{c}.json'))['rows'];source=json.load(open(base/f'{c}-source.json'))['records']
   else:rows=json.load(open(base/f'predictions-{c}.json'))['rows'];source=json.load(open(R/f'results/longer-fixed-thermal/recovery/inputs/{c}/source-manifest.json'))['records']
   ratios=[]
   for row in rows:
    i=row['pair'];dr=source[i+1]['cameras']['rgb']['left']['recorded_time_ns']-source[i]['cameras']['rgb']['left']['recorded_time_ns'];dt=source[i+1]['cameras']['thr']['left']['recorded_time_ns']-source[i]['cameras']['thr']['left']['recorded_time_ns'];ratios.append(dt/dr)
   v=np.array(ratios);summary[cohort][c]={'intervals':len(v),'duration_ratio_min':float(v.min()),'median':float(np.median(v)),'max':float(v.max()),'outside10percent':int(np.sum(np.abs(v-1)>.1))}
 (OUT/'duration-inventory.json').write_text(json.dumps({'summary':summary,'scope':'Recorded timestamp differences only; +/-10% is descriptive, not a synchronization or exposure qualification threshold. Posthoc inventory, no offset fitting or frame search.'},indent=2)+'\n')
if __name__=='__main__':main()

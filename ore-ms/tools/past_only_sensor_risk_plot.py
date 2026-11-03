"""Standalone original-rig replication, recording-level effects and timing budgets."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/past-only-sensor-risk/run-001'
def read(p):return json.loads(Path(p).read_text())
def main():
 series=read(OUT/'persistence-replication.json')['series'];scores=read(OUT/'scores.json')['horizons']['500000000'];fig,axes=plt.subplots(1,3,figsize=(15,4.8));ax=axes[0]
 for domain,color,label in [('td3','#246bb2','TD3 original rig'),('ms2','#de6b22','MS²')]:
  s=[s for s in series if s['domain']==domain];ax.scatter([s['lags']['1']['raw']['rho'] for s in s],[s['lags']['1']['rate']['rho'] for s in s],color=color,label=label,s=30,alpha=.8)
 ax.plot([-1,1],[-1,1],color='grey',ls='--');ax.set_xlim(-1,1);ax.set_ylim(-1,1);ax.set_xlabel('raw adjacent covariance rank correlation');ax.set_ylabel('after dividing covariance by recorded dt²');ax.set_title('Interval normalization: different rigs');ax.legend(fontsize=8);ax.grid(alpha=.2)
 names=['td3/20251027/0_salon_so_day','td3/20251027/1_salon_mac_cov_day','td3/20251027/night_mac_salon_warehousefig8','td3/20251027/night_thermal_so_salon2','ms2/_2021-08-06-11-23-45','ms2/_2021-08-06-16-19-00','ms2/_2021-08-13-21-18-04'];short=['TD3 day0','TD3 day1','TD3 night1','TD3 night2','MS² day','MS² rain','MS² night'];values=[];counts=[]
 for n in names:
  r=scores['records'][n]['current_history_matched'];values.append(100*(r['history']['translation_m']/r['current']['translation_m']-1));counts.append(r['current']['scored'])
 ax=axes[1];ax.barh(short,values,color=['#246bb2']*4+['#de6b22']*3);ax.axvline(0,color='black',lw=.8);ax.invert_yaxis();ax.set_xlabel('history vs latest-state T RMSE change (%)');ax.set_title('0.5 s forecast; positive = worse')
 for i,(v,n) in enumerate(zip(values,counts)):ax.text(max(v,0)+.12,i,f'n={n}',va='center',fontsize=8)
 ax.set_xlim(-.5,max(values)+4);ax.grid(axis='x',alpha=.2)
 ax=axes[2];med=[scores['budget'][d]['median_prefix_delay_s'] for d in ['td3','ms2']];p95=[scores['budget'][d]['p95_prefix_delay_s'] for d in ['td3','ms2']];ax.bar(['TD3','MS²'],med,color=['#246bb2','#de6b22']);ax.errorbar([0,1],med,yerr=[np.zeros(2),np.asarray(p95)-med],fmt='none',ecolor='black',capsize=5,label='95th percentile');ax.axhline(.5,color='red',ls='--',label='primary 0.5 s lead');ax.axhline(1,color='grey',ls=':',label='1 s sensitivity');ax.set_ylim(0,1.65);ax.set_ylabel('instrumented offline prefix delay (s)');ax.set_title('Feature-interface timing budget');ax.legend(fontsize=8)
 fig.suptitle('EXP115: exposed-data development probe, seven complete-recording exclusions\nProvided-reference losses; no prospective/live deadline or sensor-skipping claim',fontsize=11);fig.tight_layout(rect=[0,0,1,.89]);dest=ROOT/'docs/research/figures'
 for ext in ['png','pdf']:fig.savefig(dest/f'past-only-sensor-risk-2026-11-03.{ext}',dpi=180)
 print('PLOTTED original-rig timing and fixed future-risk comparison')
if __name__=='__main__':main()

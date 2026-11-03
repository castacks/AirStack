"""Standalone record/window traces; no pooling of correlated frames as independent."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/ms2-broader-range/run-001'
def read(p):return json.loads(Path(p).read_text())
def main():
 plan=read(OUT/'execution-plan.json');scores=read(OUT/'scores.json');labels=read(OUT/'fresh-labels.json');rows=read(OUT/'locked-actions.json')['rows'];fig,axes=plt.subplots(3,3,figsize=(13,9),sharey=True)
 for ax,c in zip(axes.flat,plan['cases']):
  subset=[r for r in rows if r['window']==c['case']];t=np.asarray([row['rgb_left']['time_ns'] for row in c['rows']],np.int64);t=(t-t[0])/1e9
  for cam,col in [('rgb','#246bb2'),('thermal','#de6b22')]:
   x=[];y=[]
   for r in subset:
    l=labels.get(r['sample_id'],{}).get(cam)
    if l is not None:x.append(t[r['pair_ordinal']+1]);y.append(max(l['translation_m'],1e-6))
   ax.plot(x,y,color=col,lw=1,label=cam.upper())
  chosen=[r for r in subset if r['sample_id'] in labels and r['actions']['geometry_admission']!='ABSTAIN'];ax.scatter([t[r['pair_ordinal']+1] for r in chosen],[max(labels[r['sample_id']][r['actions']['geometry_admission'].lower()]['translation_m'],1e-6) for r in chosen],s=7,color='black',alpha=.5,label='fixed geometry choice')
  n=scores['windows'][c['case']]['groups']['both_supported']['population'];balance=scores['translation_winner_balance_by_window'][c['case']];ax.set_title(f"{c['case']}: both={n}, RGB/T wins={balance.get('rgb',0)}/{balance.get('thermal',0)}",fontsize=9);ax.set_yscale('log');ax.grid(alpha=.2);ax.set_xlabel('recorded RGB time in window (s)')
 for ax in axes[:,0]:ax.set_ylabel('provided-reference translation error (m)')
 axes[0,0].legend(fontsize=7);fig.suptitle('MS²: frozen 100-group windows; original RGB and thermal experts\nAdjacent errors are correlated; provided references and constant-twist transport remain qualified',fontsize=12);fig.tight_layout(rect=[0,0,1,.94]);dest=ROOT/'docs/research/figures';dest.mkdir(exist_ok=True)
 for ext in ['png','pdf']:fig.savefig(dest/f'ms2-broader-range-2026-11-02.{ext}',dpi=180)
 print('PLOTTED',9,'windows')
if __name__=='__main__':main()

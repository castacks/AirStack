"""Saved EXP149 per-image ambiguity/error associations, no fit or selection."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];s=json.loads((ROOT/'results/native-match-ambiguity/run-001/summary.json').read_text());fig,axs=plt.subplots(3,2,figsize=(13,10))
variables=[('descriptor_margin','Descriptor Hamming margin (expected negative)'),('descriptor_ratio','Descriptor best/second ratio (expected positive)'),('patch_relative_gap','Separated patch relative gap (expected negative)')]
for col,mode in enumerate(['rgb','thermal']):
 for row,(key,label) in enumerate(variables):
  ax=axs[row,col]
  for group,color,marker,labelgroup in [('all_primary','#2676a7','o','All primary'),('locally_consistent','#d47b20','x','5x5 consistent reference')]:
   values=[(c['index'],c['groups'][group]['correlations'][key]['rho']) for c in s['cases'] if c['modality']==mode and c['groups'][group]['correlations'][key]['rho'] is not None]
   ax.scatter([x for x,y in values],[y for x,y in values],color=color,marker=marker,s=32,label=labelgroup+f' (n={len(values)})')
  ax.axhline(0,color='gray',ls='--',lw=1);ax.set_ylim(-1,1);ax.set_xlim(-50,1950);ax.grid(alpha=.2);ax.set_ylabel('Per-image Spearman rho');ax.set_title(('RGB' if mode=='rgb' else 'Thermal')+' — '+label,fontsize=10);ax.legend(fontsize=8,loc='lower right')
  if row==2:ax.set_xlabel('Original source index')
fig.suptitle('Initialized native stereo replay: ambiguity versus projected-depth proxy error',fontsize=15,y=.98)
fig.subplots_adjust(top=.92,bottom=.14,hspace=.35,wspace=.25)
fig.text(.5,.025,'All 1,398 evaluated points reproduce historical and native-cost geometry exactly; 2 other first-frame features differ.\nAt least 10 faithful finite points per correlation; empty/small/constant cohorts remain unavailable.\nExposed single night prefix and conditional LiDAR proxy; no calibrated confidence, selector or physical truth claim.',ha='center',fontsize=10)
f=ROOT/'docs/research/figures/native-match-ambiguity-2026-11-05'
for ext in ['png','pdf']:fig.savefig(str(f)+'.'+ext,dpi=150)
print(f)

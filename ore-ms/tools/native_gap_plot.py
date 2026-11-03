"""Standalone native gap engineering figure; saved summaries only."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
s=json.loads((ROOT/'results/native-gap-replay/run-001/scores.json').read_text());items=list(s['groups'].items())
labels=[k.replace('/','\n') for k,g in items];x=np.arange(len(items));fig,axes=plt.subplots(1,3,figsize=(14,4.4))
for arm,offset,color in [('dense',-.18,'#496b91'),('sparse',.18,'#c08649')]:
 axes[0].bar(x+offset,[g[arm+'_cost']['frontend_pair_calls'] for k,g in items],width=.36,color=color,label=arm)
 axes[1].bar(x+offset,[g[arm+'_cost']['run_seconds'] for k,g in items],width=.36,color=color)
 vals=[g['strata']['all']['metrics'][arm+'_proxy']['translation_m']['rmse'] for k,g in items]
 axes[2].bar(x+offset,[v if v is not None else np.nan for v in vals],width=.36,color=color)
for ax,title,ylabel in zip(axes,['Actual native pair calls','Synchronized native processing','Matched endpoint proxy (log scale)'],['Calls','Seconds (startup/depth probe excluded)','Translation RMSE (m)']):
 ax.set_title(title,fontsize=10);ax.set_ylabel(ylabel);ax.set_xticks(x,labels,fontsize=8);ax.grid(axis='y',alpha=.2)
axes[2].set_yscale('log')
for i,(key,g) in enumerate(items):
 vals=[g['strata']['all']['metrics'][arm+'_proxy']['translation_m']['rmse'] for arm in ['dense','sparse']]
 top=max(v for v in vals if v is not None)
 label='n='+str(g['strata']['all']['common_supported'])+(' (terminal)' if key=='middle/rgb' else '')
 axes[2].text(i,top*1.2,label,ha='center',fontsize=7)
axes[2].set_ylim(.006,12)
axes[0].legend();fig.suptitle('EXP118 · Frozen open-loop gaps, not an adaptive controller',fontsize=13)
fig.text(.5,.01,'All requested intervals retained in support counts; proxy bars show point estimates, not confidence intervals. Recorded-SLAM / nominal-frame proxy.',ha='center',fontsize=8)
fig.tight_layout(rect=(0,.04,1,.94));out=ROOT/'docs/research/figures';out.mkdir(exist_ok=True)
for suffix in ['png','pdf']:fig.savefig(out/('native-gap-replay-2026-11-03.'+suffix),dpi=180)
print('Saved native gap cost/support-qualified proxy figure')

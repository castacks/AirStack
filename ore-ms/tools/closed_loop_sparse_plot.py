"""Standalone cost/availability/proxy figure from locked EXP119 summaries."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];s=json.loads((ROOT/'results/closed-loop-sparse/run-001/scores.json').read_text())
items=list(s['groups'].items());x=np.arange(len(items));fig,axes=plt.subplots(2,2,figsize=(11,7))
for arm,offset,color,label in [('full_read_cache',-.18,'#496b91','full read'),('bounded_probe',.18,'#c08649','own-feedback sparse')]:
    values=[[g[arm]['cost']['total_pair_calls'] for k,g in items],
            [g[arm]['eligible'] for k,g in items],
            [g[arm]['cost']['total_run_seconds'] for k,g in items],
            [g['matched_proxy_metrics'][arm]['translation_m']['rmse'] for k,g in items]]
    for ax,v in zip(axes.flat,values):ax.bar(x+offset,[a if a is not None else np.nan for a in v],width=.36,color=color,label=label)
for ax,title,ylabel in zip(axes.flat,['Actual native pair calls','Guarded preselected adjacent deliveries','Synchronized native processing','Matched selected-motion SLAM proxy'],['Calls','Eligible core source slots','Seconds (model startup/depth probe excluded)','Translation RMSE (m)']):
    ax.set_xticks(x,[k for k,g in items]);ax.set_title(title,fontsize=10);ax.set_ylabel(ylabel);ax.grid(axis='y',alpha=.2)
for i,(k,g) in enumerate(items):
    axes[0,1].text(i,g['full_read_cache']['slots']+2,'total '+str(g['full_read_cache']['slots']),ha='center',fontsize=8)
    v=[g['matched_proxy_metrics'][a]['translation_m']['rmse'] for a in ['full_read_cache','bounded_probe']]
    if all(a is not None for a in v):axes[1,1].text(i,max(v)*1.05,'matched n='+str(len(g['matched_common_slots'])),ha='center',fontsize=8)
axes[0,0].legend(fontsize=8);fig.suptitle('EXP119 · Native sparse feedback on its own observed prefix and clock',fontsize=13)
fig.text(.5,.01,'Serial offline execution on exposed windows. Proxy bars are point estimates; no physical-accuracy, live-queue or generalization claim.',ha='center',fontsize=8)
fig.tight_layout(rect=(0,.04,1,.95));out=ROOT/'docs/research/figures';out.mkdir(exist_ok=True)
for suffix in ['png','pdf']:fig.savefig(out/('closed-loop-sparse-2026-11-03.'+suffix),dpi=180)
print('Saved EXP119 figure')

"""Standalone return-delivery latency and actual-call comparison."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];s=json.loads((ROOT/'results/readiness-continuation/run-001/scores.json').read_text())
arms=['full_read_cache','bounded_probe','readiness_probe'];names=['full read','original bounded','readiness continuation'];colors=['#496b91','#b3824b','#348267']
fig,axes=plt.subplots(2,1,figsize=(11,7));x=np.arange(4);maxdelay=0
for a,label,color,offset in zip(arms,names,colors,[-.25,0,.25]):
    lat=s['groups'][a]['transition_latency'];values=[l['first_on_time_adjacent_return']['source_groups_after_anchor'] if l['first_on_time_adjacent_return'] else None for l in lat]
    axes[0].bar(x+offset,[v if v is not None else np.nan for v in values],width=.24,label=label,color=color)
    for i,v in enumerate(values):
        if v is None:axes[0].scatter(i+offset,.12,marker='x',color=color,s=35)
        else:axes[0].text(i+offset,v+.1,str(v),ha='center',fontsize=8);maxdelay=max(maxdelay,v)
axes[0].set_ylim(0,maxdelay+2);axes[0].set_xticks(x,['RGB return35','thermal return65','joint return80:RGB','joint return80:thermal'],fontsize=9)
axes[0].set_ylabel('Source groups until adjacent motion delivery');axes[0].set_title('Actual return output receipt; x means not observed within the horizon',fontsize=11);axes[0].legend(fontsize=9)
axes[1].bar(np.arange(3),[s['groups'][a]['cost']['total_pair_calls'] for a in arms],color=colors)
axes[1].set_xticks(np.arange(3),names);axes[1].set_ylabel('Actual native pair calls')
for i,a in enumerate(arms):
    g=s['groups'][a];axes[1].text(i,g['cost']['total_pair_calls']+2,'eligible '+str(g['eligible'])+'/95',ha='center',fontsize=9)
axes[1].set_ylim(0,max(s['groups'][a]['cost']['total_pair_calls'] for a in arms)*1.18)
for ax in axes:ax.grid(axis='y',alpha=.2)
fig.suptitle('EXP121 · Bounded adjacent-output continuation under the fixed synthetic outages',fontsize=12)
fig.text(.5,.01,'Same exposed day100, native estimator and transport masks. Readiness overlay:4 attempts/session,20 extra requests/run. Serial offline results.',ha='center',fontsize=8)
fig.tight_layout(rect=(0,.04,1,.94));dest=ROOT/'docs/research/figures'
for ext in ['png','pdf']:fig.savefig(dest/('readiness-continuation-2026-11-03.'+ext),dpi=180)
print('Saved EXP121 latency/call comparison')

"""Target51 mechanism and actual native costs, with fixed fixtures identified."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];s=json.loads((ROOT/'results/buffered-handover/run-001/scores.json').read_text())
arms=['full_read_cache','bounded_probe','fixture_off','fixture_on','buffered_probe']
names=['full read','original bounded','frozen trace:off','frozen trace:catch-up','live buffered']
fig,axes=plt.subplots(2,1,figsize=(12,7))
for y,(arm,label) in enumerate(zip(arms,names)):
    rows=[r for r in s['rows'] if r['arm']==arm and 47<=r['pair']+1<=54]
    good=[r['pair']+1 for r in rows if r['selected_native_eligible']]
    bad=[r['pair']+1 for r in rows if r['action']!='ABSTAIN' and not r['selected_native_eligible']]
    axes[0].scatter(good,[y]*len(good),color='#28744b',s=45,label='eligible selected target' if y==0 else None)
    axes[0].scatter(bad,[y]*len(bad),marker='x',color='#a32929',s=45,label='selected target unavailable' if y==0 else None)
axes[0].axvline(51,color='#c58a39',alpha=.5);axes[0].set_yticks(range(5),names);axes[0].invert_yaxis()
axes[0].set_xticks(range(47,55));axes[0].set_xlabel('Target ending source frame; target51 must be written by group52')
axes[0].set_title('Same failing request trace off/on isolates the handover mechanism',fontsize=11);axes[0].legend(loc='upper right',fontsize=8)
colors=['#496b91','#b3824b','#b0a18d','#56a28a','#28744b']
calls=[s['groups'][a]['cost']['total_pair_calls'] for a in arms]
axes[1].bar(range(5),calls,color=colors);axes[1].set_xticks(range(5),names);axes[1].set_ylabel('Actual native pair calls')
for i,a in enumerate(arms):
    g=s['groups'][a];axes[1].text(i,calls[i]+2,str(g['eligible'])+'/95 eligible',ha='center',fontsize=9)
axes[1].set_ylim(0,max(calls)*1.18)
for ax in axes:ax.grid(axis='x' if ax is axes[0] else 'y',alpha=.2)
fig.suptitle('EXP122 · Bounded retained-input handover catch-up under the unchanged synthetic outages',fontsize=12)
fig.text(.5,.01,'All arms retain encoded current/prior frames. Each catch-up is charged; original source/group deadlines remain unchanged. Serial offline test.',ha='center',fontsize=8)
fig.tight_layout(rect=(0,.04,1,.94));out=ROOT/'docs/research/figures'
for ext in ['png','pdf']:fig.savefig(out/('buffered-handover-2026-11-03.'+ext),dpi=180)
print('Saved EXP122 handover/cost figure')

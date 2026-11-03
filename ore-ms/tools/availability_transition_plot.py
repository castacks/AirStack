"""Standalone synthetic environment/action/delivery timeline from locked scores."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];s=json.loads((ROOT/'results/availability-transitions/run-001/scores.json').read_text())
plan=json.loads((ROOT/'experiments/availability-transitions-2026-11-03.json').read_text())
fig,axes=plt.subplots(4,1,figsize=(12,8),sharex=True)
for ax,(arm,g) in zip(axes,s['groups'].items()):
    rows=[r for r in s['rows'] if r['arm']==arm];x=[r['pair']+1 for r in rows];y=[{'ABSTAIN':0,'RGB':1,'THERMAL':2}[r['action']] for r in rows]
    for o,color in zip(plan['outages'],['#dca0a0','#e8cc97','#aab4c2']):ax.axvspan(o['start_source']-.5,o['end_source']+.5,color=color,alpha=.5)
    ax.step(x,y,where='mid',color='#496b91',linewidth=1)
    ax.scatter([r['pair']+1 for r in rows if r['selected_native_eligible']],[{'RGB':1,'THERMAL':2}[r['action']] for r in rows if r['selected_native_eligible']],s=12,color='#28744b',label='eligible selected delivery')
    ax.scatter([r['pair']+1 for r in rows if r['action']!='ABSTAIN' and not r['selected_native_eligible']],[{'RGB':1,'THERMAL':2}[r['action']] for r in rows if r['action']!='ABSTAIN' and not r['selected_native_eligible']],marker='x',s=25,color='#a32929',label='selected delivery unavailable')
    ax.set_yticks([0,1,2],['ABSTAIN','RGB','thermal'],fontsize=8);ax.set_ylim(-.3,2.8)
    ax.set_title(arm+' · eligible '+str(g['eligible'])+'/95 · pair calls '+str(g['cost']['total_pair_calls']),fontsize=10,loc='left');ax.grid(axis='x',alpha=.2)
axes[0].legend(loc='upper right',ncol=2,fontsize=8)
axes[-1].set_xlabel('Target ending source frame (ordinary delivery deadline is the following group)')
axes[-1].set_xlim(2,98)
fig.suptitle('EXP120 · Synthetic input outages: RGB 20–34, thermal 50–64, both 75–79',fontsize=12)
fig.text(.5,.01,'Green/red marks judge the preselected camera at exact adjacent endpoints; gap/late/terminal results remain unavailable. Serial offline test.',ha='center',fontsize=8)
fig.tight_layout(rect=(0,.04,1,.95));out=ROOT/'docs/research/figures'
for ext in ['png','pdf']:fig.savefig(out/('availability-transitions-2026-11-03.'+ext),dpi=180)
print('Saved EXP120 action/delivery timeline')

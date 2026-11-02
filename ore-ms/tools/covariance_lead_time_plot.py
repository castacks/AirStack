"""Render archived lead-diagnostic statistics; no new scientific calculation."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[1];O=R/'results/covariance-lead-time/run-001'
d=json.loads((O/'diagnostic.json').read_text())
fig,axes=plt.subplots(1,3,figsize=(11.4,3.7),sharey=True)
for ax,window in zip(axes,['early','middle','late']):
    rows=[s for s in d['summaries'] if s['source']=='long_night' and s['group']==window+'-thermal']
    x=[s['horizon'] for s in rows]
    for key,label,color in [('covariance_persistence','Covariance → future covariance','#1967a3'),('covariance_future_disagreement','Covariance → future disagreement','#bd4b27')]:
        ax.plot(x,[s['associations'][key]['rho'] for s in rows],'o-',label=label,color=color)
    ax.axhline(0,color='gray',lw=.7);ax.set_title(window.capitalize()+' thermal');ax.set_xticks([0,.1,.5,1]);ax.set_ylim(-1,1)
    ax.set_xlabel('Future pair start lead (s)');ax.grid(alpha=.15)
axes[0].set_ylabel('Within-window Spearman correlation')
handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=2,bbox_to_anchor=(.5,.01),frameon=False)
fig.suptitle('Persistent covariance does not imply persistent error predictiveness',fontsize=12)
fig.text(.5,.15,'One exposed night recording; independent-algorithm disagreement is a proxy. Pair-end leads usually add 0.1 s.',ha='center',fontsize=8)
fig.tight_layout(rect=[0,.18,1,.92])
for ext in ['png','pdf']:fig.savefig(O/f'thermal-lead-associations.{ext}',dpi=180,bbox_inches='tight')

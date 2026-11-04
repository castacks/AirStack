"""Exposed dense native availability by whole recording; no pose quality labels."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from exposed_adequacy_study import ROOT,OUT,read
s=read(OUT/'inventory.json');names=list(s['recordings']);groups=[s['recordings'][n] for n in names]
labels=['MS2 day','MS2 rain','MS2 night','TD3 day0','TD3 day1','TD3 night fig8','TD3 night salon2']
x=np.arange(len(names));fig,ax=plt.subplots(figsize=(11,4.5),constrained_layout=True)
for off,field,label,color in [(-.18,'rgb_eligible','RGB','#397da3'),(.18,'thermal_eligible','Thermal','#b3824b')]:
 values=[100*g[field]/g['slots'] for g in groups];bars=ax.bar(x+off,values,.36,label=label,color=color)
 for bar,g in zip(bars,groups):ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+1,format(100*g[field]/g['slots'],'.1f').rstrip('0').rstrip('.'),ha='center',fontsize=8)
ax.set_xticks(x,[label+'\nN='+str(g['slots']) for label,g in zip(labels,groups)],rotation=20,ha='right');ax.set_ylabel('Native eligible coverage (%)');ax.set_ylim(0,112)
ax.set_title("In the audited windows, one fixed camera covers each recording's native union")
ax.legend(loc='lower left');ax.axhline(100,color='gray',ls=':',lw=1)
fig.text(.5,-.035,'18 inspected windows / 7 recordings; all per-recording and per-window headroom bounds are zero. Camera-local structural availability, not accuracy.',ha='center',fontsize=8)
for ext in ['png','pdf']:
 p=ROOT/'docs/research/figures'/('exposed-data-adequacy-2026-11-03.'+ext);assert not p.exists();fig.savefig(p,dpi=180,bbox_inches='tight')

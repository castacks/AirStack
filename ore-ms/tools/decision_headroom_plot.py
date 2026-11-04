"""Retrospective availability ceilings conditional on actual native histories."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from decision_headroom_study import ROOT,OUT,read
s=read(OUT/'accounting.json');fig,axes=plt.subplots(1,2,figsize=(12,4.5),constrained_layout=True)
for ax,condition,arms in [(axes[0],'day',['full_read_cache','bounded_probe','leased_probe']),
                         (axes[1],'night',['full_read_cache','bounded_probe','leased_probe','fixed_rgb','fixed_thermal'])]:
 g=[s['groups'][condition+'/'+a] for a in arms];x=np.arange(len(g));width=.25
 for offset,values,label,color in [(-width,[a['selected_eligible'] for a in g],'Original preselection','#397da3'),
    (0,[max(a['rgb_constant_readout'],a['thermal_constant_readout']) for a in g],'Best constant readout, same outputs','#929292'),
    (width,[a['conditional_union_ceiling'] for a in g],'Conditional union ceiling','#71a17b')]:
  bars=ax.bar(x+offset,values,width,label=label,color=color)
  for b,v in zip(bars,values):ax.text(b.get_x()+b.get_width()/2,v+1,str(v),ha='center',fontsize=8)
 labels={'full_read_cache':'Full read','bounded_probe':'Original sparse','leased_probe':'Candidate','fixed_rgb':'Native fixed RGB','fixed_thermal':'Native fixed thermal'}
 ax.set_xticks(x,[labels[a] for a in arms],rotation=25,ha='right');ax.set_ylim(0,101);ax.set_ylabel('Eligible intervals / 95')
 ax.set_title(condition.capitalize()+' retained execution histories')
axes[0].legend(loc='lower left',fontsize=8)
fig.suptitle('Availability headroom is conditional on scheduling and decision time',fontsize=13)
fig.text(.5,-.035,'Retrospective; no new policy, pose labels or inference. Conditional ceilings do not bound a different request schedule or camera accuracy.',ha='center',fontsize=8)
for ext in ['png','pdf']:
 p=ROOT/'docs/research/figures'/('decision-headroom-2026-11-03.'+ext);assert not p.exists();fig.savefig(p,dpi=180,bbox_inches='tight')

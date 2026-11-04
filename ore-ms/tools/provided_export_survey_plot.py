"""Fixed provided-export native support survey, distinct from failed MSA controls."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from provided_export_survey import ROOT,OUT,read
s=read(OUT/'native-survey.json');names=list(s['groups']);g=[s['groups'][n] for n in names];x=np.arange(len(names))
fig,ax=plt.subplots(figsize=(10,4.5),constrained_layout=True)
for offset,key,label,color in [(-.25,'rgb_eligible','RGB','#397da3'),(0,'thermal_eligible','Thermal','#b3824b'),(.25,'union_eligible','Conditional union','#71a17b')]:
 values=[r[key] for r in g];bars=ax.bar(x+offset,values,.25,label=label,color=color)
 for b,v in zip(bars,values):ax.text(b.get_x()+b.get_width()/2,v+1,str(v),ha='center',fontsize=9)
ax.set_xticks(x,[n.replace('night_thermal_so_','').replace('/',' / ') for n in names],rotation=20,ha='right')
ax.set(ylim=(0,105),ylabel='Native eligible intervals / 95',title='EXP129: provided-export native survey; no MSA regeneration')
ax.legend(loc='lower left');ax.grid(axis='y',alpha=.15)
fig.text(.5,-.035,'Four metadata-qualified windows in two development recordings. Other 11 thirds remain ineligible. No pose accuracy or adaptive-policy claim.',ha='center',fontsize=8)
for ext in ['png','pdf']:
 p=ROOT/'docs/research/figures'/('provided-export-survey-2026-11-04.'+ext);assert not p.exists();fig.savefig(p,dpi=180,bbox_inches='tight')

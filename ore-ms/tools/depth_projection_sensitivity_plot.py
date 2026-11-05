"""Render EXP148 frozen groups and conditional local-label sign ranges."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'results/depth-projection-sensitivity/run-001';s=json.loads((out/'summary.json').read_text());fig,axs=plt.subplots(2,2,figsize=(13,9))
groups=['all','insufficient','locally_consistent','mixed_depth'];labels=['All primary','<3 labels','Consistent','Mixed depth'];x=np.arange(4)
for col,mode in enumerate(['rgb','thermal']):
 for radius,color,offset in [(1,'#2676a7',-.12),(2,'#d47b20',.12)]:
  row=next(v for v in s['aggregates'] if v['modality']==mode and v['radius']==radius)
  vals=[row['groups'][g]['metrics']['depth_ratio']['median'] for g in groups];axs[0,col].scatter(x+offset,[v if v is not None else np.nan for v in vals],c=color,s=45,label=f'{2*radius+1}x{2*radius+1}')
  for i,g in enumerate(groups):
   data=row['groups'][g];axs[0,col].text(i+offset,.43 if radius==1 else .37,f"{data['points']}/{data['images_nonempty']}",ha='center',fontsize=8,color=color)
 axs[0,col].axhline(1,color='gray',ls='--',lw=1);axs[0,col].set_xticks(x,labels);axs[0,col].set_ylim(.3,1.12);axs[0,col].set_ylabel('Median per-image original depth ratio');axs[0,col].set_title(('RGB' if mode=='rgb' else 'Thermal')+' — unchanged primary errors by group');axs[0,col].legend(loc='upper right');axs[0,col].grid(alpha=.2)
 rows=[next(v for v in s['aggregates'] if v['modality']==mode and v['radius']==r)['groups']['all'] for r in [1,2]];bottom=np.zeros(2)
 for key,label,color in [('short_for_all','Shorter than all labels','#2676a7'),('contains_unit','Within label depth range','#999999'),('long_for_all','Longer than all labels','#d47b20')]:
  vals=np.array([100*r['interval_counts'][key]/r['points'] for r in rows]);axs[1,col].bar([0,1],vals,bottom=bottom,color=color,label=label,width=.5)
  for i,v in enumerate(vals):
   if v>=3:axs[1,col].text(i,bottom[i]+v/2,f'{v:.1f}%',ha='center',va='center',color='white',fontsize=10)
  bottom+=vals
 axs[1,col].set_xticks([0,1],['3x3 labels','5x5 labels']);axs[1,col].set_ylim(0,100);axs[1,col].set_ylabel('Original primary points (%)');axs[1,col].legend(fontsize=8,loc='upper left',bbox_to_anchor=(0,-.12))
fig.suptitle('Fixed local-reference sensitivity: same points, unchanged central-pixel errors',fontsize=15,y=.97)
fig.subplots_adjust(top=.9,bottom=.18,hspace=.4,wspace=.25)
fig.text(.5,.025,'Group annotations: points / nonempty images; no marker for empty cohorts.\nConsistent: at least 3 eligible labels and relative depth span <=10%; this does not certify visibility.\nIntervals enumerate observed local labels, not physical error bounds. No fitted correction or added points.',ha='center',fontsize=10)
f=ROOT/'docs/research/figures/depth-projection-sensitivity-2026-11-05'
for ext in ['png','pdf']:fig.savefig(str(f)+'.'+ext,dpi=150)
print(f)

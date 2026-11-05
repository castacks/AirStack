"""All six EXP143 outcomes, with quality compared on identical ORB fragments."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/orb-full-record/run-001'
s=json.loads((OUT/'summary.json').read_text());m=json.loads((OUT/'matched-summary.json').read_text())
orb={r['case']:r for r in s['runs']};droid={r['case']:r for r in s['droid_context']['runs']};matched={r['case']:r for r in m['runs']}
cases=[c+'-'+v for c in ['day','night','rain'] for v in ['rgb','thermal']]
labels=[c.replace('-','\n').replace('rgb','RGB').replace('thermal','Thermal').title().replace('Rgb','RGB') for c in cases]
fig,axes=plt.subplots(3,2,figsize=(12.6,10.5),layout='constrained')
fields=[('pose_coverage','Returned pose coverage','Percent of original source slots',False,100),
        ('rpe_coverage','Reference-qualified RPE coverage','Percent of eligible intervals',False,100),
        ('segment_count','Published map-continuity segments','Segments (not timestamp-gap counts)',False,1),
        ('ate_origin_rmse_m','Origin ATE on identical ORB output fragments','RMSE (m); fragment diagnostic',True,1),
        ('rpe_translation_rmse_m','Translation RPE on identical ORB intervals','RMSE (m); approximately 1 s',True,1),
        ('rpe_rotation_rmse_deg','Rotation RPE on identical ORB intervals','RMSE (degrees); approximately 1 s',True,1)]
x=np.arange(len(cases));width=.36
for ax,(key,title,unit,diagnostic,factor) in zip(axes.flat,fields):
    values=[]
    for offset,which,color,label in [(-width/2,'droid','#2475A8','DROID'),(width/2,'orb','#D87923','ORB-SLAM3')]:
        if diagnostic:
            vals=[matched[c]['droid_on_identical_orb_fragments' if which=='droid' else 'orb'][key] for c in cases]
        else:vals=[(droid if which=='droid' else orb)[c][key] for c in cases]
        vals=np.array([float(v)*factor if v is not None else np.nan for v in vals]);values.extend(vals[np.isfinite(vals)])
        bars=ax.bar(x+offset,vals,width,color=color,label=label)
        for position,value in zip(x+offset,vals):
            if np.isfinite(value):ax.annotate(f'{value:.3g}',(position,value),xytext=(0,3),textcoords='offset points',ha='center',fontsize=7)
            else:ax.annotate('n/a',(position,0),xytext=(0,3),textcoords='offset points',ha='center',fontsize=7)
    ax.set_ylim(0,max(values or [1])*1.25 if max(values or [1])>0 else 1)
    if factor==100:
        ax.set_ylim(0,110);ax.set_yticks(np.arange(0,101,20))
    ax.set_xticks(x,labels,fontsize=8);ax.set_title(title,fontsize=10);ax.set_ylabel(unit,fontsize=9)
    ax.spines[['top','right']].set_visible(False);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
axes[0,0].legend(frameon=False,fontsize=8)
fig.suptitle('Full-record ORB-SLAM3 and saved DROID: coverage, breaks and matched-support error',fontsize=13)
fig.supxlabel('Provided-camera development references; physical timing unverified. Fragment error is not a complete-trajectory win.\nBottom three panels mask DROID to ORB outputs/map fragments; native DROID outputs and reset behavior are unchanged.',fontsize=9)
dest=ROOT/'docs/research/figures/orb-full-record-2026-11-05';dest.parent.mkdir(exist_ok=True)
for suffix in ['.png','.pdf']:fig.savefig(dest.with_suffix(suffix),dpi=180)
print(dest)

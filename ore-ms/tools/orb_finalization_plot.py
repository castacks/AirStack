"""Render all frozen same-prefix output-stage scores on identical support."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
summary=json.loads((ROOT/'results/orb-finalization/run-001/summary.json').read_text())
fig,axes=plt.subplots(3,2,figsize=(10.8,8.2),layout='constrained')
arms=['received','final','droid'];labels=['Received\nORB','Finalized\nORB','Offline\nDROID']
fields=[('ate_se3_rmse_m','SE3-aligned ATE','RMSE (m)'),('rpe_translation_rmse_m','Approximately 1 s translation RPE','RMSE (m)'),('rpe_rotation_rmse_deg','Approximately 1 s rotation RPE','RMSE (degrees)')]
for col,mode in enumerate(['rgb','thermal']):
    run=next(r for r in summary['runs'] if r['modality']==mode)
    for row,(field,title,unit) in enumerate(fields):
        ax=axes[row,col];values=[run['results'][a][field] for a in arms]
        bars=ax.bar(range(3),values,color=['#7AAEC7','#D87923','#2475A8'],width=.65)
        ax.bar_label(bars,labels=[f'{v:.4f}' for v in values],padding=3,fontsize=8)
        ax.set_xticks(range(3),labels,fontsize=8);ax.set_ylabel(unit);ax.set_ylim(0,max(values)*1.22)
        ax.set_title((('RGB' if mode=='rgb' else 'Thermal')+' — '+title),fontsize=10)
        ax.spines[['top','right']].set_visible(False);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
fig.suptitle('ORB output-stage control: first 2,000 original night groups, both cameras',fontsize=13)
fig.supxlabel('Identical support: RGB 1,812 poses / 7 fragments; thermal 2,000 poses / 1 fragment.\nSame future-data horizon, no scale/time fit. Provided references; physical timing unverified.\nORB final is a post-worker native snapshot, not an additional global bundle adjustment.',fontsize=9)
dest=ROOT/'docs/research/figures/orb-finalization-2026-11-05';dest.parent.mkdir(exist_ok=True)
for suffix in ['.png','.pdf']:fig.savefig(dest.with_suffix(suffix),dpi=180)
print(dest)

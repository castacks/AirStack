"""Render the fixed EXP142 provided-reference scores without selecting cases."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
source=ROOT/'results/provided-reference-full-record/run-001/summary.json'
rows={r['case']:r for r in json.loads(source.read_text())['runs']}
conditions=['day','night','rain'];x=np.arange(3);width=.34
fig,axes=plt.subplots(1,3,figsize=(11.6,3.6),layout='constrained')
for ax,(metric,title,unit) in zip(axes,[('ate_se3_rmse_m','SE3-aligned trajectory ATE','RMSE (m)'),
                                       ('rpe_translation_rmse_m','Approximately 1 s translation RPE','RMSE (m)'),
                                       ('rpe_rotation_rmse_deg','Approximately 1 s rotation RPE','RMSE (degrees)')]):
    for offset,modality,color,label in [(-width/2,'rgb','#2475A8','RGB'),(width/2,'thermal','#D87923','Thermal')]:
        values=[rows[c+'-'+modality][metric] for c in conditions]
        bars=ax.bar(x+offset,values,width,color=color,label=label)
        ax.bar_label(bars,labels=[f'{v:.2f}' if v>=1 else f'{v:.3f}' for v in values],fontsize=8,padding=3)
    ax.set_xticks(x,[c.capitalize() for c in conditions]);ax.set_ylabel(unit);ax.set_title(title,fontsize=10)
    ax.spines[['top','right']].set_visible(False);ax.set_ylim(0,ax.get_ylim()[1]*1.16)
    ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
axes[0].legend(frameon=False,fontsize=8)
fig.suptitle('Saved full-record DROID outputs vs provided camera-local references',fontsize=13)
fig.supxlabel('Development data; own camera clocks; no scale fit. Physical timing/accuracy and adaptive benefit remain unverified.',fontsize=8)
dest=ROOT/'docs/research/figures/provided-reference-full-record-2026-11-05'
dest.parent.mkdir(parents=True,exist_ok=True)
for suffix in ['.png','.pdf']:fig.savefig(dest.with_suffix(suffix),dpi=180)
print(dest)

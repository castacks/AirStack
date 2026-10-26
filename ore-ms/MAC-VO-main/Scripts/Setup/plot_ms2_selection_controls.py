"""Median per-image selection controls; no pooling or cross-modal winner inference."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--previous',type=Path,required=True);a=p.parse_args()
    current=json.loads((a.run/'report.json').read_text())['aggregate'];prior=json.loads(a.previous.read_text())['aggregate']
    keys=[(w,m) for w in ('night-opening','day-opening','day-middle','day-late') for m in ('rgb','thr')]
    cur=[next(r for r in current if r['primary'] and (r['window'],r['modality'])==key) for key in keys]
    old=[next(r for r in prior if r['primary'] and (r['window'],r['modality'])==key) for key in keys]
    fig,axes=plt.subplots(2,1,figsize=(13,8),sharex=True);x=np.arange(8)
    axes[0].bar(x-.18,[r['ratio_min_median_max'][1] for r in old],.36,label='Unconstrained uncertainty / random')
    axes[0].bar(x+.18,[r['stratified_ratio']['min_median_max'][1] for r in cur],.36,label='Within-bin uncertainty / matched random')
    axes[0].set_ylabel('Median MSE ratio');axes[0].legend(fontsize=9);axes[0].set_title('Effect of preserving depth/location composition')
    for j,b in enumerate(('far_depth','near_depth','central','peripheral')):
        axes[1].bar(x+(j-1.5)*.19,[r['uncertainty_to_baseline'][b]['min_median_max'][1] for r in cur],.19,label=b.replace('_',' '))
    axes[1].set_ylabel('Median uncertainty / baseline MSE');axes[1].set_title('Unconstrained selection compared with each fixed baseline')
    axes[1].legend(fontsize=9,ncol=4)
    axes[1].set_xticks(x,[w.replace('opening','start').replace('day-','day\n').replace('night-','night\n')+'\n'+m.upper() for w,m in keys])
    for ax in axes:ax.axhline(1,color='black',linestyle='--',linewidth=1);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    fig.suptitle('50% retention: single-scan own support, median of20 image ratios\nProvided-reference diagnostics; lower is better; cameras have different supports',fontsize=12)
    fig.tight_layout()
    for ext in ('png','pdf'):
        out=a.run/('primary-selection-controls.'+ext);assert not out.exists();fig.savefig(out,dpi=170)

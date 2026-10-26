"""Compare fixed selection policies under squared and Euclidean residual loss."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--previous',type=Path,required=True);a=p.parse_args()
    data=json.loads((a.run/'report.json').read_text());old=json.loads(a.previous.read_text())
    keys=[(w,m) for w in ('night-opening','day-opening','day-middle','day-late') for m in ('rgb','thr')]
    new=[next(r for r in data['aggregate'] if r['primary'] and (r['window'],r['modality'])==k) for k in keys]
    previous=[next(r for r in old['aggregate'] if r['primary'] and (r['window'],r['modality'])==k) for k in keys]
    mse_global=[]
    for w,m in keys:
        rows=[r for r in old['rows'] if (r['window'],r['modality'],r['variant'],r['support'])==(w,m,'depth','own_support')]
        points=[next(p for p in r['curve'] if p['coverage']==.5)['risks'] for r in rows]
        mse_global.append(float(np.median([p['uncertainty']/p['random'] for p in points])))
    fig,axes=plt.subplots(2,1,figsize=(13,8),sharex=True);x=np.arange(8)
    series=[('Global / random: squared',mse_global),('Global / random: mean L2',[r['global_ratio']['min_median_max'][1] for r in new]),
            ('Within-bin / random: squared',[r['stratified_ratio']['min_median_max'][1] for r in previous]),
            ('Within-bin / random: mean L2',[r['stratified_ratio']['min_median_max'][1] for r in new])]
    for j,(label,values) in enumerate(series):axes[0].bar(x+(j-1.5)*.19,values,.19,label=label)
    axes[0].axhline(1,color='black',linestyle='--',linewidth=1);axes[0].set_ylabel('Median selected / random risk');axes[0].legend(fontsize=8,ncol=2)
    axes[0].set_title('Same selectors and 50% retained mass; loss function changes')
    tails=[next(t for t in data['tail_summary'] if (t['window'],t['modality'],t['variant'],t['support'],t['tail_fraction'])==(w,m,'depth','own_support',.01)) for w,m in keys]
    axes[1].bar(x-.18,[100*t['mse_contribution_min_median_max'][1] for t in tails],.36,label='Squared error')
    axes[1].bar(x+.18,[100*t['mean_l2_contribution_min_median_max'][1] for t in tails],.36,label='L2 magnitude')
    axes[1].set_ylabel('Median fraction of total error (%)');axes[1].legend(fontsize=9);axes[1].set_title('Contribution from worst 1% of pixels: descriptive, no pixels removed')
    axes[1].set_xticks(x,[w.replace('opening','start').replace('day-','day\n').replace('night-','night\n')+'\n'+m.upper() for w,m in keys])
    for ax in axes:ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    fig.suptitle('Single-scan own support: each median summarizes20 per-image values\nProvided-reference diagnostics; camera supports differ',fontsize=12);fig.tight_layout()
    for ext in ('png','pdf'):
        target=a.run/('primary-loss-tail-sensitivity.'+ext);assert not target.exists();fig.savefig(target,dpi=170)

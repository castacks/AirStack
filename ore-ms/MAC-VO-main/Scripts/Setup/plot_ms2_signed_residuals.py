"""Per-recording empirical accounting, no pooled risk."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def plot(run):
    report=json.loads((run/'report.json').read_text())['recordings'];names=list(report);x=np.arange(len(names))
    fig,axes=plt.subplots(2,1,figsize=(12,8),layout='constrained')
    for cam,color,shift in [('rgb','#225ea8',-.18),('thr','#d95f0e',.18)]:
        summaries=[v['summary']['midpoint'][cam] for v in report.values()]
        for key,hatch,label in [('joint_mean_fraction','//','range × grid mean component'),('global_mean_fraction',None,'global mean component')]:
            axes[0].bar(x+shift,[s[key]*100 for s in summaries],width=.32,color=color,alpha=.35 if hatch else 1,hatch=hatch,label=f'{cam.upper()}: {label}')
    axes[0].set_ylabel('Median per-frame MSE fraction (%)');axes[0].set_ylim(0,105);axes[0].legend(fontsize=8,ncol=2);axes[0].grid(axis='y',alpha=.2)
    for key,marker,label in [('mse_m2','o','Total squared error'),('centered_m2','s','After removing global mean residual')]:
        axes[1].plot(x,[v['summary']['midpoint'][f'median_rgb_over_thr_{key}'] for v in report.values()],marker+'-',label=label)
    axes[1].axhline(1,color='black',ls='--');axes[1].set_yscale('log');axes[1].set_ylabel('Median paired RGB / thermal ratio');axes[1].legend(fontsize=9);axes[1].grid(alpha=.2)
    labels=[n.replace('original_','Dev ').replace('exp063_','063 ').replace('exp064_','064 ') for n in names]
    for ax in axes:ax.set_xticks(x,labels)
    fig.suptitle('Signed residual accounting on seven examined MS² captures\nMidpoint supplied reference; 20 dependent frames per capture',fontsize=14)
    fig.supxlabel('Reference-assisted spatial means; no physical bias identification or deployable correction. Joint groups include sparse cells.',fontsize=10)
    output=run/'figures';output.mkdir();fig.savefig(output/'residual-accounting.png',dpi=180);fig.savefig(output/'residual-accounting.pdf');plt.close(fig)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);plot(p.parse_args().run)

"""Compare frozen rain/day baseline results, preserving separate recording summaries."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(base):
    paths=[base/'rain-day-comparison.png',base/'rain-day-comparison.pdf']
    if any(p.exists() for p in paths):raise FileExistsError('Plot already exists')
    data={name:json.loads((base/folder/'metric/report.json').read_text())['rows'] for name,folder in [('Rain','rain-control'),('Day','day')]}
    fig,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    for name,rows in data.items():
        color='#2878ad' if name=='Day' else '#b97026';x=np.arange(20)
        for ax,loss in [(axes[0,0],'mse'),(axes[0,1],'mean_l2')]:
            vals=[r['scores']['midpoint'][loss]['learned_propagated']['uncertainty']/r['scores']['midpoint'][loss]['camera_scale']['uncertainty'] for r in rows]
            ax.plot(x,vals,'o-',label=name,color=color);ax.axhline(1,color='black',linestyle='--',linewidth=.7)
        axes[1,0].plot(x,[r['scores']['midpoint']['mean_l2']['learned_propagated']['uncertainty_over_rgb'] for r in rows],'o-',label=name,color=color)
        axes[1,1].plot(x,[r['scores']['midpoint']['mean_l2']['learned_propagated']['lower_error_choice_fraction'] for r in rows],'o-',label=name+': learned selector',color=color)
        axes[1,1].plot(x,[r['scores']['midpoint']['mean_l2']['learned_propagated']['rgb_lower_error_fraction'] for r in rows],'.--',label=name+': always RGB',color=color)
    axes[0,0].set(title='Learned / per-image camera scale: squared 3D error',ylabel='Paired risk ratio')
    axes[0,1].set(title='Learned / per-image camera scale: mean 3D distance',ylabel='Paired risk ratio')
    axes[1,0].axhline(1,color='black',linestyle='--',linewidth=.7);axes[1,0].set(title='Always RGB remains a necessary counterexample',ylabel='Learned / always-RGB mean-distance risk',yscale='log')
    axes[1,1].set(title='Lower-error choice fraction, identical candidate support',ylabel='Fraction per frame',ylim=(0,1))
    for ax in axes.flat:
        ax.set(xlabel='Original ID within each separate recording',xticks=np.arange(0,20,2));ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Frozen MS² cross-modal replication: day benefit differs from rain\n20 adjacent frames per recording; candidate/shared-reference benchmark, not physical ground truth',fontsize=12)
    for p in paths:fig.savefig(p,dpi=170)
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);plot(p.parse_args().base)

"""Display frozen comparisons separately for three MS2 candidate-support windows."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot(root,output):
    paths=[output/'three-recording-controls-final.png',output/'three-recording-controls-final.pdf']
    if any(p.exists() for p in paths):raise FileExistsError('Figures already exist')
    data={name:json.loads((root/folder/'report.json').read_text())['rows'] for name,folder in [('Rain','results/ms2-day-cross-modal-replication/rain-control/metric'),('Day','results/ms2-day-cross-modal-replication/day/metric'),('Night','results/ms2-night-cross-modal-replication/night/metric')]}
    fig,axes=plt.subplots(2,3,figsize=(15,8),constrained_layout=True)
    for row,(loss,description) in enumerate([('mean_l2','Mean distance'),('mse','Squared error')]):
        for col,(comparator,label) in enumerate([('camera_scale','camera scale'),('rgb','always RGB'),('random','random')]):
            ax=axes[row,col]
            for name,color in [('Rain','#b97026'),('Day','#2878ad'),('Night','#7c4ca4')]:
                values=[]
                for record in data[name]:
                    scores=record['scores']['midpoint'][loss];a=scores['learned_propagated'];den=a[comparator] if comparator in ['rgb','random'] else scores[comparator]['uncertainty'];values.append(a['uncertainty']/den)
                ax.plot(np.arange(20),values,'o-',label=name,color=color,markersize=4)
            ax.axhline(1,color='black',linestyle='--',linewidth=.8)
            ax.set(title=f'{description}: learned / {label}',xlabel='Original source ID within each separate recording',ylabel='Per-image paired risk ratio',xticks=np.arange(0,20,4))
            if col:ax.set_yscale('log')
            ax.legend(fontsize=8);ax.grid(alpha=.2)
    fig.suptitle('Frozen MS² camera-selection controls: night selector is worse than RGB and random\nThree distinct 20-frame windows; no pooled-pixel inference or independent physical-truth claim',fontsize=12)
    for path in paths:fig.savefig(path,dpi=170)
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();plot(a.root,a.output)

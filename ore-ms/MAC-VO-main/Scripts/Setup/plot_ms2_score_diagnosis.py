"""Plot fixed score interventions, keeping reference-assisted controls explicit."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot(source,output):
    paths=[output/'score-diagnosis-final.png',output/'score-diagnosis-final.pdf']
    if any(p.exists() for p in paths):raise FileExistsError('Figure already exists')
    rows=json.loads((source/'report.json').read_text())['rows']
    policies=['original','median_normalized','reference_depth','reference_depth_median_normalized'];labels=['Pred','Pred / med','Ref*','Ref* / med'];colors=['#2878ad','#d18326','#37845b','#8050a3']
    fig,axes=plt.subplots(2,3,figsize=(13,8),constrained_layout=True)
    for col,window in enumerate(['rain','day','night']):
        group=[r for r in rows if r['window']==window]
        for row,(key,title) in enumerate([('uncertainty_over_rgb','Always RGB'),('uncertainty_over_random','Equal random expectation')]):
            ax=axes[row,col];medians=[]
            for x,(name,color) in enumerate(zip(policies,colors)):
                vals=[r['scores']['midpoint']['mean_l2'][name][key] for r in group];medians.append(np.median(vals))
                assert all(.01 < value < 2.2 for value in vals), 'Plot would clip observations'
                ax.scatter(x+np.linspace(-.08,.08,len(vals)),vals,s=16,color=color,alpha=.7)
                ax.plot([x-.17,x+.17],[np.median(vals)]*2,color='black',linewidth=2)
            ax.axhline(1,color='black',linestyle='--',linewidth=.8)
            ax.set(title=f'{window.title()}: selector / {title}',ylabel='Per-image mean-distance risk ratio',xticks=range(4),xticklabels=labels,ylim=(.01,2.2),yscale='log');ax.grid(alpha=.2)
    fig.suptitle('Score diagnosis on fixed MS² candidate support: neither intervention gives a universal repair\nPred: predicted-depth Jacobian; med: per-camera median normalization; Ref*: supplied-reference-depth Jacobian\nRef* controls use labels and are not deployable; dots are 20 adjacent-frame ratios, bars are medians',fontsize=12)
    for p in paths:fig.savefig(p,dpi=170)
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();plot(a.source,a.output)

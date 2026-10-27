"""Descriptive per-image risks for all fixed model families and excluded recordings."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(source,output):
    if output.exists():raise FileExistsError(output)
    folds=json.loads((source/'report.json').read_text())['folds']
    names=['constant','uncertainty','geometry','combined']
    fig,axes=plt.subplots(2,3,figsize=(14,8),layout='constrained')
    for col,(rec,f) in enumerate(folds.items()):
        for row,loss in enumerate(['mean_l2','mse']):
            ax=axes[row,col];rows=f['rows']
            values=[[r['risks']['midpoint'][loss][name]/r['risks']['midpoint'][loss]['random'] for r in rows] for name in names]
            boxes=ax.boxplot(values,patch_artist=True,showfliers=False,widths=.55,tick_labels=['Constant','U only','Geometry','Combined'])
            for patch,color in zip(boxes['boxes'],['#bdbdbd','#9ecae1','#a1d99b','#fdae6b'],strict=True):patch.set_facecolor(color)
            for i,v in enumerate(values,1):ax.scatter(np.linspace(i-.12,i+.12,len(v)),v,s=9,color='#333333',alpha=.5,zorder=3)
            ax.axhline(1,color='black',lw=1,ls='--',label='Equal random')
            for policy,color,label in [('always_rgb','#225ea8','Always RGB median'),('always_thr','#d95f0e','Always thermal median')]:
                val=np.median([r['risks']['midpoint'][loss][policy]/r['risks']['midpoint'][loss]['random'] for r in rows]);ax.axhline(val,color=color,lw=1.3,ls=':',label=label)
            ax.set_yscale('log');ax.grid(axis='y',alpha=.2)
            ax.set_title(f'Excluded {rec}: '+('mean 3D distance' if loss=='mean_l2' else 'squared 3D error'))
            ax.set_ylabel('Per-image model risk / equal-random risk')
            if col==0 and row==0:ax.legend(fontsize=8,loc='lower left')
    fig.suptitle('Recording-excluded camera selection with prediction-based geometry\nAll four families, fixed ridge; previously inspected windows, not untouched validation',fontsize=14)
    fig.supxlabel('20 dependent per-image observations per box; supplied-reference support; midpoint anchor shown',fontsize=10)
    output.mkdir(parents=True);fig.savefig(output/'geometry-risk-selection.png',dpi=180);fig.savefig(output/'geometry-risk-selection.pdf');plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();plot(a.source,a.output)

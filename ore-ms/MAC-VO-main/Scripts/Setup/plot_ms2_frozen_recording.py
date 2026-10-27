"""Standalone descriptive frozen-recording result, including primary incremental test."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(base,output):
    if output.exists():raise FileExistsError(output)
    r=json.loads((base/'frozen-evaluation/report.json').read_text());rows=[x for x in r['rows'] if x['availability']['both_valid']]
    assert rows,'No available conditional risks to plot'
    names=['constant','uncertainty','geometry','combined'];fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    for col,loss in enumerate(['mean_l2','mse']):
        ax=axes[0,col]
        values=[[x['risks']['midpoint'][loss][m]/x['risks']['midpoint'][loss]['always_rgb'] for x in rows] for m in names]
        bp=ax.boxplot(values,patch_artist=True,showfliers=False,tick_labels=['Constant','U only','Geometry','Combined'])
        for patch,color in zip(bp['boxes'],['#bdbdbd','#9ecae1','#a1d99b','#fdae6b'],strict=True):patch.set_facecolor(color)
        for i,v in enumerate(values,1):ax.scatter(np.linspace(i-.12,i+.12,len(v)),v,s=10,c='#333333',alpha=.5)
        ax.axhline(1,color='black',ls='--',label='Always RGB')
        t=np.median([x['risks']['midpoint'][loss]['always_thr']/x['risks']['midpoint'][loss]['always_rgb'] for x in rows]);ax.axhline(t,color='#d95f0e',ls=':',label='Always thermal median')
        ax.set_yscale('log');ax.set_ylabel('Per-image risk / always-RGB risk');ax.set_title('Mean 3D distance' if loss=='mean_l2' else 'Squared 3D error');ax.grid(axis='y',alpha=.2);ax.legend(fontsize=8)
        ax=axes[1,col];v=[x['risks']['midpoint'][loss]['combined']/x['risks']['midpoint'][loss]['uncertainty'] for x in rows]
        ax.plot([int(x['stem']) for x in rows],v,'o-',color='#d95f0e',ms=4);ax.axhline(1,color='black',ls='--')
        ax.set_xlabel('Original source ID');ax.set_ylabel('Combined / uncertainty-only risk');ax.set_title('Primary incremental-geometry comparison\nBelow 1 favors combined',fontsize=11);ax.set_yscale('log');ax.set_xticks([0,4,9,14,19],['000000','000004','000009','000014','000019']);ax.grid(alpha=.2)
    fig.suptitle('Frozen models on previously unexamined MS² '+r['sequence']+'\nNo refitting; original 20 supplied pairs, conditional reference-supported benchmark',fontsize=13)
    fig.supxlabel('Midpoint anchor; repeated per-image observations, not independent statistical samples',fontsize=10)
    output.mkdir(parents=True);fig.savefig(output/'frozen-recording.png',dpi=180);fig.savefig(output/'frozen-recording.pdf');plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();plot(a.base,a.output)

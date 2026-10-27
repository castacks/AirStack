"""Export fixed range/tail summaries; error ratios are conditional reference diagnostics."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(source,output):
    if output.exists():raise FileExistsError(output)
    r=json.loads((source/'report.json').read_text())['recordings']
    fig,axes=plt.subplots(2,3,figsize=(14,7),layout='constrained')
    for col,(rec,x) in enumerate(r.items()):
        s=x['summary']['midpoint'];ax=axes[0,col]
        for mod,color,label in [('rgb','#225ea8','RGB'),('thr','#d95f0e','Thermal')]:
            values=[s['groups'][f'range_{i}']['median_stats'][mod]['mean_l2'] if s['groups'][f'range_{i}']['median_stats'] else np.nan for i in range(6)]
            ax.plot(range(6),values,'o-',color=color,label=label)
        ax.set_yscale('log');ax.set_xticks(range(6),['<5','5–10','10–20','20–40','40–80','≥80'])
        ax.set_title(rec.title());ax.set_xlabel('Reference midpoint range (m)');ax.set_ylabel('Median per-frame mean 3D error (m)')
        ax.grid(alpha=.2);ax.legend()
        text='Median n: '+', '.join(str(int(s['groups'][f'range_{i}']['counts_min_median_max'][1])) for i in range(6))
        ax.text(.01,.98,text,transform=ax.transAxes,va='top',fontsize=8)
        ax=axes[1,col]
        for loss,color in [('mean_l2','#238b45'),('mse','#756bb1')]:
            values=[s['groups']['all']['median_stats'][f'rgb_over_thr_{loss}']]+[s['tails'][f][loss]['median_rgb_over_thr'] for f in ['0.01','0.05']]
            ax.plot(range(3),values,'o-',label='Mean distance' if loss=='mean_l2' else 'Squared error',color=color)
        ax.axhline(1,color='black',ls='--',lw=1);ax.set_yscale('log')
        ax.set_xticks(range(3),['Full support','Remove top 1%','Remove top 5%'])
        ax.set_ylabel('Median per-frame RGB / thermal risk');ax.grid(alpha=.2);ax.legend(fontsize=9)
    fig.suptitle('Fixed RGB–thermal candidate support: range and error-tail structure\nPreviously inspected 20-frame windows; supplied-reference errors, not physical ground truth',fontsize=13)
    fig.supxlabel('Tail removal uses shared midpoint max-camera error: reference-assisted diagnostic only',fontsize=10)
    output.mkdir(parents=True)
    fig.savefig(output/'error-structure.png',dpi=180);fig.savefig(output/'error-structure.pdf');plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();plot(a.source,a.output)

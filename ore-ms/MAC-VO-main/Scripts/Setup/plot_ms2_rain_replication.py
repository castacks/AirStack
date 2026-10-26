"""Descriptive rain replication against frozen per-window selection summaries."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--base',type=Path,required=True);a=p.parse_args()
    old_mse=json.loads((a.root/'results/ms2-selection-controls/run-001/report.json').read_text())
    old_l2=json.loads((a.root/'results/ms2-loss-tail-sensitivity/run-001/report.json').read_text())
    rain_mse=json.loads((a.base/'mse-run-001/report.json').read_text());rain_l2=json.loads((a.base/'mean-l2-run-001/report.json').read_text())
    labels=['night-opening','day-opening','day-middle','day-late','rain-opening'];x=np.arange(5)
    fig,axes=plt.subplots(1,2,figsize=(13,4.8),sharey=True)
    for ax,m in zip(axes,('rgb','thr')):
        for offset,old,new,label in [(-.18,old_mse,rain_mse,'Squared residual'),(.18,old_l2,rain_l2,'L2 magnitude')]:
            data=old['aggregate']+new['aggregate']
            rows=[next(r for r in data if r['primary'] and r['window']==w and r['modality']==m) for w in labels]
            values=[r['stratified_ratio']['min_median_max'][1] for r in rows]
            bars=ax.bar(x+offset,values,.36,label=label)
            for b,r in zip(bars,rows):ax.annotate(str(r['stratified_ratio']['images_below_one'])+'/20',(b.get_x()+b.get_width()/2,b.get_height()),ha='center',va='bottom',fontsize=8)
        ax.axhline(1,color='black',linestyle='--',linewidth=1);ax.set_title(m.upper());ax.set_xticks(x,[w.replace('-','\n') for w in labels]);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True);ax.set_ylim(0,1.12)
    axes[0].set_ylabel('Median within-bin uncertainty / matched random risk');axes[0].legend(fontsize=9)
    fig.suptitle('50% retention: single-scan own support; labels show images below random\nDistinct rain recording, same frozen method; different supports, no camera-winner test',fontsize=11);fig.tight_layout()
    for ext in ('png','pdf'):
        target=a.base/('selection-comparison.'+ext);assert not target.exists();fig.savefig(target,dpi=170)

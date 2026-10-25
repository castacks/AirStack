"""Plot descriptive median risk ratios; each median summarizes twenty images."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
    data=json.loads((a.run/'report.json').read_text())['aggregate']
    fig,axes=plt.subplots(2,4,figsize=(13,6),sharex=True)
    for j,w in enumerate(('night-opening','day-opening','day-middle','day-late')):
        for i,s in enumerate(('own_support','common_support')):
            ax=axes[i,j]
            for m in ('rgb','thr'):
                g=sorted([r for r in data if r['window']==w and r['support']==s and r['modality']==m and r['variant']=='depth'],key=lambda r:r['coverage'])
                ax.plot([r['coverage']*100 for r in g],[r['ratio_min_median_max'][1] for r in g],marker='o',label=m.upper())
            ax.axhline(1,color='gray',linestyle='--',label='Random expectation')
            ax.set_title(w+' / '+s.replace('_support',''));ax.set_xticks([25,50,75,100]);ax.grid(alpha=.2)
            if i==1:ax.set_xlabel('Retained coverage (%)')
            if j==0:ax.set_ylabel('Median per-image MSE ratio')
    axes[0,0].legend(fontsize=8)
    fig.suptitle('Single-scan provided-reference risk: retain lowest covariance trace\nSeparate camera supports; no cross-modal winner or physical calibration claim',fontsize=11)
    fig.tight_layout()
    for ext in ('png','pdf'):
        target=a.run/('primary-risk-curves.'+ext);assert not target.exists();fig.savefig(target,dpi=170)

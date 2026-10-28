"""Coverage curves and fixed reference-range sensitivities, no pooled samples."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def plot(run):
    protocol=json.loads((run/'protocol.json').read_text());recs=json.loads((run/'report.json').read_text())['recordings'];fig,axes=plt.subplots(2,4,figsize=(17,8),layout='constrained')
    widths=protocol['widths_standard_deviations'];nominal=[protocol['hypothetical_zero_mean_gaussian_coverage'][f'{k:g}'] for k in widths]
    for j,(name,rec) in enumerate(recs.items()):
        for cam,color in [('rgb','#225ea8'),('thr','#d95f0e')]:
            stats=rec['summary'][cam]['all']['stats'];axes[0,j].plot(widths,[100*stats[f'coverage_{k:g}']['median'] for k in widths],'o-',ms=4,color=color,label=cam.upper())
            summaries=[rec['summary'][cam][f'range_{i}'] for i in range(6)];values=[100*s['stats']['coverage_1.96']['median'] if s['nonempty_frames'] else np.nan for s in summaries]
            axes[1,j].plot(range(6),values,'o-',ms=4,color=color,label=cam.upper())
            for i,s in enumerate(summaries):
                if 0<s['nonempty_frames']<20:
                    axes[1,j].plot(i,values[i],'x',ms=10,color=color)
                    axes[1,j].annotate(f"{s['nonempty_frames']}f",(i,values[i]),xytext=(3,4 if cam=='rgb' else -12),textcoords='offset points',fontsize=7,color=color)
        axes[0,j].plot(widths,np.array(nominal)*100,'k--',lw=1,label='Zero-mean Gaussian benchmark');axes[0,j].set_title(name);axes[0,j].set_xlabel('Interval half-width / predicted σ');axes[0,j].set_xticks([1,2,3]);axes[0,j].set_ylabel('Median per-frame empirical coverage (%)')
        axes[1,j].axhline(100*protocol['hypothetical_zero_mean_gaussian_coverage']['1.96'],color='black',ls='--',lw=1);axes[1,j].set_xticks(range(6),['0–5','5–10','10–20','20–40','40–80','≥80'],rotation=35);axes[1,j].set_xlabel('Midpoint reference range (m)');axes[1,j].set_ylabel('Median coverage at ±1.96σ (%)')
        for ax in axes[:,j]:ax.set_ylim(0,103);ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=8);fig.suptitle('Current uncertainty scale does not give uniform disparity coverage\nOwn supplied native reference, unchanged both-valid support; 20 dependent frames per recording',fontsize=14)
    fig.supxlabel('No recalibration. Gaussian is a hypothetical benchmark. × marks <20 nonempty frames (f); empty groups missing, sparse counts in report.',fontsize=10)
    out=run/'figures';out.mkdir();fig.savefig(out/'disparity-coverage.png',dpi=180);fig.savefig(out/'disparity-coverage.pdf');plt.close(fig)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);plot(p.parse_args().run)

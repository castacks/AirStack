"""Exploratory assumed-family sensitivity, preserving all dependent frames."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def plot(run):
    recs=json.loads((run/'report.json').read_text())['recordings'];fig,axes=plt.subplots(2,4,figsize=(17,8),layout='constrained')
    for j,(name,rec) in enumerate(recs.items()):
        rows=rec['rows']
        for method,color in [('lognormal','#225ea8'),('gamma','#d95f0e')]:
            ratios=[r['risks']['midpoint']['mse'][method]/r['risks']['midpoint']['mse']['first_order'] for r in rows]
            axes[0,j].plot(range(20),ratios,'o-',ms=3,color=color,label=method)
        axes[0,j].axhline(1,color='black',ls='--');axes[0,j].set_yscale('log');axes[0,j].set_title(name);axes[0,j].set_ylabel('Assumed-family / first-order actual MSE');axes[0,j].grid(alpha=.2)
        for cam,color in [('rgb','#225ea8'),('thr','#d95f0e')]:axes[1,j].plot(range(20),[100*r['assumptions'][cam]['gamma_infinite_fraction'] for r in rows],'o-',ms=3,color=color,label=cam.upper()+' infinite')
        axes[1,j].plot(range(20),[100*r['gamma_both_infinite_fraction'] for r in rows],'s--',ms=3,color='#756bb1',label='Both infinite → mixture')
        axes[1,j].set_ylabel('Gamma assumed-infinite risk points (%)');axes[1,j].set_ylim(bottom=0);axes[1,j].grid(alpha=.2)
        for ax in axes[:,j]:ax.set_xlabel('Original source ID');ax.set_xticks([0,4,9,14,19],['000000','000004','000009','000014','000019'])
    axes[0,0].legend(fontsize=9);axes[1,0].legend(fontsize=8)
    fig.suptitle('Mean/variance matching does not determine inverse-depth risk\nAlready examined captures; unchanged estimates, no fit; midpoint supplied-reference squared error',fontsize=15)
    fig.supxlabel('Assumed distributions are unvalidated. Gamma infinities are mathematical assumptions, not observed infinite errors. All frames retained.',fontsize=10)
    out=run/'figures';out.mkdir();fig.savefig(out/'distribution-sensitivity.png',dpi=180);fig.savefig(out/'distribution-sensitivity.pdf');plt.close(fig)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);plot(p.parse_args().run)

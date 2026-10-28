"""All three prospective recordings; primary and fixed-camera/scalar controls."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot(suite,output):
    if output.exists():raise FileExistsError(output)
    protocol=json.loads((suite/'suite-protocol.json').read_text());fig,axes=plt.subplots(2,3,figsize=(15,8),layout='constrained')
    for col,(condition,declared) in enumerate(protocol['records'].items()):
        report=json.loads((suite/condition/'frozen-evaluation/report.json').read_text());rows=[r for r in report['rows'] if r['availability']['both_valid']]
        for i,loss in enumerate(['mean_l2','mse']):
            ax=axes[i,col]
            for comparator,color,label in [('always_rgb','#225ea8','Always RGB'),('always_thr','#d95f0e','Always thermal'),('scalar_fit','#756bb1','Frozen scalar-fit')]:
                ratios=[r['risks']['midpoint'][loss]['uncertainty']/r['risks']['midpoint'][loss][comparator] for r in rows];ax.plot([int(r['stem']) for r in rows],ratios,'o-',ms=3,lw=1,color=color,label=label)
            ax.axhline(1,color='black',ls='--',lw=1);ax.set_yscale('log');ax.grid(alpha=.2);ax.set_xticks([0,4,9,14,19],['000000','000004','000009','000014','000019']);ax.set_xlabel('Original source ID');ax.set_ylabel('U-only / comparator risk')
            ax.set_title(condition.title()+': '+('mean 3D distance' if loss=='mean_l2' else 'squared 3D error')+'\n'+declared['sequence'],fontsize=10)
            if i==0 and col==0:ax.legend(fontsize=8)
    fig.suptitle('Prospective frozen uncertainty-only camera selection across three captures\nCoefficients unchanged from EXP063; midpoint supplied-reference risks; below1 favors U-only',fontsize=14)
    fig.supxlabel('20 dependent frames per recording; different cameras share reference-selected candidate support',fontsize=10)
    output.mkdir(parents=True);fig.savefig(output/'frozen-suite.png',dpi=180);fig.savefig(output/'frozen-suite.pdf');plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--suite',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();plot(a.suite,a.output)

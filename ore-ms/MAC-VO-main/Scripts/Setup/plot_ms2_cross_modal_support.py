"""Plot fixed support and conditional common-target findings, no pooled inference."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(base):
    outputs=[base/'cross-modal-summary-with-scale.png',base/'cross-modal-summary-with-scale.pdf']
    if any(p.exists() for p in outputs):raise FileExistsError('Plot outputs already exist')
    s=json.loads((base/'run-001/report.json').read_text())['rows']
    m=json.loads((base/'metric-run-001/report.json').read_text())['rows']
    c=json.loads((base/'geometry-control-run-001/report.json').read_text())['rows']
    scale=json.loads((base/'scale-control-run-001/report.json').read_text())['rows']
    x=np.arange(20)
    fig,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    axes[0,0].plot(x,[r['stages']['rgb_nonzero'] for r in s],'.-',label='All RGB depth support',color='#777777')
    axes[0,0].plot(x,[r['stages']['mutual_nearest'] for r in s],'o-',label='Mutual/frontmost candidates',color='#2878ad')
    axes[0,0].set(ylabel='Count per frame',title='Support retained without a depth-agreement filter')
    axes[0,0].legend(fontsize=8)
    axes[0,1].plot(x,[r['point_distance_m_quartiles'][2] for r in s],'o-',color='#ae5a26')
    axes[0,1].set(ylabel='Median 3D reference-point separation (m)',yscale='log',title='Reference mismatch remains visible')
    for key,label,color in [('uncertainty_over_rgb','Always RGB','#999999'),('uncertainty_over_thr','Always thermal','#d88819'),('uncertainty_over_random','Equal random expectation','#8d61ae')]:
        axes[1,0].plot(x,[r['anchors']['midpoint']['mean_l2'][key] for r in m],'.-',label=label,color=color)
    axes[1,0].plot(x,[r['scores']['midpoint']['mean_l2']['learned_over_geometry_angular'] for r in c],'o-',label='Geometry-only angular control',color='#2878ad')
    axes[1,0].plot(x,[r['scores']['midpoint']['mean_l2']['learned_over_constant_scale'] for r in scale],'s-',label='Image-wide camera-scale control',color='#29834a')
    axes[1,0].axhline(1,color='black',linestyle='--',linewidth=1)
    axes[1,0].set(ylabel='Learned selector / comparator mean-L2 risk',yscale='log',title='Below 1 favors learned propagated uncertainty')
    axes[1,0].legend(fontsize=8)
    axes[1,1].plot(x,[r['anchors']['midpoint']['mean_l2']['lower_error_choice_fraction'] for r in m],'o-',label='Lower-error choice fraction',color='#2878ad')
    axes[1,1].plot(x,[r['choose_rgb_fraction'] for r in m],'.-',label='Fraction selecting RGB',color='#ae5a26')
    axes[1,1].axhline(.5,color='black',linestyle='--',linewidth=1)
    axes[1,1].set(ylabel='Fraction of both-valid candidates',ylim=(0,1),title='Risk reduction is not perfect winner prediction')
    axes[1,1].legend(fontsize=8)
    for ax in axes.flat:
        ax.set(xlabel='Original rain source ID (000000–000019)',xticks=np.arange(0,20,2));ax.grid(alpha=.2)
    fig.suptitle('MS² rain: fixed candidate support and conditional metric comparison\n20 adjacent frames; shared supplied references; no independent physical-truth claim',fontsize=13)
    for p in outputs:fig.savefig(p,dpi=170)
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);plot(p.parse_args().base)

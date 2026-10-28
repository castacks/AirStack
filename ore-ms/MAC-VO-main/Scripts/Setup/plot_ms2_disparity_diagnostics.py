"""Exportable domain-ranking and finite inverse-depth diagnostics."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def plot(run):
    recs=json.loads((run/'report.json').read_text())['recordings'];out=run/'figures';out.mkdir()
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    keys=[('spearman_variance_disp_sq','Raw variance / disparity error²','#225ea8'),('spearman_variance_depth_sq','Raw variance / axial-depth error²','#41b6c4'),('spearman_propagated_point_sq','Propagated variance / 3D error²','#d95f0e')]
    for ax,(name,rec) in zip(axes.flat,recs.items()):
        for i,(key,label,color) in enumerate(keys):
            ax.bar(np.arange(2)+(i-1)*.22,[rec['summary'][c]['all']['stats'][key]['median'] for c in ['rgb','thr']],width=.22,color=color,label=label)
        ax.set_xticks([0,1],['RGB','Thermal']);ax.set_ylim(-1,1);ax.axhline(0,color='black',lw=.7);ax.grid(axis='y',alpha=.2);ax.set_title(name+'\n'+rec['sequence'],fontsize=10);ax.set_ylabel('Median per-frame Spearman ρ')
    handles,labels=axes.flat[0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside lower center',ncol=3,fontsize=9)
    fig.suptitle('Within-camera ranking changes with the error domain\nOwn supplied endpoint; 20 dependent frames per recording; unstratified',fontsize=14)
    fig.savefig(out/'domain-ranking.png',dpi=180);fig.savefig(out/'domain-ranking.pdf');plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13,5),layout='constrained');rec=recs['exp064_day'];ax=axes[0]
    for key,label in [('point_fraction','Point fraction'),('disparity_sq_fraction','Disparity squared-error fraction'),('depth_sq_fraction','Axial-depth squared-error fraction')]:
        ax.plot(range(20),[100*r['cameras']['rgb']['groups']['all']['bins']['0'][key] for r in rec['rows']],'o-',ms=3,label=label)
    ax.set_title('EXP064 day RGB: predicted disparity < 0.1 × reference');ax.set_ylim(0,105);ax.set_ylabel('Percent');ax.set_xlabel('Original source ID');ax.legend(fontsize=8);ax.grid(alpha=.2)
    for cam,color,offset in [('rgb','#225ea8',-.1),('thr','#d95f0e',.1)]:
        vals=[]
        for rec in recs.values():
            vals.append(np.median([r['cameras'][cam]['groups']['all']['stats']['mse_depth_m2']/r['cameras'][cam]['groups']['all']['stats']['mse_linear_depth_m2'] for r in rec['rows']]))
        axes[1].plot(np.arange(4)+offset,vals,'o',color=color,label=cam.upper())
    axes[1].set_xticks(range(4),['063 rain','064 day','064 night','064 rainy']);axes[1].axhline(1,color='black',ls='--');axes[1].set_yscale('log');axes[1].set_ylabel('Median paired exact / reference-linear depth MSE');axes[1].set_title('Finite-error depth conversion differs from linearization');axes[1].legend();axes[1].grid(alpha=.2)
    fig.suptitle('Inverse-disparity accounting, with all finite positive disparities retained',fontsize=14)
    fig.supxlabel('Reference-assisted finite-error identity; no physical accuracy, probability coverage or deployable correction claim.',fontsize=10)
    fig.savefig(out/'inverse-depth.png',dpi=180);fig.savefig(out/'inverse-depth.pdf');plt.close(fig)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);plot(p.parse_args().run)

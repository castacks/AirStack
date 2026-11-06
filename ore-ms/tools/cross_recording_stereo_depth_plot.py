"""EXP151 standalone per-image conditional depth and patch-gap diagnostics."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/cross-recording-stereo-depth/run-001'
def main():
 s=json.loads((OUT/'summary.json').read_text());fig,axes=plt.subplots(2,2,figsize=(12,7),layout='constrained');colors=dict(day='#d55e00',night='#2878a5')
 for col,mode in enumerate(['rgb','thermal']):
  for rec in ['day','night']:
   for group,style in [('all_primary','-'),('locally_consistent','--')]:
    d=[r for r in s['depth']['cases'] if r['recording']==rec and r['modality']==mode];a=[r for r in s['ambiguity']['cases'] if r['recording']==rec and r['modality']==mode];label=rec+(' / all' if group=='all_primary' else ' / consistent')
    axes[0,col].plot([r['index'] for r in d],[r['groups'][group]['metrics']['depth_ratio']['median'] if r['groups'][group]['metrics']['depth_ratio']['median'] is not None else float('nan') for r in d],style,marker='.',lw=1,color=colors[rec],label=label)
    axes[1,col].plot([r['index'] for r in a],[r['groups'][group]['correlations']['patch_relative_gap']['rho'] if r['groups'][group]['correlations']['patch_relative_gap']['rho'] is not None else float('nan') for r in a],style,marker='.',lw=1,color=colors[rec],label=label)
  axes[0,col].axhline(1,c='black',ls=':',lw=.8);axes[0,col].set(title=mode+' / native vs projected depth',ylabel='Per-image median native / reference depth',xlabel='Fixed source index');axes[0,col].legend(fontsize=8)
  axes[1,col].axhline(0,c='black',ls=':',lw=.8);axes[1,col].set(title=mode+' / patch gap vs proxy error',ylabel='Within-image Spearman rho',xlabel='Fixed source index',ylim=(-1,1));axes[1,col].legend(fontsize=8)
 fig.suptitle('EXP151: initialized stereo, exposed day/night recordings\nConditional sparse projection; missing/constant/small cohorts left blank; no physical or causal qualification')
 d=OUT/'figures';d.mkdir(exist_ok=True);fig.savefig(d/'cross-recording-depth.png',dpi=160);plt.close(fig)
if __name__=='__main__':main()

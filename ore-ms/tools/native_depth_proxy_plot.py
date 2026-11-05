"""Render saved EXP147 per-image conditional depth-proxy comparisons."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];s=json.loads((ROOT/'results/native-depth-proxy/run-001/summary.json').read_text());fig,axs=plt.subplots(3,2,figsize=(13,10))
for col,mode in enumerate(['rgb','thermal']):
 rows=[r for r in s['cases'] if r['modality']==mode];x=[r['index'] for r in rows];color='#2676a7' if mode=='rgb' else '#d47b20'
 axs[0,col].plot(x,[r['metrics']['depth_ratio']['median'] for r in rows],'o-',color=color,lw=1.1,ms=4);axs[0,col].axhline(1,color='gray',ls='--');axs[0,col].set_title(('RGB' if mode=='rgb' else 'Thermal')+' — fixed 20-image sample');axs[0,col].set_ylabel('Median depth ratio\n(native / projected LiDAR)')
 axs[1,col].plot(x,[100*r['metrics']['abs_relative_error']['median'] for r in rows],'o-',color=color,lw=1.1,ms=4);axs[1,col].set_ylabel('Median absolute relative\ndepth error (%)')
 axs[2,col].bar(x,[r['compared'] for r in rows],width=65,color=color);axs[2,col].set_ylabel('Compared native\nstereo features');axs[2,col].set_xlabel('Original source index')
 axs[0,col].set_ylim(.2,1.2);axs[1,col].set_ylim(0,75);axs[2,col].set_ylim(0,100)
 for ax in axs[:,col]:ax.grid(alpha=.2);ax.set_xlim(-50,1950)
fig.suptitle('Native stereo depth versus supplied single-scan LiDAR projection',fontsize=16,y=.98)
fig.subplots_adjust(top=.92,bottom=.14,hspace=.28,wspace=.28)
fig.text(.5,.025,'Conditional sparse-keypoint comparison: nearest projected-depth pixel, no interpolation or fitted correction.\nAll 40 cases retained; RGB 309 / thermal 1,089 compared features. Camera populations differ.\nProjection, visibility and physical timestamp association remain unqualified; these are reference proxies.',ha='center',fontsize=10)
f=ROOT/'docs/research/figures/native-depth-proxy-2026-11-05'
for ext in ['png','pdf']:fig.savefig(str(f)+'.'+ext,dpi=150)
print(f)

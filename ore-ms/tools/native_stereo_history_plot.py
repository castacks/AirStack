"""Plot immutable EXP146 native per-frame distributions and same-run motion."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'results/native-stereo-history/run-002'
s=json.loads((out/'summary.json').read_text());fig,axs=plt.subplots(3,2,figsize=(13,11))
for col,mode in enumerate(['rgb','thermal']):
 frames=json.loads((out/'analysis'/mode/'frames.json').read_text())['rows'];motion=json.loads((out/'analysis'/mode/'motion.json').read_text())['final']['rows'];entry=next(x for x in s['runs'] if x['modality']==mode)
 x=[r['index'] for r in frames]
 for row,key,label in [(0,'acceptance','Native stereo accepted / left features'),(1,'median_depth_m','Per-frame median accepted depth (m)')]:
  y=[r[key] if r[key] is not None else np.nan for r in frames];axs[row,col].plot(x,y,color='#2676a7',lw=.55,alpha=.65)
  bin_key=key
  axs[row,col].plot([b['start']+250 for b in entry['bins']],[b[bin_key]['median'] for b in entry['bins']],'o-',color='#d47b20',lw=2,label='Fixed 500-source bin median')
  axs[row,col].set_ylabel(label)
  if row==0:axs[row,col].set_ylim(0,1);axs[row,col].set_title(('RGB' if mode=='rgb' else 'Thermal')+' — native stereo history');axs[row,col].legend(fontsize=8)
 axs[2,col].scatter([v['indices'][0] for v in motion],[v['ratio'] for v in motion],s=3,alpha=.4,color='#2676a7')
 axs[2,col].plot([b['start']+250 for b in entry['bins']],[b['final_motion']['ratio']['median'] for b in entry['bins']],'o-',color='#d47b20',lw=2)
 axs[2,col].axhline(1,color='gray',ls='--',lw=1);axs[2,col].set_ylabel('Final ORB / reference translation magnitude');axs[2,col].set_xlabel('Original source index (motion pair start)')
 for ax in axs[:,col]:ax.grid(alpha=.2);ax.set_xlim(0,2000)
fig.suptitle('Read-only native stereo observations: same 2,000-source night prefix',fontsize=15,y=.98)
fig.subplots_adjust(top=.93,bottom=.12,hspace=.3,wspace=.28)
fig.text(.5,.025,'All left features of available native frames retained; two RGB frame observations are unavailable.\nNative depths are estimates. Same-run motion retains map/missing boundaries; no fitted correction.\nRecorded timing/calibration provenance unresolved. Logging can affect asynchronous map scheduling.',ha='center',fontsize=10)
f=ROOT/'docs/research/figures/native-stereo-history-2026-11-05'
for ext in ['png','pdf']:fig.savefig(str(f)+'.'+ext,dpi=150)
print(f)

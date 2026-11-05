"""Render saved EXP145 diagnostics without changing scientific outputs."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'results/stereo-reference-diagnostic/run-001'
motion=json.loads((out/'motion.json').read_text())['runs'];clocks=json.loads((out/'clocks.json').read_text())['runs'];images=json.loads((out/'images.json').read_text())['runs']
fig,axs=plt.subplots(2,2,figsize=(13,9),constrained_layout=False)
colors={'final':'#d47b20','droid':'#2676a7'}
for ax,mode in zip(axs[0],['rgb','thermal']):
 for arm in ['final','droid']:
  row=next(v for v in motion if v['modality']==mode and v['arm']==arm)
  xs=[b['start']+250 for b in row['bins']];ys=[b['summary']['ratio']['median'] for b in row['bins']]
  ax.plot(xs,ys,'o-',label='Finalized ORB' if arm=='final' else 'Offline DROID',color=colors[arm])
  for x,y in zip(xs,ys):ax.annotate(f'{y:.3f}',(x,y),xytext=(0,7 if arm=='droid' else -14),textcoords='offset points',ha='center',fontsize=9)
 ax.axhline(1,color='gray',ls='--',lw=1);ax.set(title=('RGB' if mode=='rgb' else 'Thermal')+' — motion magnitude by fixed source bin',xlabel='Source index (bin centre)',ylabel='Median predicted / reference translation',ylim=(.35,1.2));ax.legend(loc='lower left');ax.grid(alpha=.2)
ax=axs[1,0]
for v in clocks:
 vals=np.abs(v['signed_right_minus_left_ns'])/1e6
 x=np.sort(vals);y=np.arange(1,len(x)+1)/len(x)
 ax.plot(x,y,label='RGB' if v['modality']=='rgb' else 'Thermal')
ax.set(xlabel='Absolute recorded right − left time (ms)',ylabel='Fraction of original 2,000 pairs',title='Recorded stereo timing (exposure semantics unresolved)');ax.grid(alpha=.2);ax.legend()
ax=axs[1,1]
for mode in ['rgb','thermal']:
 vals=np.array([abs(v['left'][1]-v['right'][1])/row['fx'] for row in images if row['modality']==mode for v in row['matches']]);x=np.sort(vals);y=np.arange(1,len(x)+1)/len(x);ax.plot(x,y,label='RGB' if mode=='rgb' else 'Thermal')
ax.set(xlabel='Absolute vertical keypoint difference / focal length',ylabel='Fraction of mutual ratio matches',title='Generic stereo matches; 20 fixed pairs per camera',xlim=(0,.02));ax.grid(alpha=.2);ax.legend()
fig.suptitle('Same night prefix: thermal translation shortfall, timing and stereo diagnostics',fontsize=15,y=.98)
fig.subplots_adjust(top=.91,bottom=.15,hspace=.42,wspace=.28)
fig.text(.5,.025,'Own-camera references and support; ratios are descriptive, never fitted/applied as scale corrections.\nGeneric OpenCV correspondences differ from native stereo tracking; outliers retained.\nRecorded timestamps do not prove physical exposure offsets. No calibration change or new SLAM inference.',ha='center',fontsize=10)
dest=ROOT/'docs/research/figures/stereo-reference-diagnostic-2026-11-05'
for ext in ['png','pdf']:fig.savefig(str(dest)+'.'+ext,dpi=150)
print(dest)

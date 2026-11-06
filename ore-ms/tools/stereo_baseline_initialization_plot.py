"""Render EXP150 matched-support and full-source diagnostics."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/stereo-baseline-initialization/run-001'
def main():
 s=json.loads((OUT/'summary.json').read_text());fig,ax=plt.subplots(2,2,figsize=(12,7),layout='constrained');colors=dict(original='#666666',rebuilt='#2878a5',fixed='#d55e00')
 for col,run in enumerate(s['runs']):
  mode=run['modality']
  for arm in colors:
   rows=json.loads((OUT/'analysis'/mode/(arm+'-frames.json')).read_text())['rows']
   ax[0,col].plot([r['index'] for r in rows],[r['accepted'] if r['accepted'] is not None else float('nan') for r in rows],lw=.55,alpha=.7,label=arm,color=colors[arm])
   mo=json.loads((OUT/'analysis'/mode/'motion.json').read_text())[arm+'-final']['bins']
   ax[1,col].plot([b['start']+250 for b in mo],[b['summary']['ratio']['median'] for b in mo],'-o',label=arm,color=colors[arm])
  ax[0,col].set(title=mode+' native stereo acceptance',xlabel='Source index (missing observations left blank)',ylabel='Accepted features');ax[0,col].legend()
  ax[1,col].axhline(1,c='black',ls='--',lw=.8);ax[1,col].set(title=mode+' final travel / supplied reference',xlabel='Fixed 500-source bin midpoint',ylabel='Median relative travel ratio');ax[1,col].legend()
 fig.suptitle('EXP150: one native realization per arm; common pose support and union map boundaries\nExposed night prefix; physical calibration/timing unqualified')
 d=OUT/'figures';d.mkdir(exist_ok=True);fig.savefig(d/'initialization-comparison.png',dpi=160);plt.close(fig)
if __name__=='__main__':main()

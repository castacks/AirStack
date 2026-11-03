"""Publication/export artifact for all predeclared persistence diagnostics."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/ms2-broader-range/run-001'
def main():
 data=json.loads((OUT/'persistence.json').read_text())['windows'];names=list(data);values=[];columns=[]
 for camera in ['rgb','thermal']:
  for kind in ['covariance_persistence','future_translation']:
   for lag in [1,3,10]:columns.append((camera,kind,str(lag)))
 counts=[]
 for name in names:
  row=[];nn=[]
  for cam,kind,lag in columns:
   a=data[name][cam]['lags'][lag];v=a['covariance_persistence'] if kind=='covariance_persistence' else a['future_error_association']['translation_m'];row.append(v['rho'] if v['rho'] is not None else np.nan);nn.append(v['n'])
  values.append(row);counts.append(nn)
 fig,ax=plt.subplots(figsize=(14,6));im=ax.imshow(values,vmin=-1,vmax=1,cmap='RdBu_r',aspect='auto');ax.set_yticks(range(len(names)),names);ax.set_xticks(range(len(columns)),[f'{cam.upper()}\n'+('cov→cov' if kind=='covariance_persistence' else 'cov→T error')+f'\nlag {lag}' for cam,kind,lag in columns],fontsize=8)
 for i,row in enumerate(values):
  for j,rho in enumerate(row):ax.text(j,i,f'{rho:.2f}\nn={counts[i][j]}' if np.isfinite(rho) else f'NA\nn={counts[i][j]}',ha='center',va='center',fontsize=7,color='white' if np.isfinite(rho) and abs(rho)>.6 else 'black')
 for x in [2.5,5.5,8.5]:ax.axvline(x,color='black',lw=1)
 fig.colorbar(im,ax=ax,label='Spearman correlation');ax.set_title('Covariance persistence and future-error association are different quantities\nAll fixed MS² windows; lags 1/3/10 have ~0/~0.2/~0.9 s target-start lead; no measured runtime deadline',fontsize=11);fig.tight_layout();dest=ROOT/'docs/research/figures'
 for ext in ['png','pdf']:fig.savefig(dest/f'ms2-broader-persistence-2026-11-02.{ext}',dpi=180)
 print('PLOTTED all 108 covariance/translation correlation cells')
if __name__=='__main__':main()

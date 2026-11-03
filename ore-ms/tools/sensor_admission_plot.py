"""Descriptive coverage and exact-matched raw-control ratios after locked scoring."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/sensor-admission-execution/run-002'
scores=json.loads((OUT/'scores.json').read_text());records=list(scores['records']);names=['Night','Day 0','Day 1'];methods=['candidate','geometry_current','raw','history']
fig,axes=plt.subplots(1,2,figsize=(11.8,4.1));x=np.arange(len(records));width=.19
for i,m in enumerate(methods):
 values=[scores['records'][r]['methods'][m]['scored']/scores['records'][r]['decisions']*100 for r in records]
 axes[0].bar(x+(i-1.5)*width,values,width,label=m.replace('_',' '))
axes[0].set_xticks(x,names);axes[0].set_ylim(0,110);axes[0].set_ylabel('Scored decisions / all declared decisions (%)');axes[0].set_title('Coverage, including abstentions');axes[0].legend(fontsize=8,loc='lower left')
for i,metric in enumerate(['translation_m','rotation_deg']):
 values=[]
 for r in records:
  match=scores['records'][r]['matched']['raw'];a=match['candidate'][metric];b=match['raw'][metric]
  values.append(a/b if a is not None and b else np.nan)
 axes[1].bar(x+(i-.5)*.28,values,.28,label=metric.split('_')[0])
axes[1].axhline(1,color='black',lw=.8);axes[1].set_xticks(x,names);axes[1].set_ylabel('Candidate / raw RMSE on exact matched decisions');axes[1].set_title('Conditional pose-proxy error; lower is better');axes[1].legend(fontsize=8)
fig.suptitle('Fixed RGB–thermal admission: exposed-recording development comparison',fontsize=12)
fig.text(.5,.015,'Recorded-SLAM/nominal-body proxy; standalone expert selection, not joint factor fusion or independent accuracy.',ha='center',fontsize=8)
fig.tight_layout(rect=[0,.06,1,.94])
for ext in ['png','pdf']:fig.savefig(OUT/f'admission-comparison.{ext}',dpi=180,bbox_inches='tight')

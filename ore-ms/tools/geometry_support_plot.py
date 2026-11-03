"""Predeclared new-record comparison alongside explicit retrospective context."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[1];O=R/'results/geometry-support-transfer/run-001'
new=json.loads((O/'scores.json').read_text());old=json.loads((O/'prior-context/scores.json').read_text())['records']
records=[new,*old.values()];names=['New night\n(n=18)','Prior night\n(n=19)','Prior day 0\n(n=98)','Prior day 1\n(n=54)'];methods=['geometry_admission','retained_count','occupied_cells','information_rank','rgb_only','thermal_only'];labels=['Fixed geometry gate','More points','More image cells','Information rank (diagnostic)','RGB only','Thermal only']
fig,axes=plt.subplots(1,2,figsize=(12.4,4.5));x=np.arange(4);width=.13
for ax,metric,title in zip(axes,['translation_m','rotation_deg'],['Translation proxy RMSE (m)','Rotation proxy RMSE (degrees)']):
 for i,(m,label) in enumerate(zip(methods,labels)):
  values=[r['groups']['both_supported']['methods'][m][metric] for r in records];ax.bar(x+(i-2.5)*width,values,width,label=label)
 ax.set_xticks(x,names);ax.set_yscale('log');ax.set_ylabel(title);ax.grid(axis='y',alpha=.15)
handles,legend=axes[0].get_legend_handles_labels();fig.legend(handles,legend,loc='lower center',ncol=3,frameon=False,fontsize=8,bbox_to_anchor=(.5,.04))
fig.suptitle('Geometry versus support: both experts operational',fontsize=13)
fig.text(.5,.015,'New record: frozen primary test. Prior three: posthoc exposed-data context. All errors are nominal-body recorded-SLAM proxies.',ha='center',fontsize=8)
fig.tight_layout(rect=[0,.17,1,.94])
for ext in ['png','pdf']:fig.savefig(O/f'geometry-support-comparison.{ext}',dpi=180,bbox_inches='tight')

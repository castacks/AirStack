"""EXP152 standalone support-explicit thermal controlled-rank figure."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/stereo-ambiguity-controls/run-001'
def main():
 s=json.loads((OUT/'summary.json').read_text());fig,axes=plt.subplots(2,2,figsize=(12,7),layout='constrained');labels=['depth\n30','spatial\n30','joint\n30','depth\n10','spatial\n10','joint\n10']
 for row,rec in enumerate(['day','night']):
  for col,group in enumerate(['all_primary','locally_consistent']):
   data=[next(a for a in s['aggregates'] if (a['recording'],a['modality'],a['group'],a['control'],a['minimum'])==(rec,'thermal',group,c,n)) for n in [30,10] for c in ['depth','spatial','joint']];ax=axes[row,col]
   for field,color,label in [('full_raw_defined','#666666','Full raw / same images'),('retained_raw','#2878a5','Raw / retained points'),('within_bin','#d55e00','Within-bin ranks')]:ax.plot(range(6),[a[field]['median'] if a[field]['median'] is not None else float('nan') for a in data],'-o',c=color,label=label)
   ax.axhline(0,c='black',lw=.7,ls=':');ax.axvline(2.5,c='black',lw=.7,ls='--');ax.set(xticks=range(6),xticklabels=labels,ylim=(-1,1),title=rec+' thermal / '+group,ylabel='Median coefficient on defined image cases',xlabel='Control / minimum points per stratum')
   for i,a in enumerate(data):ax.text(i,-.91,str(a['within_bin']['n'])+'/20',ha='center',fontsize=8)
   ax.legend(fontsize=8,loc='upper right')
 fig.suptitle('EXP152: fixed depth/location controls of patch gap vs conditional depth error\n30-point primary; 10-point sensitivity. Labels give defined images; unavailable results stay blank.')
 dest=OUT/'figures';dest.mkdir(exist_ok=True);fig.savefig(dest/'controlled-patch-gap.png',dpi=160);plt.close(fig)
if __name__=='__main__':main()

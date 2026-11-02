"""Static research figure: per-record ranking versus coverage costs."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/expanded-fixed-model-evaluation/run-001'
s=json.loads((OUT/'summary.json').read_text());records=s['records'];identities=list(records);arms=['recurrent_diagonal_nll','final_diagonal_nll','final_trace_risk_score'];labels=['Recurrent diagonal','Final diagonal','Final trace'];colors=['#0072B2','#E69F00','#CC79A7']
figure,axes=plt.subplots(1,2,figsize=(12,6),sharey=True);y=np.arange(len(identities));height=.24
for j,(arm,label,color) in enumerate(zip(arms,labels,colors)):
 risk=[records[identity]['arms'][arm]['relative_mse50_change_percent'] for identity in identities]
 assert all(v is not None for v in risk),'undefined relative risk remains explicit in summary; cannot silently plot'
 coverage=[100*(records[identity]['arms'][arm]['absolute_coverage95_error']-records[identity]['arms']['baseline']['absolute_coverage95_error']) for identity in identities]
 axes[0].barh(y+(j-1)*height,risk,height=height,color=color,label=label)
 axes[1].barh(y+(j-1)*height,coverage,height=height,color=color,label=label)
for axis in axes:
 axis.axvline(0,color='#333333',linewidth=1);axis.grid(axis='x',alpha=.2);axis.set_axisbelow(True)
axes[0].set_yticks(y,[identity.split('/')[1] for identity in identities]);axes[0].invert_yaxis();axes[0].set_xlabel('Change in MSE at 50% retention (%)');axes[0].set_title('Ranking relative to original checkpoint')
axes[1].set_xlabel('Change in absolute 95% coverage error (percentage points)');axes[1].set_title('Coverage relative to original checkpoint');axes[1].legend(loc='best',fontsize=9)
figure.suptitle('Eight additional synthetic RGB prefixes · fixed models · 16 pairs per record',fontsize=12)
figure.text(.5,.015,'Negative values favor adaptation. Correlated prefixes and unknown base pretraining; no population or physical-calibration claim.',ha='center',fontsize=9)
figure.tight_layout(rect=(0,.04,1,.94))
for suffix in ('png','pdf'):
 path=OUT/('ranking-coverage-tradeoff.'+suffix);assert not path.exists();figure.savefig(path,dpi=180,bbox_inches='tight')
plt.close(figure);print('Saved PNG/PDF')

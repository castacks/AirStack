"""Plot all fixed group corroboration checks, including inconclusive cases."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).resolve().parents[1]/'results/fixed-group-diagnosis/run-001';d=json.load(open(O/'diagnosis.json'));p=json.load(open(O/'posthoc-duration-probe.json'));labels=[];ncc=[];lk=[];old=[];new=[];ratio=[];colors=[]
for case in d['cases']:
 e=case['event'];q=next(x for x in p['rows'] if x['event']['condition']==e['condition'] and x['event']['pair']==e['pair']);s=case['cameras']['thermal']['selected'];labels.append(f"{e['condition']} {e['event_category']}\npair {e['pair']}");ncc.append(s['patch_ncc']['median']);lk.append(s['lk_cycle_supported']/s['count']);old.append(q['groups']['selected']['original_reference_uv_residual_median_px']);new.append(q['groups']['selected']['duration_probe_uv_residual_median_px']);ratio.append(q['duration_ratio']);colors.append({'helps':'#258a55','hurts':'#c44444','neutral':'#777777'}[e['event_category']])
x=np.arange(len(labels));fig,axes=plt.subplots(3,1,figsize=(14,11),layout='constrained');axes[0].bar(x-.18,ncc,width=.36,label='Median raw patch correlation');axes[0].bar(x+.18,lk,width=.36,label='Fraction passing classical flow cycle');axes[0].set_ylim(0,1.1);axes[0].legend();axes[0].set_ylabel('Photometric / tracker agreement')
axes[1].bar(x,ratio,color=colors);axes[1].axhline(1,color='black',linewidth=1);axes[1].set_ylabel('Recorded thermal / RGB duration');axes[1].set_title('Same frame IDs do not guarantee matching recorded motion intervals')
axes[2].bar(x-.18,old,width=.36,label='Nominal same-interval reference');axes[2].bar(x+.18,new,width=.36,label='Fixed duration-ratio / constant-twist probe');axes[2].set_ylabel('Median selected-group UV disagreement (px)');axes[2].legend()
for ax in axes:ax.set_xticks(x,labels,fontsize=8)
fig.suptitle('Fixed-case diagnosis: no new fusion outputs or fitted timing offsets\nReference-assisted duration probe is conditional; physical exposure timing, depth and motion remain unqualified.',fontsize=12);fig.savefig(O/'fixed-case-diagnosis.png',dpi=150);plt.close(fig)

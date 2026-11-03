"""Standalone stage-readiness and availability figures; full primary counts retained."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from early_health_execution import ROOT,OUT,read

def main():
 stage=read(OUT/'fresh-stage-timings.json')['summaries'];scores=read(OUT/'availability-scores.json')['horizons']['500000000'];fig,axes=plt.subplots(1,3,figsize=(15,4.7));colors=['#246bb2','#de6b22'];labels=['Covariance','Tracking count','Pair end','Pose info*','Old decision']
 ax=axes[0]
 for offset,(domain,color) in zip([-.18,.18],zip(['td3','ms2'],colors)):
  names=['joint_covariance_s','joint_tracking_s','joint_pair_end_s','joint_information_s','old_decision_prefix_s'];med=[stage[domain][n]['median'] for n in names];p95=[stage[domain][n]['p95'] for n in names];x=np.arange(5)+offset;ax.bar(x,med,.34,color=color,label=domain.upper());ax.errorbar(x,med,yerr=[np.zeros(5),np.asarray(p95)-med],fmt='none',ecolor='black',capsize=2)
 ax.axhline(.5,color='red',ls='--',lw=1,label='0.5 s primary lead');ax.set_xticks(range(5),labels,rotation=20,ha='right');ax.set_ylabel('joint readiness from first end-frame release (s)');ax.set_title('Fresh engineering captures: median / P95');ax.plot([],[],color='none',label='*paired-info subset');ax.legend(fontsize=7);ax.set_ylim(0,1.65)
 ax=axes[1];methods=['early_health','early_confirmed','early_resolved','late_eligible','late_information'];short=['Early health','Two-good','Last resolved','Late eligible','Late info']
 for offset,domain,color in zip([-.18,.18],['td3','ms2'],colors):
  vals=[scores['domains'][domain]['targets']['projected']['selection'][m]['selected_unavailable'] for m in methods];ax.bar(np.arange(5)+offset,vals,.34,color=color,label=domain.upper())
 ax.set_xticks(range(5),short,rotation=25,ha='right');ax.set_ylabel('selected output unavailable at target deadline');ax.set_title('Same outcomes with earlier health observation');ax.set_ylim(0,11);ax.legend(fontsize=8)
 ax=axes[2];native=scores['domains']['ms2']['targets']['camera_local']['recovery']['early_resolved']['thermal'];projected=scores['domains']['ms2']['targets']['projected']['recovery']['early_resolved']['thermal'];rgb=scores['domains']['td3']['targets']['camera_local']['recovery']['early_resolved']['rgb'];n=[native['observed_false_to_true_runs'],projected['observed_false_to_true_runs'],rgb['observed_false_to_true_runs']];bad=[native['undetected'],projected['undetected'],rgb['undetected']];ax.bar(['MS² thermal\nnative eligibility','MS² thermal\nprojected eligibility','TD3 RGB\nnative eligibility'],np.asarray(n)-bad,label='detected',color='#51a175');ax.bar(range(3),bad,bottom=np.asarray(n)-bad,label='undetected/censored',color='#b55649');ax.set_ylabel('observed recovery runs');ax.set_title('Selection success does not establish recovery');ax.legend(fontsize=7)
 fig.suptitle('EXP116: no-fit availability persistence and observation-only native events\nAll selected-unavailable cases here are supported terminal pairs awaiting final write; primary counts remain intact',fontsize=11);fig.tight_layout(rect=[0,0,1,.87]);dest=ROOT/'docs/research/figures'
 for ext in ['png','pdf']:fig.savefig(dest/f'early-health-availability-2026-11-03.{ext}',dpi=180)
 print('PLOTTED early readiness, retained availability counts and recovery censoring')
if __name__=='__main__':main()

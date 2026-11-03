"""Standalone masked-request budget and availability results; not compute savings."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from guarded_probe_cache import ROOT,OUT,read

def main():
 s=read(OUT/'scores.json');methods=['full_read_cache','bounded_probe','bounded_no_probe','unbounded_hold','fixed_rgb','fixed_thermal'];names=['Full read','Bounded probe','No periodic probe','Unbounded hold','Fixed RGB','Fixed thermal'];fig,axes=plt.subplots(1,3,figsize=(15,5));colors=['#246bb2','#de6b22']
 for offset,domain,color in zip([-.18,.18],['td3','ms2'],colors):
  p=s['domains'][domain]['policies'];x=np.arange(len(methods))+offset;axes[0].bar(x,[100*p[m]['request_fraction_of_dense_core'] for m in methods],.34,color=color,label=domain.upper());axes[1].bar(x,[p[m].get('selected_unavailable',0)+p[m].get('uncommitted_while_available',0) for m in methods],.34,color=color,label=domain.upper())
 for ax in axes[:2]:ax.set_xticks(range(len(names)),names,rotation=30,ha='right');ax.legend(fontsize=8)
 axes[0].set_ylabel('requested pair observations / dense core (%)');axes[0].set_ylim(0,108);axes[0].set_title('Recorded-observation masking only')
 axes[1].set_ylabel('unavailable choice + no trusted choice');axes[1].set_title('Guarded pre-input availability')
 for domain,color in zip(['td3','ms2'],colors):
  p=s['domains'][domain]['policies'];axes[2].plot(['Bounded probe','No periodic probe','Unbounded hold'],[p[m]['resolved_age_max_s'] for m in ['bounded_probe','bounded_no_probe','unbounded_hold']],marker='o',color=color,label=domain.upper())
 axes[2].set_ylabel('maximum stored resolved-state age (s)');axes[2].set_title('Stored stale states remain explicit');axes[2].legend(fontsize=8);axes[2].grid(alpha=.2)
 fig.suptitle('EXP117: 1,150 guarded core pairs; request masks retain dense native state and timing\nNo measured inference savings, native sparse-gap validation, pose improvement or deployment claim',fontsize=12);fig.tight_layout(rect=[0,0,1,.88]);dest=ROOT/'docs/research/figures'
 for ext in ['png','pdf']:fig.savefig(dest/f'guarded-probe-cache-2026-11-03.{ext}',dpi=180)
 print('PLOTTED guarded masked requests/availability/stored age')
if __name__=='__main__':main()

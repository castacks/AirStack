"""All fixed-action outcomes, with no winner-selection or threshold fitting."""
import argparse,csv
from pathlib import Path
import numpy as np
from qualify_native_frontend import read,write,sha,now

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--protocol',required=True);a=ap.parse_args();p=read(a.protocol)
    for path,h in p['pins'].items():assert sha(path)==h
    folder=Path(p['score_output']);s=read(folder/'scores.json');rows=s['rows']
    columns=['condition','beta','arm','mode','successful','status','points','translation_mm','rotation_deg','matching_covariance','translation_information_std_proxy_mm','rotation_information_std_proxy_deg'];table=[]
    for x in rows:
        m=x['metrics'];table.append([x['condition'],x['beta'],x['arm'],x['mode'],x['successful'],x['status'],x['points'],None if m is None else m['translation_m']*1000,None if m is None else m['rotation_deg'],x['median_temporal_covariance_trace'],None if m is None else m['translation_information_std_proxy_m']*1000,None if m is None else m['rotation_information_std_proxy_deg']])
    with (folder/'all-outcomes.csv').open('x',newline='') as f:w=csv.writer(f);w.writerow(columns);w.writerows(table)
    comparisons=[]
    for condition in ['day','night','rain']:
        for mode in ['regenerated','common_ids']:
            q=[x for x in rows if x['condition']==condition and x['mode']==mode and x['arm']=='memory'];q.sort(key=lambda x:x['beta']);success=[x for x in q if x['successful']]
            comparisons.append({'condition':condition,'mode':mode,'planned_actions':3,'successful_actions':len(success),'translation_error_range_mm':None if len(success)!=3 else float(np.ptp([x['metrics']['translation_m']*1000 for x in q])),'rotation_error_range_deg':None if len(success)!=3 else float(np.ptp([x['metrics']['rotation_deg'] for x in q])),'beta_plus2_minus_minus2_translation_mm':None if len(success)!=3 else (q[2]['metrics']['translation_m']-q[0]['metrics']['translation_m'])*1000,'beta_plus2_minus_minus2_rotation_deg':None if len(success)!=3 else q[2]['metrics']['rotation_deg']-q[0]['metrics']['rotation_deg']})
    write(folder/'summary.json',{'created_utc':now(),'planned_pose_attempts':36,'successful_pose_fits':sum(x['successful'] for x in rows),'comparisons':comparisons,'score_sha256':sha(folder/'scores.json'),'scope':'Fixed3actions/3exposedtextures, descriptive absolute errors, no selected scheduling policy, significance or physicaltransfer.'})
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(10,6),sharex=True)
    for col,mode in enumerate(['regenerated','common_ids']):
        for c,color in zip(['day','night','rain'],['#b76d00','#2457a7','#39825c']):
            q=sorted([x for x in rows if x['condition']==c and x['mode']==mode and x['arm']=='memory'],key=lambda x:x['beta']);sharp=next(x for x in rows if x['condition']==c and x['mode']==mode and x['arm']=='sharp')
            for i,(key,factor,label) in enumerate([('translation_m',1000,'Translation error (mm)'),('rotation_deg',1,'Rotation error (deg)')]):
                axes[i,col].plot([x['beta'] for x in q],[x['metrics'][key]*factor if x['successful'] else np.nan for x in q],marker='o',color=color,label=c+' memory')
                if sharp['successful']:axes[i,col].axhline(sharp['metrics'][key]*factor,color=color,linestyle='--',alpha=.5)
                axes[i,col].set_ylabel(label);axes[i,col].grid(alpha=.2)
        axes[0,col].set_title(mode);axes[1,col].set_xlabel('Fixed schedule beta');axes[1,col].set_xticks([-2,0,2]);axes[0,col].legend(fontsize=8)
    fig.suptitle('Matched path and deadline: memory actions; dashed = shared sharp baseline')
    fig.tight_layout();fig.savefig(folder/'pose-errors.png',dpi=160);plt.close(fig)
    print('Summarized all36attempts; no action policy selected')
if __name__=='__main__':main()

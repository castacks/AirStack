"""Frozen EXP132 pooled outcomes and recording-level fixed-camera diagnostic."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from signal_transfer import OUT,read
p=read(OUT/'evaluation.json');a=p['aggregate']['methods'];records=list(p['recordings']);models=['raw_uncertainty','angular_uncertainty','ridge_combined@night_thermal_so_salon3','ridge_combined@night_thermal_so_salon'];labels=['Raw uncertainty','Angular proxy','Combined: trained salon3','Combined: trained salon']
fig,axes=plt.subplots(1,2,figsize=(12,4.7),layout='constrained')
names=['fixed_rgb','fixed_thermal']+models;short=['Fixed RGB','Fixed thermal']+labels
axes[0].barh(short,[a[n]['rotation_mean_deg'] for n in names],color=['#969da3','#969da3','#4986af','#7aa9c6','#39815f','#7bb493']);axes[0].invert_yaxis();axes[0].set_xlabel('Pooled mean rotation discrepancy (degrees)');axes[0].axvline(a['fixed_rgb']['oracle_mean_deg'],linestyle=':',color='#725a98',label='Per-interval oracle');recordbest=sum(g['rows']*min(g['methods'][n]['rotation_mean_deg'] for n in ['fixed_rgb','fixed_thermal']) for g in p['recordings'].values())/171;axes[0].axvline(recordbest,color='#c36b25',linestyle='--',label='Hindsight best fixed per recording');axes[0].legend(fontsize=7,frameon=False)
for name,label in zip(models,labels):
 ratios=[p['recordings'][r]['methods'][name]['rotation_mean_deg']/min(p['recordings'][r]['methods'][n]['rotation_mean_deg'] for n in ['fixed_rgb','fixed_thermal']) for r in records]
 axes[1].plot(range(4),ratios,marker='o',label=label)
axes[1].set_xticks(range(4),['day0\n(95)','day1\n(45)','night mac\n(16)','night salon2\n(15)']);axes[1].set_ylabel('Rotation mean / recording best fixed mean');axes[1].set_yscale('log');axes[1].axhline(1,color='#555',linestyle='--');axes[1].legend(fontsize=7,frameon=False)
for ax in axes:ax.grid(axis='y' if ax is axes[1] else 'x',alpha=.2);ax.set_axisbelow(True)
fig.suptitle('EXP132: unchanged transfer to saved MSA thermal cohort\n171 matched intervals; recorded-SLAM proxy; known collapse dominates pooled gains',fontsize=11)
for suffix in ['png','svg']:
 target=OUT/('signal-transfer.'+suffix);assert not target.exists();fig.savefig(target,dpi=180)

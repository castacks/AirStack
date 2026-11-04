"""Plot frozen EXP131 aggregate metrics; primary method declared before fitting."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rotation_signal_audit import OUT,read
p=read(OUT/'evaluation.json')['aggregate']['methods'];names=list(p);labels=['Fixed RGB','Fixed thermal','Training best fixed','Min raw trace','Min angular proxy','Max retention','Max occupancy','Ridge: uncertainty','Ridge: health','Ridge: combined (primary)']
fig,axes=plt.subplots(1,2,figsize=(11,5.4),layout='constrained')
colors=['#378060' if n=='ridge_combined' else '#4986af' if n in ['raw_uncertainty','angular_uncertainty'] else '#969da3' for n in names]
for ax,metric,title in zip(axes,['rotation_mean_deg','translation_mean_m'],['Mean rotation discrepancy (degrees)','Mean translation discrepancy (m)']):
 ax.barh(labels,[p[n][metric] for n in names],color=colors);ax.invert_yaxis();ax.set_xlabel(title);ax.axvline(p['fixed_thermal'][metric],color='#c36b25',linestyle='--',label='Fixed thermal');ax.grid(axis='x',alpha=.2);ax.set_axisbelow(True)
axes[0].axvline(p['fixed_thermal']['oracle_mean_deg'],color='#584883',linestyle=':',label='Rotation oracle');axes[0].legend(frameon=False,fontsize=8)
fig.suptitle('EXP131: completed-output arbitration on 323 development intervals\nRecorded-SLAM proxy; both experts executed; recording-disjoint fitting',fontsize=11)
for suffix in ['png','svg']:
 path=OUT/('signal-audit.'+suffix);assert not path.exists();fig.savefig(path,dpi=180)

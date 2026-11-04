"""EXP124 native delivery/cost and explicitly scoped encoded capacity figure."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from leased_handover_study import ROOT,OUT,read
s=read(OUT/'scores.json');names=['full_read_cache','bounded_probe','fixture_on','leased_probe','diagnostic_probe']
labels=['Full read','Original sparse','Fixed fixture\n4 MiB','Live leased\n4 MiB','Diagnostic\n2 MiB']
g=[s['groups'][a] for a in names]
fig,axes=plt.subplots(1,3,figsize=(12,4),constrained_layout=True)
for ax,key,title in [(axes[0],'eligible','On-time adjacent deliveries / 95'),(axes[1],'calls','Actual native pair calls')]:
 values=[x[key] if key=='eligible' else x['cost']['total_pair_calls'] for x in g]
 bars=ax.bar(range(5),values,color=['#777777','#9db6c9','#d0a05a','#397da3','#b55b55'])
 ax.set_title(title);ax.set_xticks(range(5),labels,rotation=35,ha='right');ax.set_ylim(0,max(values)*1.16)
 for b,v in zip(bars,values):ax.text(b.get_x()+b.get_width()/2,v+max(values)*.02,str(v),ha='center',fontsize=9)
ax=axes[2];ax.bar([0,1],[4,2],label='Store capacity',color='#397da3');ax.bar([0,1],[2,2],bottom=[4,2],label='Allocated shared arena',color='#d0a05a')
ax.set_xticks([0,1],['Primary','Diagnostic']);ax.set_ylabel('MiB');ax.set_ylim(0,7);ax.set_title('Encoded capacity boundary');ax.legend(fontsize=8)
fig.suptitle('Synthetic serial native replay with explicit handover leases',fontsize=13)
fig.text(.5,-.04,'Capacity excludes staged input, control/Python memory, normal decoding, GPU, RSS and OS cache. No live cadence or accuracy claim.',ha='center',fontsize=8)
for ext in ['png','pdf']:
 path=ROOT/'docs/research/figures'/('leased-handover-2026-11-03.'+ext)
 assert not path.exists();fig.savefig(path,dpi=180,bbox_inches='tight')

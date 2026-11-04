"""EXP125 night transfer: fresh native controls and conditional exact-cost readouts."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from night_transfer_study import ROOT,OUT,read
s=read(OUT/'scores.json');names=['full_read_cache','bounded_probe','leased_probe','fixed_rgb','fixed_thermal']
labels=['Full read','Original sparse','Candidate','Fixed RGB','Fixed thermal'];g=[s['groups'][a] for a in names]
fig,axes=plt.subplots(1,3,figsize=(12.5,4.3),constrained_layout=True)
for ax,key,title in [(axes[0],'eligible','Fresh native controls: eligible / 95'),(axes[1],'calls','Actual native pair calls')]:
 values=[x['eligible'] if key=='eligible' else x['cost']['total_pair_calls'] for x in g]
 bars=ax.bar(range(5),values,color=['#777777','#9db6c9','#397da3','#bd855b','#7a9c68'])
 ax.set_title(title,fontsize=11);ax.set_xticks(range(5),labels,rotation=35,ha='right');ax.set_ylim(0,max(values)*1.18+1)
 for b,v in zip(bars,values):ax.text(b.get_x()+b.get_width()/2,v+max(values)*.025,str(v),ha='center',fontsize=9)
ax=axes[2];values=[s['groups']['leased_probe']['eligible']]+[s['matched_readouts'][a]['eligible'] for a in ['matched_rgb','matched_thermal']]
bars=ax.bar(range(3),values,color=['#397da3','#bd855b','#7a9c68'])
for b in bars[1:]:b.set_hatch('//')
ax.set_xticks(range(3),['Candidate','Always RGB','Always thermal'],rotation=35,ha='right');ax.set_ylim(0,max(values)*1.18+1)
ax.set_title('Same candidate executions: eligible / 95',fontsize=11)
for b,v in zip(bars,values):ax.text(b.get_x()+b.get_width()/2,v+max(values)*.025,str(v),ha='center',fontsize=9)
fig.suptitle('Night condition transfer: previously inspected recording, unchanged candidate',fontsize=13)
fig.text(.5,-.035,'Hatched bars are conditional readout ablations sharing candidate executions and cost; not separate native runs. Synthetic offline deadlines.',ha='center',fontsize=8)
for ext in ['png','pdf']:
 p=ROOT/'docs/research/figures'/('night-transfer-2026-11-03.'+ext);assert not p.exists();fig.savefig(p,dpi=180,bbox_inches='tight')

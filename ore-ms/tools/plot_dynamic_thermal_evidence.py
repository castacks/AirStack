"""Scientific event views and time traces; no cross-camera registration implied."""
from pathlib import Path
import json,numpy as np,cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
R=Path(__file__).resolve().parents[1];OUT=R/'results/dynamic-thermal-evidence/run-001';REC=R/'results/thermal-incremental-comparison/recovery';a=json.load(open(OUT/'ablation-scores.json'));e=json.load(open(OUT/'primary-evidence.json'));examples=[]
fig,axes=plt.subplots(3,4,figsize=(18,9),layout='constrained')
for ri,c in enumerate(['day','night','rain']):
 for ci,cat in enumerate(['helps','hurts']):
  cells=[x for x in a['rows'] if x['condition']==c and x['event_category']==cat and x['removed_count'] and 'increase_over_full_fixed_m' in x['scores']['cell']];z=max(cells,key=lambda x:abs(x['scores']['cell']['increase_over_full_fixed_m']));i=z['pair'];stem=f'{i+1:06}';rgb=cv2.imread(str(REC/'images'/c/'rgb'/(stem+'.png')));thr=cv2.imread(str(REC/'images'/c/'thr'/(stem+'.png')),cv2.IMREAD_UNCHANGED);u0=(z['cell']%4)*160;v0=(z['cell']//4)*64
  with np.load(REC/f'packets/{c}/{i:03}-thermal.npz') as p:uv=p['observations'][:,:2]
  x,y=axes[ri,2*ci:2*ci+2];x.imshow(cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB));x.set_title(f'{c} {cat}, pair {i}: RGB target');lo,hi=np.percentile(thr,[5,95]);y.imshow(thr,cmap='gray',vmin=lo,vmax=hi);y.scatter(uv[:,0],uv[:,1],s=5,c='#3ed96c',alpha=.65);ids=z['removed_local_ids'];y.scatter(uv[ids,0],uv[ids,1],s=12,c='#ff4545');y.add_patch(Rectangle((u0,v0),160,64,fill=False,edgecolor='#ff4545',linewidth=2));y.set_title(f'Thermal: cell {z["cell"]}, remove {len(ids)}\ngain {z["baseline"]["translation_gain_m"]*1000:+.2f} → {z["scores"]["cell"]["thermal_gain_vs_rgb_m"]*1000:+.2f} mm')
  for ax in [x,y]:ax.axis('off')
  examples.append({k:z[k] for k in ['condition','event_category','pair','cell','removed_count','removed_local_ids','removed_depth_median_m']})
fig.suptitle('Actual target images, fixed selected events; red = largest-effect cell among all 16 tested cells\nThermal display p5–p95; RGB/thermal views are not pixel-registered. Positive gain means lower translation error than RGB.',fontsize=12);fig.savefig(OUT/'event-regions.png',dpi=140);plt.close(fig)
fig,axes=plt.subplots(3,2,figsize=(14,10),layout='constrained')
for i,c in enumerate(['day','night','rain']):
 rr=[x for x in e['rows'] if x['condition']==c];t=np.array([x['time_ns'] for x in rr],dtype=np.int64);seconds=(t-t[0])/1e9;axes[i,0].plot(seconds,[x['translation_gain_m']*1000 for x in rr],label='fixed thermal gain',color='black');axes[i,0].axhline(0,color='gray',linewidth=.7);axes[i,0].set_ylabel(f'{c}: translation gain (mm)');axes[i,0].set_xlabel('Elapsed RGB source time (s)')
 for name in ['uv_variance_ratio','patch_contrast_ratio','flow_ratio']:
  vals=np.array([x['features'][name] if x.get('features') else np.nan for x in rr]);axes[i,1].plot(seconds,vals,label=name)
 axes[i,1].set_ylabel('Log thermal/RGB descriptor ratio');axes[i,1].set_xlabel('Elapsed RGB source time (s)');axes[i,1].legend(fontsize=8)
fig.suptitle('Within-sequence dynamics: all intervals retained; descriptor association is not a causal explanation');fig.savefig(OUT/'time-traces.png',dpi=140);plt.close(fig)
(OUT/'illustrated-examples.json').write_text(json.dumps({'examples':examples,'selection':'Posthoc largest absolute cell-effect for each preselected help/hurt event; all144cells retained separately.'},indent=2)+'\n');print('Saved event-regions.png and time-traces.png')

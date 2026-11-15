"""Descriptive frozen-arm tables and planned conditioning diagnostics; no new fits."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from qualify_native_frontend import ROOT,sha

root=ROOT/'results/selection-policy-learning';scored=root/'scoring-001';scores=json.loads((scored/'scores.json').read_text());prediction=json.loads((root/'evaluation-001/predictions.json').read_text())['rows'];cache={r['id']:r for r in json.loads((ROOT/'results/selection-native-cache/run-001/predictions.json').read_text())['rows']}
primary=['native_context','uniform32_seed0','information32','pose_weight_only32','match_reward_policy32','pose_reward_policy32']
labels=['Native full pool','Uniform 32','Information 32','Pose-trained weights','Match-reward policy','Pose-reward policy']
colors=['#777777','#3274a1','#3a923a','#e1812c','#9372b2','#c03d3e']
conditioning=[];packets={}
for row in prediction:
 key=row['scene_id']
 if key not in packets:
  with np.load(cache[key]['packet_path']) as z:packets[key]={k:z[k].copy() for k in ['X0','X1','C0','C1','source_uv','length']}
 p=packets[key]
 for arm,result in row['arms'].items():
  if not result['successful']:continue
  ids=np.array(result['selected_rows']);R=np.array(result['target_to_source'])[:3,:3];t=np.array(result['target_to_source'])[:3,3];q=p['X1'][ids]@R.T+t
  def skew(x):
   a,b,c=x;return np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
  J=np.concatenate([np.broadcast_to(np.eye(3),(len(ids),3,3)),-np.stack([skew(v) for v in q])],2);C=p['C0'][ids]+R@p['C1'][ids]@R.T
  if result['utility_weights'] is not None:C=C/np.array(result['utility_weights'])[:,None,None]
  D=np.diag([float(p['length'])]*3+[1.]*3);A=np.linalg.solve(np.linalg.cholesky(C),J@D).reshape(-1,6);singular=np.linalg.svd(A,compute_uv=False);uv=p['source_uv'][ids];cells=np.floor(uv/np.array([640,256])*4).astype(int);occupied=len(set((cells[:,1]*4+cells[:,0]).tolist()))
  conditioning.append({'fold':row['fold'],'scene_id':key,'arm':arm,'selected_points':len(ids),'normalized_information_condition':float((singular[0]/singular[-1])**2),'minimum_normalized_information_eigenvalue':float(singular[-1]**2),'occupied_4x4_cells':occupied})
summary={}
for group in scores['folds']:
 summary[group]={}
 for arm in scores['folds'][group]['arms']:
  values=[r for r in conditioning if r['fold']==group and r['arm']==arm]
  summary[group][arm]={'successful_scenes':len(values),'median_information_condition':float(np.median([r['normalized_information_condition'] for r in values])) if values else None,'median_occupied_cells':float(np.median([r['occupied_4x4_cells'] for r in values])) if values else None}
(scored/'conditioning-report.json').write_text(json.dumps({'source_sha256':sha(__file__),'scope':'Planned local-information conditioning from final whitened Jacobian and source-image cell occupancy. Descriptive diagnostics; no extra fits, metric/gate changes or causal/physical-calibration claim.','rows':conditioning,'summary':summary},indent=2)+'\n')
fig,axes=plt.subplots(2,1,figsize=(12,8),layout='constrained');x=np.arange(3)
for i,(arm,label,color) in enumerate(zip(primary,labels,colors,strict=True)):
 values=[r['arms'][arm]['own'] for r in scores['folds'].values()]
 axes[0].bar(x+(i-2.5)*.135,[r['translation_rmse_m']*1000 for r in values],.13,label=label,color=color)
 axes[1].bar(x+(i-2.5)*.135,[r['rotation_rmse_deg'] for r in values],.13,color=color)
for ax in axes:ax.set_xticks(x,[k+' texture family' for k in scores['folds']]);ax.set_yscale('log');ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
axes[0].set_ylabel('Translation RMSE (mm)');axes[1].set_ylabel('Rotation RMSE (degrees)');axes[0].legend(ncols=3,frameon=False,fontsize=9)
fig.suptitle('Frozen utility-learning comparison: deterministic primary policies\n80 artificial evaluation scenes per excluded texture recording')
for suffix in ['png','pdf']:fig.savefig(scored/('primary-policy-comparison.'+suffix),dpi=160)
fig,axes=plt.subplots(1,2,figsize=(12,4.8),layout='constrained')
for j,arm in enumerate(['match_reward_policy32','pose_reward_policy32']):
 top=[r['arms'][arm]['own']['translation_rmse_m']*1000 for r in scores['folds'].values()];axes[j].plot(x,top,'s-',color='#c03d3e',label='Deterministic top-k')
 for i in range(4):axes[j].plot(x,[r['arms'][arm+f'_stochastic{i}']['own']['translation_rmse_m']*1000 for r in scores['folds'].values()],'o--',alpha=.75,label=f'Fixed stochastic context {i+1}')
 axes[j].set_xticks(x,list(scores['folds']));axes[j].set_title(arm.replace('_',' '));axes[j].set_ylabel('Translation RMSE (mm)');axes[j].set_yscale('log');axes[j].grid(alpha=.2)
axes[0].legend(frameon=False,fontsize=8);fig.suptitle('All four predeclared stochastic contexts shown; none selected as replacement')
for suffix in ['png','pdf']:fig.savefig(scored/('policy-deployment-contexts.'+suffix),dpi=160)
print(json.dumps({g:{a:summary[g][a] for a in primary} for g in summary},indent=2))

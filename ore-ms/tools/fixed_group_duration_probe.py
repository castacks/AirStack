"""Posthoc fixed timestamp-duration probe; no offset search or fusion correction."""
from pathlib import Path
import json,numpy as np
import nominal_joint_motion as geom
from pose_covariance_replay import exp_se3,log_se3
R=Path(__file__).resolve().parents[1];REC=R/'results/thermal-incremental-comparison/recovery';OUT=R/'results/fixed-group-diagnosis/run-001'
def read(p):return json.loads(Path(p).read_text())
def load(p):
 with np.load(p) as z:return {k:z[k] for k in z.files}
def main():
 rows=[]
 for event in read(OUT/'fixed-events.json')['events']:
  c,i=event['condition'],event['pair'];b=read(REC/f'binding-{c}.json')['rows'][i];tt=b['actual_camera_times_ns'];dr=tt['rgb_left'][1]-tt['rgb_left'][0];dt=tt['thermal_left'][1]-tt['thermal_left'][0];assert dr>0 and dt>0;ratio=dt/dr
  ref=np.array(read(REC/f'scores-{c}.json')['rows'][i]['reference_delta']);T=np.linalg.inv(ref);scaled=exp_se3(log_se3(T)*ratio);p=load(REC/b['cameras']['thermal']['packet']);pred,_=geom.project(p['points'],p['camera_R'],p['camera_t'],p['Kb'],T);probe,_=geom.project(p['points'],p['camera_R'],p['camera_t'],p['Kb'],scaled);ids=np.array(event['removed_local_ids']);rest=np.array([j for j in range(len(p['points'])) if j not in set(ids.tolist())]);record={'event':event,'rgb_duration_ms':dr/1e6,'thermal_duration_ms':dt/1e6,'duration_ratio':ratio,'timestamp_offsets_ms':[(tt['thermal_left'][j]-tt['rgb_left'][j])/1e6 for j in [0,1]],'groups':{}}
  for name,index in [('selected',ids),('remaining',rest)]:
   old=np.linalg.norm(pred[index,:2]-p['observations'][index,:2],axis=1);new=np.linalg.norm(probe[index,:2]-p['observations'][index,:2],axis=1);record['groups'][name]={'count':len(index),'original_reference_uv_residual_median_px':float(np.median(old)),'duration_probe_uv_residual_median_px':float(np.median(new)),'original_reference_uv_residual_mean_px':float(np.mean(old)),'duration_probe_uv_residual_mean_px':float(np.mean(new))}
  rows.append(record);print(c,event['event_category'],i,round(ratio,3),record['groups']['selected'],flush=True)
 (OUT/'posthoc-duration-probe.json').write_text(json.dumps({'rows':rows,'scope':'Posthoc fixed ratio from original timestamps; constant body twist approximation, no offset or scale fitting. Clock/exposure semantics unverified. No sensor motion estimate, fusion output, original score, or source depth is changed. This is reference-assisted diagnostic, not a deployable correction or physical timing qualification.'},indent=2)+'\n')
if __name__=='__main__':main()

"""Independent direct projection, depth-code and source-support checks."""
from pathlib import Path
import hashlib,json,math
import numpy as np,cv2
from pose_covariance_replay import exp_se3,log_se3
R=Path(__file__).resolve().parents[1];REC=R/'results/thermal-incremental-comparison/recovery';OUT=R/'results/fixed-group-diagnosis/run-001';DEP=OUT.parent/'recovery'
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
d=read(OUT/'diagnosis.json');probe=read(OUT/'posthoc-duration-probe.json');checks=0;nulls=0;depthchecks=0
for case in d['cases']:
 c,i=case['event']['condition'],case['event']['pair'];ref=np.array(read(REC/f'scores-{c}.json')['rows'][i]['reference_delta']);T=np.linalg.inv(ref);b=read(REC/f'binding-{c}.json')['rows'][i];times=b['actual_camera_times_ns'];ratio=(times['thermal_left'][1]-times['thermal_left'][0])/(times['rgb_left'][1]-times['rgb_left'][0]);S=exp_se3(log_se3(T)*ratio);assert np.allclose(exp_se3(log_se3(T)),T,rtol=0,atol=1e-10);assert np.allclose(exp_se3(log_se3(T)*1),T,rtol=0,atol=1e-10);nulls+=2
 e=next(x for x in probe['rows'] if x['event']['condition']==c and x['event']['pair']==i);q=np.load(REC/b['cameras']['thermal']['packet']);vals=[]
 for j in e['event']['removed_local_ids']:
  source=q['points'][j];target=S[:3,:3]@source+S[:3,3];camera=q['camera_R'][j]@target+q['camera_t'][j];fx,fy,cx,cy,_=q['Kb'][j];u=fx*camera[0]/camera[2]+cx;v=fy*camera[1]/camera[2]+cy;expected=q['observations'][j,:2];vals.append(math.sqrt((u-expected[0])**2+(v-expected[1])**2));checks+=1
 assert abs(float(np.median(vals))-e['groups']['selected']['duration_probe_uv_residual_median_px'])<1e-9
 source=read(REC/f'{c}-source.json')
 for cam,sensor in [('rgb','rgb'),('thermal','thr')]:
  for frame in [i,i+1]:assert sha(REC/'images'/c/sensor/f'{frame:06}.png')==source['records'][frame]['cameras'][sensor]['left']['sha256']
  assert sha(REC/b['cameras'][cam]['packet'])==b['cameras'][cam]['sha256']
 for row in case['cameras']['thermal']['rows']:
  z=row['source_provided_depth_m']
  if z is None:continue
  a=cv2.imread(str(DEP/'inputs'/c/f'{i:06}.png'),-1);x,y=[int(math.floor(t+.5)) for t in row['source_uv']];assert int(a[y,x])/256==z;depthchecks+=1
report={'cases':len(d['cases']),'independent_selected_point_projections':checks,'SE3_ratio1_null_checks':nulls,'exact_sparse_depth_decodes':depthchecks,'source_images_and_packets_unchanged':True,'no_exposure_clock_or_physicalGT_qualification':True};(OUT/'audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))

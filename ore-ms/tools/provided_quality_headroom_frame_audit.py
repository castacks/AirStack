"""Post-result independent source-graph check; does not alter EXP130 scoring."""
import collections,json
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from provided_quality_headroom import ROOT,OUT,read,write,sha,verify
p=verify();pred=read(OUT/'predictions.json');graph=collections.defaultdict(list)
for edge in yaml.safe_load((ROOT/'data/calibration/yamaha.yaml').read_text())['transform_params']:
 M=np.eye(4);M[:3,:3]=Rotation.from_quat(edge['quaternion']).as_matrix();M[:3,3]=edge['translation'];a,b=edge['from_frame'],edge['to_frame'];graph[a].append((b,M));graph[b].append((a,np.linalg.inv(M)))
results=[]
for w in p['windows']:
 for camera in ['rgb','thermal']:
  goal='multisense/left_camera_optical_frame' if camera=='rgb' else 'thermal_left/optical_frame';q=collections.deque([('vehicle',np.eye(4),['vehicle'])]);seen=set();found=None
  while q:
   frame,M,path=q.popleft()
   if frame in seen:continue
   seen.add(frame)
   if frame==goal:found=(M,path);break
   for child,T in graph[frame]:q.append((child,M@T,path+[child]))
  assert found is not None
  name='multisense_left_rect_color_camera_info' if camera=='rgb' else 'thermal_left_processed_camera_info'
  R=np.array(yaml.safe_load((ROOT/w['dataset']/'sensors'/name/'data.yaml').read_text())['r']);assert R.shape==(3,3) and np.linalg.det(R)>0 and np.max(np.abs(R.T@R-np.eye(3)))<=1e-6
  u,_,vt=np.linalg.svd(R);rect=u@vt;basis=np.eye(4);basis[:3,:3]=rect.T@np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
  expected=found[0]@basis;difference=float(np.max(np.abs(expected-pred['transforms'][w['case']][camera])));assert difference<=1e-12
  results.append({'case':w['case'],'camera':camera,'path':found[1],'max_absolute_difference':difference,'rectification_serialization_correction':float(np.max(np.abs(rect-R)))})
write(OUT/'frame-audit.json',{'pass':True,'checks':len(results),'rows':results,'physical_mounting_verified':False,'source_sha256':sha(__file__)})
print('PASS',len(results),'independent nominal graph/rectification/basis checks')

"""Explicitly exploratory support projection and motion/time nuisance checks."""
from pathlib import Path
import json,numpy as np
from scipy.stats import rankdata
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parents[1];O=R/'results/dynamic-thermal-evidence/run-001';REC=R/'results/thermal-incremental-comparison/recovery'
def read(p):return json.loads(Path(p).read_text())
def main():
 rows=[]
 for e in read(O/'illustrated-examples.json')['examples']:
  c,i=e['condition'],e['pair'];K=np.array(read(REC/f'{c}-source.json')['records'][i]['cameras']['rgb']['K']);z=[]
  for cam in ['rgb','thermal']:
   with np.load(REC/f'packets/{c}/{i:03}-{cam}.npz') as p:z.append(p['points'])
  a,b=z;ua=(a@K.T)[:,:2]/a[:,2,None];ub=(b@K.T)[:,:2]/b[:,2,None];nearest=np.linalg.norm(ub[:,None]-ua[None],axis=2).min(1);ids=e['removed_local_ids'];mask=np.ones(len(b),bool);mask[ids]=False;inside=(b[:,2]>0)&(ub[:,0]>=0)&(ub[:,0]<1224)&(ub[:,1]>=0)&(ub[:,1]<384)
  rows.append({**e,'selected_median_nearest_rgb_feature_pixels':float(np.median(nearest[ids])),'remaining_median_nearest_rgb_feature_pixels':float(np.median(nearest[mask])),'selected_nominal_rgb_in_bounds':int(inside[ids].sum()),'scope':'Posthoc nominal source projection and distance to ACCEPTED RGB feature samples, not physical co-visibility or absence of RGB texture.'})
 (O/'projected-rgb-support.json').write_text(json.dumps({'rows':rows,'posthoc':True},indent=2)+'\n')
 d=read(O/'primary-evidence.json');summary={}
 for c in ['day','night','rain']:
  rr=[x for x in d['rows'] if x['condition']==c and x['features'] is not None];pr=read(REC/f'predictions-{c}.json')['rows'];rotation=[Rotation.from_matrix(np.array(pr[x['pair']]['methods']['rgb_cov']['transform'])[:3,:3]).magnitude() for x in rr];X=np.c_[np.ones(len(rr)),rankdata([x['rgb_motion_m'] for x in rr]),rankdata(rotation),rankdata([x['pair'] for x in rr])];y=rankdata([x['translation_gain_m'] for x in rr]);ry=y-X@np.linalg.lstsq(X,y,rcond=None)[0];summary[c]={}
  for f in rr[0]['features']:
   xx=rankdata([x['features'][f] for x in rr]);rx=xx-X@np.linalg.lstsq(X,xx,rcond=None)[0];summary[c][f]=float(np.corrcoef(rx,ry)[0,1])
 (O/'posthoc-nuisance-check.json').write_text(json.dumps({'partial_rank_correlations':summary,'controls':['RGBestimatedtranslationnorm','RGBestimatedrotationangle','sourcepairindex'],'scope':'Posthoc descriptive nuisance check, not causal identification, independent prediction, or a deployedmodel.'},indent=2)+'\n')
if __name__=='__main__':main()

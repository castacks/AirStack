"""Secondary geometric association on longer, partly different saved recordings."""
from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
import dynamic_thermal_evidence as analysis
import nominal_joint_motion as geom
R=analysis.R;REC=R/'results/longer-fixed-thermal/recovery';PREV=R/'results/longer-fixed-thermal/run-001';OUT=analysis.OUT

def main():
 lock=analysis.read(PREV/'prediction-lock.json');rows=[]
 for c in ['day','night','rain']:
  entry=next(x for x in lock['rows'] if x['condition']==c);source=analysis.read(REC/'inputs'/c/'source-manifest.json');assert analysis.sha(REC/'inputs'/c/'source-manifest.json')==entry['source_manifest_sha256'];E=geom.geometry(analysis.read(REC/'inputs'/c/'calibration.json'));graphs={};events={}
  for cam in ['rgb','thermal']:
   with np.load(REC/'captures'/c/cam/'tensor-map.npz') as z:graphs[cam]={k:z[k] for k in z.files}
   events[cam]={}
   for line in (REC/'captures'/c/cam/'arbiter-events/events.jsonl').read_text().splitlines():
    e=json.loads(line)
    if e['kind']=='motion_result_written' and e['first_write_for_submission']:events[cam][e['from_idx'],e['frame_idx']]=e
  p=PREV/f'predictions-{c}.json';assert analysis.sha(p)==entry['prediction_sha256'];predictions=analysis.read(p)['rows'];prior=analysis.read(PREV/'scores.json')['summary'][c]['methods'];errs={m:{r['pair']:r for r in prior[m]['increment_errors']} for m in ['rgb','fixed_point1']}
  for row in predictions:
   i=row['pair'];new={'condition':c,'pair':i,'features':None}
   if i not in errs['rgb'] or i not in errs['fixed_point1']:new['reason']='NO_MATCHED_MOTION';rows.append(new);continue
   a,b=errs['rgb'][i],errs['fixed_point1'][i];gain=a['translation_m']-b['translation_m'];new.update(translation_gain_m=gain,rotation_gain_deg=a['rotation_deg']-b['rotation_deg'],category='helps' if gain>=.001 else 'hurts' if gain<=-.001 else 'neutral')
   try:
    ps=[geom.bind_camera(graphs[cam],events[cam].get((i,i+1)),i,source['records'],sensor,E[cam])[0] for cam,sensor in [('rgb','rgb'),('thermal','thr')]];T=np.asarray(row['methods']['rgb']['source_to_target_body']);aa,bb=analysis.stats(ps[0],T,(384,1224)),analysis.stats(ps[1],T,(256,640));new['features']=dict(zip(analysis.GEOMETRY,np.log((bb+1e-12)/(aa+1e-12)).tolist()))
   except ValueError as e:new['reason']=str(e)
   rows.append(new)
  print('SECONDARY',c,flush=True)
 primary=analysis.read(OUT/'primary-evidence.json')['associations'];secondary=analysis.associations(rows,analysis.GEOMETRY);comparison={c:{feature:{'primary_level':primary[c][feature]['spearman_level'],'secondary_level':secondary[c][feature]['spearman_level'],'primary_changes':primary[c][feature]['spearman_adjacent_changes'],'secondary_changes':secondary[c][feature]['spearman_adjacent_changes']} for feature in analysis.GEOMETRY} for c in ['day','night','rain']}
 analysis.write(OUT/'secondary-evidence.json',{'rows':rows,'associations':secondary,'comparison':comparison,'scope':'Exploratory corroboration; day/night recordings differ, rain extended; all exposed, no independent predictive-validation or environmental-cause claim. Newday/night images absent, image descriptors not invented.'})
if __name__=='__main__':main()

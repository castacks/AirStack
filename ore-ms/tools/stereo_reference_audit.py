"""EXP145 independent scalar relative-translation and raw correspondence audit."""
import json,hashlib,math
from copy import deepcopy
from pathlib import Path
import numpy as np
import provided_reference_full_record_audit as original
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/stereo-reference-diagnostic/run-001'
PRIOR=ROOT/'results/orb-finalization/run-001'
COUNT=0

def read(p):return json.loads(Path(p).read_text())
def need(v,label):
 global COUNT
 COUNT+=1
 if not v:raise ValueError(label)
def close(a,b,label):
 if a is None or b is None:need(a is b,label);return
 need(math.isclose(a,b,rel_tol=2e-7,abs_tol=2e-8),label)
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def norm(a):return math.sqrt(sum(x*x for x in a))
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def relative_translation(a,b):
 return [sum(a[k][c]*(b[k][3]-a[k][3]) for k in range(3)) for c in range(3)]
def deriv(g,e):
 gn=norm(g);en=norm(e);error=[y-x for x,y in zip(g,e)]
 parallel=dot(error,g)/gn if gn else None
 perpendicular=norm([error[k]-parallel*g[k]/gn for k in range(3)]) if gn else None
 return dict(reference_m=gn,prediction_m=en,ratio=en/gn if gn else None,cosine=dot(g,e)/(gn*en) if gn and en else None,parallel_error_m=parallel,perpendicular_error_m=perpendicular,error_m=norm(error))
def check_stats(record,values):
 need(record['n']==len(values),'full statistic population')
 if not values:
  need(all(record[k] is None for k in ['min','median','p90','max','rms']),'empty cohort retained');return
 close(record['min'],min(values),'minimum');close(record['max'],max(values),'maximum')
 close(record['median'],float(np.median(values)),'median');close(record['p90'],float(np.percentile(values,90)),'p90')
 close(record['rms'],math.sqrt(sum(x*x for x in values)/len(values)),'RMS')
def check_summary(s,rows):
 need(s['pairs']==len(rows),'pair population')
 for key in ['reference_m','prediction_m','ratio','cosine','parallel_error_m','perpendicular_error_m','error_m']:
  check_stats(s[key],[r[key] for r in rows if r[key] is not None])
 total=sum(r['error_m']**2 for r in rows)
 close(s['parallel_squared_error_share'],sum(r['parallel_error_m']**2 for r in rows if r['parallel_error_m'] is not None)/total if total else None,'parallel share')
 need(s['zero_reference_pairs']==sum(r['reference_m']==0 for r in rows),'zero motion exclusion')
def check_matches(s,matches,fx):
 dx=[v['left'][0]-v['right'][0] for v in matches];dy=[abs(v['left'][1]-v['right'][1]) for v in matches]
 need(s['matches']==len(matches),'all matches')
 need(s['positive_disparity']==sum(x>0 for x in dx),'left-minus-right positive disparity')
 need(s['within_vertical']==sum(y<=2 for y in dy),'fixed epipolar diagnostic')
 need(s['positive_and_vertical']==sum(x>0 and y<=2 for x,y in zip(dx,dy)),'joint condition')
 check_stats(s['signed_disparity_px'],dx);check_stats(s['abs_vertical_px'],dy);check_stats(s['abs_vertical_normalized'],[y/fx for y in dy])
def qualifiers(s):
 for k in ['physical_sync_verified','benchmark_execution_ready','scale_fitted','time_fitted','new_native_inference']:need(s[k] is False,'scope '+k)

def audit():
 plan=read(ROOT/'experiments/stereo-reference-diagnostic-2026-11-05.json')
 for n,h in plan['source_sha256'].items():need(sha(ROOT/n)==h,'source pin '+n)
 source=read(PRIOR/'source-manifest.json');need(len(source['records'])==2000,'fixed prefix2000')
 replay=read(OUT/'producer-replay.json');need(replay['passed'] is True and replay['opencv']=='4.6.0' and len(replay['images'])==80,'original producer replay80images')
 summary=read(OUT/'summary.json');qualifiers(summary);motion=read(OUT/'motion.json');images=read(OUT/'images.json');clocks=read(OUT/'clocks.json');contract=read(OUT/'contract.json');cal=read(ROOT/'results/ms2-night-pilot/calibration.json')
 need(len(motion['runs'])==6 and len(images['runs'])==40 and len(clocks['runs'])==2,'all declared populations')
 need(images['native_tracker_correspondences'] is False and images['physical_rectification_verified'] is False,'generic matches qualifier')
 for mode,m in [('rgb','rgb'),('thermal','thr')]:
  c=next(x for x in contract['runs'] if x['modality']==mode);k=cal['K_'+m+'L']['values'];b=-cal['T_'+m+'R']['values'][0][0]/1000
  close(c['baseline_m'],b,'mm to metres');close(c['bf'],b*k[0][0],'baseline focal product');close(c['metric_factor'],b/.1,'DROID units')
  need(c['K']==k==cal['K_'+m+'R']['values'],'supplied intrinsics equal')
  rows=source['records'];clock=next(x for x in clocks['runs'] if x['modality']==mode)
  l=[r['cameras'][m]['left']['recorded_time_ns'] for r in rows];r=[v['cameras'][m]['right']['recorded_time_ns'] for v in rows];delta=[a-b for a,b in zip(r,l)]
  need(clock['left_times_ns']==l and clock['right_times_ns']==r and clock['signed_right_minus_left_ns']==delta,'full exact stereo axis')
  check_stats(clock['signed_ms'],[x/1000000 for x in delta]);check_stats(clock['absolute_ms'],[abs(x)/1000000 for x in delta]);check_stats(clock['left_cadence_ms'],[(b-a)/1000000 for a,b in zip(l,l[1:])])
  need(clock['within_2ms']==sum(abs(x)<=2000000 for x in delta),'timing diagnostic unchanged')
  need(clock['right_before_left']==sum(x<0 for x in delta) and clock['right_after_left']==sum(x>0 for x in delta) and clock['equal']==sum(x==0 for x in delta),'signed times counts')
  need(clock['physical_exposure_offset_known'] is False,'no exposure assumption')
  refs=[original.rigid(np.loadtxt(ROOT/'results/provided-reference-full-record/run-001/reference'/source['sequence']/m/(v['stem']+'.txt')).reshape(4,4)) for v in rows]
  for arm in ['received','final','droid']:
   report=next(x for x in motion['runs'] if x['modality']==mode and x['arm']==arm);view=read(PRIOR/'scores'/mode/(arm+'-view.json'));score=read(PRIOR/'scores'/mode/(arm+'.json'))
   need([v['indices'] for v in report['rows']]==[v['source_indices'] for v in score['rpe_rows']],'all frozen scorer pairs')
   for row,old in zip(report['rows'],score['rpe_rows']):
    i,j=row['indices'];g=relative_translation(refs[i],refs[j]);e=relative_translation(original.rigid(view['poses'][i]),original.rigid(view['poses'][j]));want=deriv(g,e)
    need(row['duration_ns']==l[j]-l[i],'source duration')
    for key,val in want.items():close(row[key],val,'scalar decomposition '+key)
    close(want['error_m'],old['translation_m'],'unchanged RPE')
    if want['parallel_error_m'] is not None:close(want['parallel_error_m']**2+want['perpendicular_error_m']**2,want['error_m']**2,'Pythagorean decomposition')
   check_summary(report['summary'],report['rows'])
   need([ [v['start'],v['end_exclusive']] for v in report['bins']]==[[0,500],[500,1000],[1000,1500],[1500,2000]],'fixed bins')
   for v in report['bins']:check_summary(v['summary'],[x for x in report['rows'] if v['start']<=x['indices'][0]<v['end_exclusive']])
   need(summary['runs'][0 if mode=='rgb' else 1]['motion'][arm]==report['summary'],'summary motion exact')
  cohort=[v for v in images['runs'] if v['modality']==mode]
  need([v['index'] for v in cohort]==list(range(0,2000,100)),'fixed image sample')
  for v in cohort:
   need(v['stem']==rows[v['index']]['stem'],'sample identity')
   close(v['fx'],k[0][0],'normalization focal length')
   need(len({x['left_id'] for x in v['matches']})==len(v['matches'])==len({x['right_id'] for x in v['matches']}),'mutual one-to-one correspondences')
   for match in v['matches']:
    need(0<=match['left_id']<v['features_left'] and 0<=match['right_id']<v['features_right'],'keypoint index')
    need(0<=match['distance']<=256,'Hamming distance')
    for side in ['left','right']:
     h,w=rows[v['index']]['cameras'][m][side]['shape'][:2];x,y=match[side];need(0<=x<w and 0<=y<h,'image coordinates')
   check_matches(v['summary'],v['matches'],v['fx'])
  agg=summary['runs'][0 if mode=='rgb' else 1]
  need(agg['image_pairs']==20 and agg['empty_pairs']==sum(not v['matches'] for v in cohort),'empty samples retained')
  check_matches(agg['image_summary'],[x for v in cohort for x in v['matches']],k[0][0])
 mutations=0
 for kind in ['scope','units','motion','matches','population']:
  try:
   if kind=='scope':bad=deepcopy(summary);bad['physical_sync_verified']=True;qualifiers(bad)
   elif kind=='units':close(contract['runs'][0]['baseline_m']*1000,contract['runs'][0]['baseline_m'],'unit corruption')
   elif kind=='motion':bad=deepcopy(motion['runs'][0]['summary']);bad['error_m']['rms']+=.1;check_summary(bad,motion['runs'][0]['rows'])
   elif kind=='matches':bad=deepcopy(images['runs'][0]['summary']);bad['positive_disparity']+=1;check_matches(bad,images['runs'][0]['matches'],images['runs'][0]['fx'])
   else:check_summary(motion['runs'][0]['summary'],motion['runs'][0]['rows'][:-1])
  except ValueError:mutations+=1
  else:raise ValueError('missed corruption '+kind)
 need(mutations==5,'all semantic corruptions rejected')
 result=dict(passed=True,independent_checks=COUNT,mutations_rejected=mutations,auditor_sha256=sha(__file__),plan_sha256=sha(ROOT/'experiments/stereo-reference-diagnostic-2026-11-05.json'),scope='source pins, scalar motion decomposition, full pair/clock populations, raw coordinate summaries; not descriptor algorithm reimplementation')
 with (OUT/'independent-audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':audit()

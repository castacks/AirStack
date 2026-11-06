"""EXP150 independent raw native, common-support and metric reconstruction."""
import json,hashlib,math
from pathlib import Path
from copy import deepcopy
import numpy as np
import native_stereo_history_audit as native
import orb_finalization_audit as final
import provided_reference_full_record_audit as metric
import stereo_reference_audit as motion
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/stereo-baseline-initialization/run-001';COUNT=0
ARMS=['original','rebuilt','fixed']
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def need(v,label):
 global COUNT
 COUNT+=1
 if not v:raise ValueError(label)
def geometry(a,b):
 if a.get('available') is False or b.get('available') is False:return dict(comparable=False,coordinates_equal=None,accepted_changes=None,right_changes=None,depth_changes=None)
 x=a['features'];y=b['features'];same=len(x)==len(y) and all(u[:3]==v[:3] for u,v in zip(x,y))
 if not same:return dict(comparable=True,coordinates_equal=False,accepted_changes=None,right_changes=None,depth_changes=None)
 return dict(comparable=True,coordinates_equal=True,accepted_changes=sum((u[4]>0)!=(v[4]>0) for u,v in zip(x,y)),right_changes=sum(u[3]!=v[3] for u,v in zip(x,y)),depth_changes=sum(u[4]!=v[4] for u,v in zip(x,y)))
def identities():
 data=read(OUT/'runtime-identities.json');a=data['original']['native_sha256']
 for arm in ARMS[1:]:
  b=data[arm]['native_sha256'];changed=[k for k in a if a[k]!=b[k]]
  need((set(changed)<= {'/opt/ORB_SLAM3/lib/libORB_SLAM3.so'} if arm=='rebuilt' else set(changed)=={'/opt/ORB_SLAM3/lib/libORB_SLAM3.so','/opt/ORB_SLAM3/src/Frame.cc'}),'only intended runtime/source identities changed')
  need(data[arm]['system_header_sha256']==data['original']['system_header_sha256'],'observer header unchanged')
 original=(OUT/'source/Frame.original.cc').read_text();fixed=(OUT/'source/Frame.fixed.cc').read_text()
 begin=original.index('Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight,');end=original.index('Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth,',begin)
 ctor=original[begin:end];need(ctor.count('    mb = mbf/fx;\n')==1,'one original late assignment')
 expected=original[:begin]+ctor.replace('    mb = mbf/fx;\n','').replace('    ComputeStereoMatches();\n','    mb = mbf / mK.at<float>(0,0);\n    ComputeStereoMatches();\n')+original[end:]
 need(expected==fixed,'independent minimal constructor source diff')
 reports=read(OUT/'probes/driver.json');fixed_rows=[read(ROOT/r['output']) for r in reports if r['arm']=='fixed' and r['returncode']==0]
 need(len(fixed_rows)==3 and sorted(r['pattern'] for r in fixed_rows)==[0,63,85],'fixed all storage patterns complete')
 need(all(r['features']==fixed_rows[0]['features'] for r in fixed_rows),'constructor storage invariance')
 fs=fixed_rows[0]['features'];ds=sorted(v[0]-v[3] for v in fs if v[4]>0)
 need(len(ds)>0 and abs(float(np.median(ds))-4)<.5,'known synthetic disparity')
 need(all(math.isclose(r['baseline_m'],.3,rel_tol=1e-6) for r in fixed_rows),'first-frame current-intrinsics baseline')
def audit():
 plan=read(ROOT/'experiments/stereo-baseline-initialization-2026-11-05.json');lock=read(OUT/'prediction-lock.json')
 need(sha(ROOT/'experiments/stereo-baseline-initialization-2026-11-05.json')==sha(OUT/'study-lock.json')==lock['study_sha256'],'study/prediction binding')
 for n,h in {**plan['source_sha256'],**lock['source_sha256']}.items():need(sha(ROOT/n)==h,'frozen source/output '+n)
 identities();summary=read(OUT/'summary.json');source=read(OUT/'source-manifest.json');need(len(source['records'])==2000,'complete source prefix')
 for key in ['physical_sync_verified','benchmark_execution_ready','scale_fitted','time_fitted','default_runtime_changed']:need(summary[key] is False,'scope qualifier')
 reports=[];sample=None;sampleplan=None
 for mode,m in [('rgb','rgb'),('thermal','thr')]:
  entry=next(r for r in summary['runs'] if r['modality']==mode);streams=[];names=[]
  for arm in ARMS:
   d=OUT/'captures'/(mode+'-'+arm);p=read(d/'plan.json');status=read(d/'run-status.json');received=read(d/'received.json')
   need(status['terminal_success'] is True and status['source_frames']==2000,'all one-attempt calls complete')
   rows=[r for r in map(json.loads,(d/'native.jsonl').read_text().splitlines()) if 'index' in r];need(len(rows)==2000,'native source population')
   active=False;sid=-1;last=None
   for i,r in enumerate(rows):
    need(r['index']==i and r['time_ns']==p['time_ns'][i] and r['right_time_ns']==p['right_time_ns'][i],'exact received clocks')
    if r['T_camera_from_world'] is None:need(received['poses'][i] is None and received['segments'][i] is None,'received hole');active=False;continue
    t=np.asarray(r['T_camera_from_world']);inv=np.eye(4);inv[:3,:3]=t[:3,:3].T;inv[:3,3]=-t[:3,:3].T@t[:3,3]
    need(np.allclose(inv,received['poses'][i],rtol=0,atol=1e-10),'received rigid transpose')
    if not active or r['map_id']!=last:sid+=1
    need(received['segments'][i]==sid,'native map/hole boundaries');active=True;last=r['map_id']
   report=final.export_audit(d);report.update(modality=mode,arm=arm);reports.append(report)
   saved=read(OUT/'analysis'/mode/(arm+'-frames.json'))['rows'];frames=[]
   with (d/'native.jsonl.stereo.jsonl').open() as f:
    for i,line in enumerate(f):
     row=json.loads(line);v=native.native_frame(row,p,i);need(v==saved[i],'independent complete feature summary');frames.append(v)
     if sample is None and row.get('available') is not False:sample=row;sampleplan=p
   need(len(frames)==len(saved)==2000,'all stereo observation slots');s=entry['native_stats'][arm]
   need(s['available']==sum(v['features'] is not None for v in frames) and s['missing_indices']==[v['index'] for v in frames if v['features'] is None],'unavailable observations explicit')
   need(s['observed_features']==sum(v['features'] or 0 for v in frames) and s['observed_accepted']==sum(v['accepted'] or 0 for v in frames),'feature/acceptance population')
   for key,st in s['frame_medians'].items():motion.check_stats(st,[v[key] for v in frames if v[key] is not None])
   for stage in ['received','final']:streams.append(read(d/(stage+'.json')));names.append(arm+'-'+stage)
  mask=[all(s['poses'][i] is not None for s in streams) for i in range(2000)];segments=[];active=False;sid=-1;last=None
  for i,ok in enumerate(mask):
   if not ok:segments.append(None);active=False;continue
   label=[s['segments'][i] for s in streams]
   if not active or label!=last:sid+=1
   segments.append(sid);active=True;last=label
  need(entry['common_pose_frames']==sum(mask),'all-six common support')
  reflock=read(ROOT/'results/provided-reference-full-record/run-001/reference-lock.json');raw=[]
  for r in source['records']:
   f=ROOT/'results/provided-reference-full-record/run-001/reference'/source['sequence']/m/(r['stem']+'.txt');need(sha(f)==reflock['member_sha256'][str(f.relative_to(ROOT))],'raw reference pin');raw.append(np.loadtxt(f).reshape(4,4))
  motions=read(OUT/'analysis'/mode/'motion.json')
  for name,original in zip(names,streams):
   for scope in ['own','common']:
    view=read(OUT/'scores'/mode/scope/(name+'-view.json'));score=read(OUT/'scores'/mode/scope/(name+'.json'))
    if scope=='own':need(view==original,'own support and poses untouched')
    else:need(view['poses']==[v if ok else None for v,ok in zip(original['poses'],mask)] and view['segments']==segments,'common support union boundaries and untouched poses')
    need(view['time_ns']==original['time_ns'] and view['source_ids']==original['source_ids'],'score source axis')
    metric.compare(score,metric.independently_derive(view['time_ns'],raw,view))
    need(entry['scores'][scope+'-'+name]=={k:v for k,v in score.items() if k not in ['rpe_rows','segment_ate','reference_exclusions']},'score summary binding')
    if scope=='common':
     mo=motions[name];need([r['indices'] for r in mo['rows']]==[r['source_indices'] for r in score['rpe_rows']],'motion pairs')
     for r in mo['rows']:
      i,j=r['indices'];g=motion.relative_translation(metric.rigid(raw[i]),metric.rigid(raw[j]));e=motion.relative_translation(metric.rigid(view['poses'][i]),metric.rigid(view['poses'][j]));want=motion.deriv(g,e)
      for k,v in want.items():motion.close(r[k],v,'independent scalar '+k)
     motion.check_summary(mo['summary'],mo['rows']);need(entry['motion_summary'][name]==mo['summary'],'motion summary binding')
     need([[b['start'],b['end_exclusive']] for b in mo['bins']]==[[0,500],[500,1000],[1000,1500],[1500,2000]],'prespecified bins')
     for b in mo['bins']:motion.check_summary(b['summary'],[r for r in mo['rows'] if b['start']<=r['indices'][0]<b['end_exclusive']])
  for a,b in [('original','rebuilt'),('rebuilt','fixed'),('original','fixed')]:
   label=a+'-vs-'+b;saved=read(OUT/'analysis'/mode/(label+'.json'))['rows'];expected=[]
   with (OUT/'captures'/(mode+'-'+a)/'native.jsonl.stereo.jsonl').open() as fa,(OUT/'captures'/(mode+'-'+b)/'native.jsonl.stereo.jsonl').open() as fb:
    for i,(la,lb) in enumerate(zip(fa,fb)):v=geometry(json.loads(la),json.loads(lb));v['index']=i;need(v==saved[i],'independent feature differences');expected.append(v)
   need(len(expected)==len(saved)==2000,'comparison source population');s=entry['stereo_comparisons'][label]
   for key,field in [('accepted_changes','accepted_changes'),('right_changes','right_changes'),('depth_changes','depth_changes')]:need(s[key]==sum(v[field] or 0 for v in expected),'difference totals')
   need(s['coordinate_mismatches']==sum(v['coordinates_equal'] is False for v in expected) and s['comparable']==sum(v['comparable'] for v in expected),'comparison availability/coordinates')
   need(s['changed_source_indices']==[v['index'] for v in expected if v['right_changes'] or v['depth_changes']],'changed source indices')
 mutations=0
 for kind in ['depth','sentinel','population','clock']:
  bad=deepcopy(sample)
  if kind=='depth':bad['features'][next(i for i,v in enumerate(bad['features']) if v[4]>0)][4]*=2
  elif kind=='sentinel':bad['features'][0][3]=-2
  elif kind=='population':bad['features'].pop()
  else:bad['time_ns']+=1
  try:native.native_frame(bad,sampleplan,bad['index'])
  except ValueError:mutations+=1
  else:raise ValueError('undetected corruption '+kind)
 result=dict(passed=True,independent_checks=COUNT+native.COUNT+final.COUNT+metric.COUNT+motion.COUNT,mutations_rejected=mutations,score_populations=24,runs=reports,auditor_sha256=sha(__file__),physical_sync_verified=False,benchmark_execution_ready=False)
 with (OUT/'independent-audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':audit()

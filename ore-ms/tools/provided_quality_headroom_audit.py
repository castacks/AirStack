"""Independent EXP130 population, source-motion, interpolation and oracle audit."""
import collections,json,sys,zlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from provided_quality_headroom import ROOT,OUT,PRIOR,PLAN,read,write,sha,verify,scientific

def main():
 p=verify();scientific();checks=0
 def check(value,label):
  nonlocal checks
  assert value,label
  checks+=1
 prediction=read(OUT/'predictions.json');scores=read(OUT/'scores.json');lock=read(OUT/'prediction-lock.json')
 check(lock['predictions_sha256']==sha(OUT/'predictions.json'),'prediction lock')
 expected={(r['case'],r['pair']) for r in read(PRIOR/'native-survey.json')['rows'] if r['mask']=='both'}
 check(len(expected)==323 and {(r['case'],r['pair']) for r in prediction['rows']}==expected,'fixed joint population')
 check([(r['case'],r['pair']) for r in scores['rows']]==[(r['case'],r['pair']) for r in prediction['rows']],'all scored/excluded row IDs retained')
 for phase in ['metadata','poses']:
  for report in read(OUT/(phase+'-intake-lock.json'))['reports']:
   for n,item in report['files'].items():
    b=(ROOT/n).read_bytes();check(sha(ROOT/n)==item['sha256'] and len(b)==item['bytes'] and zlib.crc32(b)&0xffffffff==item['crc32'],'CRC/SHA')
 windows={w['case']:w for w in p['windows']};logs={};references={}
 from decimal import Decimal
 for case,w in windows.items():
  logs[case]=[json.loads(l) for l in (PRIOR/'captures'/case/'timeline.jsonl').read_text().splitlines()]
  clip=w['clip']
  if clip not in references:
   d=OUT/'reference'/clip/'super_odometry/odometry';t=np.array([int((Decimal(s)*1000000000).to_integral_value()) for s in (d/'interp_timestamps.txt').read_text().splitlines()],dtype=np.int64);poses=np.loadtxt(d/'interp_data.txt');references[clip]=(t,poses)
 for r,s in zip(prediction['rows'],scores['rows']):
  w=windows[r['case']];i=r['pair'];common=[w['rows'][j]['rgb_left']['time_ns'] for j in [i,i+1]];check(r['common_interval_ns']==common,'common acquisition endpoints')
  for c,e in r['experts'].items():
   times=[w['rows'][j][c+'_left']['time_ns'] for j in [i,i+1]]
   matches=[x for x in logs[r['case']] if x['kind']=='observer_event' and x['camera']==c and x['event']['kind']=='motion_result_written' and x['event']['first_write_for_submission'] and x['event']['time1_ns']==times[1]];check(len(matches)==1,'unique native motion')
   m=matches[0]['event'];check([m['time0_ns'],m['time1_ns']]==times and m['stage']=='pair_processing' and m['original_need_interp'] is False,'native motion provenance')
   v=m['derived_camera_delta_xyzw'];D=np.eye(4);D[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();D[:3,3]=v[:3];T=np.array(prediction['transforms'][r['case']][c]);body=T@D@np.linalg.inv(T)
   check(np.allclose(body,e['body_delta'],atol=1e-12,rtol=0),'body conjugation')
   # Independent matrix exponential/logarithm verifies SE(3) duration rescaling.
   from scipy.linalg import expm,logm
   ratio=(common[1]-common[0])/(times[1]-times[0]);model=expm(logm(body)*ratio)
   check(e['valid_model_projection'] and np.allclose(model,e['delta_model'],atol=1e-9,rtol=0),'independent screw projection')
  if s['exclusion'] is not None:continue
  t,poses=references[w['clip']];targets=[]
  for time in common:
   j=int(np.searchsorted(t,time));check(0<j<len(t),'reference interpolation bracket')
   fraction=(time-int(t[j-1]))/int(t[j]-t[j-1]);check(t[j]-t[j-1]<=p['max_reference_gap_ns'],'reference gap')
   M=np.eye(4);M[:3,3]=poses[j-1,:3]+fraction*(poses[j,:3]-poses[j-1,:3]);M[:3,:3]=Slerp([0.,1.],Rotation.from_quat(poses[j-1:j+1,3:7]))([fraction]).as_matrix()[0];targets.append(M)
  target=np.linalg.inv(targets[0])@targets[1];check(np.allclose(target,s['reference_delta'],atol=1e-10,rtol=0),'independent reference endpoint interpolation')
  for c,e in r['experts'].items():
   P=np.array(e['delta_model']);tr=float(np.linalg.norm(P[:3,3]-target[:3,3]));angle=float(np.degrees(Rotation.from_matrix(target[:3,:3].T@P[:3,:3]).magnitude()))
   check(abs(tr-s['loss'][c]['translation_m'])<1e-10 and abs(angle-s['loss'][c]['rotation_deg'])<1e-10,'independent residual loss')
 def aggregate(rows,reported):
  check(len(rows)==reported['rows'],'summary matched denominator')
  for metric in ['translation_m','rotation_deg']:
   a=np.array([r['loss']['rgb'][metric] for r in rows]);b=np.array([r['loss']['thermal'][metric] for r in rows]);q=reported[metric]
   check(np.isclose(a.mean(),q['rgb_mean']) and np.isclose(b.mean(),q['thermal_mean']),'fixed means')
   check(np.isclose(np.minimum(a,b).mean(),q['oracle_mean']) and q['oracle_mean']<=min(a.mean(),b.mean())+1e-12,'oracle bound')
   check(np.isclose(min(a.mean(),b.mean())-np.minimum(a,b).mean(),q['oracle_reduction']),'headroom definition')
   check(sum(q['winners'].values())==len(rows),'winner denominator')
 valid=[r for r in scores['rows'] if r['exclusion'] is None];aggregate(valid,scores['aggregate'])
 for case,q in scores['groups'].items():aggregate([r for r in valid if r['case']==case],q)
 for clip,q in scores['recordings'].items():aggregate([r for r in valid if windows[r['case']]['clip']==clip],q)
 # Semantic checks use analytic motion and oracle examples, independent of results.
 from Scripts.Setup.causal_interval_model import project
 from Scripts.Setup.td3_reference import PoseReference
 from provided_quality_headroom import summarize
 D=np.eye(4);D[0,3]=2.;check(np.allclose(project(D,[0,10],[0,5])['delta_model'],np.diag([1.,1.,1.,1.])+np.array([[0,0,0,1],[0,0,0,0],[0,0,0,0],[0,0,0,0]])),'analytic duration scaling')
 check(not project(D,[0,10],[30,40])['valid_model_projection'],'projection horizon refusal')
 ref=PoseReference([0,10],[[0,0,0,0,0,0,1],[2,0,0,0,0,0,1]])
 check(np.isclose(ref.at([5])[0,0,3],1.),'analytic reference interpolation')
 rejected=False
 try:ref.at([-1])
 except ValueError:rejected=True
 check(rejected,'reference extrapolation refusal')
 fixture=[{'case':'x','pair':i,'loss':{c:{m:v for m in ['translation_m','rotation_deg']} for c,v in zip(['rgb','thermal'],values)}} for i,values in enumerate([(1.,3.),(3.,1.)])]
 q=summarize(fixture);check(q['translation_m']['oracle_reduction']==1. and q['rotation_deg']['adjacent_winner_changes']==1,'alternating quality oracle')
 write(OUT/'audit.json',{'pass':True,'checks':checks,'prediction_sha256':sha(OUT/'predictions.json'),'scores_sha256':sha(OUT/'scores.json'),'plan_sha256':sha(PLAN),'independent_projection':'matrix log/exponential','semantic_checks':5})
 print('PASS',checks,'independent checks')
if __name__=='__main__':main()

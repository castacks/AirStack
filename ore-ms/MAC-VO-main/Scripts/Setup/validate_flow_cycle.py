"""Independent raw-corner, eligibility, fixed-choice and proxy-population audit."""
import argparse,hashlib,itertools,json,math
from pathlib import Path
import numpy as np
POLICIES=('rgb','thermal','variance','closure')
CASES={'early':'end-to-end-solver-control','middle':'solver-collapse-replication','day':'solver-day-replication'}
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def close(a,b):
 if a is None or b is None:assert a is None and b is None
 else:np.testing.assert_allclose(a,b,rtol=5e-12,atol=1e-12)
def moment(values,reported):
 a=np.array(values);assert reported['count']==len(a);close(reported['mean'],float(a.mean()) if len(a) else None);close(reported['rmse'],float(np.sqrt(np.mean(a*a))) if len(a) else None)


def check_summary(meta,expected):
 for key,value in expected.items():
  if isinstance(value,list):
   assert len(meta[key])==len(value)
   for a,b in zip(meta[key],value):close(a,b)
  else:close(meta[key],value)


def capture_checks(root,output):
 p=read(output/'inference.json');execution=read(output/'execution.json');assert execution['status']=='PASS_ALL_FORWARD_CONTROLS' and execution['inference_sha256']==sha(output/'inference.json')
 for section in ('source_sha256','input_sha256','checkpoint_sha256'):
  for path,h in p[section].items():assert sha(root/path)==h
 evidence={};points=valid_points=reverse=forward=0;first_summary=None
 for spec in p['runs']:
  window,camera=spec['window'],spec['camera'];folder=root/spec['output_relative'];cap=read(folder/'capture.json');assert cap['status']=='PASS_CAPTURE_CONTROLS' and cap['label_guard_enabled'] and not cap['denied_label_reads'];n=spec['frames']-1
  assert cap['completed_pairs']==cap['forward_calls']==cap['reverse_calls']==len(cap['pairs'])==n and cap['initial_depth_calls']==2;reverse+=cap['reverse_calls'];forward+=cap['forward_calls']
  manifest=read(root/spec['dataset_relative']/'manifest.json')['pairs'];events=[json.loads(s) for s in (root/spec['original_relative']/'arbiter-events/events.jsonl').read_text().splitlines()];refine=min(e['sequence'] for e in events if e['kind']=='refinement_begin')
  for i in range(n):
   meta=cap['pairs'][i];assert meta['pair_ordinal']==i and meta['forward_bitwise'];times=[manifest[j][camera+'_left']['time_ns'] for j in (i,i+1)];assert meta['sensor_interval_ns']==times
   with np.load(root/spec['original_relative']/'uncertainty'/f'pair-{i:03d}.npz',allow_pickle=False) as old,np.load(folder/f'pair-{i:03d}.npz',allow_pickle=False) as a:
    old={key:old[key] for key in old.files};a={key:a[key] for key in a.files}
    for new,key in [('uv0','grid_uv0'),('uv1','grid_uv1'),('original_inbound','grid_inbound'),('K','K'),('raw_covariance','grid_raw_flow_cov'),('height_width','height_width')]:assert a[new].dtype==old[key].dtype and a[new].shape==old[key].shape and a[new].tobytes()==old[key].tobytes()
    assert a['time_ns'].tolist()==times and int(old['time0_ns'])==times[0];h,w=a['height_width'];uv=a['uv1'];sampled=[];inside=[];valid=[]
    digest=str(a['reverse_field_sha256']);assert len(digest)==64 and all(c in '0123456789abcdef' for c in digest)
    for index,(x,y) in enumerate(uv):
     is_inside=bool(np.isfinite([x,y]).all() and 0<=x<=w-1 and 0<=y<=h-1);inside.append(is_inside)
     if not is_inside:
      assert (a['indices'][index]==-1).all() and np.isnan(a['corners'][index]).all() and np.isnan(a['weights'][index]).all();sampled.append([np.nan,np.nan]);valid.append(False);continue
     x,y=float(x),float(y);x0,y0=math.floor(x),math.floor(y);x1,y1=min(x0+1,w-1),min(y0+1,h-1);dx,dy=x-x0,y-y0
     expected=[[x0,y0],[x1,y0],[x0,y1],[x1,y1]];assert a['indices'][index].tolist()==expected
     weights=[(1-dx)*(1-dy),dx*(1-dy),(1-dx)*dy,dx*dy];assert np.all(np.abs(a['weights'][index]-weights)<=1e-12+5e-12*np.abs(weights));corners=a['corners'][index].astype(float);ok=bool(np.isfinite(corners).all());valid.append(ok)
     sampled.append([math.fsum(weights[j]*corners[j,c] for j in range(4)) for c in range(2)] if ok else [np.nan,np.nan])
    sampled=np.array(sampled);inside=np.array(inside);sample_valid=np.array(valid);assert np.array_equal(a['in_bounds'],inside) and np.array_equal(a['sample_valid'],sample_valid);close(a['sampled'],sampled)
    closure=np.array(uv,dtype=float)+sampled-a['uv0'];mask=sample_valid&a['original_inbound']&np.isfinite(closure).all(1);assert np.array_equal(mask,a['closure_valid']);close(a['closure'],closure);k=a['K'].reshape(3,3);angle=np.sqrt(np.square(closure[:,0]/float(k[0,0]))+np.square(closure[:,1]/float(k[1,1])));pixel=np.sqrt(np.square(closure).sum(1))
    cov=old['grid_raw_flow_cov'];cov_valid=np.isfinite(cov).all(1)&(cov[:,0]>0)&(cov[:,1]>0);trace=cov[:,0]/k[0,0]**2+cov[:,1]/k[1,1]**2
    q=lambda x:np.percentile(x,[25,50,75]).tolist() if len(x) else [None]*3
    expected={'grid_count':len(uv),'original_inbound_count':int(a['original_inbound'].sum()),'valid_closure_count':int(mask.sum()),'valid_fraction':float(mask.mean()),'closure_angular_q25_q50_q75':q(angle[mask]),'closure_pixel_q25_q50_q75':q(pixel[mask]),'covariance_valid_count':int(cov_valid.sum()),'angular_covariance_median':float(np.median(trace[cov_valid])) if cov_valid.any() else None};check_summary(meta,expected);points+=len(uv);valid_points+=int(mask.sum())
   if first_summary is None:first_summary=(meta,expected)
   health,=[e for e in events if e['kind']=='pair_end' and [e['time0_ns'],e['time1_ns']]==times]
   motions=[e for e in events if e['kind']=='motion_result_written' and e['first_write_for_submission'] and e['from_idx']==i and e['frame_idx']==i+1 and [e['time0_ns'],e['time1_ns']]==times];assert len(motions)<=1
   available=bool(motions and not health['original_need_interp'] and health['execution_error'] is None and motions[0]['sequence']<refine and not motions[0]['original_need_interp'] and motions[0]['derived_delta_unavailable_reason'] is None)
   evidence[window,camera,i]={'motion_available':available,'motion_stage':motions[0]['stage'] if motions else None,'sensor_interval_ns':times,'closure_median':meta['closure_angular_q25_q50_q75'][1],'angular_covariance_median':expected['angular_covariance_median'],'valid_closure_count':expected['valid_closure_count'],'grid_count':expected['grid_count']}
 assert forward==reverse==434
 return evidence,{'grid_points_checked':points,'valid_closure_points':valid_points,'forward_controls':forward,'reverse_pairs':reverse},first_summary


def output_checks(root,output,evidence):
 pred=read(output/'predictions.json');ep=read(output/'evaluation-protocol.json');evaluated=read(output/'evaluation.json');pp=read(output/'prediction-protocol.json')
 assert pred['prediction_protocol_sha256']==sha(output/'prediction-protocol.json') and evaluated['evaluation_protocol_sha256']==sha(output/'evaluation-protocol.json')
 assert not pred['labels_used'] and not pred['fitted'] and pred['label_guard_enabled'] and not pred['denied_label_reads'];assert not pp['labels_used'] and not pp['fitted'] and not ep['fit_performed'] and not ep['independent_ground_truth'] and not evaluated['fit_performed'] and not evaluated['independent_ground_truth']
 for protocol in (pp,ep):
  for path,h in protocol['input_sha256'].items():assert sha(root/path)==h
 assert len(pred['rows'])==len(evaluated['rows'])==217;lookup={(r['window'],r['pair_ordinal']):r for r in pred['rows']};assert len(lookup)==217;rows={(r['window'],r['pair_ordinal']):r for r in evaluated['rows']};assert set(rows)==set(lookup)
 for window,previous in CASES.items():
  old=read(root/'results'/previous/'run-001/evaluation.json');proof=read(root/'results'/previous/'run-001/validation.json');assert proof['status']=='PASS' and proof['evaluation_sha256']==sha(root/'results'/previous/'run-001/evaluation.json');labels={(r['camera'],r['pair_ordinal']):r for r in old['rows'] if r['arm']=='original'};ids=list(range(len(labels)//2));w=evaluated['windows'][window];assert w['all_slots']==len(ids)
  for i in ids:
   p=lookup[window,i];r=rows[window,i];expected_choices={}
   for c in ('rgb','thermal'):assert p['cameras'][c]==evidence[window,c,i] and r['cameras'][c]==p['cameras'][c]
   for policy in POLICIES:
    if policy in ('rgb','thermal'):chosen=policy if evidence[window,policy,i]['motion_available'] else None
    else:
     key='closure_median' if policy=='closure' else 'angular_covariance_median';scores=[evidence[window,c,i][key] if evidence[window,c,i]['motion_available'] and evidence[window,c,i][key] is not None else np.inf for c in ('rgb','thermal')];chosen=('rgb','thermal')[int(np.argmin(scores))] if np.isfinite(scores).any() else None
    expected_choices[policy]=chosen
   assert p['choices']==r['choices']==expected_choices;assert r['common_interval_ns']==labels['rgb',i]['common_interval_ns'];assert r['camera_losses']=={c:labels[c,i]['common_loss'] for c in ('rgb','thermal')}
   for policy,c in expected_choices.items():assert r['policy_losses'][policy]==(labels[c,i]['common_loss'] if c else None)
  both=[i for i in ids if all(evidence[window,c,i]['motion_available'] for c in ('rgb','thermal'))];union=[i for i in ids if any(evidence[window,c,i]['motion_available'] for c in ('rgb','thermal'))];processing=[i for i in both if all(evidence[window,c,i]['motion_stage']=='pair_processing' for c in ('rgb','thermal'))]
  for population,ids_ in [('both',both),('union',union),('both_processing_only',processing)]:
   g=w['populations'][population];assert g['slots']==len(ids_) and g['pair_ordinals']==ids_
   for policy in POLICIES:
    selected=[i for i in ids_ if rows[window,i]['choices'][policy] is not None];f=g['full'][policy];assert f['available']==len(selected) and f['missing']==len(ids_)-len(selected) and f['pair_ordinals']==selected
    assert f['choices']=={str(c) if c is not None else 'null':sum(rows[window,i]['choices'][policy]==c for i in ids_) for c in ('rgb','thermal',None)}
    for loss in ('translation_m','rotation_deg'):moment([rows[window,i]['policy_losses'][policy][loss] for i in selected],f['losses'][loss])
   for a,b in itertools.combinations(POLICIES,2):
    selected=[i for i in ids_ if all(rows[window,i]['choices'][p] is not None for p in (a,b))];m=g['matched'][a+'/'+b];assert m['pair_ordinals']==selected
    for p in (a,b):
     for loss in ('translation_m','rotation_deg'):moment([rows[window,i]['policy_losses'][p][loss] for i in selected],m[p][loss])
 return {'prediction_slots_checked':217,'policy_choices_checked':868}


def validate(root,output):
 evidence,capture,first=capture_checks(root,output);result={'status':'PASS',**capture,**output_checks(root,output,evidence),'evaluation_sha256':sha(output/'evaluation.json'),'predictions_sha256':sha(output/'predictions.json'),'inference_sha256':sha(output/'inference.json'),'validator_sha256':sha(Path(__file__)),'scope':'independent cached-corner/mask/closure/action/loss arithmetic; dense reverse field identified by hash, not independently re-inferred; no physical/online claim'}
 (output/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(result)


if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();validate(a.root,a.output)

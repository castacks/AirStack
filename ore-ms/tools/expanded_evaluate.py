"""Fixed common cohorts with source-referenced covariance drift; no selection."""
import os
os.environ['CUBLAS_WORKSPACE_CONFIG']=':4096:8'
from pathlib import Path
import datetime,hashlib,json,math
import numpy as np
import torch
from Train.MatchingNet.retraining_runner import build_model,configure
from Train.MatchingNet.retraining_manifest import load_pair,sha256
from Train.MatchingNet.covariance_contract import sequence_covariance_nll
ROOT=Path('/root/AirStack/ore-ms');OUT=ROOT/'results/expanded-fixed-model-evaluation/run-001';plan=json.loads((OUT/'protocol.json').read_text());manifest=json.loads((ROOT/plan['training_manifest_relative']).read_text());options=manifest['optimization'];evaluation_manifest=json.loads((ROOT/plan['evaluation_manifest']['path']).read_text());assert evaluation_manifest['task']=='frozen_covariance_recording_evaluation' and evaluation_manifest['optimizer_steps']==0;common=evaluation_manifest['samples'];assert len(common)==128
torch.manual_seed(0);torch.cuda.manual_seed_all(0);torch.set_num_threads(1);torch.use_deterministic_algorithms(True);torch.backends.cudnn.benchmark=False
produced=json.loads((OUT/'produced-model-pins.json').read_text())
for pin in produced.values(): assert sha256(ROOT/pin['path'])==pin['sha256']
models={}
for arm in plan['models']:
 model,_=build_model(ROOT,manifest,'cuda')
 if arm!='baseline':
  state=torch.load(ROOT/plan['models'][arm]['path'],weights_only=True,map_location='cuda');assert state['step']==48;model.load_state_dict(state['model'],strict=True)
 configure(model,training=False);models[arm]=model
def digest(model):
 h=hashlib.sha256()
 for name,tensor in model.state_dict().items():
  h.update(name.encode());h.update(str(tensor.dtype).encode());h.update(str(tuple(tensor.shape)).encode());h.update(tensor.detach().cpu().contiguous().numpy().tobytes())
 return h.hexdigest()
state_before={arm:digest(model) for arm,model in models.items()}
results={a:[] for a in models}; capture=[]; (OUT/'arrays').mkdir(exist_ok=False)
with torch.no_grad():
 for j,sample in enumerate(common):
  pair={k:v.to('cuda') for k,v in load_pair(ROOT,sample).items()};valid=(pair['valid_mask'][:,0]&torch.isfinite(pair['flow']).all(dim=1)&(torch.linalg.vector_norm(pair['flow'],dim=1)<options['max_flow'])).cpu().numpy();gt=pair['flow'].cpu().numpy().astype(np.float64).transpose(0,2,3,1)[valid];teacher=[];teacher_means=[]; arrays={'target':gt.astype(np.float32),'flat_index':np.flatnonzero(valid)}
  for arm,model in models.items():
   means,raw=model(pair['image1'],pair['image2']);rec=sequence_covariance_nll(means,raw,pair['flow'],pair['valid_mask'],gamma=options['gamma'],max_flow=options['max_flow']);final=sequence_covariance_nll([means[-1]],[raw[-1]],pair['flow'],pair['valid_mask'],gamma=options['gamma'],max_flow=options['max_flow']);stages=[];reference=0.
   for i,(mean,cov) in enumerate(zip(means,raw)):
    a=mean.cpu().numpy();b=cov.cpu().numpy();prediction=a.astype(np.float64).transpose(0,2,3,1)[valid];r=b.astype(np.float64).transpose(0,2,3,1)[valid];mean_hash=hashlib.sha256(a.tobytes()).hexdigest()
    if arm=='baseline':teacher.append(r.copy());teacher_means.append(mean_hash)
    assert mean_hash==teacher_means[i];difference=r-teacher[i];kl=.5*(np.exp(-2*difference)+2*difference-1);assert np.isfinite(kl).all() and kl.mean()>=-1e-12
    alternate_kl=.5*(np.exp(2*teacher[i])/np.exp(2*r)+np.log(np.exp(2*r)/np.exp(2*teacher[i]))-1);assert np.allclose(kl,alternate_kl,rtol=1e-8,atol=1e-12)
    nll=float((.5*((prediction-gt)**2/np.exp(2*r)+2*r)).mean());weight=options['gamma']**(len(raw)-i-1);reference+=weight*nll;stages.append({'stage':i,'weight':weight,'numpy64_nll':nll,'mean_sha256':mean_hash,'raw_sha256':hashlib.sha256(b.tobytes()).hexdigest(),'teacher_to_student_kl':float(kl.mean()),'raw_shift_mse':float((difference**2).mean()),'raw_shift_mean':float(difference.mean()),'mean_exp_minus_two_shift':float(np.exp(-2*difference).mean())})
   assert math.isclose(rec.loss.item(),reference,rel_tol=2e-5,abs_tol=2e-6);assert math.isclose(final.loss.item(),stages[-1]['numpy64_nll'],rel_tol=2e-5,abs_tol=2e-6);weights=sum(s['weight'] for s in stages)
   if arm=='baseline': arrays['prediction']=prediction.astype(np.float32)
   arrays[arm]=r.astype(np.float32)
   results[arm].append({'cohort':sample['cohort'],'recording_id':sample['recording_id'],'sample_sha256':sample['file']['sha256'],'valid_components':rec.valid_components,'valid_mask_sha256':hashlib.sha256(valid.tobytes()).hexdigest(),'cuda32_recurrent_nll':rec.loss.item(),'numpy64_recurrent_nll':reference,'cuda32_final_nll':final.loss.item(),'numpy64_final_nll':stages[-1]['numpy64_nll'],'weighted_kl':sum(s['weight']*s['teacher_to_student_kl'] for s in stages)/weights,'weighted_raw_shift_mse':sum(s['weight']*s['raw_shift_mse'] for s in stages)/weights,'final_kl':stages[-1]['teacher_to_student_kl'],'stages':stages})

  array_file=OUT/'arrays'/f'{j:02d}.npz'; np.savez_compressed(array_file,**arrays);capture.append({'index':j,'recording_id':sample['recording_id'],'cohort':sample['cohort'],'sample_sha256':sample['file']['sha256'],'pixels':len(gt),'path':str(array_file.relative_to(ROOT)),'sha256':sha256(array_file)})
  print('evaluated',j,sample['recording_id'],flush=True)

state_after={arm:digest(model) for arm,model in models.items()};assert state_before==state_after
assert all(parameter.grad is None for model in models.values() for parameter in model.parameters())
keys=('cuda32_recurrent_nll','numpy64_recurrent_nll','cuda32_final_nll','numpy64_final_nll','weighted_kl','weighted_raw_shift_mse','final_kl')
output={}
for arm,rows in results.items():
 per_record={}
 for record in evaluation_manifest['records']:
  identity=record['recording_id'];selected=[row for row in rows if row['recording_id']==identity];assert len(selected)==record['pairs'];components=sum(row['valid_components'] for row in selected)
  per_record[identity]={'pairs':len(selected),'valid_components':components,'cohort':record['cohort'],**{key:sum(row[key]*row['valid_components'] for row in selected)/components for key in keys}}
 groups={}
 for group in ('expanded_only',):
  records=[identity for identity,record in per_record.items() if group=='all_three' or record['cohort']==group];selected=[row for row in rows if row['recording_id'] in records];components=sum(row['valid_components'] for row in selected)
  groups[group]={'recordings':records,'record_count':len(records),'pairs':len(selected),'valid_components':components,'macro':{key:sum(per_record[identity][key] for identity in records)/len(records) for key in keys},'micro':{key:sum(row[key]*row['valid_components'] for row in selected)/components for key in keys}}
 output[arm]={'rows':rows,'per_record':per_record,'groups':groups};print(arm,json.dumps(groups),flush=True)
result={'status':'PASS_FROZEN_MULTIRECORD_EVALUATION','created_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'protocol_sha256':sha256(OUT/'protocol.json'),'evaluation_manifest_sha256':sha256(ROOT/plan['evaluation_manifest']['path']),'arms':output,'model_state_digest_before':state_before,'model_state_digest_after':state_after,'optimizer_steps':0,'model_exported':False,'primary':plan['primary'],'scope':'frozen syntheticRGBprovided-reference evaluation;unknownbasepretrainingoverlap;no thermal/physical/adoptionclaim'}
with (OUT/'evaluation.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')

with (OUT/'capture.json').open('x') as f: json.dump({'rows':capture,'state_before':state_before,'state_after':state_after,'optimizer_steps':0},f,indent=2);f.write('\n')

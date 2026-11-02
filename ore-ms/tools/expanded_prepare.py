"""Native provided-flow packing and evidence-backed exposure accounting."""
from pathlib import Path
import json
import numpy as np
from PIL import Image
from Scripts.Setup.prepare_tartanair_flow import prepare_pair
from Scripts.Setup.audit_recording_exposure import validate_ledger
from Train.MatchingNet.retraining_manifest import load_pair,inspect_manifest,sha256
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/expanded-fixed-model-evaluation/run-001'
p=json.loads((OUT/'data-protocol.json').read_text());acq=json.loads((OUT/'acquisition.json').read_text());assert acq['status']=='PASS_DECLARED_MEMBER_ACQUISITION' and len(acq['members'])==392
pin=lambda path:{'path':str(path.relative_to(ROOT)),'sha256':sha256(path)}
for row in acq['members']:assert sha256(ROOT/row['path'])==row['sha256']
ledger=json.loads((ROOT/p['previous_ledger']['path']).read_text());validate_ledger(ledger,ROOT);before=json.loads(json.dumps(ledger['records']));assert len(before)==28
training=json.loads((ROOT/p['training_manifest_relative']).read_text());old_eval=json.loads((ROOT/'experiments/frozen-multirecord-evaluation-2026-11-01.json').read_text());prior_eval=json.loads((ROOT/'experiments/prospective-objective-evaluation-2026-11-02.json').read_text());old_samples=training['samples']+old_eval['samples']+prior_eval['samples'];old_ids={s['recording_id'] for s in old_samples}
old_images=set()
for old in old_samples:
 with np.load(ROOT/old['file']['path'],allow_pickle=False) as arrays:
  for key in ('image1','image2'):
   import hashlib
   old_images.add(hashlib.sha256(arrays[key].tobytes()).hexdigest())
records=[];samples=[];proof=[];quotes=[];checks=0;new_images=set()
for case in p['cases']:
 env,traj,identity=case['environment'],case['trajectory'],case['recording_id'];assert identity not in old_ids
 source=ROOT/p['raw_root_relative']/env/'Easy'/traj;members=[r for r in acq['members'] if r['member'].startswith(f'{env}/Easy/{traj}/')];assert len(members)==49
 evidence={'recording_id':identity,'role':'expanded_frozen_candidate_evaluation_only','label_kind':'provided_dense_temporal_flow','protocol':pin(OUT/'data-protocol.json'),'acquisition':pin(OUT/'acquisition.json'),'members':members,'base_pretraining_membership':'unknown'};f=OUT/(env+'-label-evidence.json')
 with f.open('x') as stream:json.dump(evidence,stream,indent=2);stream.write('\n')
 quote=f"EXP108 decoded {identity} first17leftRGBimages and16providedforwardflow/mask pairs only for prospective frozen-candidate evaluation; source {f.relative_to(ROOT)} SHA256 {sha256(f)}";quotes.append(quote)
 ledger['records'].append({'id':identity,'dataset':'tartanair_v1','aliases':[identity+'/'+name for name in ('left','right','image_left','image_right')],'unit_kind':'temporal_recording','exposure_status':'development_exposed','image_observation':'observed','reference_observation':'observed first16providedforwardflow/mask pairs','MACT_Offroad_training_membership':'unknown','RGB_checkpoint_training_membership':'unknown','evidence':[{'source':'expanded_objective_ingestion','quote':quote}],'scope_note':'prospective evaluation of alreadyfrozen candidates;not added to finetuning;unknownbasepretraining'})
 records.append({'recording_id':identity,'cohort':'expanded_only','pairs':16,'label_evidence':pin(f)})
 folder=ROOT/p['prepared_root_relative']/env/traj;folder.mkdir(parents=True,exist_ok=False)
 for i in range(case['pairs']):
  raw=[source/'image_left'/f'{i:06d}_left.png',source/'image_left'/f'{i+1:06d}_left.png',source/'flow'/f'{i:06d}_{i+1:06d}_flow.npy',source/'flow'/f'{i:06d}_{i+1:06d}_mask.npy'];target=folder/f'{i:06d}_{i+1:06d}.npz';stats=prepare_pair(*raw,target);assert stats['shape']==[480,640]
  with np.load(target,allow_pickle=False) as arrays:
   for j,key in ((0,'image1'),(1,'image2')):
    with Image.open(raw[j]) as image:assert image.mode=='RGB' and image.size==(640,480);expected=(np.asarray(image).astype(np.float32)/np.float32(255)).transpose(2,0,1)
    assert arrays[key].dtype==np.float32 and np.array_equal(arrays[key],expected);checks+=3
    digest=hashlib.sha256(arrays[key].tobytes()).hexdigest();assert digest not in old_images;new_images.add(digest)
   flow=np.load(raw[2],allow_pickle=False);mask=np.load(raw[3],allow_pickle=False);assert arrays['flow'].dtype==np.float32 and np.array_equal(arrays['flow'],flow.transpose(2,0,1),equal_nan=True);checks+=3
   assert arrays['valid_mask'].dtype==np.bool_ and np.array_equal(arrays['valid_mask'],(mask==0)[None]);checks+=2
  sample={'recording_id':identity,'cohort':'expanded_only','file':pin(target)};pair=load_pair(ROOT,sample);flow=pair['flow'].numpy();valid=pair['valid_mask'].numpy()[:,0]&np.isfinite(flow).all(axis=1)&(np.linalg.norm(flow,axis=1)<400);assert valid.any();samples.append(sample);proof.append({'sample':sample,'raw':list(map(pin,raw)),**stats,'admitted_pixels':int(valid.sum())})
source=OUT/'development-exposure-source.txt';source.write_text('Prospective frozen-candidate observations\n'+'\n'.join(quotes)+'\n');ledger['sources']['expanded_objective_ingestion']={'repository_path':str(source.relative_to(ROOT)),'commit':p['source_commit'],'sha256':sha256(source),'snapshot_relative':str(source.relative_to(ROOT))};ledger.update(as_of='2026-11-02',scope='old28records unchanged plus8prospectivefrozenmodel evaluation trajectories; nowdevelopmentexposed,nottraining');assert ledger['records'][:28]==before;validate_ledger(ledger,ROOT)
ledger_path=ROOT/'experiments/recording-exposure-ledger-2026-11-02-expanded-fixed-models.json'
with ledger_path.open('x') as stream:json.dump(ledger,stream,indent=2);stream.write('\n')
recipe={'name':'unchangedtartanair-v1nativeRGBunit/forwardflow/source-mask0valid','original_recipe':training['input_recipe'],'expanded_evaluation_protocol':pin(OUT/'data-protocol.json'),'transforms_changed':False};recipe_path=ROOT/'experiments/expanded-fixed-model-recipe-2026-11-02.json'
with recipe_path.open('x') as stream:json.dump(recipe,stream,indent=2);stream.write('\n')
manifest={'schema_version':1,'task':'frozen_covariance_recording_evaluation','models':p['models'],'ledger':pin(ledger_path),'recipe':pin(recipe_path),'source_protocol':pin(OUT/'data-protocol.json'),'records':records,'samples':samples,'optimizer_steps':0,'training_assignment_created':False,'base_pretraining_membership':'unknown'};manifest_path=ROOT/'experiments/expanded-fixed-model-evaluation-2026-11-02.json'
with manifest_path.open('x') as stream:json.dump(manifest,stream,indent=2);stream.write('\n')
assert len(samples)==128
try:inspect_manifest(ROOT,manifest_path)
except ValueError as error:rejection=str(error)
else:raise AssertionError('evaluation manifest must not train')
result={'status':'PASS_PROSPECTIVE_DATA_ADMISSION','manifest':pin(manifest_path),'old28ledgerrecords_unchanged':True,'ledger_record_count':36,'independent_array_checks':checks,'new_array_fields_exact':512,'distinct_new_images':len(new_images),'old_image_hash_overlap':0,'training_record_overlap':False,'training_manifest_rejection':rejection,'samples':proof,'optimizer_steps':0,'protected_reserves_accessed':False}
with (OUT/'data-validation.json').open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
print('PASS',len(samples),'pairs',checks,'arraychecks',len(new_images),'distinctimages',flush=True)

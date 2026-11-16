"""Independent learned-control filter, coverage and score audit."""
from collections import Counter
import hashlib,json,statistics
from pathlib import Path
import numpy as np
from audit_smokeseer_sift import evaluate,angle_rotations

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/smokeseer-learned'
def read(p):return json.loads(p.read_text())
def sha(p):
 with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def main():
 checks=0;freeze=read(OUT/'execution-freeze.json');lock=read(OUT/'prediction-lock.json')
 for name,digest in freeze['pins'].items():assert sha(ROOT/name)==digest,name;checks+=1
 assert sha(OUT/'execution-freeze.json')==lock['execution_freeze_sha256']
 assert sha(OUT/'predictions.jsonl')==lock['predictions_sha256']
 assert sha(OUT/'journal.jsonl')==lock['journal_sha256']
 assert lock['rows']==lock['inference_batches']==3465 and lock['directional_pairs']==6930 and lock['model_states_unchanged']
 # Whole input population recheck after inference; SIFT normalization stays fixed.
 for member,pin in read(OUT/'gray-pins.json').items():assert sha(ROOT/pin['path'])==pin['sha256'];checks+=1
 preds=[json.loads(x) for x in (OUT/'predictions.jsonl').read_text().splitlines()]
 journal=[json.loads(x) for x in (OUT/'journal.jsonl').read_text().splitlines()]
 assert sorted(journal,key=lambda x:x['slot'])==preds
 references=read(ROOT/'results/smokeseer-preflight/declared-pairs.json');scores=read(OUT/'scores.json');summary=read(OUT/'summary.json')
 assert len(preds)==len(references)==len(scores)==3465
 axis=np.rint(np.linspace(8,791,64));grid=np.array([(x,y) for y in axis for x in axis]);assert grid.shape==(4096,2)
 keys=set();computed=[]
 for ref,pred,score in zip(references,preds,scores):
  key=(ref['scene'],ref['arm'],ref['index']);assert key not in keys;keys.add(key)
  for name in ['scene','arm','index','source','target']:assert ref[name]==pred[name];checks+=1
  assert pred['slot']==len(keys)-1 and pred['query_budget']==4096
  packet=ROOT/pred['packet'];assert sha(packet)==pred['packet_sha256']
  with np.load(packet,allow_pickle=False) as z:
   np.testing.assert_array_equal(z['grid'],grid)
   assert z['forward'].shape==z['target'].shape==z['source_covariance'].shape==(4096,2)
   assert np.isfinite(z['forward']).all() and np.isfinite(z['source_covariance']).all() and (z['source_covariance']>0).all()
   target=grid+z['forward'];np.testing.assert_array_equal(target,z['target'])
   inside=np.logical_and.reduce([np.all(np.isfinite(target),axis=1),np.min(target,axis=1)>=0,np.max(target,axis=1)<=799])
   np.testing.assert_array_equal(inside,z['inside']);ids=np.flatnonzero(inside);np.testing.assert_array_equal(ids,z['inside_ids'])
   assert z['backward_at_target'].shape==(len(ids),2) and np.isfinite(z['backward_at_target']).all()
   delta=z['forward'][ids]+z['backward_at_target'];cycle=np.sqrt(np.einsum('ij,ij->i',delta,delta))
   np.testing.assert_allclose(cycle,z['cycle'],atol=1e-12,rtol=1e-12)
   accepted=ids[cycle<=1];np.testing.assert_array_equal(accepted,z['accepted_ids'])
   np.testing.assert_array_equal(grid[accepted],np.asarray(pred['points0']).reshape(-1,2))
   np.testing.assert_array_equal(target[accepted],np.asarray(pred['points1']).reshape(-1,2))
   assert len(accepted)==pred['matches']==pred['cycle_accepted'] and int(inside.sum())==pred['inside']
   checks+=13
  if pred['status']=='ok':
   r=np.array(pred['R']);t=np.array(pred['t']);np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-8)
   assert abs(np.linalg.det(r)-1)<1e-8 and abs(np.linalg.norm(t)-1)<1e-8
   assert pred['matches']>=8 and pred['cheirality_inliers']>=8
   assert sum(bool(x) for x in pred['pose_mask'])==pred['cheirality_inliers']
   assert all(not bool(a) or bool(b) for a,b in zip(pred['pose_mask'],pred['essential_mask']));checks+=5
  supported,primary,joint,rerr,terr=evaluate(pred,ref)
  assert (score['supported'],score['primary_success'],score['joint_success'])==(supported,primary,joint)
  for value,name in [(rerr,'rotation_error_deg'),(terr,'translation_error_deg')]:
   assert score[name] is None if value is None else abs(value-score[name])<1e-5
   checks+=1
  identity=angle_rotations(np.eye(3),np.asarray(ref['source_to_next_optical'])[:3,:3])<=5
  assert score['identity_success']==identity
  computed.append(dict(scene=ref['scene'],arm=ref['arm'],status=pred['status'],supported=supported,primary=primary,joint=joint,rerr=rerr,terr=terr,identity=identity));checks+=2
 for arm in ['rgb_smoke','rgb_clean','thermal']:
  for scene in [None]+sorted({x['scene'] for x in computed}):
   group=[x for x in computed if x['arm']==arm and (scene is None or x['scene']==scene)];sup=[x for x in group if x['supported']]
   stored=summary['pooled'][arm] if scene is None else summary['scenes'][arm][scene]
   values={'pairs':len(group),'supported':len(sup),'primary_success':sum(x['primary'] for x in group),'joint_success':sum(x['joint'] for x in group),'identity_success':sum(x['identity'] for x in group)}
   for k,v in values.items():assert stored[k]==v;checks+=1
   assert stored['status_counts']==dict(Counter(x['status'] for x in group))
   for field,count in [('primary_rate','primary_success'),('joint_rate','joint_success'),('support_rate','supported')]:assert abs(stored[field]-values[count]/len(group))<1e-12;checks+=1
   for field,metric in [('conditional_rotation_median_deg','rerr'),('conditional_translation_median_deg','terr')]:
    value=statistics.median(x[metric] for x in sup) if sup else None
    assert stored[field] is None if value is None else abs(stored[field]-value)<1e-5;checks+=1
  for field in ['primary_rate','support_rate','joint_rate']:
   assert abs(summary['macro'][arm][field]-statistics.mean(x[field] for x in summary['scenes'][arm].values()))<1e-12;checks+=1
 # Explicit boundary/sign guards, independent of estimator outputs.
 assert np.linalg.norm([1.,0.])<=1 and not np.linalg.norm([1.0001,0.])<=1
 identity=np.eye(4);identity[0,3]=1;ref={'source_to_next_optical':identity.tolist(),'translation_direction_defined':True}
 assert evaluate({'status':'ok','R':np.eye(3).tolist(),'t':[1,0,0]},ref)[2]
 assert not evaluate({'status':'ok','R':np.eye(3).tolist(),'t':[-1,0,0]},ref)[2]
 assert not evaluate({'status':'insufficient_matches'},ref)[1]
 result={'status':'pass','checks':checks,'guards':4,'rows':3465,'sparse_packets':3465,'scope':'All pins/input hashes, canonical/journal coverage, complete fixed query/filter evidence, supported poses and independent angular scores/scene denominators/macro rates; full dense fields are not archived','prediction_sha256':sha(OUT/'predictions.jsonl')}
 with (OUT/'audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(json.dumps(result,indent=2))

if __name__=='__main__':main()

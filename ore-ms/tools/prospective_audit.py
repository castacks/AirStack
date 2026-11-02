"""Independent raw-data, exposure, fixed-model, support and score audit."""
from pathlib import Path
import copy
import hashlib
import json
import math
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/prospective-objective-evaluation/run-001'
p=json.loads((OUT/'data-protocol.json').read_text());manifest=json.loads((ROOT/'experiments/prospective-objective-evaluation-2026-11-02.json').read_text());data=json.loads((OUT/'data-validation.json').read_text());checks=0

def check(value,label):
 global checks
 if not value:raise AssertionError(label)
 checks+=1

def close(a,b,label):check(math.isclose(a,b,rel_tol=2e-12,abs_tol=2e-12),label)

for name,digest in p['protected_pins'].items():check(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name)
for pin in p['models'].values():check(hashlib.sha256((ROOT/pin['path']).read_bytes()).hexdigest()==pin['sha256'],'modelpin')
check(manifest['optimizer_steps']==0 and not manifest['training_assignment_created'],'evalonly')
check(len(manifest['samples'])==len(data['samples'])==32,'complete32')
old=json.loads((ROOT/p['previous_ledger']['path']).read_text());ledger=json.loads((ROOT/manifest['ledger']['path']).read_text());check(ledger['records'][:26]==old['records'] and len(ledger['records'])==28,'ledgerpreservation')
new=[r['id'] for r in ledger['records'][26:]];check(new==[c['recording_id'] for c in p['cases']],'canonicalrecords')
old_ids={r['id'] for r in old['records']};check(not old_ids.intersection(new),'recorddisjointness')
expected=[]
for sample,proof in zip(manifest['samples'],data['samples']):
 check(sample==proof['sample'],'sampleorder')
 for pin in [sample['file'],*proof['raw']]:check(hashlib.sha256((ROOT/pin['path']).read_bytes()).hexdigest()==pin['sha256'],'inputpin')
 raw=[ROOT/v['path'] for v in proof['raw']]
 with np.load(ROOT/sample['file']['path'],allow_pickle=False) as arrays:
  check(set(arrays.files)=={'image1','image2','flow','valid_mask'},'arrayschema')
  for j,key in enumerate(['image1','image2']):
   with Image.open(raw[j]) as image:
    rgb=np.array(image,dtype=np.uint8);check(rgb.shape==(480,640,3),'nativeRGB')
   reference=np.moveaxis(rgb.astype('float32')/np.float32(255),-1,0)
   check(np.array_equal(reference,arrays[key]),'independentpixels')
  flow=np.load(raw[2],allow_pickle=False);mask=np.load(raw[3],allow_pickle=False)
  check(flow.dtype==np.float32 and mask.dtype==np.uint8,'nativeformats')
  check(np.array_equal(np.moveaxis(flow,-1,0),arrays['flow'],equal_nan=True),'providedflow')
  check(np.array_equal(mask==0,arrays['valid_mask'][0]),'providedsupport')
  norm=np.sqrt(np.square(flow.astype('float32')).sum(-1));valid=(mask==0)&np.isfinite(flow).all(-1)&(norm<400)
  check(valid.any() and int(valid.sum())==proof['admitted_pixels'],'nonemptysupport')
  expected.append({'sample':sample,'target':flow[valid],'flat_index':np.flatnonzero(valid)})
phase='data';mutations=[]
if (OUT/'metrics.json').exists():
 phase='evaluation';m=json.loads((OUT/'metrics.json').read_text());e=json.loads((OUT/'evaluation.json').read_text());capture=json.loads((OUT/'capture.json').read_text());old_e=json.loads((ROOT/'results/objective-comparison/run-001/evaluation.json').read_text())
 check(e['model_state_digest_before']==e['model_state_digest_after']==old_e['model_state_digest_after'],'fixedmodelstate');check(len(capture['rows'])==len(m['rows'])==32,'everypair')
 for j,(row,mr,reference) in enumerate(zip(capture['rows'],m['rows'],expected)):
  check(row['sample_sha256']==reference['sample']['file']['sha256'] and row['recording_id']==reference['sample']['recording_id'],'referenceaccounting')
  path=ROOT/row['path'];check(hashlib.sha256(path.read_bytes()).hexdigest()==row['sha256'],'savedarraypin')
  with np.load(path,allow_pickle=False) as a:
   check(np.array_equal(a['target'],reference['target']) and np.array_equal(a['flat_index'],reference['flat_index']),'evaluationtargetsupport')
   residual=a['prediction'].astype('float64')-a['target'].astype('float64');loss=np.einsum('ij,ij->i',residual,residual);n=len(loss);check(n==row['pixels']==mr['pixels'],'population')
   for arm in p['models']:
    raw=a[arm].astype('float64');variance=np.exp(raw*2);q=np.sum(residual**2/variance,1);trace=variance.sum(1);entry=mr['arms'][arm]
    final=float(np.mean(residual**2/(2*variance)+raw));close(final,entry['final_nll'],'arrayNLL');close(final,e['arms'][arm]['rows'][j]['numpy64_final_nll'],'torchreferenceNLL')
    for level in p['coverage_levels']:check(int(np.count_nonzero(q<=-2*math.log1p(-level)))==entry['coverage_counts'][str(level)],'coverage')
    close(float(q.sum()),entry['mahalanobis_sum'],'qsum')
    for axis in range(2):
     standardized=residual[:,axis]/np.sqrt(variance[:,axis]);close(float(standardized.sum()),entry['normalized_sum'][axis],'standardizedbias');close(float(np.dot(standardized,standardized)),entry['normalized_squared_sum'][axis],'standardizedsecondmoment')
    for i,f in enumerate(p['retained_fractions']):
     k=max(1,int(f*n));cutoff=np.partition(trace,k-1)[k-1];before=np.flatnonzero(trace<cutoff);ties=np.flatnonzero(trace==cutoff);chosen=np.r_[before,ties[:k-len(before)]];risk=entry['risks'][i]
     check(risk['retained']==len(chosen)==k,'budget');close(float(loss[chosen].sum()),risk['selected_error_sum'],'rankedsum');close(float(np.partition(loss,k-1)[:k].sum()),risk['oracle_error_sum'],'oraclesum');close(float(np.mean(loss)*k),risk['random_expected_error_sum'],'randomexpectation')
    for stage,base in zip(e['arms'][arm]['rows'][j]['stages'],e['arms']['baseline']['rows'][j]['stages']):check(stage['mean_sha256']==base['mean_sha256'],'sharedmeans')
 def aggregation_check(metrics):
  for identity,record in metrics['records'].items():
   rr=[r for r in metrics['rows'] if r['recording_id']==identity];count=sum(r['pixels'] for r in rr)
   for arm in p['models']:
    for i in range(5):
     k=sum(r['arms'][arm]['risks'][i]['retained'] for r in rr)
     for key,source in [('selected_mse','selected_error_sum'),('oracle_mse','oracle_error_sum'),('random_mse','random_expected_error_sum')]:close(sum(r['arms'][arm]['risks'][i][source] for r in rr)/k,record['arms'][arm]['risks'][i][key],'recordrisk')
    for level in p['coverage_levels']:close(sum(r['arms'][arm]['coverage_counts'][str(level)] for r in rr)/count,record['arms'][arm]['coverage'][str(level)],'recordcoverage')
  group=metrics['groups']['prospective_only'];check(len(group['recordings'])==2,'tworecordmacro')
  for arm in p['models']:
   for i in range(5):
    for key in ('selected_mse','oracle_mse','random_mse'):close(sum(metrics['records'][identity]['arms'][arm]['risks'][i][key] for identity in group['recordings'])/2,group['arms'][arm]['risks'][i][key],'riskmacro')
   for level in p['coverage_levels']:close(sum(metrics['records'][identity]['arms'][arm]['coverage'][str(level)] for identity in group['recordings'])/2,group['arms'][arm]['coverage'][str(level)],'coveragemacro')
 aggregation_check(m)
 for name in ('primarymacro','coverage95','retainedpopulation'):
  corrupt=copy.deepcopy(m)
  if name=='primarymacro':corrupt['groups']['prospective_only']['arms']['final_trace_risk_score']['risks'][2]['selected_mse']+=.01
  elif name=='coverage95':corrupt['records'][next(iter(corrupt['records']))]['arms']['baseline']['coverage']['0.95']+=.01
  else:corrupt['rows'][0]['arms']['baseline']['risks'][2]['retained']+=100
  try:aggregation_check(corrupt)
  except AssertionError:mutations.append({'mutation':name,'rejected':True})
  else:raise AssertionError('semantic mutation escaped: '+name)
with (OUT/('audit-'+phase+'.json')).open('x') as stream:json.dump({'status':'PASS','checks':checks,'mutations':mutations,'optimizer_steps':0,'old26ledgerrecords_unchanged':True,'protected_reserves_accessed':False},stream,indent=2);stream.write('\n')
print('PASS',phase,checks,'checks',len(mutations),'mutations',flush=True)

"""Reexecute the frozen predictor with guarded reads and in-memory writes only."""
import copy,contextlib,io
import signal_transfer as study
original_read,original_write=study.read,study.write
captured=[];access=[];labels=original_read(study.OUT/'labels.json')
def run(poisoned):
 captured.clear();access.clear()
 def guarded_read(path):
  path=study.Path(path);access.append(str(path))
  if path==study.OUT/'labels.json':return copy.deepcopy(poisoned)
  assert path.name not in ['scores.json','evaluation.json'],'prediction attempted outcome access'
  return original_read(path)
 def in_memory_write(path,value):captured.append((str(path),copy.deepcopy(value)))
 study.read,study.write=guarded_read,in_memory_write
 try:
  with contextlib.redirect_stdout(io.StringIO()):study.predict()
 finally:study.read,study.write=original_read,original_write
 predictions=next(v for path,v in captured if path==str(study.OUT/'predictions.json'))
 assert str(study.OUT/'labels.json') not in access,'prediction attempted transfer-label read'
 return predictions,list(access)
first,reads=run(labels);poisoned=copy.deepcopy(labels)
for row in poisoned['rows']:
 if row['exclusion'] is None:
  row['loss']['rgb']['rotation_deg']=1e9;row['loss']['thermal']['rotation_deg']=0.
second,again=run(poisoned)
assert first==second==original_read(study.OUT/'predictions.json') and reads==again
original_write(study.OUT/'causal-audit.json',{'pass':True,'checks':3,'semantic_mutations':1,'predictor_reexecutions':2,'transfer_labels_read':False,'outcome_file_reads':False,'read_paths':reads,'source_sha256':study.sha(__file__),'predictions_sha256':study.sha(study.OUT/'predictions.json')})
print('PASS guarded predictor reexecution; evaluation-label poisoning invariant; original outputs untouched')

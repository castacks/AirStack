"""Past-only inputs, analytical ridge and recording exclusion are executable contracts."""
import copy
import numpy as np
from Scripts.Setup.future_sensor_risk import current_history,fit_fold,predict,bind_target

def fixture():
 rows=[];labels={}
 for rec in ['a','b','c']:
  for i in range(6):
   sid=rec+str(i);rows.append({'recording_id':rec,'forecast_id':sid,'target_id':sid,'current':[float(i),1.],'history':[float(i),1.,float(i%2)]});labels[sid]={'rgb':{'translation_m':.01+i*.01},'thermal':{'translation_m':.04}}
 return rows,labels

def test_record_exclusion_and_training_scaler_invariance():
 rows,labels=fixture();model=fit_fold(rows,labels,'c','current');other=copy.deepcopy(rows);mutated=copy.deepcopy(labels)
 for r in other:
  if r['recording_id']=='c':r['current']=[1e12,-1e12];del mutated[r['target_id']]
 assert fit_fold(other,mutated,'c','current')==model
 assert set(model['training_recordings'])=={'a','b'} and model['gradient_inf']<1e-12
 np.testing.assert_allclose(model['mean'],[2.5,1.]);assert model['scale'][1]==1.
 assert not any(x.startswith('c') for x in model['train_ids'])

def test_analytic_ridge_against_augmented_lstsq():
 rows,labels=fixture();model=fit_fold(rows,labels,'c','history');train=[r for r in rows if r['recording_id']!='c'];x=np.asarray([r['history'] for r in train]);z=(x-model['mean'])/model['scale'];w=np.asarray(model['weights']);y=np.asarray([np.log1p(labels[r['target_id']]['rgb']['translation_m']/.1)-np.log1p(labels[r['target_id']]['thermal']['translation_m']/.1) for r in train]);design=np.c_[z,np.ones(len(z))];reg=np.c_[np.eye(3),np.zeros(3)];a=np.r_[np.sqrt(w)[:,None]*design,reg];b=np.r_[np.sqrt(w)*y,np.zeros(3)];expected=np.linalg.lstsq(a,b,rcond=None)[0]
 np.testing.assert_allclose(np.r_[model['coefficients'],model['intercept']],expected,atol=1e-12)

def test_prediction_reads_no_files_or_labels(monkeypatch):
 rows,labels=fixture();model=fit_fold(rows,labels,'c','current')
 import builtins
 monkeypatch.setattr(builtins,'open',lambda *a,**k:(_ for _ in ()).throw(AssertionError('prediction I/O')))
 assert predict(model,[1.,1.])['action'] in ['RGB','THERMAL']

def test_covariance_history_order_and_unused_fields():
 state={c:{'available':True,'information':10.,'retained':10,'cells':9,'fraction':.8} for c in ['rgb','thermal']};v={c:[.04,.01,.04] for c in state};dt={c:[.2,.1,.1] for c in state};a,b=current_history(state,v,dt);assert len(a)==16 and len(b)==20
 np.testing.assert_allclose(b[-4:],[0,np.log(4),0,np.log(4)],atol=1e-12)
 for c in state:state[c]['future_error']=1e99
 assert current_history(state,v,dt)==(a,b)

def test_target_matching_no_overlap_tie_earlier_and_tolerance():
 rows=[{'rgb_left':{'time_ns':i*100_000_000}} for i in range(20)]
 assert bind_target(rows,2,500_000_000)==8
 assert bind_target(rows,2,550_000_000)==8
 assert bind_target(rows,18,500_000_000) is None

def test_no_training_keeps_abstention():
 rows,labels=fixture();model=fit_fold(rows,{},'a','current');assert predict(model,[0,1])=={'action':'ABSTAIN','margin':None}

"""Causal features, analytic fitting contract and whole-group exclusion."""
import copy,math
import numpy as np
import pytest
from Scripts.Setup.temporal_quality_gate import prefix_features,event_values,FAMILIES,cost_weights,objective,fit,fit_fold,choose


def fixture():
 rows=[{c+'_left':{'time_ns':1000+i*100} for c in ('rgb','thermal')} for i in range(5)]
 events={c:[] for c in ('rgb','thermal')}
 for c in events:
  for i in range(4):events[c].append({'kind':'pair_end','sequence':i,'time0_ns':1000+i*100,'time1_ns':1100+i*100,
   'selected_count':10,'inbound_count':8,'retained_count':6,'occupied_3x3_cells':9,'grid_count':20,'grid_positive_uu_vv_count':15,
   'grid_trace_q10_q50_q90_px2':[1.,3.,7.],'original_need_interp':False,'execution_error':None})
 return rows,events


def test_feature_formula_order_and_control_coordinates():
 rows,events=fixture();values,evidence=prefix_features(rows,2,events)
 expected=[.6,.8,1.,.75,math.log(4),math.log(8)-math.log(2),1.]
 np.testing.assert_allclose(values,expected*6)
 assert evidence['health_sequences']=={'rgb':[2,1,0],'thermal':[2,1,0]}
 assert FAMILIES['current']==list(range(7))+list(range(21,28))
 assert [len(FAMILIES[k]) for k in ('combined','uncertainty','health','current')]==[42,12,30,14]
 assert set(FAMILIES['uncertainty']).isdisjoint(FAMILIES['health'])


def test_future_and_labels_cannot_change_prefix_features():
 rows,events=fixture();before=copy.deepcopy(events);expected=prefix_features(rows,2,events)
 for c in events:
  events[c][-1].update(grid_trace_q10_q50_q90_px2=[999]*3,reference_loss=-999,future_failure=True)
 assert prefix_features(rows,2,events)==expected
 assert before['rgb'][0]==events['rgb'][0]


def test_cold_start_missing_quantiles_and_zero_counts():
 rows,events=fixture();assert prefix_features(rows,1,events)[0] is None
 events['rgb'][0]['grid_trace_q10_q50_q90_px2']=None;assert prefix_features(rows,2,events)[0] is None
 event=copy.deepcopy(events['thermal'][0]);event.update(selected_count=0,inbound_count=0,retained_count=0,occupied_3x3_cells=0,grid_count=0,grid_positive_uu_vv_count=0,original_need_interp=True)
 assert event_values(event)[:4]==[0,0,0,0]
 assert event_values(event)[-1]==0


def test_no_feature_file_access(monkeypatch):
 import builtins
 rows,events=fixture()
 monkeypatch.setattr(builtins,'open',lambda *a,**k:(_ for _ in ()).throw(AssertionError('file read')))
 assert len(prefix_features(rows,2,events)[0])==42


@pytest.mark.parametrize('fault',['duplicate','counts','boolean','quantile_order'])
def test_corrupt_observation_rejected(fault):
 rows,events=fixture()
 if fault=='duplicate':events['rgb'].append(copy.deepcopy(events['rgb'][0]))
 elif fault=='counts':events['rgb'][0]['retained_count']=11
 elif fault=='boolean':events['rgb'][0]['original_need_interp']=1
 else:events['rgb'][0]['grid_trace_q10_q50_q90_px2']=[7,3,1]
 with pytest.raises(ValueError):prefix_features(rows,2,events)


def test_record_balancing_and_zero_cost():
 weights,totals=cost_weights([1,3,2,0],['a','a','b','c'])
 np.testing.assert_allclose(weights,[.125,.375,.5,0]);assert totals['c']==0


def test_analytic_gradient_against_central_finite_difference():
 z=np.array([[1.,2.],[-1.,.5],[.1,-.8]]);y=np.array([0.,1.,1.]);weights=np.array([.2,.3,.5]);theta=np.array([.4,-.2,.1])
 value,gradient=objective(theta,z,y,weights);numeric=[]
 for i in range(3):
  step=np.zeros(3);step[i]=1e-6;numeric.append((objective(theta+step,z,y,weights)[0]-objective(theta-step,z,y,weights)[0])/2e-6)
 np.testing.assert_allclose(gradient,numeric,atol=2e-10,rtol=1e-7)
 assert math.isfinite(value)


def test_constant_column_exact_normalization_and_analytic_intercept():
 # True constant8/9 can acquire tiny nonzero np.std through mean roundoff.
 x=np.full((3,2),8/9);model=fit(x,[0,1,0],[1,3,2],['a','a','b'],['0','1','2'])
 assert model['status']=='PASS_FIT' and model['scale']==[1.,1.]
 np.testing.assert_array_equal(model['mean'],x[0]);np.testing.assert_allclose(model['coefficients'],0,atol=1e-12)
 assert model['intercept']==pytest.approx(math.log(.375/.625),abs=1e-7)


@pytest.mark.parametrize('costs,labels,reason',[([0,0],[0,1],'no_positive_cost'),([1,1],[1,1],'one_weighted_class')])
def test_target_failures_are_artifacts_not_replacement_models(costs,labels,reason):
 model=fit([[0],[1]],labels,costs,['a','a'],['0','1'])
 assert model['status']=='FAILED_FIT' and model['reason']==reason
 assert choose(model,[0]*42,True,'THERMAL',[0])['action']=='THERMAL'


def test_training_only_scaling_and_excluded_label_feature_mutation():
 rows=[];labels={}
 for group in ('a','b'):
  for i in range(4):
   identity=group+str(i);rows.append({'sample_id':identity,'recording_id':group,'eligible':True,'vector':[float(i-2)]+[1.]*41})
   labels[identity]={'rgb':{'translation_m':1. if i>=2 else 2.},'thermal':{'translation_m':2. if i>=2 else 1.}}
 first=fit_fold(rows,labels,'combined','b');assert first['status']=='PASS_FIT'
 assert first['train_sample_ids']==['a0','a1','a2','a3'] and first['training_recordings']==['a']
 assert first['mean'][0]==pytest.approx(-.5)
 for r in rows:
  if r['recording_id']=='b':r['vector']=[1e100]*42;del labels[r['sample_id']]
 assert fit_fold(rows,labels,'combined','b')==first


def test_fallback_and_zero_margin_keep_fixed_preference():
 model={'status':'PASS_FIT','mean':[0.],'scale':[1.],'coefficients':[0.],'intercept':0.}
 assert choose(model,[0]*42,True,'THERMAL',[0])['action']=='RGB'
 assert choose(model,None,False,'ABSTAIN',[0])['action']=='ABSTAIN'


def test_prediction_adapter_reads_no_labels_or_files_and_preserves_fallback(monkeypatch):
 import builtins
 from Scripts.Setup.run_temporal_gate_development import make_predictions
 recordings=['a','b','c'];models={}
 for family,indices in FAMILIES.items():
  for i,r in enumerate(recordings):models[family,i]={'status':'PASS_FIT','excluded_recording':r,'training_recordings':[g for g in recordings if g!=r],
   'mean':[0.]*len(indices),'scale':[1.]*len(indices),'coefficients':[0.]*len(indices),'intercept':-1.}
 rows=[{'sample_id':r,'recording_id':r,'eligible':i==0,'vector':[0.]*42 if i==0 else None,
  'baseline_actions':{'rgb_only':'RGB' if i==0 else 'ABSTAIN','thermal_only':'THERMAL' if i<2 else 'ABSTAIN','raw_advice':('RGB','THERMAL','ABSTAIN')[i],'history':'ABSTAIN'},
  'reference_loss':'forbidden metadata'} for i,r in enumerate(recordings)]
 monkeypatch.setattr(builtins,'open',lambda *a,**k:(_ for _ in ()).throw(AssertionError('prediction file access')))
 pred=make_predictions(rows,models,recordings)
 for family in FAMILIES:assert [r['actions'][family] for r in pred]==['THERMAL','THERMAL','ABSTAIN']


def test_full_development_model_cannot_be_used_as_excluded_model():
 from Scripts.Setup.run_temporal_gate_development import make_predictions
 model={'status':'PASS_FIT','excluded_recording':None,'training_recordings':['a','b','c']}
 models={(f,0):model for f in FAMILIES}
 row={'sample_id':'a','recording_id':'a','eligible':False,'vector':None,'baseline_actions':{'raw_advice':'RGB'}}
 with pytest.raises(AssertionError):make_predictions([row],models,['a','b','c'])

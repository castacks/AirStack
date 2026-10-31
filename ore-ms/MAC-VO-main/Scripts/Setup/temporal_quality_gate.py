"""Fixed draft temporal gate: pure observed-prefix features and bounded CPU fit.

Margins are cost-sensitive scores, not calibrated probabilities. This module
never reads reference files, recording identities or future events at inference.
"""
import math
import numpy as np
from scipy.optimize import minimize
from scipy.special import expit

CAMERAS=('rgb','thermal')
NAMES=('retained_fraction','inbound_fraction','occupied_fraction','positive_diagonal_fraction','log_median_trace','log_trace_spread','supported')
FEATURE_NAMES=tuple(f'{c}/lag{lag}/{name}' for c in CAMERAS for lag in range(3) for name in NAMES)
FAMILIES={'combined':list(range(42)),
 'uncertainty':[i for i in range(42) if i%7 in (4,5)],
 'health':[i for i in range(42) if i%7 not in (4,5)],
 'current':[i for i in range(42) if i//7%3==0]}
OPTIONS={'maxiter':2000,'gtol':1e-9,'ftol':0.,'maxls':20,'maxcor':10,'maxfun':100000}
RIDGE=.01


def event_values(event):
    if event is None:return None
    keys=('selected_count','inbound_count','retained_count','occupied_3x3_cells','grid_count','grid_positive_uu_vv_count')
    if any(k not in event for k in (*keys,'original_need_interp','execution_error','grid_trace_q10_q50_q90_px2')):return None
    counts=[event[k] for k in keys]
    if any(type(x) is not int or x<0 for x in counts) or not counts[0]>=counts[1]>=counts[2] or counts[3]>9 or counts[5]>counts[4]:raise ValueError('invalid observed counts')
    if type(event['original_need_interp']) is not bool:raise ValueError('boolean original support required')
    q=event['grid_trace_q10_q50_q90_px2']
    if not isinstance(q,list) or len(q)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) or x<0 for x in q):return None
    if q!=sorted(q):raise ValueError('unordered covariance quantiles')
    selected,inbound,retained,cells,grid,positive=counts
    return [retained/max(selected,1),inbound/max(selected,1),cells/9,positive/max(grid,1),
        math.log1p(q[1]),math.log1p(q[2])-math.log1p(q[0]),float(not event['original_need_interp'] and event['execution_error'] is None)]


def prefix_features(rows,pair,events):
    if type(pair) is not int or not 0<=pair<len(rows)-1:raise ValueError('valid target pair required')
    if pair<2:return None,{'reason':'cold_start','health_sequences':{}}
    values=[];sequences={}
    for c in CAMERAS:
        sequences[c]=[]
        for lag in range(3):
            a,b=[rows[j][c+'_left']['time_ns'] for j in (pair-lag,pair-lag+1)]
            if type(a) is not int or type(b) is not int or b<=a:raise ValueError('exact increasing sensor endpoints required')
            found=[e for e in events[c] if e['kind']=='pair_end' and [e['time0_ns'],e['time1_ns']]==[a,b]]
            if len(found)>1:raise ValueError('duplicate observed interval health')
            vector=event_values(found[0] if found else None)
            if vector is None:return None,{'reason':'missing_observed_features','health_sequences':sequences}
            values+=vector;sequences[c].append(found[0]['sequence'])
    assert len(values)==42
    return values,{'reason':'complete','health_sequences':sequences}


def cost_weights(costs,groups):
    costs=np.asarray(costs,dtype=np.float64);groups=np.asarray(groups)
    if costs.ndim!=1 or groups.shape!=costs.shape or not np.isfinite(costs).all() or np.any(costs<0):raise ValueError('aligned nonnegative finite costs required')
    totals={g:float(costs[groups==g].sum()) for g in sorted(set(groups.tolist()))}
    active=[g for g,total in totals.items() if total>0];weights=np.zeros_like(costs)
    for g in active:weights[groups==g]=costs[groups==g]/totals[g]/len(active)
    return weights,totals


def objective(theta,z,y,weights):
    score=z@theta[:-1]+theta[-1];residual=weights*(expit(score)-y)
    value=float(np.sum(weights*(np.logaddexp(0.,score)-y*score))+.5*RIDGE*np.dot(theta[:-1],theta[:-1]))
    gradient=np.r_[z.T@residual+RIDGE*theta[:-1],residual.sum()]
    return value,gradient


def fit(x,y,costs,groups,sample_ids):
    x=np.asarray(x,dtype=np.float64);y=np.asarray(y,dtype=np.float64)
    if x.ndim!=2 or len(x)==0 or y.shape!=(len(x),) or len(sample_ids)!=len(x) or len(set(sample_ids))!=len(x):raise ValueError('unique aligned training rows required')
    if not np.isfinite(x).all() or not np.isfinite(y).all() or not np.isin(y,[0,1]).all():raise ValueError('finite features and binary labels required')
    weights,totals=cost_weights(costs,groups)
    if len(weights)!=len(x):raise ValueError('weights/rows disagree')
    base={'status':'FAILED_FIT','train_sample_ids':list(sample_ids),'record_cost_totals':totals,'train_weights':weights.tolist(),
        'ridge':RIDGE,'solver':'L-BFGS-B','options':OPTIONS.copy()}
    if not np.any(weights>0):return base|{'reason':'no_positive_cost'}
    if len(np.unique(y[weights>0]))<2:return base|{'reason':'one_weighted_class'}
    mean=x.mean(axis=0);scale=x.std(axis=0,ddof=0);constant=np.ptp(x,axis=0)==0
    mean[constant]=x[0,constant];scale[constant]=1.
    if not np.isfinite(scale).all() or np.any(scale<=0):return base|{'reason':'invalid_training_scale'}
    z=(x-mean)/scale
    result=minimize(lambda theta:objective(theta,z,y,weights),np.zeros(x.shape[1]+1,dtype=np.float64),method='L-BFGS-B',jac=True,options=OPTIONS)
    value,gradient=objective(result.x,z,y,weights);norm=float(np.max(np.abs(gradient)))
    finite=bool(np.isfinite(result.x).all() and math.isfinite(value) and math.isfinite(norm))
    success=bool(result.success and finite and norm<=OPTIONS['gtol'])
    return base|{'status':'PASS_FIT' if success else 'FAILED_FIT','reason':'converged' if success else 'solver_or_stationarity_failure',
        'mean':mean.tolist(),'scale':scale.tolist(),'constant_columns':constant.tolist(),
        'coefficients':result.x[:-1].tolist(),'intercept':float(result.x[-1]),'objective':value,'gradient_inf':norm,
        'native_success':bool(result.success),'message':str(result.message),'iterations':int(result.nit),'function_evaluations':int(result.nfev)}


def choose(model,vector,eligible,fallback,indices):
    if fallback not in ('RGB','THERMAL','ABSTAIN'):raise ValueError('unknown fallback action')
    if not eligible or vector is None:return {'action':fallback,'reason':'original_fallback','margin':None}
    if model['status']!='PASS_FIT':return {'action':fallback,'reason':'failed_fit_fallback','margin':None}
    x=np.asarray(vector,dtype=np.float64)[indices];mean=np.asarray(model['mean']);scale=np.asarray(model['scale'])
    margin=float(((x-mean)/scale)@np.asarray(model['coefficients'])+model['intercept'])
    if not math.isfinite(margin):return {'action':fallback,'reason':'nonfinite_margin_fallback','margin':None}
    return {'action':'RGB' if margin>=0 else 'THERMAL','reason':'learned_margin','margin':margin}


def fit_fold(rows,labels,family,excluded_recording=None):
    """Select whole training recordings before accessing labels or scalers."""
    indices=FAMILIES[family]
    train=[r for r in rows if r['eligible'] and r['recording_id']!=excluded_recording]
    x=np.array([r['vector'] for r in train],dtype=np.float64)[:,indices]
    target=[];costs=[]
    for r in train:
        label=labels[r['sample_id']];a=label['rgb']['translation_m'];b=label['thermal']['translation_m']
        if type(a) not in (int,float) or type(b) not in (int,float) or not math.isfinite(a) or not math.isfinite(b) or min(a,b)<0:raise ValueError('finite nonnegative training losses required')
        target.append(float(a<b));costs.append(abs(b*b-a*a))
    model=fit(x,target,costs,[r['recording_id'] for r in train],[r['sample_id'] for r in train])
    return model|{'family':family,'feature_indices':indices,'excluded_recording':excluded_recording,
        'training_recordings':sorted({r['recording_id'] for r in train})}

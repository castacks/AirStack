"""Fixed analytical future-risk probe. No estimator or architecture changes."""
import math
import numpy as np
CAMERAS=('rgb','thermal')
RIDGE=1.

def current_history(current,covariance,dt):
    """Exactly three past/current covariance values per sensor, no label fields."""
    now=[];history=[]
    for camera in CAMERAS:
        v=np.asarray(covariance[camera],float);t=np.asarray(dt[camera],float)
        if v.shape!=(3,) or t.shape!=(3,) or not np.isfinite(v).all() or not np.isfinite(t).all() or np.any(v<0) or np.any(t<=0):raise ValueError('three finite contiguous covariance intervals required')
        rate=np.log(np.maximum(v/t**2,1e-30));c=current[camera];available=bool(c['available']);info=c['information'];has_info=available and info is not None and math.isfinite(info)
        now.extend([rate[0],math.log(t[0]),math.log1p(max(info,0)) if has_info else 0.,math.log1p(c['retained']) if available else 0.,c['cells']/9 if available else 0.,c['fraction'] if available else 0.,float(available),float(has_info)])
        history.extend([rate[1]-rate[0],rate[2]-rate[0]])
    return list(map(float,now)),list(map(float,now+history))

def fit_fold(rows,labels,excluded_recording,family):
    """Exclude whole recording BEFORE target-label lookup and scaling."""
    train=[r for r in rows if r['recording_id']!=excluded_recording and r['target_id'] in labels and all(labels[r['target_id']].get(c) is not None for c in CAMERAS)]
    if not train:return {'status':'NO_TRAINING','excluded_recording':excluded_recording,'family':family,'train_ids':[]}
    x=np.asarray([r[family] for r in train],float);y=np.asarray([math.log1p(labels[r['target_id']]['rgb']['translation_m']/.1)-math.log1p(labels[r['target_id']]['thermal']['translation_m']/.1) for r in train])
    if not np.isfinite(x).all() or not np.isfinite(y).all():raise ValueError('finite fit data required')
    groups=[r['recording_id'] for r in train];unique=sorted(set(groups));counts={g:groups.count(g) for g in unique};w=np.array([1/len(unique)/counts[g] for g in groups]);mean=w@x;variance=w@((x-mean)**2);constant=np.ptp(x,axis=0)==0;mean[constant]=x[0,constant];scale=np.sqrt(variance);scale[constant]=1
    if np.any(scale<=0):raise ValueError('invalid training-only scale')
    z=(x-mean)/scale;ym=float(w@y);coef=np.linalg.solve(z.T@(w[:,None]*z)+RIDGE*np.eye(x.shape[1]),z.T@(w*(y-ym)));intercept=float(w@(y-z@coef));residual=z@coef+intercept-y;gradient=np.r_[z.T@(w*residual)+RIDGE*coef,w@residual]
    if np.max(abs(gradient))>1e-9:raise ValueError('ridge stationarity failed')
    return {'status':'PASS_FIT','excluded_recording':excluded_recording,'family':family,'training_recordings':unique,'train_ids':[r['forecast_id'] for r in train],'target_ids':[r['target_id'] for r in train],'weights':w.tolist(),'mean':mean.tolist(),'scale':scale.tolist(),'coefficients':coef.tolist(),'intercept':intercept,'ridge':RIDGE,'gradient_inf':float(np.max(abs(gradient))),'objective':float(w@(residual**2)+RIDGE*(coef@coef))}

def predict(model,vector):
    if model['status']!='PASS_FIT':return {'action':'ABSTAIN','margin':None}
    value=float(((np.asarray(vector,float)-model['mean'])/model['scale'])@model['coefficients']+model['intercept'])
    if not math.isfinite(value):raise ValueError('nonfinite prediction')
    return {'action':'THERMAL' if value>0 else 'RGB','margin':value}

def bind_target(rows,origin,horizon_ns):
    end=rows[origin+1]['rgb_left']['time_ns'];possible=[(abs(rows[j]['rgb_left']['time_ns']-end-horizon_ns),rows[j]['rgb_left']['time_ns'],j) for j in range(origin+1,len(rows)-1) if rows[j]['rgb_left']['time_ns']>=end]
    if not possible:return None
    delta,_,j=min(possible)
    return j if delta<=50_000_000 else None

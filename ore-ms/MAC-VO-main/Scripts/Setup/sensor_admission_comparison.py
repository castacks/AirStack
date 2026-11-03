"""Fixed research selector on explicit same-target causal information contracts.

Predict reads only the input contract; score separately reads locked predictions
and proxy labels. This is whole-motion selection, not joint measurement fusion.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

SCALES=np.array([.1]*3+[np.pi/180]*3)  # engineering design budgets, not calibrated errors
CAMERAS=('rgb','thermal')
METHODS=('candidate','geometry_current','raw','history','rgb_only','thermal_only')

def adequate(matrix):
    h=np.asarray(matrix,dtype=float)
    if h.shape!=(6,6) or not np.isfinite(h).all() or not np.allclose(h,h.T,rtol=0,atol=1e-10):
        raise ValueError('finite symmetric 6x6 information required')
    scaled=SCALES[:,None]*h*SCALES[None,:]
    e=np.linalg.eigvalsh(scaled)
    if min(e)<-1e-10*max(1,max(abs(e))):raise ValueError('PSD information required')
    return bool(min(e)>=1),float(min(e))


def predict(contract):
    if set(contract)!={'schema_version','frame_id','rows'} or contract['schema_version']!=1 or not isinstance(contract['frame_id'],str) or not contract['frame_id']:
        raise ValueError('strict label-free versioned frame contract required')
    rows=[];states={};ids=set()
    for row in contract['rows']:
        if set(row)!={'sample_id','recording_id','window','pair_ordinal','interval_ns','deadline_ns','cameras','baselines'}:
            raise ValueError('strict label-free row required')
        sid=row['sample_id'];key=(row['recording_id'],row['window']);ordinal=row['pair_ordinal'];interval=row['interval_ns'];deadline=row['deadline_ns']
        if sid in ids or type(ordinal)is not int or ordinal<0 or len(interval)!=2 or any(type(x)is not int for x in interval) or interval[1]<=interval[0] or type(deadline)is not int or deadline<0:
            raise ValueError('unique ID, ordinal, positive target interval and integer deadline required')
        ids.add(sid)
        old=states.get(key);streak={c:0 for c in CAMERAS}
        if old:
            if ordinal<=old['ordinal'] or interval[0]<old['end']:raise ValueError('ordered nonoverlapping targets required')
            if ordinal==old['ordinal']+1 and interval[0]==old['end']:streak=old['streak'].copy()
        if set(row['cameras'])!=set(CAMERAS) or set(row['baselines'])!={'raw','history'}:
            raise ValueError('both camera slots and original baselines required')
        available={};quality={};minimum={}
        for c in CAMERAS:
            camera=row['cameras'][c]
            if set(camera)!={'available','supported','information'} or type(camera['available'])is not bool or type(camera['supported'])is not bool:
                raise ValueError('strict camera availability/support fields required')
            available[c]=camera['available'] and camera['supported'];info=camera['information']
            if available[c]:
                if info is None:raise ValueError('supported available motion requires bound geometry; recapture needed')
                if set(info)!={'frame_id','interval_ns','received_ns','matrix','evidence_ref'} or info['frame_id']!=contract['frame_id'] or info['interval_ns']!=interval or type(info['received_ns'])is not int or not 0<=info['received_ns']<=deadline or not isinstance(info['evidence_ref'],str) or not info['evidence_ref']:
                    raise ValueError('same-target/frame, causal receipt and evidence binding required')
                quality[c],minimum[c]=adequate(info['matrix']);streak[c]=streak[c]+1 if quality[c] else 0
            else:quality[c]=False;minimum[c]=None;streak[c]=0
        confirmed=[c for c in CAMERAS if quality[c] and streak[c]>=2]
        current=[c for c in CAMERAS if quality[c]]
        selected=(confirmed or current or [None])[0]
        actions={'candidate':selected.upper() if selected else 'ABSTAIN','geometry_current':current[0].upper() if current else 'ABSTAIN',
            'rgb_only':'RGB' if available['rgb'] else 'ABSTAIN','thermal_only':'THERMAL' if available['thermal'] else 'ABSTAIN'}
        for name,action in row['baselines'].items():
            if action not in ('RGB','THERMAL','ABSTAIN') or action!='ABSTAIN' and not available[action.lower()]:
                raise ValueError('baseline selected unavailable motion')
            actions[name]=action
        reason='confirmed_adequate' if confirmed else 'reconsider_recovery_remaining_inadequate' if current else 'no_adequate_current_motion'
        rows.append({'sample_id':sid,'recording_id':row['recording_id'],'window':row['window'],'pair_ordinal':ordinal,
            'interval_ns':interval,'actions':actions,'streak':streak.copy(),'adequate':quality,'normalized_information_min_eigenvalue':minimum,
            'native_available':available,'reason':reason,'confirmation_pairs_remaining':{c:max(0,2-streak[c]) if available[c] else None for c in CAMERAS}})
        states[key]={'ordinal':ordinal,'end':interval[1],'streak':streak.copy()}
    return {'schema_version':1,'rows':rows,'labels_used':False,'scope':'whole-motion selector; conditional Gaussian information, not calibrated pose accuracy','production_defaults_changed':False}


def score(predictions,labels):
    """Labels: sample -> camera -> {translation_m, rotation_deg}, or null."""
    if set(labels)!=set(r['sample_id'] for r in predictions['rows']):raise ValueError('exact label population required')
    records={}
    for r in predictions['rows']:records.setdefault(r['recording_id'],[]).append(r)
    result={}
    for record,rows in records.items():
        out={'decisions':len(rows),'methods':{},'matched':{}}
        for method in METHODS:
            values=[]
            for r in rows:
                action=r['actions'][method]
                if action=='ABSTAIN':continue
                label=labels[r['sample_id']][action.lower()]
                if label is None:raise ValueError('selected motion lacks reference loss; preserve failure')
                if set(label)!={'translation_m','rotation_deg'} or any(not np.isfinite(v) or v<0 for v in label.values()):raise ValueError('finite nonnegative losses required')
                values.append((r['sample_id'],label))
            out['methods'][method]={'scored':len(values),'ids':[x[0] for x in values],**{k:float(np.sqrt(np.mean([l[k]**2 for _,l in values]))) if values else None for k in ['translation_m','rotation_deg']}}
        for method in METHODS[1:]:
            common=set(out['methods']['candidate']['ids'])&set(out['methods'][method]['ids'])
            chosen=[r for r in rows if r['sample_id'] in common]
            out['matched'][method]={'ids':[r['sample_id'] for r in chosen],**{m:{k:float(np.sqrt(np.mean([labels[r['sample_id']][r['actions'][m].lower()][k]**2 for r in chosen]))) if chosen else None for k in ['translation_m','rotation_deg']} for m in ['candidate',method]}}
        out['admission_costs']={}
        for camera in CAMERAS:
            discarded=[r for r in rows if r['native_available'][camera] and r['actions']['candidate']!=camera.upper()]
            deferred=[r['sample_id'] for r in discarded if r['adequate'][camera] and r['streak'][camera]==1]
            missed=[]
            for r in discarded:
                action=r['actions']['candidate'];other=labels[r['sample_id']][camera]
                if action!='ABSTAIN' and other is not None and other['translation_m']<labels[r['sample_id']][action.lower()]['translation_m']:
                    missed.append(r['sample_id'])
            out['admission_costs'][camera]={'discarded_supported_motion_ids':[r['sample_id'] for r in discarded],
                'deferred_recovery_opportunity_ids':deferred,'missed_lower_translation_proxy_ids':missed,
                'delay_unit':'decision opportunities; timestamps and confirmation state remain in predictions'}
        comparisons=out['matched']
        out['passes_strict_diagnostic']=bool(out['methods']['candidate']['scored']>=out['methods']['raw']['scored'] and all(v['ids'] and v['candidate']['translation_m']<v[m]['translation_m'] and v['candidate']['rotation_deg']<=v[m]['rotation_deg'] for m,v in comparisons.items()))
        result[record]=out
    return {'records':result,'passes_all_records':bool(result) and all(x['passes_strict_diagnostic'] for x in result.values()),'physical_accuracy_verified':False,'candidate_adopted':False}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['predict','score']);parser.add_argument('--input',type=Path,required=True);parser.add_argument('--labels',type=Path);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if args.output.exists():raise ValueError('output must be absent')
    data=json.loads(args.input.read_text())
    if args.action=='predict':
        if args.labels is not None:raise ValueError('prediction must not receive labels')
        result=predict(data)
    else:
        if args.labels is None:raise ValueError('separate scoring labels required')
        result=score(data,json.loads(args.labels.read_text()))
    result['input_sha256']=hashlib.sha256(args.input.read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')

if __name__=='__main__':main()

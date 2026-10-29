"""Label-free current/prior diagnostics from verified causal replay prefixes."""
import argparse
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import json
from pathlib import Path
import numpy as np
from Scripts.Setup.shared_arbiter_replay import advisory
from Scripts.Setup.td3_temporal_angular import angular_variance

FEATURES={'flow_trace_px2':'lower','ray_trace_rad2':'lower','retained_fraction':'higher',
          'occupied_cells':'higher','prior_supported':'higher'}
CASES=(('early','shared-arbiter-events'),('middle','arbiter-support-collapse'),('day-middle','history-day-replication'))


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def covariance_features(raw):
    uv=np.asarray(raw['grid_uv1']);c=np.asarray(raw['grid_raw_flow_cov']);k=np.asarray(raw['K']).reshape(3,3)
    if uv.ndim!=2 or uv.shape[1]!=2 or c.shape!=(len(uv),3):raise ValueError('aligned grid endpoints/covariances required')
    if not np.isfinite(k).all() or np.linalg.det(k)==0:raise ValueError('finite invertible K required')
    if len(uv):
        values,valid=angular_variance(uv,c,k)
        valid &= np.isfinite(uv).all(1)&(c[:,0]>0)&(c[:,1]>0)
    else:values=np.array([]);valid=np.zeros(0,bool)
    return {'flow_trace_px2':float(np.median(c[valid,:2].sum(1))) if valid.any() else None,
            'ray_trace_rad2':float(np.median(values[valid])) if valid.any() else None,
            'grid_count':len(uv),'valid_covariance_grid_count':int(valid.sum()),'invalid_covariance_grid_count':int((~valid).sum())}


def interval_health(groups,pair,camera,events):
    if pair<0:return None
    times=[groups[i][camera+'_left']['time_ns'] for i in (pair,pair+1)]
    matches=[e for e in events if e['kind']=='pair_end' and [e['time0_ns'],e['time1_ns']]==times]
    if len(matches)>1:raise ValueError('duplicate pair health')
    return matches[0] if matches else None


def make_features(groups,pair,camera,events,raw):
    current=interval_health(groups,pair,camera,events);prior=interval_health(groups,pair-1,camera,events)
    if current is None:raise ValueError('source lacks target health snapshot')
    if np.asarray(raw['time0_ns']).size!=1 or np.asarray(raw['time0_ns']).dtype.kind not in 'iu' or int(np.asarray(raw['time0_ns']).item())!=groups[pair][camera+'_left']['time_ns']:
        raise ValueError('raw snapshot is not the exact target interval')
    covariance=covariance_features(raw);selected=current['selected_count'];retained=current['retained_count']
    if type(selected) is not int or type(retained) is not int or not 0<=retained<=selected:raise ValueError('invalid retained/selected counts')
    inbound=np.asarray(raw['selected_inbound']);keep=np.asarray(raw['outlier_keep_inbound'])
    if (len(raw['selected_uv0'])!=selected or inbound.dtype!=np.bool_ or inbound.shape!=(selected,)
            or int(inbound.sum())!=current['inbound_count'] or keep.dtype!=np.bool_
            or keep.shape!=(int(inbound.sum()),) or int(keep.sum())!=retained or covariance['grid_count']!=current['grid_count']):
        raise ValueError('raw counts disagree with event snapshot')
    points=np.asarray(raw['selected_uv0'])[inbound][keep];height,width=np.asarray(raw['height_width']).reshape(2)
    if not height>0 or not width>0 or not np.isfinite(points).all() or not np.all((points>=0)&(points<[width,height])):
        raise ValueError('invalid retained source pixels')
    cells=np.floor(points/[width,height]*3).astype(int)
    occupied=len(np.unique(cells[:,1]*3+cells[:,0])) if len(cells) else 0
    if occupied!=current['occupied_3x3_cells']:raise ValueError('raw spatial coverage disagrees with event')
    prior_support=None if prior is None or type(prior.get('original_need_interp')) is not bool else int(prior['original_need_interp'] is False and prior.get('execution_error') is None)
    return {'values':{k:covariance[k] for k in ('flow_trace_px2','ray_trace_rad2')}|{
                'retained_fraction':retained/selected if selected else None,'occupied_cells':current['occupied_3x3_cells'],'prior_supported':prior_support},
            'covariance_population':{k:covariance[k] for k in ('grid_count','valid_covariance_grid_count','invalid_covariance_grid_count')},
            'health_event_sequence':current['sequence'],'prior_health_event_sequence':prior['sequence'] if prior else None,
            'health_observation_monotonic_ns':current['observation_monotonic_ns']}


def prepare(root,output):
    files=[];cases=[]
    for window,directory in CASES:
        run=root/'results'/directory/'run-001'
        files += [run/n for n in ('timeline.jsonl','protocol.json','validation.json')]
        for c in ('rgb','thermal'):
            files += [run/'experts'/c/'arbiter-events/events.jsonl',run.parent/(c+'-preservation.json')]
            files += sorted((run/'experts'/c/'uncertainty').glob('pair-*.npz'))
        cases.append({'window':window,'run_relative':str(run.relative_to(root))})
    files += [Path(__file__),Path(__file__).with_name('td3_temporal_angular.py'),Path(__file__).with_name('shared_arbiter_replay.py'),Path(__file__).with_name('pair_arbiter_events.py')]
    output.mkdir(parents=True,exist_ok=False)
    protocol={'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'base_commit':'758240925',
        'features':FEATURES,'cases':cases,'input_sha256':{str(p.relative_to(root)):sha(p) for p in files},
        'labels_loaded':False,'fitted':False,'scope':'offline computation from causal current/prior tensors; raw files serialized later; not measured runtime availability',
        'covariance_population':'fixed grid, finite model endpoints, finite PSD covariance with positive diagonals; identical native/angular population; full cross term',
        'unavailable_optimizer_features':['convergence','pose_covariance'],'newer_intervals_used':False}
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')


def extract(root,output):
    if (output/'features.json').exists():raise FileExistsError('features already exist')
    protocol=json.loads((output/'protocol.json').read_text())
    if protocol['features']!=FEATURES or protocol['labels_loaded'] or protocol['newer_intervals_used']:raise ValueError('feature contract changed')
    for rel,d in protocol['input_sha256'].items():
        if sha(root/rel)!=d:raise ValueError('frozen source changed')
    rows=[];telemetry={'available_first_motions':0,'convergence_nonnull':0,'pose_covariance_nonnull':0}
    for case in protocol['cases']:
        run=root/case['run_relative'];proof=json.loads((run/'validation.json').read_text())
        if proof['status']!='PASS' or proof['timeline_sha256']!=sha(run/'timeline.jsonl'):raise ValueError('unverified source timeline')
        groups=json.loads((run/'protocol.json').read_text())['input_rows'];events={c:[] for c in ('rgb','thermal')}
        for c in events:
            p=json.loads((run.parent/(c+'-preservation.json')).read_text())
            if p['status']!='PASS' or p['events_sha256']!=sha(run/'experts'/c/'arbiter-events/events.jsonl'):raise ValueError('unverified expert raw journal')
        for line in (run/'timeline.jsonl').read_text().splitlines():
            decision=json.loads(line)
            if decision['kind']=='observer_event':events[decision['camera']].append(decision['event'])
            if decision['kind']!='decision':continue
            pair=decision['pair_ordinal'];original=advisory(groups,pair,events)
            if original['availability']!=decision['availability'] or original['evidence']!=decision['evidence']:raise ValueError('source prefix mismatch')
            row={'window':case['window'],'pair_ordinal':pair,'source_decision_sequence':decision['sequence'],
                 'deadline_monotonic_ns':decision['controller_monotonic_ns'],'observed_event_counts':decision['observed_event_counts'],
                 'availability':original['availability'],'experts':{}}
            for c in events:
                path=run/'experts'/c/'uncertainty'/f'pair-{pair:03d}.npz'
                with np.load(path,allow_pickle=False) as raw:record=make_features(groups,pair,c,events[c],raw)
                if record['health_observation_monotonic_ns']>row['deadline_monotonic_ns']:raise ValueError('future health')
                record['raw_relative']=str(path.relative_to(root));row['experts'][c]=record
                if original['availability'][c]:
                    m=events[c][original['evidence'][c]['motion_event_sequence']];telemetry['available_first_motions']+=1
                    telemetry['convergence_nonnull']+=m.get('optimizer_convergence') is not None
                    telemetry['pose_covariance_nonnull']+=m.get('pose_covariance') is not None
            rows.append(row)
    for rel,d in protocol['input_sha256'].items():
        if sha(root/rel)!=d:raise ValueError('source changed during extraction')
    report={'protocol_sha256':sha(output/'protocol.json'),'rows':rows,'optimizer_telemetry':telemetry,'labels_loaded':False}
    (output/'features.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'rows':len(rows),'expert_slots':2*len(rows),'optimizer_telemetry':telemetry}))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=('prepare','extract'));p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();(prepare if a.action=='prepare' else extract)(a.root,a.output)

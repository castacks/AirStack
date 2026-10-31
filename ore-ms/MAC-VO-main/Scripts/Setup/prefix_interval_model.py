"""Unchanged interval model and policies on observed shared-controller prefixes.

Static nominal transforms are predictor metadata, not physical verification.
No labels, reference poses, file reads or optimizer feedback in predict_prefix.
"""
import numpy as np
from Scripts.Setup.arbiter_history import history_advisory
from Scripts.Setup.shared_arbiter_replay import advisory, POSE_FRAME
from Scripts.Setup.pair_arbiter_events import integer, camera_record, index_events
from Scripts.Setup.arbiter_motion_output import pose_matrix
from Scripts.Setup.causal_interval_model import project, se3

CAMERAS=('rgb','thermal')


def predict_prefix(rows,timeline,nominal_body_from_camera):
    transforms={c:se3(nominal_body_from_camera[c]) for c in CAMERAS}
    events={c:[] for c in CAMERAS};receipts={c:{} for c in CAMERAS};previous=-1;next_pair=0
    for sequence,entry in enumerate(timeline):
        now=integer(entry['controller_monotonic_ns'])
        if integer(entry['sequence'])!=sequence or now<0 or now<previous:
            raise ValueError('nonsequential timeline or backwards controller clock')
        previous=now
        if entry['kind']=='observer_event':
            camera,event=entry['camera'],entry['event']
            if camera not in CAMERAS or integer(event['sequence'])!=len(events[camera]) or event.get('modality',camera)!=camera:
                raise ValueError('invalid observer identity')
            events[camera].append(event);receipts[camera][event['sequence']]=now
        elif entry['kind']=='decision':
            pair=integer(entry['pair_ordinal'])
            if pair!=next_pair or not 0<=pair<len(rows)-1:raise ValueError('missing/duplicate target decision')
            next_pair+=1
            current=advisory(rows,pair,events)
            for key in ('action','availability','evidence','observed_event_counts'):
                if current[key]!=entry[key]:raise ValueError('source advice differs from observed prefix')
            history=history_advisory(rows,pair,events)
            common=[integer(rows[i]['rgb_left']['time_ns']) for i in (pair,pair+1)]
            experts={}
            for camera in CAMERAS:
                first,last=[integer(rows[i][camera+'_left']['time_ns']) for i in (pair,pair+1)]
                info={'available_by_prefix':current['availability'][camera],'valid_model_projection':False,
                      'reason':'source_unavailable_by_deadline','delta_model':None,'source':None}
                if current['availability'][camera]:
                    health,motions,_=index_events(events[camera])
                    record=camera_record({'coordinate_frame':POSE_FRAME},health,motions,first,last);motion=record['motion']
                    received=max(receipts[camera][motion['sequence']],receipts[camera][health[first,last]['sequence']])
                    assert received<=now
                    if motion['stage']!='pair_processing':
                        info['reason']='non_processing_stage'
                    else:
                        e=transforms[camera];native=se3(e@np.array(pose_matrix(motion['derived_camera_delta_xyzw']))@np.linalg.inv(e))
                        info.update(project(native,[first,last],common))
                        info['source']={'motion_event_sequence':motion['sequence'],'health_event_sequence':health[first,last]['sequence'],
                            'received_controller_ns':received,'native_body_delta':native.tolist()}
                experts[camera]=info
            advice={'rgb_only':'RGB' if current['availability']['rgb'] else 'ABSTAIN',
                    'thermal_only':'THERMAL' if current['availability']['thermal'] else 'ABSTAIN',
                    'raw_advice':current['action'],'history':history['action']}
            choices={name:action if action!='ABSTAIN' and experts[action.lower()]['valid_model_projection'] else 'ABSTAIN' for name,action in advice.items()}
            yield {'pair_ordinal':pair,'source_decision_sequence':sequence,'controller_deadline_ns':now,
                'controller_clock_id':'shared-controller','common_interval_ns':common,'experts':experts,
                'observed_event_counts':current['observed_event_counts'].copy(),'advice_actions':advice,'model_actions':choices,
                'history_reason':history['reason'],'admission_ready':False,'physical_frame_verified':False,
                'acquisition_clock_verified':False,'measured_live_latency_verified':False,'reference_used_for_prediction':False}

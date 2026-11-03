"""Bind new geometry journals to label-free prefix motion projections.

Requires fresh prefix predictions from the same geometry capture. Historical
predictions cannot be paired with a newly numbered or later-refined journal.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .native_pose_information import project_information


def prepare(predictions,static,timeline,recording,window):
    frame_id='nominal_body_stationary_twist';rows=[];proof=[]
    transforms=static[window]['nominal_body_from_camera']
    for r in predictions['rows']:
        if r['window']!=window:continue
        if r.get('reference_used_for_prediction') is not False:
            raise ValueError('label-free source motion projection required')
        deadline=r['controller_deadline_ns'];cameras={}
        for c in ['rgb','thermal']:
            expert=r['experts'][c];available=expert['available_by_prefix'] and expert['valid_model_projection']
            camera={'available':bool(available),'supported':bool(available),'information':None}
            if available:
                prefix=[entry for entry in timeline if entry['kind']=='observer_event' and entry['camera']==c and entry['controller_monotonic_ns']<=deadline]
                by_sequence={x['event']['sequence']:x for x in prefix}
                source=expert['source'];child=by_sequence[source['motion_event_sequence']];motion=child['event']
                if motion['kind']!='motion_result_written' or not motion['first_write_for_submission'] or motion['original_need_interp']:
                    raise ValueError('exact first supported motion receipt required')
                matches=[x for x in prefix if x['event']['kind']=='native_disparity_information' and x['event']['submission_id']==motion['submission_id']]
                if len(matches)!=1:raise ValueError('missing/duplicate causal geometry; historical capture must be repeated')
                observation=matches[0];event=observation['event'];native=[motion['time0_ns'],motion['time1_ns']]
                if native!=expert['native_interval_ns'] or [event['time0_ns'],event['time1_ns']]!=native or event['from_idx']!=motion['from_idx'] or event['frame_idx']!=motion['frame_idx']:
                    raise ValueError('geometry and motion interval/graph bindings differ')
                if event['linearization']!='returned optimizer pose before first write; copied submitted fixed factors':
                    raise ValueError('returned-pose geometry required')
                v=np.asarray(motion['derived_camera_delta_xyzw']);delta=np.eye(4);delta[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();delta[:3,3]=v[:3]
                transported=project_information(event['information']['information_6x6'],delta,transforms[c],native,r['common_interval_ns'])
                if not np.allclose(transported['delta_model'],expert['delta_model'],rtol=1e-8,atol=1e-9):raise ValueError('transport changed the original motion projection')
                received=max(child['controller_monotonic_ns'],observation['controller_monotonic_ns'],source['received_controller_ns'])
                if received>deadline:raise ValueError('motion/geometry arrived after decision')
                ref=f"{window}/{c}/controller-{observation['sequence']}/submission-{event['submission_id']}"
                camera['information']={'frame_id':frame_id,'interval_ns':r['common_interval_ns'],'received_ns':received,'matrix':transported['information_body_6x6'],'evidence_ref':ref}
                proof.append({'pair_ordinal':r['pair_ordinal'],'camera':c,'evidence_ref':ref,'transport':transported})
            cameras[c]=camera
        rows.append({'sample_id':f"{window}/{r['pair_ordinal']}",'recording_id':recording,'window':window,
            'pair_ordinal':r['pair_ordinal'],'interval_ns':r['common_interval_ns'],'deadline_ns':deadline,'cameras':cameras,
            'baselines':{'raw':r['model_actions']['raw_advice'],'history':r['model_actions']['history']}})
    if not rows:raise ValueError('no declared window rows')
    return {'schema_version':1,'frame_id':frame_id,'rows':rows},proof

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['predictions','static','timeline','output']:p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--recording-id',required=True);p.add_argument('--window',required=True);args=p.parse_args()
    proof_path=args.output.with_suffix('.proof.json')
    if args.output.exists() or proof_path.exists():raise ValueError('exclusive outputs required')
    data,proof=prepare(json.loads(args.predictions.read_text()),json.loads(args.static.read_text()),[json.loads(x) for x in args.timeline.read_text().splitlines()],args.recording_id,args.window)
    input_hashes={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in [args.predictions,args.static,args.timeline]}
    args.output.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    proof_path.write_text(json.dumps({'input_sha256':input_hashes,'bindings':proof,'labels_used':False},indent=2,allow_nan=False)+'\n')

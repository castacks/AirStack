"""Matched research capture retaining an exact native terminal zero-match failure."""
import argparse
import json
from pathlib import Path
from . import shared_arbiter_replay as controller
from .shared_arbiter_replay import worker as original_worker


def retaining_run(original,args,camera):
    """Never change native estimator code or status; permit a completed no-motion capture."""
    output=Path(args.output)/camera
    try:
        value=original(args,camera)
    except RuntimeError as error:
        if str(error)!='No visual matches were retained':raise
        report=json.loads((output/'report.json').read_text());tracking=report['tracking_diagnostics'];observer=report['arbiter_event_recording']
        if not (report['status']=='FAILED' and report['completed_frames']==args.frames
                and tracking['retained_observations']==0 and tracking['support_state']=='NO_PAIRS_SUPPORTED'
                and tracking['unsupported_pairs_before_refinement']==args.frames-1
                and observer['status']=='COMPLETE' and not observer['observer_errors'] and observer['optimization_submissions']==0):
            raise RuntimeError('zero-match terminal evidence does not satisfy capture contract') from error
        contract={'native_status':'FAILED','native_failure':'No visual matches were retained','all_frames_processed':True,
            'motion_available':False,'scope':'terminal failure retained; shared observation completed, not successful VO'}
        with (output/'terminal-capture-contract.json').open('x') as f:json.dump(contract,f,indent=2);f.write('\n')
        return None
    else:
        with (output/'terminal-capture-contract.json').open('x') as f:json.dump({'native_status':'RETURNED','zero_match_failure_suppressed':False},f,indent=2);f.write('\n')
        return value


def worker(camera,options,pipe):
    from . import run_td3_execution_v3 as runner,arbiter_event_recorder
    from .geometry_event_recorder import GeometryEventRecorder
    original_run=runner.run;original_recorder=arbiter_event_recorder.ArbiterEventRecorder
    geometry=options.pop('geometry_support_observation',False)
    runner.run=lambda args,modality:retaining_run(original_run,args,modality)
    if geometry:arbiter_event_recorder.ArbiterEventRecorder=GeometryEventRecorder
    try:return original_worker(camera,options,pipe)
    finally:runner.run=original_run;arbiter_event_recorder.ArbiterEventRecorder=original_recorder


def run(args):
    # shared controller does not pass research-specific flags; use picklable worker adapters.
    old=controller.worker;controller.worker=worker_on if args.geometry else worker
    try:return controller.run(args)
    finally:controller.worker=old


def worker_on(camera,options,pipe):
    options=dict(options,geometry_support_observation=True);return worker(camera,options,pipe)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['dataset','manifest','output']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--frames',type=int,default=20);p.add_argument('--start',type=int,default=0);p.add_argument('--timeout-seconds',type=float,default=300)
    p.add_argument('--record-root-initialization',action='store_true');p.add_argument('--acknowledge-provisional-geometry',action='store_true');p.add_argument('--geometry',action='store_true')
    run(p.parse_args())

"""Matched geometry-only versus geometry+early observations, optional native MS²."""
import argparse,contextlib
from pathlib import Path
from . import shared_arbiter_replay as controller
from .geometry_support_replay import retaining_run

def worker(camera,options,pipe):
    from . import run_td3_execution_v3 as runner,arbiter_event_recorder as recorder
    from .geometry_event_recorder import GeometryEventRecorder
    from .early_signal_events import EarlyGeometryEventRecorder
    from .ms2_execution_adapter import adapted_runner
    early=options.pop('early_signals',False);ms2=options.pop('ms2_native',False)
    oldrun,oldrecorder=runner.run,recorder.ArbiterEventRecorder
    runner.run=lambda args,cam:retaining_run(oldrun,args,cam)
    recorder.ArbiterEventRecorder=EarlyGeometryEventRecorder if early else GeometryEventRecorder
    try:
        with adapted_runner() if ms2 else contextlib.nullcontext():return worker_original(camera,options,pipe)
    finally:runner.run=oldrun;recorder.ArbiterEventRecorder=oldrecorder

def td3_off(c,o,p):return worker(c,dict(o,early_signals=False,ms2_native=False),p)
def td3_on(c,o,p):return worker(c,dict(o,early_signals=True,ms2_native=False),p)
def ms2_off(c,o,p):return worker(c,dict(o,early_signals=False,ms2_native=True),p)
def ms2_on(c,o,p):return worker(c,dict(o,early_signals=True,ms2_native=True),p)
# Keep the original shared worker importable in each spawned process.
from .shared_arbiter_replay import worker as worker_original

def run(args):
    old=controller.worker;controller.worker=(ms2_on if args.early else ms2_off) if args.ms2 else (td3_on if args.early else td3_off)
    try:return controller.run(args)
    finally:controller.worker=old

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for n in ['dataset','manifest','output']:p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--frames',type=int,default=100);p.add_argument('--start',type=int,default=0);p.add_argument('--timeout-seconds',type=float,default=300)
    for n in ['record-root-initialization','acknowledge-provisional-geometry','early','ms2']:p.add_argument('--'+n,action='store_true')
    run(p.parse_args())

"""Opt-in shared replay with returned-pose geometry events; native experts unchanged."""
import argparse
from pathlib import Path
from . import shared_arbiter_replay as shared

_ORIGINAL_WORKER=shared.worker

def geometry_worker(camera,options,pipe):
    from . import arbiter_event_recorder
    from .geometry_event_recorder import GeometryEventRecorder
    original=arbiter_event_recorder.ArbiterEventRecorder
    arbiter_event_recorder.ArbiterEventRecorder=GeometryEventRecorder
    try:return _ORIGINAL_WORKER(camera,options,pipe)
    finally:arbiter_event_recorder.ArbiterEventRecorder=original

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['dataset','manifest','output']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--frames',type=int,default=20);p.add_argument('--start',type=int,default=0)
    p.add_argument('--timeout-seconds',type=float,default=300)
    p.add_argument('--acknowledge-provisional-geometry',action='store_true')
    p.add_argument('--record-root-initialization',action='store_true')
    args=p.parse_args();shared.worker=geometry_worker;shared.run(args)

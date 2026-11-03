"""Explicit native MS² adapter composed with the unchanged geometry observer."""
import argparse
from pathlib import Path
from . import shared_arbiter_replay as controller
from .geometry_support_replay import worker as geometry_worker
from .ms2_execution_adapter import adapted_runner

def worker(camera,options,pipe):
    with adapted_runner():return geometry_worker(camera,options,pipe)
def worker_on(camera,options,pipe):
    return worker(camera,dict(options,geometry_support_observation=True),pipe)
def run(args):
    original=controller.worker;controller.worker=worker_on if args.geometry else worker
    try:return controller.run(args)
    finally:controller.worker=original
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['dataset','manifest','output']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--frames',type=int,default=100);p.add_argument('--start',type=int,default=0);p.add_argument('--timeout-seconds',type=float,default=300)
    p.add_argument('--record-root-initialization',action='store_true');p.add_argument('--acknowledge-provisional-geometry',action='store_true');p.add_argument('--geometry',action='store_true')
    run(p.parse_args())

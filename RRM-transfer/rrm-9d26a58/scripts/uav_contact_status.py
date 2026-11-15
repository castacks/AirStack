#!/usr/bin/env python3
"""Preserve contact-recorder health before launching a flight (host-side)."""
import argparse,json,time
from pathlib import Path

ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--status',required=True,type=Path);ap.add_argument('--output',required=True,type=Path);ap.add_argument('--seconds',type=float,default=180)
args=ap.parse_args();last=None;end=time.monotonic()+args.seconds
with args.output.open('x',buffering=1) as output:
    while time.monotonic()<end:
        if args.status.exists():
            v=json.loads(args.status.read_text())
            if v.get('wall_unix')!=last:output.write(json.dumps(v)+'\n');last=v.get('wall_unix')
        time.sleep(.1)

#!/usr/bin/env python3
"""Assess a probe and independent PhysX capture by bounded nearest wall receipts.
Receipt alignment does not establish sensor acquisition synchronization. Whole-flight
altitude includes takeoff/landing; phase-specific flight envelopes must be separate.
"""
import argparse,bisect,hashlib,json,math,statistics
from pathlib import Path


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('events',type=Path);ap.add_argument('truth',type=Path);ap.add_argument('--output',required=True,type=Path)
    args=ap.parse_args()
    if args.output.exists():ap.error('output must be new')
    events=[json.loads(s) for s in args.events.read_text().splitlines()]
    truth=[json.loads(s) for s in args.truth.read_text().splitlines()]
    odom=[e for e in events if e['kind']=='odom'];times=[t['wall_time_unix_s'] for t in truth]
    if not odom or not truth or any(b<=a for a,b in zip(times,times[1:])):ap.error('missing or nonmonotonic evidence')
    matched=[]
    for o in odom:
        i=bisect.bisect_left(times,o['wall_unix']);candidates=truth[max(0,i-1):min(len(truth),i+1)]
        if candidates:
            t=min(candidates,key=lambda t:abs(t['wall_time_unix_s']-o['wall_unix']))
            if abs(t['wall_time_unix_s']-o['wall_unix'])<=.15:matched.append((o,t))
    phases={}
    for phase in sorted({o['phase'] for o,t in matched}):
        pairs=[(o,t) for o,t in matched if o['phase']==phase];p=[t['rigid_body_position_m'] for o,t in pairs]
        phases[phase]=dict(matched_samples=len(pairs),min_physical_z_m=min(x[2] for x in p),max_physical_z_m=max(x[2] for x in p),
            max_physical_speed_m_s=max(math.sqrt(sum(a*a for a in t['rigid_body_linear_velocity_m_s'])) for o,t in pairs),
            max_abs_physical_yawrate_rad_s=max(abs(t['rigid_body_angular_velocity_rad_s'][2]) for o,t in pairs),
            mean_estimator_minus_physical_z_m=statistics.mean(o['data']['pose']['pose']['position']['z']-t['rigid_body_position_m'][2] for o,t in pairs))
    result=dict(scope='nearest wall receipts within 150ms; no acquisition synchronization or automatic collision claim',
        phase_metrics=phases,odom_samples=len(odom),matched_odom=len(matched),truth_records=len(truth),
        collision='UNKNOWN',qualification='NOT DEMONSTRATED',max_truth_wall_gap_s=max((b-a for a,b in zip(times,times[1:])),default=None),
        event_sha256=hashlib.sha256(args.events.read_bytes()).hexdigest(),truth_sha256=hashlib.sha256(args.truth.read_bytes()).hexdigest())
    args.output.write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':main()

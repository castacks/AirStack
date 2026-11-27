#!/usr/bin/env python3
"""Independent offline replay of retained M00-C timestamps; no runtime APIs."""
import argparse
from datetime import datetime,timezone
from decimal import Decimal
import json
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'RRM/evidence/M00-C/startup-source-records.json'
OUT=ROOT/'RRM/evidence/M00-E1A/offline-timing-replay.json'

def elapsed(a,b):
    for field in ['domain','episode','boot']:
        if not a.get(field) or a[field]!=b.get(field):raise ValueError('Missing or incompatible '+field)
    value=b['time']-a['time']
    if not value.is_finite() or value<0:raise ValueError('Invalid/reversed time interval')
    return value

def producer_time(line):
    match=re.search(r'^\[(?:ERROR|INFO)\] \[([0-9]+\.[0-9]{9})\]',line)
    if not match:raise ValueError('Missing producer timestamp')
    return Decimal(match[1])

def receipt_time(line):
    match=re.match(r'^(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)\.([0-9]{9})Z ',line)
    if not match:raise ValueError('Missing nanosecond Docker UTC receipt')
    seconds=int(datetime.strptime(match[1],'%Y-%m-%dT%H:%M:%S').replace(tzinfo=timezone.utc).timestamp())
    return Decimal(seconds)+Decimal('0.'+match[2])

def analyze(doc):
    boot=doc['original_ros_logs']['boot_id']
    if boot!=doc['isaac_processes']['boot_id']:raise ValueError('Mismatched historical boot')
    def event(domain,time,locator,line):
        return dict(domain=domain,time=time,episode='20261127_STARTUP_01',boot=boot,source=locator,raw_line=line)
    ros=[]
    for i,entry in enumerate(doc['original_ros_logs']['logs']):
        for n,line in enumerate(entry['text'].splitlines(),1):
            ros.append((i,n,line))
    deadlines=[x for x in ros if 'Actuation NOT_READY: STARTUP_DEADLINE' in x[2]]
    beats=[x for x in ros if 'CON: Got HEARTBEAT' in x[2]]
    if len(deadlines)!=1 or not beats:raise ValueError('Missing/ambiguous initial deadline or absent heartbeat')
    # Later reconnection messages are retained but are not independent startup episodes.
    first=min(beats,key=lambda x:producer_time(x[2]))
    def ros_event(x):return event('ROS_PRODUCER_EPOCH',producer_time(x[2]),f'#/original_ros_logs/logs/{x[0]}/text:line{x[1]}',x[2])
    def docker_event(needle):
        matches=[(g,i,r['text']) for g,group in enumerate(doc['filtered_logs']) if group['container']=='isaac-sim-livestream' for i,r in enumerate(group['records']) if needle in r['text']]
        if len(matches)!=1:raise ValueError('Missing/ambiguous receipt event: '+needle)
        g,i,line=matches[0]
        return event('DOCKER_RECEIPT_UTC',receipt_time(line),f'#/filtered_logs/{g}/records/{i}/text',line)
    pairs=[('L11',ros_event(deadlines[0]),ros_event(first),'First recorded MAVROS connection minus deadline error, not total PX4 startup'),
           ('L03',docker_event('Simulation App Startup Complete'),docker_event('PX4 node at'),'Broad application-complete to graph receipt interval, not exclusive scene latency')]
    records=[]
    for key,a,b,limit in pairs:
        records.append(dict(measurement_id=key,seconds=str(elapsed(a,b)),domain=a['domain'],
                            start={k:str(v) for k,v in a.items()},stop={k:str(v) for k,v in b.items()},
                            interpretation_limit=limit+'; historical n=1, clock calibration/buffering uncertainty unmeasured; not an acceptance threshold'))
    return dict(source='RRM/evidence/M00-C/startup-source-records.json',startup_episodes=1,
                heartbeat_messages_in_retained_log=len(beats),later_reconnections_are_trials=False,
                measurements=records,new_startup_trials=0,execution_authorized=False,
                acquisition_readiness_verified=False,requirement_verdict='NOT_ASSESSED')

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--check',action='store_true');args=parser.parse_args()
    result=json.dumps(analyze(json.loads(SOURCE.read_text())),indent=2)+'\n'
    if args.check:
        if OUT.read_text()!=result:raise ValueError('Nonreproducible retained-evidence replay')
    else:
        with OUT.open('x') as f:f.write(result)
    print(result,end='')
if __name__=='__main__':main()

"""Deterministic arithmetic on pinned retained text only; no runtime interaction."""
from pathlib import Path
from decimal import Decimal
import hashlib
import json
import re

ROOT=Path(__file__).resolve().parents[3]
SOURCE=ROOT/'RRM/evidence/M00-C/startup-source-records.json'
SOURCE_SHA='536f9e8b55647d813dd5be532a4444ccb2639482f74f449229413958588f859c'

def recompute():
    data=SOURCE.read_bytes()
    if hashlib.sha256(data).hexdigest()!=SOURCE_SHA:raise ValueError('Raw input identity mismatch')
    raw=json.loads(data)
    def first_line(index,needle):
        return next(line for line in raw['original_ros_logs']['logs'][index]['text'].splitlines() if needle in line)
    deadline=first_line(0,'Actuation NOT_READY: STARTUP_DEADLINE')
    connection=first_line(1,'CON: Got HEARTBEAT, connected.')
    def epoch(line):return Decimal(re.search(r'\[(\d+\.\d+)\]',line).group(1))
    def receipt(line):
        stamp=line.split()[0]
        if not stamp.endswith('Z'):raise ValueError('Explicit UTC receipt required')
        day,time=stamp[:-1].split('T');whole,fraction=time.split('.')
        h,m,s=map(int,whole.split(':'))
        return day,Decimal(h*3600+m*60+s)+Decimal('0.'+fraction)
    application=raw['filtered_logs'][0]['records'][341]['text']
    graph=raw['filtered_logs'][0]['records'][357]['text']
    if 'Simulation App Startup Complete' not in application or 'created in on-demand graph' not in graph:
        raise ValueError('Retained record locator/meaning mismatch')
    day1,start=receipt(application);day2,stop=receipt(graph)
    if day1!=day2:raise ValueError('Same-day receipt subtraction required')
    results={'deadline_error_to_recorded_mavros_connection_s':str(epoch(connection)-epoch(deadline)),
             'application_complete_to_recorded_vehicle_graph_s':str(stop-start)}
    if list(results.values())!=['19.276162179','74.228771713']:raise ValueError('Historical arithmetic discrepancy')
    return {'source':str(SOURCE.relative_to(ROOT)),'source_sha256':SOURCE_SHA,
            'captured_context_utc':raw['captured_utc'],'results':results,
            'input_records':{'deadline':deadline,'connection':connection,'application':application,'graph':graph},
            'locators':['original_ros_logs/logs/0/text','original_ros_logs/logs/1/text',
                        'filtered_logs/0/records/341','filtered_logs/0/records/357'],
            'clock_domains':['ROS producer log epoch, continuity uncalibrated','Docker UTC receipts, buffering unbounded'],
            'retained_startup_episode_count':1,'new_trials':0,
            'claim':'Exact Decimal differences between specified retained text timestamps only',
            'not_established':['Governing acceptance authority','Calibrated elapsed runtime or exact steady deadline edge',
                               'Independent repeated starts/statistical reliability','Production compatibility or successful recovery',
                               'Flight/safety acceptance or qualification']}

if __name__=='__main__':print(json.dumps(recompute(),indent=2,sort_keys=True))

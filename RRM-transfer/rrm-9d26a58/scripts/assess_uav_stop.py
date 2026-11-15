#!/usr/bin/env python3
"""Assess real monitor PAUSE, physical braking, public cancellation and recovery.
Use alongside assess_uav_contacts.py for independent complete-flight evidence.
"""
import argparse,bisect,json,math,hashlib
from pathlib import Path


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for key in ('events','truth','contact-assessment','output'):ap.add_argument('--'+key,required=True,type=Path)
    ap.add_argument('--automatic-state-fault',action='store_true')
    args=ap.parse_args()
    if args.output.exists():ap.error('output must be new')
    events=[json.loads(s) for s in args.events.read_text().splitlines()]
    truth=[json.loads(s) for s in args.truth.read_text().splitlines()]
    contact=json.loads(args.contact_assessment.read_text())
    trigger='state_fault_injected' if args.automatic_state_fault else 'supervisory_pause'
    pause=[e for e in events if e['kind']==trigger];summary=[e['data'] for e in events if e['kind']=='summary']
    checks=dict(one_pause_command=len(pause)==1,physical_motion_before_pause=False,physical_stop_within_budget=False,physical_stopping_distance=False,physical_braking_evidence_complete=False,
        monitor_received_pause=False,public_goal_canceled=False,final_disarmed=False,complete_contact_evidence=contact.get('verdict')=='PASS')
    metrics={}
    if len(pause)==1:
        command=pause[0];time0=command['wall_unix'];data=[t for t in truth if time0-.15<=t['wall_time_unix_s']<=time0+6]
        if data:
            gaps=[b['wall_time_unix_s']-a['wall_time_unix_s'] for a,b in zip(data,data[1:])]
            checks['physical_braking_evidence_complete']=(data[0]['wall_time_unix_s']<=time0+.15 and data[-1]['wall_time_unix_s']>=time0+5.85 and bool(gaps) and all(0<g<=.5 for g in gaps))
            initial=min(data,key=lambda t:abs(t['wall_time_unix_s']-time0));origin=initial['rigid_body_position_m']
            initial_speed=math.sqrt(sum(v*v for v in initial['rigid_body_linear_velocity_m_s']))
            checks['physical_motion_before_pause']=abs(initial['wall_time_unix_s']-time0)<=.15 and initial_speed>.1
            stable=None;stable_end=None;distances=[];prev=None
            for t in data:
                wall=t['wall_time_unix_s']
                if wall<time0:continue
                speed=math.sqrt(sum(v*v for v in t['rigid_body_linear_velocity_m_s']))
                distances.append(math.dist(t['rigid_body_position_m'],origin))
                if prev is not None and wall-prev>.5:stable=None
                stable=(stable if stable is not None else wall) if speed<.1 else None
                if stable is not None and wall-stable>=1:stable_end=wall;break
                prev=wall
            checks['physical_stop_within_budget']=stable_end is not None and stable_end-time0<=6
            checks['physical_stopping_distance']=bool(distances) and max(distances)<=.5
            metrics=dict(physical_speed_at_pause_m_s=initial_speed,physical_stop_latency_including_dwell_wall_s=stable_end-time0 if stable_end else None,
                         max_physical_stopping_distance_m=max(distances,default=None),scope='nearest wall receipts; independent PhysX speed/position, not reference freeze')
        checks['monitor_received_pause']=any(e['kind']=='controller_log' and 'drone_safety_monitor' in e['data']['name'] and
            (('watchdog_qualification' in e['data']['name'] and 'State health: stale_source_stamp' in e['data']['msg'])
             if args.automatic_state_fault else 'Safety command: PAUSE' in e['data']['msg']) and
            time0<=e['wall_unix']<=time0+2 for e in events)
        if args.automatic_state_fault:
            interval=[e for e in events if time0<=e['wall_unix']<=time0+6]
            relays=[e for e in interval if e['kind']=='fault_relay' and e.get('injected')]
            acknowledgements=[e for e in interval if e['kind']=='fault_ack']
            checks['automatic_input_evidence']=bool(relays) and all(
                abs(e['original_source_s']-e['forwarded_source_s']-10)<1e-6 for e in relays)
            checks['automatic_ack_transition']=any(
                a['data']['data'] is False and b['data']['data'] is True and a['wall_unix']<b['wall_unix']
                for a in acknowledgements for b in acknowledgements)
            checks['automatic_no_manual_pause']=not any(e['kind']=='supervisory_pause' or
                (e['kind']=='controller_log' and 'Safety command: PAUSE' in e['data']['msg']) for e in interval)
            checks['automatic_fault_flag']=any(e['kind']=='fault_watchdog' and e['data']['data'] is True for e in interval)
            checks['automatic_recovery_reconciled']=bool(summary) and all(summary[-1].get(k) is True for k in
                ('automatic_state_fault_stop','qualification_monitor_stopped')) and summary[-1].get('pending_goal_unreconciled') is False
    checks['public_goal_canceled']=any(e['kind']=='action_result' and e['phase']=='navigate_viewpoint' and e['status']==5 for e in events)
    if summary:
        checks['final_disarmed']=summary[-1].get('final_disarmed') is True and summary[-1].get('final_on_ground') is True
    result=dict(verdict='PASS' if all(checks.values()) else 'NOT DEMONSTRATED',checks=checks,metrics=metrics,
                scope='bounded operator-pause/cancel/land test; canceled navigation is not goal completion')
    if args.automatic_state_fault:
        result['scope']='bounded dedicated-monitor stale-source detection/automatic PAUSE; supervisor cancel/LAND, not autonomous landing'
        if len(pause)!=1:result['verdict']='NOT DEMONSTRATED'
    if pause and ((checks['physical_braking_evidence_complete'] and not checks['physical_stop_within_budget']) or (metrics.get('max_physical_stopping_distance_m') is not None and metrics['max_physical_stopping_distance_m']>.5)):result['verdict']='FAIL'
    if contact.get('verdict')=='FAIL':result['verdict']='FAIL'
    if args.automatic_state_fault:
        checks['one_fault_injection']=checks.pop('one_pause_command')
        checks['monitor_detected_source_fault']=checks.pop('monitor_received_pause')
        for old,new in [('physical_speed_at_pause_m_s','physical_speed_at_fault_injection_m_s'),
                        ('physical_stop_latency_including_dwell_wall_s','physical_stop_latency_from_fault_including_dwell_wall_s'),
                        ('max_physical_stopping_distance_m','max_extra_displacement_after_fault_m')]:
            if old in metrics:metrics[new]=metrics.pop(old)
    result['assessor_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    result['input_sha256']={name:hashlib.sha256(getattr(args,name).read_bytes()).hexdigest() for name in ('events','truth','contact_assessment')}
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':main()

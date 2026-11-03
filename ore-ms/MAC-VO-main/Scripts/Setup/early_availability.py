"""No-fit availability persistence from an exact early observed prefix."""
CAMERAS=('rgb','thermal')

def choose(states):
    return 'RGB' if states['rgb'] is True else 'THERMAL' if states['thermal'] is True else 'ABSTAIN'

def supported(event):
    if event.get('execution_error') is not None:return False
    flag=event.get('original_need_interp')
    if type(flag) is not bool:return None
    return not flag and event.get('execution_error') is None

def snapshot(rows,origin,timeline):
    if type(origin) is not int or not 0<=origin<len(rows)-1:raise ValueError('bounded origin required')
    endings={};expected={c:(rows[origin][c+'_left']['time_ns'],rows[origin+1][c+'_left']['time_ns']) for c in CAMERAS}
    for cam in CAMERAS:
        found=[e for e in timeline if e['kind']=='observer_event' and e['camera']==cam and e['event']['kind']=='pair_end' and (e['event']['time0_ns'],e['event']['time1_ns'])==expected[cam]]
        if len(found)!=1:raise ValueError('unique origin pair-end receipts required')
        endings[cam]=found[0]
    epoch=max(e['controller_monotonic_ns'] for e in endings.values());observed=[e for e in timeline if e['kind']=='observer_event' and e['controller_monotonic_ns']<=epoch]
    states={};provenance={}
    for cam in CAMERAS:
        byinterval={};writes={}
        for entry in observed:
            if entry['camera']!=cam:continue
            event=entry['event'];kind=event['kind']
            if kind=='pair_end':
                key=(event['time0_ns'],event['time1_ns'])
                if key in byinterval:raise ValueError('duplicate pair-end interval')
                byinterval[key]=entry
            elif kind=='motion_result_written' and event['first_write_for_submission']:
                writes.setdefault(event['time1_ns'],[]).append(entry)
        a,b=expected[cam];current=supported(byinterval[a,b]['event']);previous=None
        if origin>0:
            old=byinterval.get((rows[origin-1][cam+'_left']['time_ns'],a));previous=supported(old['event']) if old else None
        resolved=None;source=None;pending=0
        for i in range(origin,-1,-1):
            first,last=[rows[k][cam+'_left']['time_ns'] for k in [i,i+1]];health=byinterval.get((first,last))
            if health is None:continue
            value=supported(health['event'])
            if value is None:continue
            motion=writes.get(last,[])
            if value and not motion:pending+=1;continue
            if value:
                m=motion[0]['event'] if len(motion)==1 else None
                value=bool(m is not None and m['time0_ns']==first and m['time1_ns']==last and m.get('stage')=='pair_processing' and m.get('original_need_interp') is False and m.get('derived_camera_delta_xyzw') is not None)
            resolved=value;source={'pair':i,'health_sequence':health['event']['sequence'],'health_receipt_ns':health['controller_monotonic_ns'],'motion_sequences':[m['event']['sequence'] for m in motion],'age_from_own_origin_end_ns':b-last};break
        states[cam]={'health':current,'confirmed':None if current is None or previous is None else current and previous,'resolved':resolved}
        provenance[cam]={'origin_health_sequence':endings[cam]['event']['sequence'],'origin_health_receipt_ns':endings[cam]['controller_monotonic_ns'],'resolved':source,'supported_pending_intervals_skipped':pending}
    return {'epoch_ns':epoch,'states':states,'provenance':provenance,'actions':{name:choose({c:states[c][name] for c in CAMERAS}) for name in ['health','confirmed','resolved']}}

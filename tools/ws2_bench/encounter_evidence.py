"""Offline sampled sensor-exposure overlap with realized corridor encounters.

This module is deliberately standalone. It never schedules flights, regenerates
geometry, supplies GT to a planner, or concludes that an attack caused failure.
"""
import bisect
import json
import math
from pathlib import Path

DEFAULT_THRESHOLDS = {
    'obstacle_approach_m': 4., 'obstacle_departure_m': 2.,
    'obstacle_near_distance_m': 4., 'obstacle_passing_margin_m': .5,
    'turn_approach_m': 3., 'turn_departure_m': 2., 'turn_zone_padding_m': .5,
    'turn_vertex_angle_deg': 3., 'turn_group_gap_m': 2.,
    'max_pose_gap_s': .5, 'timestamp_tolerance_s': .05,
    'projection_ambiguity_m': .15, 'projection_separation_m': 2.,
}
LIMITATIONS = [
    'GT route/pose is privileged offline diagnostic evidence, never a target-network input.',
    'Approach and turn phases are geometric regions with explicit thresholds, not camera visibility or an actual evasive maneuver.',
    'Frame availability and worker telemetry are asynchronous; availability is not the exact inference or command time.',
    'Worker confirmation joins exact sequence and input timestamp across this episode trace. Missing joins mean unknown consumption, not evidence that a frame was ignored.',
    'Sample counts and first/last sample times are not continuous exposure durations. Unsampled intervals remain unknown.',
    'GT positions use short-gap linear interpolation without extrapolation; large gaps, clock resets and ambiguous route projections are excluded.',
    'Distance is drone-center to the realized box, not rotor clearance, camera-to-object range or the PhysX collision oracle.',
    'Observed overlap supports follow-up test design, not attack causality, a failure threshold or comparative agent advantage.',
]


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _vector(value, length):
    return isinstance(value, (list, tuple)) and len(value) >= length and all(_finite(x) for x in value[:length])


def _route(points):
    if not isinstance(points, list) or len(points) < 2 or any(not _vector(p, 2) for p in points):
        raise ValueError('Realized route must contain finite XY points')
    result=[]; distance=0.
    for a,b in zip(points,points[1:]):
        length=math.dist(a[:2],b[:2])
        if length <= 1e-9:raise ValueError('Realized route contains a zero-length segment')
        result.append(dict(a=a[:2],b=b[:2],length=length,start=distance,
                           tangent=[(b[j]-a[j])/length for j in (0,1)]))
        distance+=length
    return result,distance


def _projection(position,segments,thresholds,width):
    candidates=[]
    for s in segments:
        along=max(0.,min(s['length'],sum((position[j]-s['a'][j])*s['tangent'][j] for j in (0,1))))
        nearest=[s['a'][j]+along*s['tangent'][j] for j in (0,1)]
        candidates.append((math.dist(position[:2],nearest),s['start']+along))
    distance,progress=min(candidates)
    if distance>width/2:return None,'outside_route_envelope'
    if any(d<=distance+thresholds['projection_ambiguity_m'] and abs(p-progress)>thresholds['projection_separation_m']
           for d,p in candidates):return None,'ambiguous_route_projection'
    return dict(route_s_m=progress,route_distance_m=distance),None


def _turns(segments,length,t):
    vertices=[]
    for before,after in zip(segments,segments[1:]):
        u,v=before['tangent'],after['tangent']
        angle=math.degrees(math.atan2(u[0]*v[1]-u[1]*v[0],u[0]*v[0]+u[1]*v[1]))
        if abs(angle)>=t['turn_vertex_angle_deg']:
            vertices.append(dict(route_s=after['start'],angle=angle))
    groups=[]
    for vertex in vertices:
        if (not groups or vertex['route_s']-groups[-1][-1]['route_s']>t['turn_group_gap_m'] or
                vertex['angle']*groups[-1][-1]['angle']<=0):groups.append([])
        groups[-1].append(vertex)
    return [dict(index=i,route_start_m=max(0.,group[0]['route_s']-t['turn_zone_padding_m']),
                 route_end_m=min(length,group[-1]['route_s']+t['turn_zone_padding_m']),
                 signed_heading_change_deg=sum(v['angle'] for v in group),
                 direction='left' if group[0]['angle']>0 else 'right',vertex_count=len(group))
            for i,group in enumerate(groups)]


def _box_distance(position,box):
    dx,dy=position[0]-box['center'][0],position[1]-box['center'][1]
    c,s=math.cos(box['yaw']),math.sin(box['yaw'])
    local=(dx*c+dy*s,-dx*s+dy*c,position[2]-box['center'][2])
    return math.sqrt(sum(max(abs(local[j])-box['size'][j]/2,0.)**2 for j in range(3)))


def _pose_at(stamp,times,positions,max_gap):
    index=bisect.bisect_left(times,stamp)
    if index<len(times) and abs(times[index]-stamp)<1e-8:return positions[index],0.,None
    if index==0 or index==len(times):return None,None,'pose_extrapolation_required'
    gap=times[index]-times[index-1]
    if gap>max_gap:return None,gap,'pose_gap_too_large'
    f=(stamp-times[index-1])/gap
    return [positions[index-1][j]*(1-f)+positions[index][j]*f for j in range(3)],gap,None


def _window(schedule,elapsed):
    if schedule is None:return True
    if (not isinstance(schedule,dict) or set(schedule)!={'start_s','duration_s','period_s'} or
            any(not _finite(v) or v<0 for v in schedule.values())):return None
    if schedule['period_s'] and not 0<schedule['duration_s']<schedule['period_s']:return None
    if elapsed is None:return False
    if not _finite(elapsed):return None
    if elapsed<schedule['start_s']:return False
    offset=elapsed-schedule['start_s']
    if schedule['period_s']:offset%=schedule['period_s']
    return schedule['duration_s']==0 or offset<schedule['duration_s']


def _states(row,condition,available,tolerance):
    disturbance=row.get('latest_input_disturbance')
    if not isinstance(disturbance,dict):return dict(noise='unknown',delay='unknown'),['missing_frame_disturbance']
    states={};issues=[]
    for factor,setting,key,schedule in [('noise','rgb_noise','rgb_noise_stddev','noise_schedule'),
                                         ('delay','delay','fixed_delay_s','delay_schedule')]:
        amplitude=disturbance.get(key);requested=condition.get(setting,0.)
        state='unknown'
        window=_window(condition.get(schedule),disturbance.get('schedule_elapsed_s'))
        if (not _finite(amplitude) or amplitude<0 or not _finite(requested) or requested<0 or window is None or
                disturbance.get(schedule)!=condition.get(schedule)):
            issues.append(factor+'_invalid_metadata')
        elif amplitude!=(requested if window else 0.):
            issues.append(factor+'_schedule_or_magnitude_mismatch')
        elif factor=='noise':
            rmse=row.get('latest_input_rgb_noise_rmse')
            if not _finite(rmse) or rmse<0 or (rmse>0)!=(amplitude>0):issues.append('noise_missing_or_inconsistent_pixel_evidence')
            else:state='active' if amplitude>0 else 'inactive'
        else:
            if amplitude>0 and available-row['latest_input_stamp']<amplitude-tolerance:
                issues.append('delay_missing_latency_evidence')
            else:state='active' if amplitude>0 else 'inactive'
        states[factor]=state
    return states,issues


def _bucket():
    return dict(frame_count=0,worker_confirmed_frames=0,worker_confirmation_unknown_frames=0,
                first_sample_sim_s=None,last_sample_sim_s=None,
                noise=dict(active=0,inactive=0,unknown=0),delay=dict(active=0,inactive=0,unknown=0),
                worker_confirmed_noise=dict(active=0,inactive=0,unknown=0),
                worker_confirmed_delay=dict(active=0,inactive=0,unknown=0))


def _add(bucket,frame,stamp):
    bucket['frame_count']+=1;bucket['worker_confirmed_frames']+=int(frame['worker_confirmed'])
    bucket['worker_confirmation_unknown_frames']+=int(not frame['worker_confirmed'])
    # Capture order and observed availability order may differ. These fields
    # are timestamp extrema, never incidental iteration endpoints.
    bucket['first_sample_sim_s']=stamp if bucket['first_sample_sim_s'] is None else min(bucket['first_sample_sim_s'],stamp)
    bucket['last_sample_sim_s']=stamp if bucket['last_sample_sim_s'] is None else max(bucket['last_sample_sim_s'],stamp)
    for factor in ('noise','delay'):
        bucket[factor][frame['states'][factor]]+=1
        if frame['worker_confirmed']:
            bucket['worker_confirmed_'+factor][frame['states'][factor]]+=1


def _worker_index(trace,issue):
    """Index observed worker identities, not rows or current bridge frames.

    The recorder owns run association. Nested consumed metadata, when present,
    must agree with its outer worker identity; a conflicting record cannot fall
    back to the legacy join. Optional revision/pixel metadata further constrain
    a join without treating missing legacy fields as zero-valued measurements.
    """
    result={}
    for row in trace:
        if not isinstance(row,dict):continue
        worker=row.get('worker')
        if not isinstance(worker,dict):continue
        sequence,stamp=worker.get('sequence'),worker.get('sim_stamp')
        if type(sequence) is not int or sequence<0 or not _finite(stamp):
            issue('invalid_worker_identity');continue
        # The scene sample precedes bridge/worker HTTP reads. It is not an
        # observation clock and must not reject a slightly later input stamp.
        direct=worker.get('consumed_input')
        proof=dict(source='exact_episode_sequence_stamp_join',revision=None,
                   disturbance=None,rgb_noise_rmse=None)
        if direct is not None:
            def reject_direct(name):
                issue(name)
                marker=dict(invalid_direct=True)
                records=result.setdefault((sequence,stamp),[])
                if marker not in records:records.append(marker)
            if (not isinstance(direct,dict) or type(direct.get('schema_version')) is not int
                    or direct['schema_version']!=1 or type(direct.get('sequence')) is not int
                    or direct['sequence']!=sequence or not _finite(direct.get('sim_stamp'))
                    or direct['sim_stamp']!=stamp):
                reject_direct('invalid_consumed_input_identity');continue
            revision=direct.get('disturbance_revision')
            if revision is not None and (type(revision) is not int or revision<0):
                reject_direct('invalid_consumed_input_revision');continue
            disturbance=direct.get('sensor_disturbance')
            if disturbance is not None and not isinstance(disturbance,dict):
                reject_direct('invalid_consumed_input_disturbance');continue
            rmse=direct.get('rgb_noise_rmse')
            if rmse is not None and (not _finite(rmse) or rmse<0):
                reject_direct('invalid_consumed_input_pixel_measurement');continue
            proof.update(source='worker_consumed_input_metadata',revision=revision,
                         disturbance=disturbance,rgb_noise_rmse=rmse)
        records=result.setdefault((sequence,stamp),[])
        if proof not in records:records.append(proof)
    return result


def _worker_confirmation(frame,proofs,identity_count,issue):
    sources=set()
    if not frame['metadata_consistent']:
        issue('worker_join_conflicting_bridge_metadata');return sources
    if any(proof.get('invalid_direct') for proof in proofs):
        issue('worker_join_invalid_direct_metadata');return sources
    for field in ('revision','disturbance','rgb_noise_rmse'):
        known=[proof[field] for proof in proofs if proof[field] is not None]
        if known and any(value!=known[0] for value in known[1:]):
            issue('worker_join_conflicting_consumed_metadata');return sources
    for proof in proofs:
        for field,problem in [('revision','worker_frame_revision_mismatch'),
                              ('disturbance','worker_frame_disturbance_mismatch'),
                              ('rgb_noise_rmse','worker_frame_pixel_measurement_mismatch')]:
            if proof[field] is not None and proof[field]!=frame[field]:
                issue(problem);return sources
    for proof in proofs:
        if proof['revision'] is None and identity_count>1:
            issue('ambiguous_worker_frame_revision');continue
        sources.add(proof['source'])
    return sources


def analyze_encounters(trace,geometry,condition,thresholds=None):
    """Return sampled overlap at input capture and first observed availability.

    Threshold overrides are explicit and recorded in the output. Temporal pose
    matching uses only saved GT samples; it never extrapolates a trajectory.
    """
    t=dict(DEFAULT_THRESHOLDS)
    if thresholds:
        if set(thresholds)-set(t):raise ValueError('Unknown encounter thresholds')
        t.update(thresholds)
    if any(not _finite(v) or v<=0 for v in t.values()):raise ValueError('Encounter thresholds must be positive finite numbers')
    if not isinstance(trace,list) or not isinstance(geometry,dict) or not isinstance(condition,dict):
        raise ValueError('Expected trace list, saved geometry and condition mappings')
    segments,length=_route(geometry.get('route'))
    width=geometry.get('spec',{}).get('width')
    if not _finite(width) or width<=0:raise ValueError('Saved physical corridor width required')
    obstacles=geometry.get('obstacles',[])
    if not isinstance(obstacles,list):raise ValueError('Saved obstacles must be an array')
    for box in obstacles:
        if (not isinstance(box,dict) or not _vector(box.get('center'),3) or not _vector(box.get('size'),3) or
                any(v<=0 for v in box['size']) or not _finite(box.get('yaw')) or not _finite(box.get('route_s'))):
            raise ValueError('Saved obstacle needs center, size, yaw and route_s')
    turns=_turns(segments,length,t)
    output=dict(schema='encounter_evidence_v2',status='complete',geometry_id=geometry.get('placement_id'),
        thresholds=t,route_length_m=length,trace_rows=len(trace),factors={
            'noise':dict(configured_amplitude=condition.get('rgb_noise',0.),schedule=condition.get('noise_schedule')),
            'delay':dict(configured_amplitude=condition.get('delay',0.),schedule=condition.get('delay_schedule'))},
        alignment=dict(capture='GT interpolation at captured input timestamp',
                       available='GT interpolation when the bridge frame was first observed available; not exact model consumption',
                       worker='Exact same-episode sequence/input timestamp join across all saved worker observations; consumed_input metadata checked when provided'),
        obstacles=[dict(index=i,route_s_m=b['route_s'],center_m=b['center'],size_m=b['size'],yaw_rad=b['yaw'],
                        at_capture={},when_available={}) for i,b in enumerate(obstacles)],
        turns=[dict(v,at_capture={},when_available={}) for v in turns],
        overall=_bucket(),gaps={},issues={},limitations=list(LIMITATIONS))
    def issue(name):output['issues'][name]=output['issues'].get(name,0)+1
    times=[];positions=[]
    for row in trace:
        if not isinstance(row,dict) or not _finite(row.get('sim_time_s')) or not _vector(row.get('position_m'),3):
            issue('invalid_gt_row');continue
        stamp=row['sim_time_s']
        if times and stamp<times[-1]:
            output['status']='unavailable_clock_regression';issue('simulation_clock_regressed');return output
        if times and stamp==times[-1]:issue('duplicate_gt_time');continue
        times.append(stamp);positions.append(row['position_m'][:3])
    gaps=[b-a for a,b in zip(times,times[1:])]
    output['gaps'].update(valid_gt_samples=len(times),maximum_gt_gap_s=max(gaps,default=None),
                          gaps_exceeding_pose_limit=sum(g>t['max_pose_gap_s'] for g in gaps))
    if not times:output['status']='unavailable_no_gt';return output
    workers=_worker_index(trace,issue)
    frames={}
    for row in trace:
        if not isinstance(row,dict):continue
        stamp=row.get('latest_input_stamp');sequence=row.get('camera_sequence')
        if not _finite(stamp):issue('missing_input_timestamp');continue
        d=row.get('latest_input_disturbance')
        if not isinstance(d,dict):d={}
        if type(sequence) is not int or sequence<0:
            issue('invalid_bridge_input_sequence');continue
        revision=d.get('revision')
        if revision is not None and (type(revision) is not int or revision<0):
            issue('invalid_bridge_disturbance_revision');continue
        identity=(d.get('revision'),sequence,stamp)
        if identity in frames:
            previous=frames[identity]
            if (previous['disturbance']!=d or
                    previous['rgb_noise_rmse']!=row.get('latest_input_rgb_noise_rmse')):
                previous['metadata_consistent']=False
                previous['states']=dict(noise='unknown',delay='unknown')
                issue('conflicting_duplicate_frame_metadata')
            continue
        live=row.get('sensor_disturbance') or {}
        start,elapsed=live.get('schedule_start_sim_s'),live.get('schedule_elapsed_s')
        available=start+elapsed if _finite(start) and _finite(elapsed) else row.get('sim_time_s')
        if not _finite(available) or available<stamp-t['timestamp_tolerance_s']:
            issue('invalid_availability_timestamp');continue
        states,problems=_states(row,condition,available,t['timestamp_tolerance_s'])
        for problem in problems:issue(problem)
        frames[identity]=dict(capture=stamp,available=available,states=states,worker_confirmed=False,
                              sequence=sequence,revision=revision,disturbance=d,
                              rgb_noise_rmse=row.get('latest_input_rgb_noise_rmse'),
                              metadata_consistent=True,
                              availability_clock_aligned=available>=stamp-1e-8)
    frame_identities={}
    for frame in frames.values():
        key=(frame['sequence'],frame['capture'])
        frame_identities[key]=frame_identities.get(key,0)+1
    joined=set();source_counts={}
    for frame in frames.values():
        key=(frame['sequence'],frame['capture'])
        sources=_worker_confirmation(frame,workers.get(key,[]),frame_identities[key],issue)
        frame['worker_confirmed']=bool(sources)
        if sources:joined.add(key)
        for source in sources:source_counts[source]=source_counts.get(source,0)+1
    output['worker_evidence']=dict(unique_observed_input_identities=len(workers),
        joined_input_identities=len(joined),inputs_without_matching_available_frame=len(set(workers)-joined),
        confirmation_sources=source_counts,
        limitation='Unjoined worker inputs have no verified bridge availability/encounter assignment here; unconfirmed bridge frames have unknown consumption.')
    output['gaps']['unique_captured_frames']=len(frames)
    output['gaps']['pose_unmatched_capture']=output['gaps']['pose_unmatched_available']=0
    for frame in sorted(frames.values(),key=lambda f:f['capture']):
        _add(output['overall'],frame,frame['capture'])
        for when,field in [('capture','at_capture'),('available','when_available')]:
            stamp=frame[when]
            if when=='available' and not frame['availability_clock_aligned']:
                # A small clock skew may be within the latency audit tolerance,
                # but cannot place availability before acquisition physically.
                issue('available_precedes_capture_within_tolerance')
                output['gaps']['pose_unmatched_available']+=1
                continue
            position,gap,error=_pose_at(stamp,times,positions,t['max_pose_gap_s'])
            if error:
                issue(when+'_'+error);output['gaps']['pose_unmatched_'+when]+=1;continue
            projection,error=_projection(position,segments,t,width)
            if error:issue(when+'_'+error);continue
            progress=projection['route_s_m']
            for box,entry in zip(obstacles,output['obstacles']):
                if _box_distance(position,box)>t['obstacle_near_distance_m']:continue
                delta=box['route_s']-progress
                passing=max(box['size'][:2])/2+t['obstacle_passing_margin_m']
                phase=('passing' if abs(delta)<=passing else
                       'approach' if 0<delta<=t['obstacle_approach_m'] else
                       'departing' if -t['obstacle_departure_m']<=delta<0 else None)
                if phase:_add(entry[field].setdefault(phase,_bucket()),frame,stamp)
            for entry in output['turns']:
                first,last=entry['route_start_m'],entry['route_end_m']
                phase=('turn' if first<=progress<=last else
                       'approach' if first-t['turn_approach_m']<=progress<first else
                       'departing' if last<progress<=last+t['turn_departure_m'] else None)
                if phase:_add(entry[field].setdefault(phase,_bucket()),frame,stamp)
    if not frames:output['status']='unavailable_no_input_frames'
    return output


def analyze_episode(directory,thresholds=None):
    """Read saved episode artifacts, return evidence, write nothing."""
    directory=Path(directory)
    def read(name):return json.loads((directory/name).read_text())
    scenario=read('scenario.json')
    result=analyze_encounters(read('diagnostic_trace.json'),read('corridor_geometry.json'),scenario['condition'],thresholds)
    result.update(episode_directory=str(directory.resolve()),planner=scenario.get('planner'))
    return result

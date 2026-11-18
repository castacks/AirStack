"""Measured camera-program coverage; absence of evidence is never a success."""
import json
from pathlib import Path

from camera_program import identity


def interruption_at_guard(rows, condition, run_id):
    """Identify a live injected interruption, never merely a configured one.

    The episode watchdog keeps its original threshold. Recent arriving frames
    must actually be suppressed while the same-run worker input stays fixed.
    A dead camera/bridge cannot advance these counters and stays an infra error.
    """
    if not rows or not condition.get('camera_program'):
        return None
    end = rows[-1]['sim_time_s']
    recent = [row for row in rows if end-.75 <= row['sim_time_s'] <= end]
    if len(recent) < 2:
        return None
    first, last = recent[0], recent[-1]
    program_id = identity(condition['camera_program'])
    before = (first.get('sensor_disturbance') or {}).get('camera_program_telemetry') or {}
    after = (last.get('sensor_disturbance') or {}).get('camera_program_telemetry') or {}
    current = (last.get('sensor_disturbance') or {}).get('camera_program_state') or {}
    if any(value.get('program_id') != program_id for value in (before, after, current)):
        return None
    old, new = first.get('worker') or {}, last.get('worker') or {}
    if (old.get('run_id') != run_id or new.get('run_id') != run_id
            or new.get('sequence') is None or new.get('sim_stamp') is None
            or (old.get('sequence'), old.get('sim_stamp')) != (new['sequence'], new['sim_stamp'])):
        return None
    factors = []
    for factor, key in [('frame_drop', 'dropped_arrivals'), ('frame_freeze', 'frozen_arrivals')]:
        a, b = before.get(key), after.get(key)
        if (factor in current.get('active_factors', []) and type(a) is int and type(b) is int and b > a >= 0):
            factors.append(factor)
    if not factors:
        return None
    return dict(schema='injected_camera_guard_v1', program_id=program_id,
                observed_factors=factors, observation_start_sim_s=first['sim_time_s'],
                observation_end_sim_s=end, worker_sequence=new['sequence'],
                worker_input_stamp_sim_s=new['sim_stamp'],
                interpretation='Bench watchdog intervention after measured input suppression; not a collision or proof of a network defect.')


def audit(rows, condition, run_id):
    program_id=identity(condition['camera_program'])
    configured={e['factor'] for e in condition['camera_program']}
    counts={f:set() for f in ('rgb_noise','delay')}
    suppressed={f:0 for f in ('frame_drop','frame_freeze')}
    stalls={f:0 for f in suppressed}
    previous=None
    for row in rows:
        d=row.get('sensor_disturbance') or {}
        stats=d.get('camera_program_telemetry') or {}
        if stats.get('program_id')!=program_id:continue
        state=d.get('camera_program_state') or {}
        for f,key in [('frame_drop','dropped_arrivals'),('frame_freeze','frozen_arrivals')]:
            count=stats.get(key,0)
            if type(count) is int and count>=0:suppressed[f]=max(suppressed[f],count)
            # Measured suppression plus a same-run worker input that stopped
            # advancing. This is an observation of unavailable fresh input,
            # not an assertion that the network repeatedly processed it.
            worker=row.get('worker') or {}
            if previous and f in state.get('active_factors',[]) and worker.get('run_id')==run_id:
                old_worker=previous.get('worker') or {}
                t=row.get('sim_time_s');old_t=previous.get('sim_time_s')
                if (worker.get('sequence') is not None and worker.get('sim_stamp') is not None
                        and worker.get('sequence')==old_worker.get('sequence')
                        and worker.get('sim_stamp')==old_worker.get('sim_stamp')
                        and old_worker.get('run_id')==run_id and isinstance(t,(int,float))
                        and isinstance(old_t,(int,float)) and t>old_t
                        and count>(previous.get('sensor_disturbance',{}).get('camera_program_telemetry',{}).get(key,0))):
                    stalls[f]+=1
        sample=row.get('latest_input_disturbance') or {}
        p=sample.get('camera_program_state') or {}
        if p.get('program_id')==program_id:
            frame=(row.get('camera_sequence'),row.get('latest_input_stamp'))
            if all(v is not None for v in frame):
                for f,field,value in [('rgb_noise','rgb_noise_stddev','rgb_noise'),('delay','fixed_delay_s','delay')]:
                    if f not in p.get('active_factors',[]) or sample.get(field)!=condition[value] or condition[value]<=0:continue
                    if f=='rgb_noise' and not (row.get('latest_input_rgb_noise_rmse') or 0)>0:continue
                    if f=='delay':
                        age=row.get('camera_age_sim_s')
                        if not isinstance(age,(int,float)) or age<condition[value]-.05:continue
                    counts[f].add(frame)
        previous=row
    factors={}
    for f,key in [('rgb_noise','noise'),('delay','delay')]:
        exposed=bool(counts[f]);requested=f in configured
        factors[key]=dict(requested=requested,active_frames=len(counts[f]),exposure_verified=exposed,
                          status='exposed' if exposed else 'not_observed')
    for f in suppressed:
        verified=suppressed[f]>0 and stalls[f]>0
        factors[f]=dict(requested=f in configured,suppressed_arrivals=suppressed[f],
                       stalled_worker_observations=stalls[f],exposure_verified=verified,
                       status='exposed' if verified else 'not_observed')
    return dict(schema='camera_program_exposure_v1',program_id=program_id,status='complete',factors=factors,
                exposure_verified=all(v['exposure_verified'] for v in factors.values() if v['requested']),
                limitation='Recorded coverage only. Suppression and unchanged observed worker input do not establish a failure cause or exact model processing latency.')


def record(folder, condition):
    from agent_context import _trace_document
    from episode import RUNTIME
    folder=Path(folder).resolve()
    result=json.loads((folder/'result.json').read_text())
    result['result_dir']=str(folder)
    rows,provenance=_trace_document(result,RUNTIME)
    if rows is None:raise ValueError('Unverified camera trace: '+str(provenance))
    saved=json.loads((folder/'scenario.json').read_text())['condition']
    if saved!=condition:raise ValueError('Camera exposure condition differs from executed scenario')
    return dict(audit(rows,condition,str(folder)),source=provenance)

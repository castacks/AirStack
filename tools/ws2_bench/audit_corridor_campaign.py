"""Read-only audit of adaptive corridor flights, independent of LLM claims."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from episode import fingerprint,resolved
from corridor_space import validate_action,action_to_episode
from corridors import geometry,patch_corners
from campaign import clean_twin
from audit_expanded import audit_patch
from mission import SUCCESSES,EXCLUDED
from diagnostic_evaluation import workflow_evidence

BEHAVIOR=('episode.py','flight_action.py','conditions.py','mission.py','corridors.py',
          'corridor_scene.py','launch_office.py','model_adapters.py')


def read(path):return json.loads(Path(path).read_text())


def _active(schedule,elapsed):
    if schedule is None:return True
    if elapsed is None or elapsed<schedule['start_s']:return False
    offset=elapsed-schedule['start_s']
    if schedule['period_s']:offset%=schedule['period_s']
    return schedule['duration_s']==0 or offset<schedule['duration_s']


def audit_scheduled_sensors(folder,condition,initial):
    """Verify captured pixels/latency, not merely requested ROS parameters.

    A short flight can end before its attack window. It is a valid execution,
    but exposure_verified is false and cannot support an attack-effect claim.
    """
    configured={dst:condition[src] for dst,src in [('seed','seed'),('rgb_noise_stddev','rgb_noise'),
                 ('fixed_delay_s','delay'),('depth_noise_stddev_m','depth_noise')]}
    traces=read(Path(folder)/'diagnostic_trace.json')
    frames=[dict(stamp=initial['stamp'],now=initial['stamp'],disturbance=initial['sensor_disturbance'],
                 rmse=initial['rgb_noise_rmse'],initial=True)]
    seen=set()
    unaligned_scene_frames=0;clock_sources={}
    for row in traces:
        disturbance=row.get('latest_input_disturbance');stamp=row.get('latest_input_stamp')
        if disturbance is None or stamp is None:continue
        key=(stamp,disturbance.get('revision'))
        if key in seen:continue
        seen.add(key)
        # Scene JSON is read before the HTTP bridge call. The bridge may have
        # advanced several simulation ticks by then. Its own live schedule
        # clock shares the captured-frame timebase; scene-frame subtraction can
        # otherwise be negative even for an undelayed frame.
        live=row.get('sensor_disturbance') or {}
        start,elapsed=live.get('schedule_start_sim_s'),live.get('schedule_elapsed_s')
        aligned=all(type(value) in (int,float) and math.isfinite(value) for value in (start,elapsed))
        now=start+elapsed if aligned else row['sim_time_s']
        source='bridge_sim_clock' if aligned else 'scene_snapshot_clock_fallback'
        clock_sources[source]=clock_sources.get(source,0)+1
        if row['sim_time_s']-stamp<-.05:unaligned_scene_frames+=1
        frames.append(dict(stamp=stamp,now=now,disturbance=disturbance,
                           rmse=row.get('latest_input_rgb_noise_rmse'),initial=False))
    assert len(frames)>1,'Scheduled sensor validation requires captured-frame diagnostic telemetry'
    counts={key:dict(active_frames=0,inactive_frames=0,unknown_frames=0) for key in ('noise','delay')}
    for frame in frames:
        d=frame['disturbance'];elapsed=d.get('schedule_elapsed_s')
        assert d.get('configured')==configured,'configured sensor magnitudes differ'
        for short,key,magnitude,effective in [('noise','noise_schedule','rgb_noise','rgb_noise_stddev'),
                                              ('delay','delay_schedule','delay','fixed_delay_s')]:
            assert d.get(key)==condition.get(key),'sensor schedule differs: '+key
            expected=condition[magnitude] if _active(condition.get(key),elapsed) else 0.
            assert d[effective]==expected,'sensor schedule effective value mismatch: '+key
            if not frame['initial']:
                # A positive request without timestamp support is unknown,
                # never counted as measured exposure. Zero delay makes no
                # positive latency claim, so asynchronous scene sampling does
                # not invalidate an otherwise unperturbed captured frame.
                state=('unknown_frames' if short=='delay' and expected>0 and frame['now']-frame['stamp']<expected-.05
                       else 'active_frames' if expected>0 else 'inactive_frames')
                counts[short][state]+=1
        assert type(frame['rmse']) in (int,float),'missing measured RGB disturbance'
        assert (frame['rmse']>0)==(d['rgb_noise_stddev']>0),'image pixels do not match scheduled noise'
    requested={'noise':condition['rgb_noise']>0,'delay':condition['delay']>0}
    for key in counts:
        counts[key]['requested']=requested[key]
        counts[key]['exposure_verified']=not requested[key] or counts[key]['active_frames']>0
        counts[key]['status']=('disabled' if not requested[key] else 'exposed' if counts[key]['active_frames'] else
                               'exposure_unknown' if counts[key]['unknown_frames'] else 'not_exposed_before_termination')
    return dict(captured_frames=len(frames)-1,factors=counts,
                exposure_verified=all(v['exposure_verified'] for v in counts.values()),
                timing_evidence=dict(measurement_sources=clock_sources,asynchronous_scene_frame_samples=unaligned_scene_frames,
                                     unknown_delay_frames=counts['delay']['unknown_frames']),
                limitation='Timing follows captured frames; finite frames do not validate unsampled windows or attack causality.')


def scheduled_effects(result_dir,condition):
    """Immediate per-flight exposure evidence for agent/report consumption.

    Return None for legacy constant-disturbance trials. Validation errors are
    explicit evidence errors; callers must not interpret them as zero exposure.
    """
    if not any(k in condition for k in ('noise_schedule','delay_schedule')):return None
    initial=read(Path(result_dir)/'condition_evidence.json')['frame']
    return audit_scheduled_sensors(result_dir,condition,initial)


def audit(root,expected_sources=None):
    root=Path(root);config=read(root/'config.json');history=read(root/'history.json')
    rows=[];errors=[];runtime=None;model=None
    for entry in history:
        decision=entry['decision'];number=decision['round'];pair=entry['pairs'][0];cases={};evidence={}
        try:
            action=validate_action(decision['action'],config['planners'][0])
            expected=resolved(action_to_episode(action,config['planners'][0],config['seed'],
                f"{config['backend']} round {number} {config['planners'][0]}",config['mission']))
            assert expected['condition']==decision['condition'],'resolved action differs from saved condition'
            if decision.get('diagnosis'):
                from matched_policy import audit_diagnosis_decision
                call=(read(root/'llm_calls'/(decision['llm_call_id']+'.json'))
                      if decision.get('llm_call_id') else None)
                audit_diagnosis_decision(decision,history[:6],call=call,
                    first_decision=history[6]['decision'] if len(history)>6 else None)
            elif decision.get('llm_call_id'):
                call=read(root/'llm_calls'/(decision['llm_call_id']+'.json'))
                assert call['status']=='validated','unvalidated LLM selection'
                output=call['attempts'][-1]['output']
                assert validate_action(output['action'],config['planners'][0])==action,'executed action differs from Claude output'
                assert output['experiment']==decision['experiment'],'experiment purpose differs from Claude output'
            for role in ('clean','perturbed'):
                saved=pair[role]
                if saved.get('executed') is False:
                    assert role=='perturbed' and pair['clean']['outcome'] not in SUCCESSES,'invalid skipped slot'
                    continue
                folder=Path(saved['result_dir']);result=read(folder/'result.json');case=read(folder/'scenario.json')
                cases[role]=case;ev=read(folder/'condition_evidence.json');evidence[role]=ev
                assert result['outcome'] not in EXCLUDED,'infrastructure/operator interruption'
                assert result['configuration_hash']==fingerprint(case)==saved['configuration_hash'],'scenario/result hash mismatch'
                assert case==(clean_twin(expected) if role=='clean' else expected),'executed scenario differs from selected action'
                assert not result['cleanup_errors'],'cleanup failed'
                g=geometry(case['condition']['corridor'],case['condition']['layout_seed'])
                assert result['scene_placement_id']==g['placement_id']==read(folder/'corridor_geometry.json')['placement_id'],'actual geometry mismatch'
                assert ev['scene']['realized_layout']['placement_id']==g['placement_id'],'simulator geometry mismatch'
                assert ev['requested']==case['condition'],'applied condition request mismatch'
                c=case['condition'];frame=ev['frame'];applied=frame['sensor_disturbance']
                scheduled=None
                if 'noise_schedule' in c or 'delay_schedule' in c:
                    scheduled=audit_scheduled_sensors(folder,c,frame)
                else:
                    for actual,key in [('seed','seed'),('rgb_noise_stddev','rgb_noise'),('fixed_delay_s','delay'),('depth_noise_stddev_m','depth_noise')]:
                        assert applied[actual]==c[key],'sensor application mismatch: '+key
                    assert (frame['rgb_noise_rmse']>0)==(c['rgb_noise']>0),'missing/unexpected image noise'
                    assert ev['scene']['sim_time']-frame['stamp']>=c['delay']-.05,'delay missing from timestamps'
                assert ev['scene']['condition']['light']==c['light'],'lighting mismatch'
                patch_errors=audit_patch(folder,case,ev)
                assert not patch_errors,'; '.join(patch_errors)
                if c.get('patch_mount'):
                    actual=ev['scene']['patch_corners_world'];wanted=patch_corners(g,c['patch_size'],c['patch_mount'])
                    assert len(actual)==4 and all(math.dist(a,b)<1e-4 for a,b in zip(actual,wanted)),'physical patch mount mismatch'
                preview=read(folder/'inference.json')
                assert preview['run_id']==str(folder.resolve()),'inference from another run'
                samples=read(folder/'samples.json')
                assert len(samples)>1 and all(b['sim_time_s']>a['sim_time_s'] for a,b in zip(samples,samples[1:])),'invalid GT timing'
                if result['outcome']=='route_completed':assert result['metrics']['route_completed'],'finish gate not reached'
                if result['outcome']=='insufficient_progress':assert result['termination']['reason']=='corridor_no_progress','unexpected progress scoring'
                p=read(folder/'provenance.json')
                if expected_sources:
                    assert all(p['runtime_sources'].get(k)==v for k,v in expected_sources.items()),'source differs from frozen comparison protocol'
                sources={k:p['runtime_sources'][k] for k in BEHAVIOR}
                if runtime is not None:assert sources==runtime,'bench/scorer behavior changed within campaign'
                runtime=sources
                identity=(p['worker_sources'],p['model_weights'],p['control_launch_sha256'],p['worker_image'],p['runtime_images'])
                if model is not None:assert identity==model,'model/controller/runtime identity changed'
                model=identity
                rows.append(dict(round=number,role=role,outcome=result['outcome'],placement_id=g['placement_id'],
                    rgb_noise_rmse=frame['rgb_noise_rmse'],scheduled_sensor_evidence=scheduled,directory=str(folder)))
            if 'perturbed' in cases:
                assert cases['clean']==clean_twin(cases['perturbed']),'unmatched pair'
                assert math.dist(evidence['clean']['scene']['position'],evidence['perturbed']['scene']['position'])<=.01,'initial poses differ by >1cm'
        except (AssertionError,OSError,ValueError,KeyError,TypeError) as exc:
            errors.append(dict(round=number,error=str(exc)))
    workflow=workflow_evidence(history);errors.extend(workflow['invalid_experiments'])
    expected_count=sum(p[role].get('executed',True) for r in history for p in r['pairs'] for role in ('clean','perturbed'))
    data=dict(passed=bool(history) and not errors and len(rows)==expected_count,completed_rounds=len(history),
              audited_flights=len(rows),expected_executed_flights=expected_count,errors=errors,rows=rows,workflow=workflow,
              scheduled_effects_verified=all(r['scheduled_sensor_evidence']['exposure_verified'] for r in rows if r.get('scheduled_sensor_evidence')),
              model_identity_sha256=hashlib.sha256(json.dumps(model,sort_keys=True).encode()).hexdigest() if model else None)
    (root/'corridor_artifact_audit.json').write_text(json.dumps(data,indent=2)+'\n')
    return data


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);a=p.parse_args();result=audit(a.campaign)
    print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)

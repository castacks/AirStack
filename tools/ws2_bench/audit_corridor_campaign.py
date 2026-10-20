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
            if decision.get('llm_call_id'):
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
                    rgb_noise_rmse=frame['rgb_noise_rmse'],directory=str(folder)))
            if 'perturbed' in cases:
                assert cases['clean']==clean_twin(cases['perturbed']),'unmatched pair'
                assert math.dist(evidence['clean']['scene']['position'],evidence['perturbed']['scene']['position'])<=.01,'initial poses differ by >1cm'
        except (AssertionError,OSError,ValueError,KeyError,TypeError) as exc:
            errors.append(dict(round=number,error=str(exc)))
    workflow=workflow_evidence(history);errors.extend(workflow['invalid_experiments'])
    expected_count=sum(p[role].get('executed',True) for r in history for p in r['pairs'] for role in ('clean','perturbed'))
    data=dict(passed=bool(history) and not errors and len(rows)==expected_count,completed_rounds=len(history),
              audited_flights=len(rows),expected_executed_flights=expected_count,errors=errors,rows=rows,workflow=workflow,
              model_identity_sha256=hashlib.sha256(json.dumps(model,sort_keys=True).encode()).hexdigest() if model else None)
    (root/'corridor_artifact_audit.json').write_text(json.dumps(data,indent=2)+'\n')
    return data


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);a=p.parse_args();result=audit(a.campaign)
    print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)

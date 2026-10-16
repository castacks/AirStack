import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from episode import resolved,fingerprint,patch_active
from episode import worker_command
from campaign import clean_twin,candidates,verdict
from ravi_reporting import build_campaign_report
import json
import campaign

def test_kim_reference_camera_profile_is_explicit_and_paired():
    legacy=resolved({'planner':'kim'})
    assert (legacy['input_fov_deg'],legacy['camera_pitch_deg'],legacy['input_rate_hz'])==(0,0,3)
    c=resolved(dict(legacy,input_fov_deg=62.8,camera_pitch_deg=20,input_rate_hz=5,
                    condition={'rgb_noise':32}))
    twin=clean_twin(c)
    for key in ('input_fov_deg','camera_pitch_deg','input_rate_hz'):assert twin[key]==c[key]
    command=worker_command(c)
    assert command[command.index('--input-fov')+1]=='62.8'
    assert command[command.index('--rate')+1]=='5.0'
    for bad in ({'input_fov_deg':1},{'camera_pitch_deg':-20},{'input_rate_hz':True}):
        with pytest.raises(ValueError):resolved(dict(legacy,**bad))
    with pytest.raises(ValueError):resolved({'planner':'pulp_dronet','input_fov_deg':0})

def test_privileged_depth_is_explicit_and_cannot_be_an_attack():
    for planner, normal in [('kim','fcrn'),('mononav','zoe')]:
        base=resolved({'planner':planner})
        assert 'diagnostic_depth_source' not in base
        command=worker_command(base)
        assert command[command.index('--depth-source')+1]==normal
        diagnostic=resolved(dict(base,diagnostic_depth_source='ground-truth'))
        command=worker_command(diagnostic)
        assert command[command.index('--depth-source')+1]=='ground-truth'
        assert fingerprint(base)!=fingerprint(diagnostic)
        for key,value in [('rgb_noise',10),('delay',.1),('depth_noise',.1)]:
            with pytest.raises(ValueError,match='unperturbed'):
                resolved(dict(diagnostic,condition=dict(diagnostic['condition'],**{key:value})))
    with pytest.raises(ValueError,match='unperturbed'):
        resolved({'planner':'kim','diagnostic_depth_source':'ground-truth','condition':{'patch_enabled':True}})

def test_depth_diagnostic_marks_legacy_crash_as_inconclusive():
    from depth_diagnostic import interpretation_status
    crash={'outcome':'planner_stopped','termination':{'reason':'planner_process_exit','detail':1}}
    assert interpretation_status(crash)=='runtime_error_inconclusive'
    assert crash['outcome']=='planner_stopped'  # Preserve the original evidence.
    assert interpretation_status({'outcome':'planner_stopped','termination':{'reason':'planner_terminal'}})=='completed_diagnostic_trial'
    assert interpretation_status({'outcome':'collision','termination':{'reason':'physx_contact'}})=='completed_diagnostic_trial'

def test_diagnostic_summary_never_launches_missing_flights(tmp_path,monkeypatch):
    import depth_diagnostic
    monkeypatch.setattr(depth_diagnostic,'RUNTIME',tmp_path)
    scenario=tmp_path/'scenario.json'
    scenario.write_text(json.dumps({'planner':'mononav'}))
    def forbidden(*args,**kwargs):
        pytest.fail('Summary requested, but a flight was launched')
    monkeypatch.setattr(depth_diagnostic,'run_episode',forbidden)
    with pytest.raises(ValueError,match='no flight will be launched'):
        depth_diagnostic.run(tmp_path/'report',[scenario],summarize_only=True)

def test_pair_preserves_realized_scene():
    c=resolved({'planner':'kim','condition':{'layout':'furnished_b','layout_seed':7,'seed':19,'light':800,
                'rgb_noise':20,'delay':.2,'patch_enabled':True,'patch_size':.8}})
    clean=clean_twin(c)
    assert clean['condition']['rgb_noise']==0 and clean['condition']['delay']==0
    assert not clean['condition']['patch_enabled']
    for k in ['layout','layout_seed','seed','light','patch_size']:assert clean['condition'][k]==c['condition'][k]
    assert c['condition']['rgb_noise']==20
    assert fingerprint(clean)!=fingerprint(c)

def test_resume_proposals_and_budget():
    assert candidates('random',4,17,['kim','mononav'])==candidates('random',4,17,['kim','mononav'])
    assert len(candidates('grid',4,17,['kim','mononav']))==2
    with pytest.raises(ValueError):candidates('random',3,17,['kim'])
    with pytest.raises(ValueError):candidates('random',4,17,['kim','kim'])
    matrix=candidates('random',8,17,['kim','mononav'],matrix=True)
    assert len(matrix)==4
    assert matrix[0]['condition']==matrix[1]['condition']
    assert matrix[2]['condition']==matrix[3]['condition']
    assert matrix[0]['planner']!=matrix[1]['planner']
    with pytest.raises(ValueError):candidates('random',6,17,['kim','mononav'],matrix=True)

@pytest.mark.parametrize('bad',[{'timeout':float('nan')},{'height':True},{'planner':'bogus'},{'shell':'bad'},{'fault':'bad'}])
def test_invalid_episode(bad):
    with pytest.raises(ValueError):resolved(bad)

def test_failed_clean_not_attack_success():
    assert verdict({'outcome':'timeout'},{'outcome':'collision'})=='invalid_clean_baseline'
    assert verdict({'outcome':'goal_reached'},{'outcome':'infrastructure_error'})=='infrastructure_error'

def test_patch_schedule_and_clean_twin():
    c=resolved({'condition':{'patch_enabled':True},'patch_start_s':2,'patch_duration_s':3})
    assert not patch_active(c,1.99) and patch_active(c,2) and patch_active(c,4.99) and not patch_active(c,5)
    assert not patch_active(clean_twin(c),3)
    with pytest.raises(ValueError):resolved({'patch_start_s':-1})

def test_patch_campaign_profiles():
    with pytest.raises(ValueError,match='ZoeDepth'):candidates('grid',4,42,['kim','mononav'],True,'patch')
    c=candidates('grid',2,42,['kim'],True,'patch')
    assert c[0]['condition']['patch_enabled'] and c[0]['condition']['rgb_noise']==0
    combined=candidates('random',4,42,['kim'],profile='combined')
    assert combined==candidates('random',4,42,['kim'],profile='combined')
    assert all(.3<=x['condition']['patch_size']<=.9 and x['condition']['rgb_noise']>0 for x in combined)
    for backend in ['random','grid']:
        for profile in ['patch','combined']:
            trials=candidates(backend,8,42,['kim'],profile=profile)
            assert len({x['condition']['patch_size'] for x in trials})>1
            for x in trials:
                assert {k for k in x['condition'] if k.startswith('patch_')}=={'patch_enabled','patch_size'}

def test_legacy_campaign_cannot_resume_with_new_patch_semantics(tmp_path,monkeypatch):
    monkeypatch.setattr(campaign,'RUNTIME',tmp_path)
    root=tmp_path/'legacy';root.mkdir()
    config={'backend':'random','flight_budget':2,'seed':42,'planners':['kim'],'infrastructure_retries':1}
    saved=json.dumps(config);(root/'config.json').write_text(saved)
    with pytest.raises(ValueError,match='patch policy changed'):
        campaign.run_campaign(root,'random',2,42,['kim'])
    assert (root/'config.json').read_text()==saved

def test_infrastructure_excluded_from_rate():
    report=build_campaign_report({'trials':[{'outcome':x} for x in ['goal_reached','collision','infrastructure_error']]})
    assert report['summary']['success_percent']==50
    assert report['summary']['evaluable_trial_count']==2

def test_retry_only_infrastructure_and_resume_without_duplicate_runs(tmp_path,monkeypatch):
    monkeypatch.setattr(campaign,'RUNTIME',tmp_path)
    calls=[]
    def fake_run(c,folder):
        folder.mkdir();calls.append(folder)
        outcome='infrastructure_error' if len(calls)==1 else 'goal_reached' if len(calls)==2 else 'collision'
        result={'configuration_hash':fingerprint(c),'outcome':outcome,'result_dir':str(folder),'metrics':{}}
        (folder/'result.json').write_text(json.dumps(result));return result
    monkeypatch.setattr(campaign,'run_episode',fake_run)
    first=campaign.run_campaign(tmp_path/'campaign','random',2,42,['kim'],1)
    assert len(calls)==3
    assert first['trials'][0]['pairs'][0]['verdict']=='autonomy_failure'
    assert first['actual_attempts']==3
    second=campaign.run_campaign(tmp_path/'campaign','random',2,42,['kim'],1)
    assert len(calls)==3 and second['actual_attempts']==3

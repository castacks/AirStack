import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from episode import resolved,fingerprint,patch_active
from campaign import clean_twin,candidates,verdict
from ravi_reporting import build_campaign_report
import json
import campaign

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
    c=candidates('grid',4,42,['kim','mononav'],True,'patch')
    assert c[0]['condition']==c[1]['condition']
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

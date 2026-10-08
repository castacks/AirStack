import copy,json
from pathlib import Path
import pytest
import agent_campaign
from campaign import clean_twin
from episode import resolved,fingerprint
from expanded_space import ExpandedPolicy,reference_action,action_to_episode,validate_action,bounds
from generated_layouts import generate,validate_placement,identity,translated,geometry

def test_new_positions_are_replayable_and_not_catalogue_selection():
    seen=set()
    for density,count in [('easy',3),('medium',7),('hard',11)]:
        for seed in (812,813):
            p=generate(seed,density)
            assert validate_placement(json.loads(json.dumps(p)))==p
            assert generate(seed,density)==p and len(p['offsets'])==count
            seen.add(p['placement_id'])
    assert len(seen)==6

def test_even_rehashed_geometry_cannot_enter_protected_flight_corridor():
    p=copy.deepcopy(generate(42));box=geometry()['sources']['plant']['bounds']
    p['offsets']['plant']=[-(box[0][0]+box[1][0])/2,-(box[0][1]+box[1][1])/2,0.]
    p['bounds_source_m']['plant']=translated(box,p['offsets']['plant'])
    p['placement_id']=identity({k:v for k,v in p.items() if k!='placement_id'})
    with pytest.raises(ValueError,match='Unsafe'):validate_placement(p)

def test_expanded_twins_keep_realized_positions_and_lighting():
    a=reference_action();a.update(rgb_noise_stddev=21.,light_intensity=923.,delay_s=.23,patch_enabled=True)
    from mission import defaults
    c=resolved(action_to_episode(a,'kim',17,'paired',defaults('kim')));clean=clean_twin(c)
    for key in ('placement','layout_seed','light','seed','patch_size'):assert clean['condition'][key]==c['condition'][key]
    assert clean['condition']['rgb_noise']==clean['condition']['delay']==0
    assert not clean['condition']['patch_enabled']

@pytest.mark.parametrize('change',[{'rgb_noise_stddev':33},{'light_intensity':float('nan')},{'layout_seed':True},{'patch_strength':.5}])
def test_expanded_bounds_reject_invalid_or_legacy_controls(change):
    with pytest.raises(ValueError):validate_action(dict(reference_action(),**change))

@pytest.mark.parametrize('method',['random','search','agent_search'])
def test_confirmation_is_identical_and_budgeted_for_every_method(method):
    action=reference_action();action.update(rgb_noise_stddev=15.)
    h=[{'decision':{'round':1,'rule':'initial','action':action},'pairs':[{'clean':{'outcome':'goal_reached'},'perturbed':{'outcome':'collision'}}]}]
    d=ExpandedPolicy(method,42).choose_next(h,1)
    assert d['rule']=='confirm_failure' and d['action']==action

def test_expanded_campaign_resume_preserves_exact_offsets_and_legacy_mode(tmp_path,monkeypatch):
    monkeypatch.setattr(agent_campaign,'RUNTIME',tmp_path);calls=[]
    def fly(c,folder,**kwargs):
        folder.mkdir();calls.append(c)
        r={'configuration_hash':fingerprint(c),'outcome':'goal_reached','metrics':{},'result_dir':str(folder)}
        (folder/'result.json').write_text(json.dumps(r));return r
    monkeypatch.setattr(agent_campaign,'run_episode',fly)
    root=tmp_path/'expanded'
    args={'retries':0,'pause_seconds':0,'clean_validation_runs':0,'clean_failure_policy':'record','action_space':'expanded'}
    h=agent_campaign.run_adaptive_campaign(root,'random',4,17,'mononav',**args)
    assert len(calls)==4 and h[0]['decision']['condition']['placement']!=h[1]['decision']['condition']['placement']
    for i in (0,2):assert calls[i]['condition']['placement']==calls[i+1]['condition']['placement']
    agent_campaign.run_adaptive_campaign(root,'random',4,17,'mononav',**args)
    assert len(calls)==4
    assert json.loads(json.dumps(bounds()))==bounds()

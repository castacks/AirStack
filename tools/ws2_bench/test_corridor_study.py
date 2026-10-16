"""A failing clean route must never qualify an attack comparison."""
import json
import pytest
import corridor_study as study


def test_privileged_depth_cannot_qualify_rgb_attack_study(tmp_path):
    with pytest.raises(ValueError,match='GT-depth'):
        study.run_study({'planner':'mononav','diagnostic_depth_source':'ground-truth',
                         'condition':{'layout':'corridor'}},tmp_path)


def fake_runner(monkeypatch, tmp_path, outcomes):
    monkeypatch.setattr(study,'RUNTIME',tmp_path)
    calls=[]
    def run(config,folder,**kwargs):
        calls.append(config)
        return {'outcome':next(outcomes),'metrics':{},'result_dir':str(folder),'cleanup_errors':[]}
    monkeypatch.setattr(study,'run_episode',run)
    return calls


def test_failed_clean_prevents_attacks(monkeypatch,tmp_path):
    calls=fake_runner(monkeypatch,tmp_path,iter(['collision']))
    report=study.run_study({'planner':'kim','condition':{'layout':'corridor','corridor':{'shape':'elbow'}}},
                           tmp_path/'study',2,('rgb_noise','patch'))
    assert len(calls)==1
    assert not calls[0]['condition']['patch_enabled']
    assert report['evaluated_flights']==1
    assert report['phase']=='clean_qualification_failed'
    assert not report['paired_comparisons']
    assert 'attack effects were not evaluated' in report['conclusion']


def test_isolated_attacks_have_fresh_controls_and_correct_flight_count(monkeypatch,tmp_path):
    calls=fake_runner(monkeypatch,tmp_path,iter(['route_completed']*4+['route_completed','collision']))
    report=study.run_study({'planner':'kim','condition':{'layout':'corridor','corridor':{'shape':'elbow'}}},
                           tmp_path/'study',2,('rgb_noise','delay'))
    assert len(calls)==report['evaluated_flights']==6
    assert report['phase']=='complete'
    assert len(report['paired_comparisons'])==2
    assert [c['condition']['rgb_noise'] for c in calls]==[0,0,0,32,0,0]
    assert [c['condition']['delay'] for c in calls]==[0,0,0,0,0,.5]
    assert len(report['findings'])==1
    assert report['findings'][0]['status']=='candidate_needs_repeat'


def test_scene_generalization_pairs_still_share_exact_geometry(monkeypatch,tmp_path):
    calls=fake_runner(monkeypatch,tmp_path,iter(['route_completed']*6))
    study.run_study({'planner':'mononav','condition':{'layout':'corridor',
        'corridor':{'shape':'u_obstacles','width':4.,'bend_radius':2.5,'obstacles':4}}},
        tmp_path/'study',2,('rgb_noise','delay'),{'rgb_noise':'zigzag','delay':'curve'})
    for clean,attack in [(calls[2],calls[3]),(calls[4],calls[5])]:
        assert clean['condition']['corridor']==attack['condition']['corridor']
        assert clean['condition']['layout_seed']==attack['condition']['layout_seed']
        assert clean['condition']['corridor']['obstacles']==0
    assert calls[0]['condition']['corridor']['obstacles']==4
    assert calls[2]['condition']['corridor']['shape']=='zigzag'
    assert calls[4]['condition']['corridor']['shape']=='curve'
    assert 'bend_radius' not in calls[4]['condition']['corridor']


def test_reused_qualification_is_not_counted_as_new_flights(monkeypatch,tmp_path):
    calls=fake_runner(monkeypatch,tmp_path,iter(['route_completed']*4))
    monkeypatch.setattr(study,'qualified_runs',lambda *args:[{'outcome':'route_completed'}]*2)
    output=tmp_path/'study'
    report=study.run_study({'planner':'pulp_dronet','condition':{'layout':'corridor'}},
                          output,2,('rgb_noise','delay'),qualification=[tmp_path/'old1',tmp_path/'old2'])
    assert len(calls)==report['evaluated_flights']==4
    assert len(report['paired_comparisons'])==2
    evidence=json.loads((output/'qualification.json').read_text())
    assert evidence['clean_completed']==2
    assert len(evidence['reused_clean_directories'])==2
    assert len(evidence['flights'])==4

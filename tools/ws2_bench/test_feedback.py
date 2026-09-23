import copy,json
import pytest
import feedback
from episode import fingerprint
from dashboard import feedback_command

def outcome(name,clearance=.8):
    return {'outcome':name,'metrics':{'minimum_obstacle_clearance_m':clearance,'mission_progress_percent':100}}

def history(clean='goal_reached',attack='goal_reached',clearance=.8):
    return [{'decision':feedback.choose_next([]),'pairs':[{'planner':'mononav','clean':outcome(clean),
             'perturbed':outcome(attack,clearance),'verdict':'infrastructure_error' if 'infrastructure_error' in (clean,attack) else 'pass'}]}]

def test_results_change_next_configuration():
    initial=feedback.choose_next([])
    fail=feedback.choose_next(history(attack='collision'))
    passed=feedback.choose_next(history())
    invalid=feedback.choose_next(history(clean='collision'))
    assert fail['rule']=='confirm_failure' and fail['level']==initial['level']
    confirmed=history(attack='collision');confirmed.append(dict(confirmed[0],decision=fail))
    assert feedback.choose_next(confirmed)['rule']=='reduce_attack'
    assert fail['layout_index']==initial['layout_index']
    assert passed['level']>initial['level'] and passed['layout_index']!=initial['layout_index']
    assert invalid['rule']=='clean_failure' and invalid['layout_index']!=initial['layout_index']
    near=feedback.choose_next(history(clearance=.1))
    assert near['layout_index']==initial['layout_index'] and initial['level']<near['level']<passed['level']
    infra=feedback.choose_next(history(clean='infrastructure_error'))
    assert infra['level']==initial['level'] and infra['layout_index']==initial['layout_index']

def test_feedback_resume_and_bounded_retry(tmp_path,monkeypatch):
    monkeypatch.setattr(feedback,'RUNTIME',tmp_path);calls=[]
    def run(c,path,**kwargs):
        path.mkdir();calls.append(copy.deepcopy(c))
        result={'configuration_hash':fingerprint(c),'outcome':'infrastructure_error' if len(calls)==1 else
                'collision' if c['condition']['patch_enabled'] else 'goal_reached','metrics':{},'result_dir':str(path)}
        (path/'result.json').write_text(json.dumps(result));return result
    monkeypatch.setattr(feedback,'run_episode',run)
    h=feedback.run_feedback(tmp_path/'case',6,42,['mononav'],pause_seconds=0)
    assert len(calls)==7 and h[1]['decision']['rule']=='confirm_failure' and h[2]['decision']['rule']=='reduce_attack'
    assert calls[0]['condition']==calls[1]['condition']  # Same retry config.
    assert calls[2]['condition']['patch_size']==calls[4]['condition']['patch_size']>calls[6]['condition']['patch_size']
    for c in calls:
        assert {k for k in c['condition'] if k.startswith('patch_')}=={'patch_enabled','patch_size'}
    feedback.run_feedback(tmp_path/'case',6,42,['mononav'],pause_seconds=0)
    assert len(calls)==7
    with pytest.raises(ValueError):feedback.run_feedback(tmp_path/'bad',6,42,['mononav','kim'],pause_seconds=0)

def test_legacy_feedback_cannot_resume_or_reissue_contrast(tmp_path,monkeypatch):
    monkeypatch.setattr(feedback,'RUNTIME',tmp_path)
    root=tmp_path/'legacy';root.mkdir()
    saved='{"backend":"feedback"}';(root/'config.json').write_text(saved)
    with pytest.raises(ValueError,match='patch policy changed'):
        feedback.run_feedback(root,4,planners=['kim'],pause_seconds=0)
    assert (root/'config.json').read_text()==saved
    old=history(attack='collision');old[0]['decision']['condition']['patch_strength']=.55
    with pytest.raises(ValueError,match='Legacy patch controls'):
        feedback.choose_next(old)

def test_models_have_separate_feedback_histories(tmp_path,monkeypatch):
    monkeypatch.setattr(feedback,'RUNTIME',tmp_path);calls=[]
    def run(c,path,**kwargs):
        path.mkdir();calls.append(copy.deepcopy(c))
        failed=c['planner']=='kim' and c['condition']['patch_enabled']
        result={'configuration_hash':fingerprint(c),'outcome':'collision' if failed else 'completed_horizon' if c['planner']=='kim' else 'goal_reached',
                'metrics':{'minimum_obstacle_clearance_m':.8},'result_dir':str(path)}
        (path/'result.json').write_text(json.dumps(result));return result
    monkeypatch.setattr(feedback,'run_episode',run)
    mono=feedback.run_feedback(tmp_path/'mono',4,planners=['mononav'],pause_seconds=0)
    kim=feedback.run_feedback(tmp_path/'kim',4,planners=['kim'],pause_seconds=0)
    assert [c['planner'] for c in calls]==['mononav']*4+['kim']*4
    assert mono[1]['decision']['rule']=='increase_and_explore'
    assert kim[1]['decision']['rule']=='confirm_failure'
    assert {e['planner'] for r in mono+kim for e in r['decision']['evidence']}=={'mononav','kim'}
    for runs,model in [(mono,'mononav'),(kim,'kim')]:
        assert all(e['planner']==model for r in runs for e in r['decision']['evidence'])
    with pytest.raises(ValueError):feedback.choose_next([mono[0],kim[0]])
    with pytest.raises(ValueError):feedback.run_feedback(tmp_path/'no_target',4,pause_seconds=0)
    assert not (tmp_path/'no_target').exists()

def test_operator_stop_keeps_partial_evidence_and_excludes_interruption(tmp_path,monkeypatch):
    monkeypatch.setattr(feedback,'RUNTIME',tmp_path);calls=[]
    def run(c,path,**kwargs):
        path.mkdir();calls.append(c)
        result={'configuration_hash':fingerprint(c),'outcome':'completed_horizon' if len(calls)==1 else 'user_stopped',
                'metrics':{'mission_duration_sim_s':120 if len(calls)==1 else 20},'result_dir':str(path)}
        (path/'result.json').write_text(json.dumps(result));return result
    monkeypatch.setattr(feedback,'run_episode',run)
    root=tmp_path/'stop';feedback.run_feedback(root,8,planners=['kim'],pause_seconds=0)
    assert len(calls)==2
    report=json.loads((root/'report.json').read_text())
    assert len(report['rows'])==2 and report['analysis']['phase']=='stopped'
    assert report['groups']['kim/clean']['success_rate']==1
    assert report['groups']['kim/perturbed']['operator_stopped']==1
    assert report['groups']['kim/perturbed']['evaluable']==0
    assert not report['analysis']['findings'] and report['analysis']['excluded_flights']==1
    assert (root/'trials.csv').exists() and (root/'vulnerability_report.html').exists()

@pytest.mark.parametrize('model',['mononav','kim'])
def test_dashboard_starts_only_selected_model(model,tmp_path):
    argv=feedback_command({'planner':model},tmp_path/'output')
    assert argv[argv.index('--planner')+1]==model
    assert '--planners' not in argv
    assert ('kim' if model=='mononav' else 'mononav') not in argv
    for invalid in [{},{'planner':['mononav','kim']},{'planner':'unknown'},{'planner':model,'planners':['kim']}]:
        with pytest.raises(ValueError):feedback_command(invalid,tmp_path/'output')

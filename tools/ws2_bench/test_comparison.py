import json
from pathlib import Path
from compare_policies import method_result,report_study
from expanded_space import reference_action
from dashboard import adaptive_command

def write(root,name,value):
    root.mkdir(parents=True,exist_ok=True);(root/name).write_text(json.dumps(value))

def history(clean='goal_reached',attack='collision'):
    return [{'decision':{'round':i,'action':reference_action()},'pairs':[{
        role:{'outcome':outcome,'metrics':{},'wall_duration_s':2.} for role,outcome in [('clean',clean),('perturbed',attack)]}]} for i in (1,2)]

def test_clean_failures_and_infrastructure_are_not_agent_successes(tmp_path):
    root=tmp_path/'random';write(root,'history.json',history('collision','collision'))
    result=method_result(root)
    assert result['clean_failure_pairs']==2 and result['distinct_candidate_conditions']==0
    write(root,'history.json',history('infrastructure_error','infrastructure_error'))
    result=method_result(root)
    assert result['infrastructure_errors']==4 and result['evaluated_pairs']==0

def test_comparison_counts_repeat_once_and_does_not_invent_advantage(tmp_path):
    protocol={'order':['random','search','agent_search'],'flights_per_method':4,'qualification_directories':['test-reference']}
    write(tmp_path,'protocol.json',protocol)
    for name in protocol['order']:
        write(tmp_path/name,'config.json',{});write(tmp_path/name,'history.json',history())
    report=report_study(tmp_path)
    assert report['comparison_valid']
    assert all(r['distinct_candidate_conditions']==r['reproduced_conditions']==1 for r in report['results'])
    assert 'did not show a higher' in report['conclusion']
    assert all(r['flights_to_first_candidate']==2 for r in report['results'])

def test_expanded_viewer_needs_no_saved_layout_selector(tmp_path):
    cmd=adaptive_command({'planner':'mononav','policy':'random','action_space':'expanded','budget':4},tmp_path)
    assert cmd[cmd.index('--action-space')+1]=='expanded'
    assert '--qualified-layout' not in cmd

def test_llm_evidence_does_not_assign_kim_controls_to_mononav():
    from vulnerability_report import analyze
    from agent_analysis import evidence_for_report
    from mission import defaults
    for planner in ('mononav','kim'):
        evidence=evidence_for_report(analyze([],[],{'planners':[planner],'mission':defaults(planner)},'complete'))
        if planner=='mononav':
            assert 'maximum_speed' not in evidence['mission']
            assert 'governor' not in ' '.join(evidence['limitations'])
        else:
            assert 'velocity' not in evidence['mission']
            assert 'depth-based speed governor' in ' '.join(evidence['limitations'])
        assert 'does not establish' in evidence['parameter_definitions']['attack_effect_evaluable']

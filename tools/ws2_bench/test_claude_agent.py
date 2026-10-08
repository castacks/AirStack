import json
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

import agent_campaign
from agent_analysis import validate_analysis, write_analysis, evidence_for_report
from agent_policies import AgentSearchPolicy
from agent_schema import validate_action, validate_intent
from claude_provider import ClaudeSubscriptionProvider, validate_proposal
from dashboard import adaptive_command
from episode import fingerprint
from vulnerability_report import analyze


ACTION = validate_action({'layout': 'easy', 'layout_seed': 2, 'delay_s': .15,
                          'patch_enabled': True, 'patch_size_m': .6,
                          'patch_start_s': 5., 'patch_duration_s': 10.})


class DirectProvider:
    chooses_action = True
    last_call_id = 'test_call'

    def __init__(self):
        self.selections = 0
        self.analyses = 0

    def metadata(self):
        return {'provider': 'mock_claude', 'requested_model': 'fake'}

    def propose(self, context):
        self.selections += 1
        return {'hypothesis': 'test delayed observation', 'reason': 'isolate the selected setting', 'action': ACTION}

    def complete(self, purpose, system, evidence, schema, validate):
        self.analyses += 1
        return validate({'summary': 'Synthetic evidence only.', 'findings': [
            {'claim': 'Attack flight failed after a passing clean run.', 'status': 'candidate',
             'evidence_rounds': [evidence['rounds'][0]['id']]}],
            'limitations': ['Too few runs for causal attribution.'], 'next_tests': ['Repeat with patch disabled.'], 'defense_candidates':[]})


def test_exact_configuration_is_selected_by_model_not_ranker(monkeypatch):
    provider = DirectProvider()
    policy = AgentSearchPolicy(42, provider, [ACTION])
    monkeypatch.setattr(policy, '_rank', lambda *_: pytest.fail('ranker must not override Claude'))
    decision = policy.choose_next([], 2)
    assert decision['action'] == ACTION
    assert decision['rule'] == 'llm_selected_configuration'
    policy = AgentSearchPolicy(42, provider, [{**ACTION, 'layout_seed': 4}])
    with pytest.raises(ValueError, match='unavailable'):
        policy.choose_next([], 2)


def test_missing_fields_do_not_silently_become_an_llm_decision():
    with pytest.raises(ValueError, match='missing'):
        validate_intent({})
    with pytest.raises(ValueError):
        validate_proposal({'hypothesis': 'x', 'reason': 'x', 'action': {}})


def test_web_command_contains_qualified_layouts_and_claude_provider(monkeypatch):
    monkeypatch.setattr(ClaudeSubscriptionProvider, 'check_auth', lambda self: {'loggedIn': True})
    cmd = adaptive_command({'planner': 'mononav', 'policy': 'agent_search', 'provider': 'claude',
                            'qualified_layouts': ['easy:2', 'easy:4']}, Path('/tmp/test-campaign'))
    assert cmd.count('--qualified-layout') == 2
    assert cmd[cmd.index('--provider')+1] == 'claude'
    limited = adaptive_command({'planner':'mononav','policy':'search',
                                'qualified_layouts':['easy:2'],'budget':2,
                                'clean_validation_runs':0,'infrastructure_retries':0,
                                'clean_failure_policy':'record'}, Path('/tmp/test-campaign'))
    assert limited[limited.index('--clean-validation-runs')+1] == '0'
    assert limited[limited.index('--infrastructure-retries')+1] == '0'
    assert limited[limited.index('--clean-failure-policy')+1] == 'record'
    with pytest.raises(ValueError, match='clean-checked'):
        adaptive_command({'planner':'mononav', 'policy':'search'}, Path('/tmp/test'))


def test_schedule_is_part_of_repeated_condition_identity():
    condition = {'layout':'easy','layout_seed':2,'patch_enabled':True,'patch_size':.6,'delay':.15}
    history = [{'decision':{'round':i,'condition':condition,'patch_start_s':start,'patch_duration_s':5},
                'pairs':[{'planner':'mononav','clean':{'outcome':'goal_reached','metrics':{}},
                          'perturbed':{'outcome':'collision','metrics':{}}}]} for i,start in [(1,5),(2,10)]]
    report = analyze(history, [], {'planners':['mononav']}, 'complete')
    assert report['evaluated_flights'] == 4
    assert len(report['findings']) == 2
    assert all(f['status'] == 'candidate_needs_repeat' for f in report['findings'])


def test_final_analysis_resumes_without_new_flights_or_llm_calls(tmp_path, monkeypatch):
    monkeypatch.setattr(agent_campaign, 'RUNTIME', tmp_path)
    flights = []
    def fly(config, folder, **kwargs):
        folder.mkdir()
        flights.append(config)
        result = {'configuration_hash':fingerprint(config), 'result_dir':str(folder),
                  'outcome':'collision' if config['condition']['patch_enabled'] else 'goal_reached',
                  'metrics':{'path_length_m':4.0, 'mission_duration_sim_s':15.}}
        (folder/'result.json').write_text(json.dumps(result))
        return result
    monkeypatch.setattr(agent_campaign, 'run_episode', fly)
    provider = DirectProvider(); root = tmp_path/'campaign'
    args = dict(provider=provider, planner='kim', retries=0, pause_seconds=0, clean_validation_runs=0)
    agent_campaign.run_adaptive_campaign(root, 'agent_search', 4, **args)
    assert len(flights) == 4
    assert provider.selections == provider.analyses == 1
    report = json.loads((root/'llm_analysis.json').read_text())
    assert report['status'] == 'complete'
    assert report['analysis']['findings'][0]['evidence_rounds'] == ['round_01']
    agent_campaign.run_adaptive_campaign(root, 'agent_search', 4, **args)
    assert len(flights) == 4
    assert provider.selections == provider.analyses == 1
    report = json.loads((root/'vulnerability_report.json').read_text())
    assert report['evaluated_flights'] == 4
    assert len(report['flight_results']) == 4
    assert json.loads((root/'report.json').read_text())['actual_attempts'] == 4


def test_analysis_rejects_fabricated_evidence_and_false_candidates():
    evidence = {'rounds':[{'id':'round_01','attack_effect_evaluable':False,
                           'attacked':{'outcome':'collision'}}]}
    raw = {'summary':'x','findings':[{'claim':'x','status':'candidate','evidence_rounds':['round_02']}],
           'limitations':['small sample'],'next_tests':[], 'defense_candidates':[]}
    with pytest.raises(ValueError, match='existing'):
        validate_analysis(raw, evidence)
    raw['findings'][0]['evidence_rounds'] = ['round_01']
    with pytest.raises(ValueError, match='clean-pass'):
        validate_analysis(raw, evidence)


def test_analysis_failure_preserves_metric_report(tmp_path):
    report = {'target':'mononav','mission':{},'phase':'complete','evaluated_flights':2,'excluded_flights':0,
              'outcomes':{},'limitations':[], 'paired_comparisons':[{
                  'round':1,'condition':{},'attack_effect_evaluable':True,
                  'clean':{'outcome':'goal_reached'},'perturbed':{'outcome':'collision'},
                  'delta_attack_minus_clean':{}}]}
    provider = DirectProvider()
    provider.complete = lambda *a: (_ for _ in ()).throw(RuntimeError('quota exhausted'))
    (tmp_path/'vulnerability_report.json').write_text('original metrics')
    assert write_analysis(tmp_path, report, provider)['status'] == 'unavailable'
    assert (tmp_path/'vulnerability_report.json').read_text() == 'original metrics'


@pytest.fixture
def subscription_env(monkeypatch):
    for name in ('ANTHROPIC_API_KEY','ANTHROPIC_AUTH_TOKEN','ANTHROPIC_BASE_URL','ANTHROPIC_PROFILE',
                 'CLAUDE_CODE_USE_BEDROCK','CLAUDE_CODE_USE_VERTEX','CLAUDE_CODE_USE_FOUNDRY',
                 'CLAUDE_CODE_USE_ANTHROPIC_AWS'):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setattr('claude_provider.shutil.which', lambda _: '/fake/claude')


def test_official_cli_is_tool_free_and_repairs_bad_json(tmp_path, monkeypatch, subscription_env):
    calls = []
    def run(argv, **kwargs):
        if 'status' in argv:
            return SimpleNamespace(returncode=0, stdout=json.dumps({'loggedIn':True,'apiProvider':'firstParty',
                                                                   'authMethod':'oauth_token','subscriptionType':'team'}))
        calls.append((argv, kwargs))
        value = {} if len(calls)==1 else {'hypothesis':'x','reason':'y','action':ACTION}
        return SimpleNamespace(returncode=0, stdout=json.dumps({'subtype':'success','is_error':False,
                               'structured_output':value, 'usage':{'input_tokens':1}, 'modelUsage':{'fake':{}}}))
    monkeypatch.setattr('claude_provider.subprocess.run', run)
    provider = ClaudeSubscriptionProvider(tmp_path/'calls')
    decision = AgentSearchPolicy(42, provider, [ACTION]).choose_next([], 2)
    assert decision['action'] == ACTION
    assert len(calls) == 2
    for argv, kwargs in calls:
        assert argv[argv.index('--tools')+1] == ''
        assert '--safe-mode' in argv and '--no-session-persistence' in argv
        assert '--strict-mcp-config' in argv and '--bare' not in argv
        assert 'shell' not in kwargs
        assert 'available_action_keys' not in kwargs['input']
    audit = json.loads(next((tmp_path/'calls').glob('*.json')).read_text())
    assert audit['status'] == 'validated'
    assert len(audit['attempts']) == 2


def test_subscription_does_not_fall_back_to_api_billing(monkeypatch, subscription_env):
    monkeypatch.setenv('ANTHROPIC_API_KEY','fake-test-only')
    with pytest.raises(ValueError, match='ANTHROPIC_API_KEY'):
        ClaudeSubscriptionProvider().check_auth()


def test_timeout_does_not_execute_or_fabricate_a_configuration(monkeypatch, subscription_env):
    provider = ClaudeSubscriptionProvider()
    monkeypatch.setattr(provider, 'check_auth', lambda: {})
    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired('claude',180)
    monkeypatch.setattr('claude_provider.subprocess.run', timeout)
    with pytest.raises(RuntimeError, match='timed out'):
        AgentSearchPolicy(42, provider, [ACTION]).choose_next([],2)


def test_report_supplies_zero_duration_and_collision_semantics():
    report={'target':'mononav','mission':{'velocity':.3,'initial_speed':.4,'maximum_speed':.5},'phase':'complete','evaluated_flights':0,
            'excluded_flights':0,'outcomes':{},'limitations':[],'paired_comparisons':[]}
    evidence=evidence_for_report(report)
    assert evidence['mission']=={'velocity':.3}
    definitions=evidence['parameter_definitions']
    assert 'rest of the mission' in definitions['patch_duration_s']
    assert 'negative is not itself' in definitions['clearance']


def test_isolated_checkout_cannot_start_original_containers(monkeypatch,tmp_path):
    import episode
    checkout=tmp_path/'new';runtime=checkout/'robot/ros_ws/ws2_runtime'
    monkeypatch.setattr(episode,'HERE',checkout/'tools/ws2_bench')
    monkeypatch.setattr(episode,'RUNTIME',runtime)
    calls=[]
    def inspect(argv,**kwargs):
        calls.append(argv)
        return json.dumps([{'Source':str(tmp_path/'old'),'Destination':'/isaac-sim/AirStack'}])
    monkeypatch.setattr(episode,'cmd',inspect)
    with pytest.raises(RuntimeError,match='no container was started'):
        episode.run_episode({'planner':'mononav'},runtime/'trial')
    assert all(argv[:2]==['docker','inspect'] for argv in calls)
    assert not (runtime/'trial').exists()


def test_matching_runtime_mounts_pass_read_only_check(monkeypatch,tmp_path):
    import episode
    checkout=tmp_path/'stack';runtime=checkout/'robot/ros_ws/ws2_runtime'
    monkeypatch.setattr(episode,'HERE',checkout/'tools/ws2_bench')
    monkeypatch.setattr(episode,'RUNTIME',runtime)
    def inspect(argv,**kwargs):
        if argv[-1]=='isaac-sim':return json.dumps([{'Source':str(checkout),'Destination':'/isaac-sim/AirStack'}])
        return json.dumps([{'Source':str(checkout/'robot/ros_ws'),'Destination':'/root/AirStack/robot/ros_ws'}])
    monkeypatch.setattr(episode,'cmd',inspect)
    episode.verify_runtime_mounts()

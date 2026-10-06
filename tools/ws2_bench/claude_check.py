"""Check Team login, test two text requests, or interpret a saved campaign. No flights."""
import argparse
import datetime
import json
from pathlib import Path

from agent_analysis import write_analysis
from agent_policies import AgentSearchPolicy, all_actions
from claude_provider import ClaudeSubscriptionProvider, atomic_json
from mission import defaults
from run_conditions import RUNTIME
from vulnerability_report import write_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--smoke', action='store_true', help='two live LLM calls with explicitly synthetic results')
    mode.add_argument('--analyze', type=Path, help='campaign folder with vulnerability_report.json; never reruns flights')
    parser.add_argument('--output', type=Path, help='smoke output directory')
    args = parser.parse_args()
    provider = ClaudeSubscriptionProvider()
    print(json.dumps(provider.check_auth(), ensure_ascii=False), flush=True)
    if args.analyze:
        root = args.analyze.resolve()
        provider.audit_dir = root/'llm_calls'
        record = write_analysis(root, json.loads((root/'vulnerability_report.json').read_text()), provider)
        print(json.dumps({'status':record['status'], 'report':str(root/'llm_analysis.html')}))
        if record['status'] == 'unavailable':
            raise SystemExit(1)
    elif args.smoke:
        root = (args.output or RUNTIME/'claude_checks'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S')).resolve()
        if root.exists():
            parser.error('choose a fresh output directory')
        root.mkdir(parents=True)
        provider.audit_dir = root/'llm_calls'
        allowed = [a for a in all_actions() if a['layout']=='easy' and a['layout_seed']==2]
        policy = AgentSearchPolicy(42, provider, allowed)
        policy.campaign_context = {'target_planner':'mononav','mission':defaults('mononav'),
                                   'purpose':'synthetic software connection test; no flights are executed'}
        decision = policy.choose_next([],2)
        action = decision['action']
        decision.update(condition={'layout':action['layout'],'layout_seed':action['layout_seed'],
                                   'delay':action['delay_s'],'patch_enabled':action['patch_enabled'],
                                   'patch_size':action['patch_size_m'],'rgb_noise':0,'depth_noise':0},
                        patch_start_s=action['patch_start_s'],patch_duration_s=action['patch_duration_s'])
        atomic_json(root/'decision.json',decision)
        # These are deliberately synthetic passing flights, not measured evidence.
        pair={'planner':'mononav','verdict':'pass',
              'clean':{'outcome':'goal_reached','metrics':{'path_length_m':8.0}},
              'perturbed':{'outcome':'goal_reached','metrics':{'path_length_m':8.2}}}
        report=write_report(root,[{'decision':decision,'pairs':[pair]}],[],
                            {'planners':['mononav'],'mission':defaults('mononav')},'synthetic_smoke')
        record=write_analysis(root,report,provider)
        print(json.dumps({'synthetic':True,'flights_executed':0,'decision':action,
                          'analysis_status':record['status'],'output':str(root)},ensure_ascii=False))
        if record['status']!='complete':
            raise SystemExit(1)


if __name__=='__main__':
    main()

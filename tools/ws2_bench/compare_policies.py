"""Fixed-budget pilot comparison on the same model, mission and expanded space."""
import argparse, csv, html, json, random, time
from pathlib import Path
from agent_campaign import run_adaptive_campaign
from claude_provider import ClaudeSubscriptionProvider
from episode import RUNTIME, atomic
from expanded_space import bounds, action_key
from mission import SUCCESSES, EXCLUDED, defaults

METHODS=('random','search','agent_search')

def read(path):return json.loads(Path(path).read_text())

def method_result(root):
    history=read(root/'history.json') if (root/'history.json').exists() else []
    groups={};clean_pass=0;attacked_fail=0;baseline_fail=0;infra=0;flights=0;seconds=0.;first=None
    for r in history:
        p=r['pairs'][0];key=action_key(r['decision']['action'])
        for role in ('clean','perturbed'):
            result=p[role];flights+=1;seconds+=result.get('wall_duration_s',0.)
            if result['outcome']=='infrastructure_error':infra+=1
        if any(p[role]['outcome'] in EXCLUDED for role in ('clean','perturbed')):continue
        if p['clean']['outcome'] not in SUCCESSES:baseline_fail+=1;continue
        clean_pass+=1
        if p['perturbed']['outcome'] not in SUCCESSES:
            attacked_fail+=1;groups[key]=groups.get(key,0)+1
            if first is None:first=2*r['decision']['round']
    calls=[];usage={};selection_s=0.;analysis_s=0.;cost=0.
    for path in sorted((root/'llm_calls').glob('*.json')):
        d=read(path);calls.append(d['call_id'])
        for attempt in d.get('attempts',[]):
            elapsed=attempt.get('elapsed_s',0.)
            if d['call_id'].startswith('analysis_'):analysis_s+=elapsed
            else:selection_s+=elapsed
            cost+=attempt.get('reported_cost_usd') or 0.
            for name,value in (attempt.get('usage') or {}).items():
                if isinstance(value,(int,float)):usage[name]=usage.get(name,0)+value
    evaluated_pairs=sum(all(p[role]['outcome'] not in EXCLUDED for role in ('clean','perturbed')) for r in history for p in r['pairs'])
    return {'method':root.name,'scheduled_flights':flights,'evaluated_pairs':evaluated_pairs,
            'clean_pass_pairs':clean_pass,'clean_failure_pairs':baseline_fail,
            'attack_failure_pairs_with_clean_pass':attacked_fail,'distinct_candidate_conditions':len(groups),
            'reproduced_conditions':sum(n>=2 for n in groups.values()),'flights_to_first_candidate':first,
            'infrastructure_errors':infra,'flight_wall_seconds':round(seconds,2),
            'llm_requests':len(calls),'llm_selection_seconds':round(selection_s,2),
            'llm_analysis_seconds':round(analysis_s,2),'cli_list_price_estimate_usd':round(cost,6),
            'token_usage':usage,'run_directory':str(root)}

def report_study(root):
    root=Path(root).resolve();protocol=read(root/'protocol.json')
    results=[method_result(root/name) for name in protocol['order'] if (root/name/'config.json').exists()]
    complete=len(results)==3 and all(r['scheduled_flights']==protocol['flights_per_method'] for r in results)
    valid=complete and all(r['infrastructure_errors']==0 for r in results)
    # Report exact-condition yield separately from unique root causes.
    conclusion=('This small pilot is complete. It does not establish statistical superiority of any policy.' if valid else
                'The pilot is still incomplete; results are provisional.' if not complete else
                'The pilot contains infrastructure errors; do not infer a policy ranking.')
    if valid:
        counts={r['method']:r['distinct_candidate_conditions'] for r in results}
        conclusion=(f"Observed distinct candidate settings: Claude {counts['agent_search']}, random {counts['random']}, search {counts['search']}. "+
                    ('Claude found more candidate settings in this pilot. ' if counts['agent_search']>max(counts['random'],counts['search']) else
                     'This pilot did not show a higher candidate yield for Claude. ')+
                    'The sample is too small to establish statistical superiority; inspect reproduced settings and clean failures separately.')
    limitations=['Two pairs per method at the default budget; no statistical significance or generalization claim.',
                 'This pilot covers MonoNav on one Office route with a protected straight corridor, one seed and one method order.',
                 'A distinct condition is not necessarily a distinct failure mechanism.',
                 'Every policy uses identical limits, pairing, mission and one-repeat confirmation; confirmation consumes budget.',
                 'A failed clean pair is not counted as an additional sensor/patch attack effect.',
                 'Changing several factors together does not establish which factor caused a failure.',
                 'CLI dollar values are list-price estimates, not a subscription invoice.',
                 'LLM call totals include any retained report-only revisions; selection and analysis latency are recorded separately.',
                 'Separate clean qualification is outside the comparison budget and listed separately.',
                 'All three methods run without manual configuration changes during the campaign; reduced manual effort by Claude is not established.']
    data={'status':'complete' if complete else 'in_progress','comparison_valid':valid,'protocol':protocol,
          'results':results,'conclusion':conclusion,'limitations':limitations}
    atomic(root/'comparison.json',data)
    columns=['method','scheduled_flights','clean_pass_pairs','clean_failure_pairs','attack_failure_pairs_with_clean_pass',
             'distinct_candidate_conditions','reproduced_conditions','flights_to_first_candidate','infrastructure_errors',
             'flight_wall_seconds','llm_requests','llm_selection_seconds','cli_list_price_estimate_usd']
    with (root/'comparison.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=columns,extrasaction='ignore');writer.writeheader();writer.writerows(results)
    lines=['# WS2 policy comparison pilot','',conclusion,'',
           '| Method | Flights | Clean pass / fail | Attack failures with clean pass | Distinct candidate settings | Reproduced settings | Flights to first candidate |',
           '|---|---:|---:|---:|---:|---:|---:|']
    for r in results:lines.append(f"| {r['method']} | {r['scheduled_flights']} | {r['clean_pass_pairs']} / {r['clean_failure_pairs']} | {r['attack_failure_pairs_with_clean_pass']} | {r['distinct_candidate_conditions']} | {r['reproduced_conditions']} | {r['flights_to_first_candidate'] or 'not found'} |")
    lines+=['','## Cost and elapsed time','']
    for r in results:lines.append(f"- {r['method']}: {r['flight_wall_seconds']}s flight lifecycle; {r['llm_requests']} LLM requests; {r['llm_selection_seconds']}s selection; list-price estimate ${r['cli_list_price_estimate_usd']:.4f}.")
    lines+=['','## Interpretation limits','',*['- '+s for s in limitations],
            '','## Qualification evidence','',*['- '+s for s in protocol['qualification_directories']]]
    text='\n'.join(lines)+'\n';(root/'comparison.md').write_text(text)
    esc=lambda v:html.escape(str(v))
    page=['<meta charset="utf-8"><title>WS2 policy comparison</title><style>body{font:16px Arial;max-width:1250px;margin:32px auto;line-height:1.6}table{border-collapse:collapse}td,th{padding:10px;border:1px solid #ccd}pre{white-space:pre-wrap}a{color:#146b89}</style>',
          '<h1>WS2 policy comparison pilot</h1><p>'+esc(conclusion)+'</p><table><tr>',*['<th>'+esc(c)+'</th>' for c in columns[:9]],'</tr>']
    for r in results:page+=['<tr>',*['<td>'+esc(r[c])+'</td>' for c in columns[:9]],'</tr>']
    page+=['</table><h2>Evidence and limits</h2><pre>'+esc(text)+'</pre>'];(root/'comparison.html').write_text(''.join(page))
    return data

def run_study(root,flights=4,seed=17,qualification=()):
    root=Path(root).resolve();root.relative_to(RUNTIME.resolve())
    if flights<4 or flights%2:raise ValueError('Use an even budget of at least four flights per method')
    if not qualification:raise ValueError('Supply clean qualification evidence')
    qualified=[]
    for directory in qualification:
        values=read(Path(directory)/'qualification.json')
        if len(values)<2 or any(r['outcome'] not in SUCCESSES for r in values):raise ValueError('Qualification needs two successful clean flights')
        from expanded_space import reference_action,action_to_episode
        from episode import resolved,fingerprint
        from audit_results import audit_episode
        expected=resolved(action_to_episode(reference_action(),'mononav',42,'reference',defaults('mononav')))
        if len({r['result_dir'] for r in values})!=len(values):raise ValueError('Qualification needs independent flights')
        for result in values:
            scenario=read(Path(result['result_dir'])/'scenario.json')
            meaningful=lambda c:{k:v for k,v in c.items() if k!='name' and not (k=='patch_size' and not c['patch_enabled'])}
            if result['configuration_hash']!=fingerprint(scenario) or meaningful(scenario['condition'])!=meaningful(expected['condition']):
                raise ValueError('Qualification did not use the registered reference condition')
            if any(scenario[k]!=expected[k] for k in defaults('mononav')):raise ValueError('Qualification mission differs')
            if not audit_episode(result['result_dir'])['passed']:raise ValueError('Qualification artifact audit failed')
        qualified.append(str(Path(directory).resolve()))
    provider=ClaudeSubscriptionProvider(model='claude-sonnet-5-5');provider.check_auth()
    order=list(METHODS);random.Random(seed).shuffle(order)
    protocol={'schema':'ws2_comparison_pilot_v1','target':'mononav','flights_per_method':flights,'total_flights':3*flights,
              'seed':seed,'order':order,'mission':defaults('mononav'),'action_space':bounds(),
              'qualified_reference':'generated seed42 easy, corridor half-width0.9m, side bias0, light1800, no perturbations',
              'qualification_directories':qualified,'llm_provider':provider.metadata(),
              'rules':'Equal budget; paired clean/attack; identical confirmation for all policies; no infrastructure retries; no cherry-picking runs.'}
    root.mkdir(parents=True,exist_ok=True)
    if (root/'protocol.json').exists() and read(root/'protocol.json')!=protocol:raise ValueError('Study protocol changed')
    atomic(root/'protocol.json',protocol)
    for method in order:
        atomic(RUNTIME/'live_comparison.json',{'output':str(root),'active_method':method,'status':'running'})
        print('Starting comparison method:',method,flush=True)
        kwargs={'provider':ClaudeSubscriptionProvider(audit_dir=root/method/'llm_calls',model=provider.model)} if method=='agent_search' else {}
        run_adaptive_campaign(root/method,method,flights,seed,'mononav',retries=0,pause_seconds=0,
            clean_validation_runs=0,clean_failure_policy='record',action_space='expanded',**kwargs)
        report_study(root)
        if read(root/method/'presentation.json')['phase']!='complete':
            atomic(RUNTIME/'live_comparison.json',{'output':str(root),'status':'stopped','active_method':method});return
    atomic(RUNTIME/'live_comparison.json',{'output':str(root),'status':'complete','active_method':None})
    print(json.dumps(report_study(root),indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--flights-per-method',type=int,default=4)
    p.add_argument('--seed',type=int,default=17);p.add_argument('--qualification',action='append',default=[]);p.add_argument('--report-only',action='store_true');a=p.parse_args()
    if a.report_only:print(json.dumps(report_study(a.output),indent=2))
    else:run_study(a.output,a.flights_per_method,a.seed,a.qualification)

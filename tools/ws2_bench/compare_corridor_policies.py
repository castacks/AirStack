"""Equal-budget corridor comparison, one independently selected target per campaign."""
import argparse
import csv
import hashlib
import html
import json
import random
from pathlib import Path
from agent_campaign import run_adaptive_campaign
from claude_provider import ClaudeSubscriptionProvider
from corridor_space import bounds,mission_for
from diagnostic_evaluation import write_audit
from audit_corridor_campaign import audit as artifact_audit
from compare_policies import method_result
from episode import RUNTIME,atomic

METHODS=('random','search','agent_search')
TARGETS=('mononav','kim','pulp_dronet')
OBJECTIVE=('Find reproducible and diagnostically useful failure conditions within six paired experiments. '
    'Use a passing clean control, confirm candidates, and where informative isolate combined factors or probe a failure bracket. '
    'Do not spend the entire budget collecting trivial maximum-noise failures. Explore scene interaction or lower-dose failures when evidence supports it. '
    'Choose shape, obstacle count/positions and attacks within the shared bounds; preserve all non-treatment settings for follow-ups. '
    'No prior recorded flight results from the earlier fixed matrix are supplied to any selection policy.')


def read(path):return json.loads(Path(path).read_text())


def report(root):
    root=Path(root);protocol=read(root/'protocol.json');rows=[]
    for job in protocol['order']:
        directory=root/job['target']/job['method']
        if not (directory/'config.json').exists():continue
        value=method_result(directory);audit=write_audit(directory);workflow=audit['workflow']
        artifact=read(directory/'corridor_artifact_audit.json') if (directory/'corridor_artifact_audit.json').exists() else {}
        value.update(target=job['target'],artifact_audit_passed=artifact.get('passed',False),model_identity_sha256=artifact.get('model_identity_sha256'),isolations=len(workflow['isolation_evidence']),
            factor_contrasts=len(workflow['distinct_failure_factor_contrasts']),bracket_probes=len(workflow['threshold_evidence']),reproduced_failure_families=len(workflow['reproduced_failure_families']),
            flights_to_first_reproduced=workflow['flights_to_first_reproduced'],invalid_experiments=len(workflow['invalid_experiments']),
            english_report=audit['defenses']['report_status'],defense_proposals=len(audit['defenses']['candidates']))
        rows.append(value)
    identities={t:{r['model_identity_sha256'] for r in rows if r['target']==t and r['model_identity_sha256']} for t in protocol['targets']}
    identity_consistent=all(len(v)<=1 for v in identities.values())
    complete=len(rows)==len(protocol['order']) and all(r['scheduled_flights']==protocol['flights_per_method'] for r in rows)
    valid=complete and identity_consistent and all(r['infrastructure_errors']==0 and r['invalid_experiments']==0 and r['artifact_audit_passed'] for r in rows)
    findings=[]
    for target in protocol['targets']:
        group={r['method']:r for r in rows if r['target']==target}
        if set(group)!=set(METHODS):continue
        keys=('reproduced_conditions','reproduced_failure_families','factor_contrasts','isolations','bracket_probes')
        findings.append(dict(target=target,metrics={k:{m:group[m][k] for m in METHODS} for k in keys},
            interpretation='Observed counts in one budget-limited campaign per method; no statistical superiority or generalization claim.'))
    limits=[
        '12 reserved flight slots per method = six matched pairs. Failed clean controls skip the attack; skipped slots are not reused and executed costs are reported separately.',
        'One campaign seed per target/method. This is an initial comparison, not statistical proof.',
        'Search is a documented result-guided local heuristic, not Bayesian optimization or a state-of-the-art optimizer.',
        'Random and Search both receive rule-based repeat, isolation and bisection follow-ups; Claude allocates its own experiments.',
        'The same Claude provider writes final reports for ALL policies from their saved evidence; report generation is excluded from selection cost.',
        'Factor contrasts require an actually failed combined condition and a matched single-factor observation, independent of the declared experiment purpose; repeated identical contrasts count once.',
        'A scene/factor/outcome family is descriptive, not proof of a distinct internal failure mechanism.',
        'Defense field/reference checks do not establish usefulness or effectiveness. Semantic review is separate; proposed defenses are not tested.',
        'No human baseline was run. Reduced manual effort or better-than-human performance is not established.',
        'No changes to model/controller presets or collision scoring during comparison. Geometry/light are exactly matched within pairs.',
        'Functional validation flights and historical fixed-matrix flights are excluded from comparison outcomes.']
    data=dict(status='complete' if complete else 'in_progress',comparison_valid=valid,identity_consistent=identity_consistent,protocol=protocol,results=rows,findings=findings,limitations=limits)
    atomic(root/'comparison.json',data)
    columns=['target','method','scheduled_flights','executed_flights','clean_pass_pairs','clean_failure_pairs',
             'distinct_candidate_conditions','reproduced_conditions','reproduced_failure_families','factor_contrasts','isolations','bracket_probes',
             'flights_to_first_reproduced','infrastructure_errors','english_report','defense_proposals','llm_selection_seconds','llm_analysis_seconds']
    with (root/'comparison.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=columns,extrasaction='ignore');w.writeheader();w.writerows(rows)
    lines=['# Corridor agent comparison','',f"Status: {data['status']}. Executed {sum(r['executed_flights'] for r in rows)} of {protocol['total_reserved_flights']} reserved flights.",'',
           '| Target | Method | Executed / reserved | Clean failures | Reproduced conditions | Families | Distinct factor contrasts | Bracket probes |',
           '|---|---|---:|---:|---:|---:|---:|---:|']
    for r in rows:lines.append(f"| {r['target']} | {r['method']} | {r['executed_flights']} / {r['scheduled_flights']} | {r['clean_failure_pairs']} | {r['reproduced_conditions']} | {r['reproduced_failure_families']} | {r['factor_contrasts']} | {r['bracket_probes']} |")
    lines+=['','## Interpretation limits','',*['- '+s for s in limits]]
    text='\n'.join(lines)+'\n';(root/'comparison.md').write_text(text)
    (root/'comparison.html').write_text('<meta charset="utf-8"><title>Corridor comparison</title><style>body{font:16px Arial;margin:32px;line-height:1.5}pre{white-space:pre-wrap}</style><pre>'+html.escape(text)+'</pre>')
    return data


def run(root,flights=12,seed=1810,targets=TARGETS):
    root=Path(root).resolve();root.relative_to(RUNTIME.resolve());root.mkdir(parents=True,exist_ok=True)
    if type(flights) is not int or flights<4 or flights%2:raise ValueError('Even budget >=4 required')
    if len(set(targets))!=len(targets) or not set(targets)<=set(TARGETS):raise ValueError('Choose unique registered targets')
    provider=ClaudeSubscriptionProvider();provider.check_auth()
    order=[]
    for target in targets:
        methods=list(METHODS);random.Random(seed+TARGETS.index(target)).shuffle(methods)
        order.extend(dict(target=target,method=m) for m in methods)
    protocol=dict(schema='ws2_corridor_comparison_v1',targets=list(targets),flights_per_method=flights,
        total_reserved_flights=len(targets)*len(METHODS)*flights,seed=seed,order=order,
        action_spaces={t:bounds(t) for t in targets},missions={t:mission_for(t) for t in targets},
        provider=provider.metadata(),objective=OBJECTIVE,
        source_hashes={name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() for name in
                       ('corridor_space.py','corridors.py','experiment_protocol.py','agent_campaign.py','episode.py','agent_analysis.py','diagnostic_evaluation.py',
                        'claude_provider.py','campaign.py','vulnerability_report.py','diagnostics.py','ravi_metrics.py','conditions.py','corridor_scene.py','launch_office.py','model_adapters.py','mission.py','flight_action.py','audit_corridor_campaign.py','compare_corridor_policies.py')},
        pairing='Every round includes its own new clean control. Failed clean skips attack; slot not reused. No infra retries.',
        reporting='Same final Claude report for every policy; no LLM selection calls in baselines. No fixed-matrix prior evidence supplied.')
    if (root/'protocol.json').exists() and read(root/'protocol.json')!=protocol:raise ValueError('Frozen comparison protocol/source changed; preserve existing results')
    atomic(root/'protocol.json',protocol)
    snapshot=root/'source_snapshot';snapshot.mkdir(exist_ok=True)
    for name,expected_hash in protocol['source_hashes'].items():
        destination=snapshot/name
        if destination.exists():
            if hashlib.sha256(destination.read_bytes()).hexdigest()!=expected_hash:raise ValueError('Saved source snapshot was modified')
        else:destination.write_bytes(Path(__file__).with_name(name).read_bytes())
    report(root)
    for job in order:
        target,method=job['target'],job['method'];directory=root/target/method
        atomic(RUNTIME/'live_comparison.json',dict(output=str(root),status='running',active_method=method,target=target))
        print('CAMPAIGN',target,method,flush=True)
        p=ClaudeSubscriptionProvider(audit_dir=directory/'llm_calls',model=provider.model)
        run_adaptive_campaign(directory,method,flights,seed+TARGETS.index(target),target,retries=0,pause_seconds=0,
            clean_validation_runs=0,clean_failure_policy='record',action_space='corridor',study_objective=OBJECTIVE,provider=p)
        artifact_audit(directory,expected_sources=protocol['source_hashes'])
        result=report(root)
        latest=next(r for r in result['results'] if r['target']==target and r['method']==method)
        print('RESULT',json.dumps(latest),flush=True)
        if not result['identity_consistent'] or latest['infrastructure_errors'] or latest['invalid_experiments'] or not latest['artifact_audit_passed'] or read(directory/'presentation.json')['phase']!='complete':
            raise RuntimeError('Comparison paused for infrastructure/protocol review; existing evidence retained')
    atomic(RUNTIME/'live_comparison.json',dict(output=str(root),status='complete',active_method=None))
    return report(root)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--flights-per-method',type=int,default=12)
    p.add_argument('--seed',type=int,default=1810);p.add_argument('--targets',nargs='+',choices=TARGETS,default=list(TARGETS));p.add_argument('--report-only',action='store_true')
    a=p.parse_args();result=report(a.output) if a.report_only else run(a.output,a.flights_per_method,a.seed,a.targets)
    print(json.dumps(result,indent=2))

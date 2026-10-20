"""Deterministic evidence audit; not an LLM's self-rating or a causal oracle."""
import copy
import hashlib
import json
from pathlib import Path
from mission import SUCCESSES, EXCLUDED
import experiment_protocol as experiments


def workflow_evidence(history):
    from corridor_space import action_key as corridor_key
    from expanded_space import action_key as office_key
    from agent_schema import action_key as saved_key
    first=history[0]['decision']['action'] if history else {}
    key=corridor_key if 'shape' in first else saved_key if 'layout' in first else office_key
    counts={kind:0 for kind in experiments.KINDS};isolations=[];brackets=[];groups={};invalid=[];flights=0;first_reproduced=None
    for i,r in enumerate(history):
        d=r['decision'];p=r['pairs'][0];e=d.get('experiment');a=d['action']
        flights+=sum(p[role].get('executed',True) for role in ('clean','perturbed'))
        if e:
            try:experiments.validate(e,a,history[:i],key)
            except ValueError as exc:invalid.append(dict(round=d['round'],error=str(exc)));continue
            counts[e['kind']]+=1
        valid=p['clean']['outcome'] in SUCCESSES and p['perturbed']['outcome'] not in EXCLUDED
        if not valid:continue
        failed=p['perturbed']['outcome'] not in SUCCESSES
        if failed and experiments.active(a):
            g=groups.setdefault(key(a),dict(rounds=[],shape=a.get('shape',a.get('scene_challenge','protected')),
                                            factors=experiments.active(a),outcomes=[]))
            g['rounds'].append(d['round']);g['outcomes'].append(p['perturbed']['outcome'])
            if len(g['rounds'])==2 and first_reproduced is None:first_reproduced=flights
        if e and e['kind']=='isolate':
            source=next(x for x in history[:i] if x['decision']['round']==e['source_round'])
            isolations.append(dict(round=d['round'],source_round=e['source_round'],factor=e['factor'],
                combined_outcome=source['pairs'][0]['perturbed']['outcome'],isolated_outcome=p['perturbed']['outcome'],
                interpretation='Measured one-factor intervention; internal cause and probability remain unproven.'))
        if e and e['kind']=='threshold':
            source=next(x for x in history[:i] if x['decision']['round']==e['source_round'])
            field=experiments.FACTORS[e['factor']][0];high=source['decision']['action'][field]
            low=0. if not e['passing_round'] else next(x['decision']['action'][field] for x in history[:i] if x['decision']['round']==e['passing_round'])
            value=a[field]
            brackets.append(dict(round=d['round'],factor=e['factor'],source_round=e['source_round'],passing_round=e['passing_round'],
                fixed_other_factors={k:a[k] for factor in experiments.active(a) if factor!=e['factor'] for k in experiments.FACTORS[factor] if k in a},
                prior_pass=low,prior_fail=high,probe=value,probe_outcome=p['perturbed']['outcome'],
                observed_pass=low if failed else value,observed_fail=value if failed else high,
                width_before=high-low,width_after=value-low if failed else high-value,
                interpretation='Observed bracket only, not a monotonic or statistically confirmed failure threshold.'))
    # Count information in the measured designs, regardless of the policy's
    # declared purpose. Repeating an isolation does not create a new contrast.
    contrasts={}
    valid=[r for r in history if r['pairs'][0]['clean']['outcome'] in SUCCESSES and r['pairs'][0]['perturbed']['outcome'] not in EXCLUDED]
    for source in valid:
        base=source['decision']['action']
        if len(experiments.active(base))<2 or source['pairs'][0]['perturbed']['outcome'] in SUCCESSES:continue
        for isolated in valid:
            a=isolated['decision']['action'];factors=experiments.active(a)
            if len(factors)!=1:continue
            factor=factors[0];expected=copy.deepcopy(base)
            for other in experiments.FACTORS:
                if other!=factor:experiments.disable(expected,other)
            if key(a)!=key(expected):continue
            identity=hashlib.sha256((key(base)+'|'+key(a)).encode()).hexdigest()
            c=contrasts.setdefault(identity,dict(factor=factor,combined_rounds=[],isolated_rounds=[]))
            for name,row in [('combined_rounds',source),('isolated_rounds',isolated)]:
                if row['decision']['round'] not in c[name]:c[name].append(row['decision']['round'])
    families={(g['shape'],tuple(g['factors']),o) for g in groups.values() if len(g['rounds'])>=2 for o in set(g['outcomes']) if g['outcomes'].count(o)>=2}
    return dict(experiment_counts=counts,invalid_experiments=invalid,isolation_evidence=isolations,threshold_evidence=brackets,
        distinct_failure_factor_contrasts=list(contrasts.values()),
        reproduced_condition_count=sum(len(g['rounds'])>=2 for g in groups.values()),
        reproduced_failure_families=[dict(shape=s,factors=list(f),outcome=o) for s,f,o in sorted(families)],
        flights_to_first_reproduced=first_reproduced,
        family_definition='Predefined scene shape x active perturbation set x repeated outcome; a descriptive family, not an identified root cause.')


def defense_audit(record):
    rounds={r['id']:r for r in record.get('evidence',{}).get('rounds',[])}
    items=[]
    for d in record.get('analysis',{}).get('defense_candidates',[]):
        refs=d['evidence_rounds'];support=[rounds[i] for i in refs if i in rounds]
        items.append(dict(proposal=d['proposal'],evidence_rounds=refs,
            cited_conditions=[dict(round=r['id'],condition=r['condition'],clean=r['clean']['outcome'],attacked=r['attacked']['outcome']) for r in support],
            all_references_exist=len(support)==len(refs),has_observed_failure=any(r['clean']['outcome'] not in SUCCESSES or (r['attack_executed'] and r['attacked']['outcome'] not in SUCCESSES) for r in support),
            fields_present=all(bool(d.get(k,'').strip()) for k in ('proposal','failure_hypothesis','rationale','tradeoffs','validation_plan')),
            status=d['status'],semantic_correctness='Requires evidence review; field/citation checks do not prove technical correctness.',effectiveness='Not tested'))
    return dict(report_status=record.get('status'),candidates=items,
        limitation='No automated usefulness score. Review concrete intervention, evidence fit, tradeoffs and executable validation plan separately.')


def write_audit(root):
    root=Path(root);h=json.loads((root/'history.json').read_text()) if (root/'history.json').exists() else []
    report=json.loads((root/'llm_analysis.json').read_text()) if (root/'llm_analysis.json').exists() else {}
    data=dict(workflow=workflow_evidence(h),defenses=defense_audit(report))
    (root/'diagnostic_audit.json').write_text(json.dumps(data,indent=2)+'\n')
    return data

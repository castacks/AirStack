"""Frozen development labels; neither labels nor resolver output go to the model."""
from copy import deepcopy


def _fact(identity,predicate,truth='TRUE',obj=None,stamp=10.):
    return dict(key=dict(subject=identity,predicate=predicate,obj=obj),truth=truth,
        provenance='INFERRED',source_ref='authored-semantic-evidence/v1',
        observed_monotonic_s=stamp,received_monotonic_s=stamp,max_age_s=1.)


def _evidence(blue_a='TRUE',blue_b='FALSE'):
    return [fact for identity,blue in (('marker-a',blue_a),('marker-b',blue_b))
            for fact in (_fact(identity,'exists'),_fact(identity,'kind',obj='marker'),
                         _fact(identity,'color',blue,obj='blue'),
                         _fact(identity,'color','FALSE' if blue=='TRUE' else 'TRUE',obj='orange'))]


def specification():
    base=_evidence()
    missing=[f for f in base if not(f['key']['subject']=='marker-a' and f['key']['predicate']=='color')]
    conflict=base+[_fact('marker-a','color','FALSE',obj='blue')]
    stale=deepcopy(base)
    for f in stale:
        if f['key']['subject']=='marker-a':
            f.update(observed_monotonic_s=1.,received_monotonic_s=1.)
    definitions=(
        ('blue-unique','navigate to the blue marker',base,'READY','marker-a'),
        ('orange-unique','go to the orange marker',base,'READY','marker-b'),
        ('blue-paraphrase','approach the blue marker',base,'READY','marker-a'),
        ('explicit-id','go to marker-b',base,'READY','marker-b'),
        ('duplicate-blue','navigate to the blue marker',_evidence('TRUE','TRUE'),'NEEDS_CLARIFICATION',None),
        ('missing-color','navigate to the blue marker',missing,'NEEDS_CLARIFICATION',None),
        ('conflicting-color','navigate to the blue marker',conflict,'NEEDS_CLARIFICATION',None),
        ('stale-target','navigate to the blue marker',stale,'NEEDS_CLARIFICATION',None),
        ('absent-blue','navigate to the blue marker',_evidence('FALSE','FALSE'),'NEEDS_CLARIFICATION',None),
        ('unsupported-inspection','inspect the blue marker',base,'UNSUPPORTED',None),
        ('unknown-qualifier','go to the nicest marker',base,'NEEDS_CLARIFICATION',None),
        ('unknown-competitor','navigate to the blue marker',_evidence('TRUE','UNKNOWN'),'NEEDS_CLARIFICATION',None),
    )
    cases=[]
    for index,(name,objective,evidence,status,target) in enumerate(definitions):
        # Case/adjudication names stay in the evaluator, never in model-visible metadata.
        task_id=f'intent-task-{index:03}'
        cases.append(dict(case_id=name,candidate_entity_ids=['marker-a','marker-b'],
            scope_ref='authored-finite-candidate-scope/v1',context=dict(
                task=dict(task_id=task_id,revision='task/v1',objective=objective,context_refs=['semantic-development/v1'],
                    constraints_revision='constraints/v1',issuer_id='fixture',permission_revision='none',
                    requested_embodiment_id='opaque-body'),
                snapshot=dict(snapshot_id=f'intent-observation-{index:03}',revision='state/v1',task_id=task_id,
                    episode_id='authored-semantic-development',evidence=evidence,complete_domains=[]),
                capabilities=dict(embodiment_id='opaque-body',revision='capabilities/v1',operations=['NAVIGATE_TO'],
                    resources=['mobility'],available_resources=['mobility'],limits_ref='fixture/limits'),
                now_monotonic_s=10.2),
            adjudication=dict(status=status,target=target,source='authored-label/v1')))
    return deepcopy(dict(schema='rrm-independent-intent-corpus/v2',scope='SYNTHETIC_SEMANTIC_DEVELOPMENT',cases=cases))

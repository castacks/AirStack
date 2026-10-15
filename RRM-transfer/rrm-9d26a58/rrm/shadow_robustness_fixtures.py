"""Authored sensor-neutral replay tape; not robot observations or learned output.

Adjudication labels are declared here before evaluation, never derived from the
candidate's acceptance or completion verdicts. No physical qualification follows.
"""
from copy import deepcopy
import json


def _fact(subject, predicate, truth='TRUE', obj=None, stamp=10.):
    return dict(key=dict(subject=subject,predicate=predicate,obj=obj),truth=truth,
        provenance='INFERRED',source_ref='authored-tape/'+predicate,
        observed_monotonic_s=stamp,received_monotonic_s=stamp,max_age_s=1.)


def _snapshot(evidence, revision):
    return dict(snapshot_id='tape/'+revision,revision=revision,task_id='task-1',
                episode_id='authored-shadow',evidence=evidence,complete_domains=[])


def _proposal(target='target-1', verb='NAVIGATE_TO'):
    return json.dumps(dict(status='READY',grounded_goal=dict(name='near',subject='$self',obj=target),
        grounded_entities=[target],ambiguity_refs=[],explanation='Authored replay proposal.',
        actions=[dict(id='action-1',verb=verb,targets=[target],dependencies=[])],recovery_budget=1))


def _step(*, pre=None, post=None, backend=True, proposal_valid=True, actual_goal=True,
          raw='default', permission=True, stopped=False, delay=0.):
    if pre is None:
        pre=[_fact('target-1','exists')]
    if post is None:
        post=[_fact('$self','near',obj='target-1')]
    return dict(context=dict(
        task=dict(task_id='task-1',revision='task/v1',objective='navigate to target-1',
            context_refs=['authored-tape/v1'],constraints_revision='constraints/v1',issuer_id='fixture',
            permission_revision='fixture/permission',requested_embodiment_id='opaque-body'),
        snapshot=_snapshot(pre,'before'),
        capabilities=dict(embodiment_id='opaque-body',revision='capabilities/v1',
            operations=['NAVIGATE_TO'],resources=['mobility'],available_resources=['mobility'],
            limits_ref='fixture/limits/v1'),now_monotonic_s=10.2),
        task_effect=dict(subject='$self',predicate='near',obj='target-1'),
        post_snapshot=_snapshot(post,'after'),post_now_monotonic_s=10.3,
        raw_response=_proposal() if raw=='default' else raw,
        backend_feedback=dict(success=backend),permission=permission,stop_active=stopped,
        synthetic_proposal_delay_s=delay,
        adjudication=dict(proposal_permitted=proposal_valid,actual_goal=actual_goal,
                          source='authored-independent-label/v1'))


def specification():
    """One deterministic development census; repeats do not create new trials."""
    absent=[_fact('$self','near','FALSE',obj='target-1')]
    exists=_fact('target-1','exists')
    kind=_fact('target-1','kind',obj='marker')
    cases = [
        ('nominal',[_step()]),
        ('property-only-identity',[_step(pre=[kind])]),
        ('unknown-localization',[_step(pre=[exists,_fact('target-1','localized','UNKNOWN')])]),
        ('contradictory-existence',[_step(pre=[exists,_fact('target-1','exists','FALSE'),kind],
            proposal_valid=False,post=absent,actual_goal=False)]),
        ('unknown-existence',[_step(pre=[_fact('target-1','exists','UNKNOWN'),kind],
            proposal_valid=False,post=absent,actual_goal=False)]),
        ('false-existence',[_step(pre=[_fact('target-1','exists','FALSE'),kind],
            proposal_valid=False,post=absent,actual_goal=False)]),
        ('contradictory-property-only',[_step(pre=[kind,_fact('target-1','kind','FALSE',obj='marker')],
            proposal_valid=False,post=absent,actual_goal=False)]),
        ('stale-identity',[_step(pre=[_fact('target-1','exists',stamp=1.)],
            proposal_valid=False,post=absent,actual_goal=False)]),
        ('invented-target',[_step(raw=_proposal('invented'),proposal_valid=False,post=absent,actual_goal=False)]),
        ('unsupported-verb',[_step(raw=_proposal(verb='GRASP'),proposal_valid=False,post=absent,actual_goal=False)]),
        ('wrong-valid-target',[_step(pre=[exists,_fact('target-2','exists')],raw=_proposal('target-2'),
            post=absent+[_fact('$self','near',obj='target-2')],proposal_valid=False,actual_goal=False)]),
        ('unmet-effect',[_step(post=absent,actual_goal=False)]),
        ('stale-positive-effect',[_step(post=[_fact('$self','near',obj='target-1',stamp=1.)],actual_goal=False)]),
        ('contradictory-effect',[_step(post=absent+[_fact('$self','near',obj='target-1')],actual_goal=None)]),
        ('missing-effect',[_step(post=[],actual_goal=False)]),
        ('backend-failure-observed-goal',[_step(backend=False)]),
        ('missing-feedback-and-effect',[_step(post=[],backend=None,actual_goal=None)]),
        ('recoverable-failure',[_step(post=absent,backend=False,actual_goal=False),_step()]),
        ('silent-recoverable-failure',[_step(post=absent,actual_goal=False),_step()]),
        ('persistent-failure',[_step(post=absent,actual_goal=False),_step(post=absent,actual_goal=False)]),
        ('permission-denied',[_step(permission=False,proposal_valid=False,post=[],actual_goal=None)]),
        ('stop-active',[_step(stopped=True,proposal_valid=False,post=[],actual_goal=None)]),
        ('late-proposal',[_step(delay=2.,proposal_valid=False,post=[],actual_goal=None)]),
        ('missing-proposal',[_step(raw=None,backend=None,post=[],proposal_valid=None,actual_goal=None)]),
    ]
    # The task effect and labels are fixture-author records, not model-generated goals.
    return deepcopy(dict(schema='rrm-shadow-corpus/v1',evidence_kind='AUTHORED_SYNTHETIC_TAPE',
        identities=dict(backend='frozen-skill-feedback/v1',adapter='opaque-adapter/v1',
            observations='semantic-tape/v1',proposals='authored-not-model-generated/v1',
            model=None,scene='no-physical-scene'),
        budgets=dict(max_steps_per_case=2,proposal_deadline_s=1.),
        cases=[dict(case_id=name,steps=steps) for name,steps in cases]))

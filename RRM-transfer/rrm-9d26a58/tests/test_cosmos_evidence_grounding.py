"""Sensor-neutral identity grounding: contradiction is not positive evidence."""
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import unittest

from rrm.contracts import CapabilityDeclaration
from rrm.cosmos_reason2 import (CosmosCandidateStatus, CosmosReasoningInput,
                               parse_cosmos_candidate, render_cosmos_prompt)
from rrm.state_contracts import FactEvidence, StateSnapshot
from rrm.task_contracts import TaskRequest


def fact(predicate, truth='TRUE', *, obj=None, received=10., source='SENSOR'):
    return FactEvidence.model_validate(dict(
        key=dict(subject='target-1',predicate=predicate,obj=obj),truth=truth,
        provenance=source,source_ref=f'fixture/{source}/{predicate}/{truth}',
        observed_monotonic_s=received,received_monotonic_s=received,max_age_s=1.))


def context(evidence):
    return CosmosReasoningInput(
        task=TaskRequest(task_id='grounding-probe',revision='task/v1',
            objective='navigate to target-1',constraints_revision='constraints/v1',
            issuer_id='fixture',permission_revision='none',requested_embodiment_id='body-A'),
        snapshot=StateSnapshot(snapshot_id='fixture/state',revision='state/v1',
            task_id='grounding-probe',episode_id='synthetic',evidence=tuple(evidence)),
        capabilities=CapabilityDeclaration(embodiment_id='body-A',revision='cap/v1',
            operations=frozenset({'NAVIGATE_TO','TAKEOFF'}),resources=frozenset({'mobility'}),
            available_resources=frozenset({'mobility'}),limits_ref='fixture/limits/v1'),
        now_monotonic_s=10.2)


def raw_candidate():
    return json.dumps(dict(status='READY',grounded_goal=dict(name='near',subject='$self',obj='target-1'),
        grounded_entities=['target-1'],ambiguity_refs=[],explanation='Authored proposal, not an outcome.',
        actions=[dict(id='move-1',verb='NAVIGATE_TO',targets=['target-1'],dependencies=[])],recovery_budget=0))


def cases():
    return (
        ('exists_true',(fact('exists'),),True),
        ('exists_conflict',(fact('exists'),fact('exists','FALSE')),False),
        ('exists_false_with_kind',(fact('exists','FALSE'),fact('kind',obj='marker')),False),
        ('exists_unknown_with_kind',(fact('exists','UNKNOWN'),fact('kind',obj='marker')),False),
        ('property_conflict_only',(fact('kind',obj='marker'),fact('kind','FALSE',obj='marker')),False),
        ('property_only',(fact('kind',obj='marker'),),True),
        ('independent_property_support',(fact('kind',obj='marker'),fact('kind','FALSE',obj='marker'),
                                         fact('color',obj='blue')),True),
        ('exists_true_unknown_localization',(fact('exists'),fact('localized','UNKNOWN')),True),
        ('exists_true_false_localization',(fact('exists'),fact('localized','FALSE')),True),
        ('stale_counterevidence',(fact('exists'),fact('exists','FALSE',received=1.)),True),
        ('stale_support_only',(fact('exists',received=1.),),False),
        ('fresh_false_stale_support',(fact('exists','FALSE'),fact('exists',received=1.)),False),
        ('unknown_property_only',(fact('kind','UNKNOWN',obj='marker'),),False),
        ('true_plus_unknown_presence',(fact('exists'),fact('exists','UNKNOWN')),False),
    )


def probe_rows():
    rows = []
    for name,evidence,expected in cases():
        ctx = context(evidence)
        prompt = render_cosmos_prompt(ctx)
        payload = json.loads(prompt.split('C01/C02/C03 input follows:\n',1)[1])
        candidate = parse_cosmos_candidate(raw_candidate(),ctx)
        rows.append(dict(case=name,expected_grounded=expected,
            prompt_entities=payload['state']['entity_ids'],status=candidate.status.value,
            reasons=candidate.reasons,context=payload,raw_response=raw_candidate(),
            prompt_sha256=hashlib.sha256(prompt.encode()).hexdigest(),execution_dispatch=False))
    return rows


class CosmosEvidenceGroundingTests(unittest.TestCase):
    def test_prompt_and_candidate_share_resolved_identity(self):
        rows = probe_rows()
        export = os.environ.get('RRM_GROUNDING_PROBE_OUTPUT')
        if export:
            with Path(export).open('x') as stream:
                json.dump(rows,stream,indent=2)
        for row in rows:
            with self.subTest(case=row['case']):
                self.assertEqual(row['prompt_entities'],['target-1'] if row['expected_grounded'] else [])
                expected = 'ACCEPTED' if row['expected_grounded'] else 'REJECTED'
                self.assertEqual(row['status'],expected)

    def test_prompt_retains_conflict_and_reports_resolved_truth(self):
        ctx = context((fact('exists'),fact('exists','FALSE')))
        payload = json.loads(render_cosmos_prompt(ctx).split('C01/C02/C03 input follows:\n',1)[1])
        self.assertEqual([e['truth'] for e in payload['state']['evidence']],['TRUE','FALSE'])
        self.assertEqual([e['resolved_truth'] for e in payload['state']['evidence']],['UNKNOWN','UNKNOWN'])

    def test_grounding_is_sensor_and_body_name_independent(self):
        for source in ('SENSOR','INFERRED','OPERATOR','SIMULATOR'):
            ctx = context((fact('exists',source=source),fact('localized','UNKNOWN',source=source)))
            other = replace(ctx,capabilities=replace(ctx.capabilities,embodiment_id='body-B'),
                task=ctx.task.model_copy(update={'requested_embodiment_id':'body-B'}))
            for variant in (ctx,other):
                self.assertIs(parse_cosmos_candidate(raw_candidate(),variant).status,CosmosCandidateStatus.ACCEPTED)

    def test_targetless_proposal_needs_no_target_or_depth(self):
        raw = json.dumps(dict(status='READY',grounded_goal=dict(name='airborne',subject='$self',obj=None),
            grounded_entities=[],ambiguity_refs=[],explanation='Proposal only.',
            actions=[dict(id='takeoff-1',verb='TAKEOFF',targets=[],dependencies=[])],recovery_budget=0))
        self.assertIs(parse_cosmos_candidate(raw,context(())).status,CosmosCandidateStatus.ACCEPTED)


if __name__ == '__main__':
    unittest.main()

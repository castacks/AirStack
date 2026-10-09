"""Teacher-free frozen request and separate identity/presence assessment.

No inference/transport/control imports. A trusted caller must validate frame/teacher
pairing before assessment. Pixel regions are not physical localization evidence.
"""
import base64
import hashlib
import math
import re
from typing import Mapping

from .contracts import Truth
from .visual_evaluation import canonical_sha256
from .visual_world_builder import (
    MediaArtifact, VisualCandidateStatus, VisualGroundingCandidate, VisualGroundingInput,
    parse_visual_candidate, complete_visual_identities,
)


def candidate_request(*, image: bytes, frame: Mapping, catalog: Mapping[str, str],
                      cycle_id: str) -> dict:
    """Build from scratch: no teacher, registration, regions or template evidence."""
    if not re.fullmatch(r'[0-9a-f]{32}', cycle_id) or not image or hashlib.sha256(image).hexdigest() != frame['sha256']:
        raise ValueError('invalid candidate image identity')
    if not catalog or any(not isinstance(key, str) or not isinstance(kind, str)
                          or not key.strip() or not kind.strip() for key, kind in catalog.items()):
        raise ValueError('invalid candidate catalog')
    return dict(cycle_id=cycle_id, step_index=0, observation_sha256=frame['sha256'],
                image_base64=base64.b64encode(image).decode('ascii'), entity_catalog=dict(catalog),
                context={
                    'task': dict(task_id=cycle_id, revision='frozen-visual-pilot/v1',
                        objective='Identify catalog entities supported by this frozen image; no control action.',
                        context_refs=['evidence-mode:frozen-sensor-only-pilot'],
                        constraints_revision='no-execution/v1', issuer_id='operator-eval',
                        permission_revision='inference-only', requested_embodiment_id='visual-pilot'),
                    'snapshot': dict(snapshot_id=cycle_id, revision=frame['scene_revision'],
                        task_id=cycle_id, episode_id=frame['episode_id'], evidence=[], complete_domains=[]),
                    'capabilities': dict(embodiment_id='visual-pilot', revision='no-execution/v1',
                        operations=[], resources=[], available_resources=[], limits_ref='no-execution/v1'),
                    'now_monotonic_s': 0.0,
                })


def assess_response(*, request: Mapping, response: Mapping, teacher: Mapping,
                    pairing_report: Mapping) -> dict:
    """Assess a structurally bound frozen pilot; does not establish live freshness.

    Only VISIBLE labels enter identity denominators. Both exists=TRUE and exact
    kind=TRUE are required to identify an entity. Localization claims are counted
    as unassessed, not promoted from a 2-D teacher region into physical knowledge.
    """
    if (pairing_report.get('status') != 'BOUND_FOR_ASSESSMENT'
            or pairing_report.get('teacher_sha256') != canonical_sha256(teacher)
            or pairing_report.get('image_sha256') != request.get('observation_sha256')
            or teacher['frame']['sha256'] != request.get('observation_sha256')
            or teacher['frame']['episode_id'] != request['context']['snapshot']['episode_id']
            or teacher['frame']['scene_revision'] != request['context']['snapshot']['revision']):
        raise ValueError('teacher pair is not bound to the pilot')
    if (response.get('execution_dispatch') is not False
            or any(response.get(key) != request[key]
                   for key in ('cycle_id', 'step_index', 'observation_sha256'))):
        raise ValueError('worker response identity mismatch')
    now = response['now_monotonic_s']
    if type(now) not in (int, float) or not math.isfinite(now) or now < 0:
        raise ValueError('invalid worker evidence clock')
    context = request['context']
    if context['snapshot']['evidence'] or context['snapshot']['complete_domains']:
        raise ValueError('pilot context must not contain prior evidence')
    candidate = VisualGroundingCandidate.model_validate(response['visual_candidate'])
    visual = VisualGroundingInput(
        task_id=context['task']['task_id'], episode_id=context['snapshot']['episode_id'],
        state_revision=context['snapshot']['revision'], entity_catalog=request['entity_catalog'],
        media=MediaArtifact(source_ref=f"live/{request['cycle_id']}/{request['step_index']}",
                            sha256=request['observation_sha256'], observed_monotonic_s=now),
        received_monotonic_s=now, max_age_s=5.0,
        model_ref='cosmos-reason2-live-entity-verifier/v1',
    )
    reparsed = parse_visual_candidate(candidate.raw_response, visual)
    if candidate != reparsed:
        raise ValueError('structured candidate differs from reparsed raw output')
    visible = {item['entity_id']: item['kind'] for item in teacher['labels']
               if item['exists'] and item['visibility'] == 'VISIBLE'}
    absent = {item['entity_id'] for item in teacher['labels']
              if item['exists'] is False and item['visibility'] == 'ABSENT'}
    snapshot = candidate.snapshot
    identified = set()
    positive_entities = set()
    positive_presence_entities = set()
    matched_facts = 0
    assessed_facts = 0
    unassessed_localization = 0
    if snapshot is not None:
        for fact in snapshot.evidence:
            if fact.key.predicate == 'localized':
                unassessed_localization += 1
                continue
            assessed_facts += 1
            expected_kind = visible.get(fact.key.subject)
            if fact.truth is Truth.TRUE:
                positive_entities.add(fact.key.subject)
                if fact.key.predicate == 'exists':
                    positive_presence_entities.add(fact.key.subject)
                if expected_kind is not None and (fact.key.predicate == 'exists'
                        or (fact.key.predicate == 'kind' and fact.key.obj == expected_kind)):
                    matched_facts += 1
        identified = (set(complete_visual_identities(snapshot, entity_catalog=visible,
                                                    now_monotonic_s=now)) if visible else set())
    candidate_identities = (() if snapshot is None else complete_visual_identities(
        snapshot, entity_catalog=request['entity_catalog'], now_monotonic_s=now))
    return dict(schema_version='rrm-frozen-visual-pilot/v1',
                candidate_status=candidate.status.value, candidate_reasons=list(candidate.reasons),
                attempt_count=1, visible_entity_denominator=len(visible),
                correctly_identified_entities=sorted(identified),
                identity_numerator=len(identified), identity_denominator=len(visible),
                exact_identity_fact_matches=matched_facts,
                candidate_identity_fact_denominator=assessed_facts,
                teacher_identity_fact_denominator=2 * len(visible),
                identity_fact_precision=None if not assessed_facts else matched_facts / assessed_facts,
                identity_fact_recall=None if not visible else matched_facts / (2 * len(visible)),
                positive_entities_without_visible_teacher_support=sorted(positive_entities - set(visible)),
                teacher_absent_entities=sorted(absent),
                absent_entity_denominator=len(absent),
                false_positive_presence_entities=sorted(positive_presence_entities & absent),
                false_positive_presence_count=len(positive_presence_entities & absent),
                absent_entities_without_positive_presence_claim=sorted(absent - positive_presence_entities),
                unassessed_localization_claims=unassessed_localization,
                raw_candidate_reparsed=True, teacher_sent_to_candidate=False,
                schema_accepted=candidate.status is VisualCandidateStatus.ACCEPTED,
                candidate_complete_identities=list(candidate_identities),
                teacher_supported_complete_identities=sorted(identified),
                identity_scope='exists-and-exact-kind-only',
                frozen_assessment=True, live_freshness_verified=False,
                execution_dispatch=False, spatial_localization_verified=False,
                accepted=candidate.status is VisualCandidateStatus.ACCEPTED)

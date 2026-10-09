"""Frozen pilot boundaries, not learned model performance."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))

from test_visual_evaluation import fixture
from rrm.visual_evaluation import validate_visual_pair
from rrm.visual_pilot import candidate_request, assess_response
from rrm.visual_world_builder import MediaArtifact, VisualGroundingInput, parse_visual_candidate

PILOT_ID = 'a' * 32


class PilotTests(unittest.TestCase):
    def setUp(self):
        self.pair = fixture()
        self.report = validate_visual_pair(**self.pair)
        self.catalog = {item['entity_id']: item['kind'] for item in self.pair['teacher']['labels']}
        self.request = candidate_request(image=self.pair['image'], frame=self.pair['frame'],
                                         catalog=self.catalog, cycle_id=PILOT_ID)

    def response(self, claims):
        context = self.request['context']
        visual = VisualGroundingInput(task_id=PILOT_ID, episode_id=context['snapshot']['episode_id'],
            state_revision=context['snapshot']['revision'], entity_catalog=self.catalog,
            media=MediaArtifact(source_ref=f'live/{PILOT_ID}/0', sha256=self.request['observation_sha256'],
                                observed_monotonic_s=100.), received_monotonic_s=100., max_age_s=5.,
            model_ref='cosmos-reason2-live-entity-verifier/v1')
        candidate = parse_visual_candidate(json.dumps(dict(status='READY', claims=claims)), visual)
        return dict(cycle_id=PILOT_ID, step_index=0, observation_sha256=self.request['observation_sha256'],
                    visual_candidate=candidate.model_dump(mode='json'), now_monotonic_s=100., execution_dispatch=False)

    def assess(self, response):
        return assess_response(request=self.request, response=response, teacher=self.pair['teacher'],
                               pairing_report=self.report)

    def test_teacher_free_request_and_loadable_context(self):
        from scripts.rrm_cosmos_reason2 import load_context_payload
        from scripts.rrm_cosmos_worker import decode_request
        decode_request(self.request)
        load_context_payload(self.request['context'])
        self.assertEqual(self.request['context']['snapshot']['evidence'], [])
        self.assertEqual(self.request['context']['capabilities']['operations'], [])
        for forbidden in ('teacher', 'labels', 'image_region', 'scene_prim', 'producer_ref'):
            self.assertNotIn(forbidden, json.dumps(self.request))

    def test_exact_identity_and_unassessed_localization(self):
        claims = [dict(subject=entity, predicate=predicate, obj=kind if predicate == 'kind' else None,
                       truth='TRUE') for entity, kind in self.catalog.items()
                  for predicate in ('exists', 'kind', 'localized')]
        result = self.assess(self.response(claims))
        self.assertEqual(result['identity_numerator'], 2)
        self.assertEqual(result['identity_fact_precision'], 1.)
        self.assertEqual(result['identity_fact_recall'], 1.)
        self.assertEqual(result['unassessed_localization_claims'], 2)
        self.assertFalse(result['spatial_localization_verified'])

    def test_refusal_retained_with_denominator(self):
        result = self.assess(self.response([]))
        self.assertFalse(result['accepted'])
        self.assertEqual(result['identity_numerator'], 0)
        self.assertEqual(result['identity_denominator'], 2)
        self.assertIsNone(result['identity_fact_precision'])
        self.assertEqual(result['identity_fact_recall'], 0.)

    def test_presence_only_does_not_infer_kind_from_catalog(self):
        claims = [dict(subject=entity, predicate='exists', obj=None, truth='TRUE')
                  for entity in self.catalog]
        result = self.assess(self.response(claims))
        self.assertTrue(result['accepted'])
        self.assertEqual(result['exact_identity_fact_matches'], 2)
        self.assertEqual(result['identity_fact_recall'], .5)
        self.assertEqual(result['identity_numerator'], 0)
        self.assertEqual(result['identity_denominator'], 2)
        self.assertTrue(result['schema_accepted'])
        self.assertEqual(result['candidate_complete_identities'], [])
        self.assertEqual(result['teacher_supported_complete_identities'], [])

    def test_candidate_catalog_cannot_override_teacher_kind(self):
        entity = next(iter(self.catalog))
        self.catalog[entity] = 'wrong candidate kind'
        self.request['entity_catalog'][entity] = 'wrong candidate kind'
        claims = [dict(subject=entity, predicate='exists', obj=None, truth='TRUE'),
                  dict(subject=entity, predicate='kind', obj='wrong candidate kind', truth='TRUE')]
        result = self.assess(self.response(claims))
        self.assertEqual(result['candidate_complete_identities'], [entity])
        self.assertEqual(result['teacher_supported_complete_identities'], [])
        self.assertEqual(result['identity_numerator'], 0)

    def test_teacher_changed_after_validation_rejected(self):
        self.pair['teacher']['labels'][0]['visibility'] = 'UNKNOWN'
        with self.assertRaises(ValueError): self.assess(self.response([]))

    def test_measured_absence_counts_false_positive_but_unknown_does_not(self):
        entity = self.pair['teacher']['labels'][0]['entity_id']
        row = self.pair['teacher']['labels'][0]
        claims = [dict(subject=entity, predicate='exists', obj=None, truth='TRUE')]
        for visibility, exists, count in [('UNKNOWN', True, 0), ('ABSENT', False, 1)]:
            row.update(visibility=visibility, exists=exists, image_region=None)
            self.report = validate_visual_pair(**self.pair)
            result = self.assess(self.response(claims))
            self.assertEqual(result['false_positive_presence_count'], count)
        result = self.assess(self.response([]))
        self.assertEqual(result['false_positive_presence_count'], 0)
        self.assertEqual(result['absent_entities_without_positive_presence_claim'], [entity])
        self.assertFalse(result['accepted'])

    def test_identity_and_structure_tampering_rejected(self):
        for key, value in [('cycle_id', 'wrong'), ('execution_dispatch', True), ('now_monotonic_s', True)]:
            response = self.response([]); response[key] = value
            with self.assertRaises(ValueError): self.assess(response)
        response = self.response([])
        response['visual_candidate']['raw_response'] = 'changed'
        with self.assertRaises(ValueError): self.assess(response)

    def test_prior_evidence_rejected(self):
        response = self.response([])
        self.request['context']['snapshot']['complete_domains'] = ['exists']
        with self.assertRaises(ValueError): self.assess(response)

    def test_image_mismatch_rejected(self):
        with self.assertRaises(ValueError):
            candidate_request(image=b'wrong', frame=self.pair['frame'], catalog=self.catalog, cycle_id=PILOT_ID)

    def test_invalid_worker_cycle_rejected_before_transport(self):
        with self.assertRaises(ValueError):
            candidate_request(image=self.pair['image'], frame=self.pair['frame'],
                              catalog=self.catalog, cycle_id='human-label')


if __name__ == '__main__': unittest.main()

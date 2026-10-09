"""Prompt and real warm-worker method contracts; fake generation, no model claim."""
import hashlib
import json
from pathlib import Path
import sys
import threading
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from rrm_cosmos_worker import CosmosWorker, VISUAL_SOURCE_SHA256
from rrm.visual_world_builder import (
    VISUAL_PROMPT_REVISION, VisualCandidateStatus, render_visual_prompt,
    parse_visual_candidate, visual_output_schema,
)
from rrm.visual_pilot import candidate_request
from test_visual_world_builder import _context
from test_visual_evaluation import fixture


class PromptContractTests(unittest.TestCase):
    def test_schema_is_real_json_with_explicit_fields_and_uncertainty(self):
        schema = json.loads(json.dumps(visual_output_schema()))
        claim = schema['properties']['claims']['items']
        self.assertEqual(set(claim['required']), {'subject', 'predicate', 'obj', 'truth'})
        self.assertEqual(claim['properties']['truth']['enum'], ['TRUE', 'FALSE', 'UNKNOWN'])
        prompt = render_visual_prompt(_context())
        self.assertIn(json.dumps(schema, sort_keys=True, separators=(',', ':')), prompt)
        self.assertNotIn('str|null', prompt)
        self.assertIn('do not claim localized TRUE', prompt)
        self.assertIn('Catalog membership is not image evidence', prompt)
        self.assertIn(VISUAL_PROMPT_REVISION, prompt)
        for forbidden in ('image_region', 'scene_prim', 'SIMULATOR'):
            self.assertNotIn(forbidden, prompt)

    def test_original_failure_is_not_repaired(self):
        raw = json.dumps(dict(status='READY', claims=[dict(subject='loading_bay_marker',
                           predicate='exists', obj='marker')]))
        result = parse_visual_candidate(raw, _context())
        self.assertIs(result.status, VisualCandidateStatus.REJECTED)
        self.assertIsNone(result.snapshot)
        self.assertEqual(result.raw_response, raw)

    def test_generic_example_is_not_target_evidence_and_rules_stay_strict(self):
        prompt = render_visual_prompt(_context())
        self.assertEqual(VISUAL_PROMPT_REVISION, 'visual-claims/v3')
        self.assertIn('TWO TRUE claims', prompt)
        self.assertIn('FORMAT EXAMPLE ONLY', prompt)
        self.assertIn('not evidence for the supplied image', prompt)
        self.assertIn('not READY with empty claims', prompt)
        example = next(line for line in prompt.splitlines() if line.startswith('{"status":"READY"'))
        value = json.loads(example)
        self.assertEqual([c['predicate'] for c in value['claims']], ['exists', 'kind'])
        # Example IDs do not bypass actual catalog validation.
        self.assertIs(parse_visual_candidate(example, _context()).status, VisualCandidateStatus.REJECTED)

    def test_all_four_claim_fields_are_required(self):
        claim = dict(subject='loading_bay_marker', predicate='exists', obj=None, truth='TRUE')
        for field in claim:
            missing = dict(claim); del missing[field]
            result = parse_visual_candidate(json.dumps(dict(status='READY', claims=[missing])), _context())
            self.assertIs(result.status, VisualCandidateStatus.REJECTED)

    def test_uncertain_claim_and_explicit_clarification(self):
        result = parse_visual_candidate(json.dumps(dict(status='READY', claims=[dict(
            subject='red_crate', predicate='exists', obj=None, truth='UNKNOWN')])), _context())
        self.assertIs(result.status, VisualCandidateStatus.ACCEPTED)
        self.assertEqual(result.snapshot.evidence[0].truth.value, 'UNKNOWN')
        for raw in ('{"status":"NEEDS_CLARIFICATION"}',
                    '{"status":"NEEDS_CLARIFICATION","claims":null}'):
            self.assertIs(parse_visual_candidate(raw, _context()).status, VisualCandidateStatus.REJECTED)
        result = parse_visual_candidate('{"status":"NEEDS_CLARIFICATION","claims":[]}', _context())
        self.assertIs(result.status, VisualCandidateStatus.NEEDS_CLARIFICATION)

    def test_real_worker_method_uses_new_prompt_and_retains_failure(self):
        pair = fixture()
        catalog = {item['entity_id']: item['kind'] for item in pair['registration']['entities']}
        payload = candidate_request(image=pair['image'], frame=pair['frame'], catalog=catalog,
                                    cycle_id='b' * 32)
        calls = []
        class FakeGenerator:
            def generate(self, **kwargs):
                calls.append(kwargs)
                self.image = Path(kwargs['image_path']).read_bytes()
                return '{"status":"READY","claims":[]}'
        worker = CosmosWorker.__new__(CosmosWorker)
        worker.generator = FakeGenerator()
        worker.lock = threading.Lock()
        worker.max_new_tokens = 768
        response = worker.verify_entities(payload)
        self.assertEqual(len(calls), 1)
        self.assertEqual(worker.generator.image, pair['image'])
        self.assertEqual(response['visual_prompt_revision'], VISUAL_PROMPT_REVISION)
        self.assertEqual(response['visual_prompt_sha256'], hashlib.sha256(calls[0]['prompt'].encode()).hexdigest())
        self.assertEqual(response['visual_world_builder_source_sha256'], VISUAL_SOURCE_SHA256)
        self.assertEqual(response['visual_candidate']['status'], 'REJECTED')
        self.assertFalse(response['execution_dispatch'])
        self.assertEqual(calls[0]['max_new_tokens'], 768)


if __name__ == '__main__': unittest.main()

"""Reference resolution is restricted, scope-aware and independent of model output."""
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import unittest

from rrm.intent_grounding_fixtures import specification
from rrm.intent_resolution import resolve_task_intent
from rrm.shadow_robustness import _context


class IntentResolutionTests(unittest.TestCase):
    def resolve(self,case):
        return resolve_task_intent(_context(case['context']),
            candidate_entity_ids=tuple(case['candidate_entity_ids']),scope_ref=case['scope_ref'])

    def test_frozen_author_labels_match_without_model_or_expected_input(self):
        corpus=specification()
        self.assertEqual(len(corpus['cases']),12)
        for case in corpus['cases']:
            with self.subTest(case=case['case_id']):
                result=self.resolve(case)
                self.assertEqual(result.status,case['adjudication']['status'])
                self.assertEqual(result.binding.grounded_goal.obj if result.binding else None,case['adjudication']['target'])

    def test_unknown_competitor_is_not_silently_excluded(self):
        case=next(c for c in specification()['cases'] if c['case_id']=='unknown-competitor')
        self.assertEqual(self.resolve(case).reasons,('unresolved_possible_competitor',))

    def test_extra_language_or_multistep_request_is_not_partially_compiled(self):
        case=specification()['cases'][0]
        for objective in ('navigate to the blue marker and then land','do not navigate to the blue marker',
                          'go to the most useful marker','navigate to invented'):
            ctx=_context(case['context'])
            result=resolve_task_intent(replace(ctx,task=ctx.task.model_copy(update={'objective':objective})),
                candidate_entity_ids=tuple(case['candidate_entity_ids']),scope_ref=case['scope_ref'])
            self.assertEqual(result.status,'NEEDS_CLARIFICATION')
            self.assertIsNone(result.binding)

    def test_invalid_scope_clock_prebinding_or_body_mismatch_cannot_bind(self):
        case=specification()['cases'][0]
        ctx=_context(case['context'])
        for ids,ref in (((), 'scope'),(('marker-a','marker-a'),'scope'),(('marker-a',),'')):
            with self.assertRaises(ValueError):resolve_task_intent(ctx,candidate_entity_ids=ids,scope_ref=ref)
        with self.assertRaises(ValueError):
            resolve_task_intent(replace(ctx,now_monotonic_s=float('nan')),candidate_entity_ids=('marker-a',),scope_ref='scope')
        binding=self.resolve(case).binding
        with self.assertRaises(ValueError):
            resolve_task_intent(replace(ctx,intent_binding=binding),candidate_entity_ids=('marker-a',),scope_ref='scope')
        result=resolve_task_intent(replace(ctx,capabilities=replace(ctx.capabilities,embodiment_id='other')),
            candidate_entity_ids=('marker-a',),scope_ref='scope')
        self.assertEqual(result.status,'NEEDS_CLARIFICATION')

    def test_no_navigation_capability_cannot_be_invented(self):
        case=specification()['cases'][0]
        ctx=_context(case['context'])
        result=resolve_task_intent(replace(ctx,capabilities=replace(ctx.capabilities,operations=frozenset())),
            candidate_entity_ids=('marker-a',),scope_ref='scope')
        self.assertEqual(result.status,'UNSUPPORTED')

    @classmethod
    def setUpClass(cls):
        path=Path(__file__).resolve().parents[1]/'scripts/independent_intent_probe.py'
        spec=importlib.util.spec_from_file_location('intent_probe',path)
        cls.probe=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.probe)

    def test_model_request_has_no_label_binding_or_resolved_target(self):
        case=specification()['cases'][0]
        request=self.probe.build_request(case,self.probe.neutral_png(),0)
        self.assertEqual(set(request),{'cycle_id','step_index','observation_sha256','context','image_base64'})
        self.assertNotIn('intent_binding',request['context'])
        self.assertNotIn('adjudication',request['context'])
        response=dict(cycle_id=request['cycle_id'],step_index=0,observation_sha256=request['observation_sha256'],execution_dispatch=False)
        self.probe.validate_response(request,response)
        response['step_index']=False
        with self.assertRaises(ValueError):self.probe.validate_response(request,response)

    def test_all_case_names_are_absent_from_worker_visible_metadata(self):
        image=self.probe.neutral_png()
        for index,case in enumerate(specification()['cases']):
            request=self.probe.build_request(case,image,index)
            context=request['context']
            self.assertEqual(context['task']['task_id'],f'intent-task-{index:03}')
            self.assertEqual(context['snapshot']['snapshot_id'],f'intent-observation-{index:03}')
            self.assertEqual(context['snapshot']['task_id'],context['task']['task_id'])
            self.assertNotIn(case['case_id'],json.dumps(request))

    def test_wrong_model_target_is_held_not_copied_into_binding(self):
        case=specification()['cases'][0]
        raw=dict(status='READY',grounded_goal=dict(name='near',subject='$self',obj='marker-b'),
            grounded_entities=['marker-b'],ambiguity_refs=[],explanation='wrong target',
            actions=[dict(id='a',verb='NAVIGATE_TO',targets=['marker-b'],dependencies=[])],recovery_budget=0)
        result=self.probe.assess(case,dict(candidate=dict(raw_response=json.dumps(raw))))
        self.assertEqual(result['reference_target'],'marker-a')
        self.assertFalse(result['model_matches_authored_intent'])
        self.assertEqual(result['pipeline_status'],'HELD_MODEL_MISMATCH')
        ambiguous=next(c for c in specification()['cases'] if c['case_id']=='duplicate-blue')
        result=self.probe.assess(ambiguous,dict(candidate=dict(raw_response=json.dumps(raw))))
        self.assertEqual(result['pipeline_status'],'NEEDS_CLARIFICATION')
        self.assertIsNone(result['binding'])
        self.assertIsNone(result['guarded_candidate'])

    def test_empty_output_and_false_clarification_are_not_grounding_success(self):
        case=specification()['cases'][0]
        for raw in ('',json.dumps(dict(status='NEEDS_CLARIFICATION',grounded_goal=None,grounded_entities=[],
            ambiguity_refs=['target'],explanation='unsure',actions=[],recovery_budget=0))):
            result=self.probe.assess(case,dict(candidate=dict(raw_response=raw)))
            self.assertFalse(result['model_matches_authored_intent'])
            self.assertEqual(result['pipeline_status'],'HELD_MODEL_MISMATCH')


if __name__=='__main__':unittest.main()

"""Independent semantic bindings constrain proposals, never authorize control."""
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import unittest

from rrm.cosmos_reason2 import parse_cosmos_candidate, render_cosmos_prompt
from rrm.intent_alignment import TaskIntentBinding
from rrm.schema import Predicate
from rrm.shadow_robustness import _context
from rrm.shadow_robustness_fixtures import specification


def fixture():
    step=next(c for c in specification()['cases'] if c['case_id']=='wrong-valid-target')['steps'][0]
    ctx=_context(step['context'])
    binding=TaskIntentBinding(binding_id='independent-task/v1',source_ref='authored-task-interpretation/v1',
        task_id=ctx.task.task_id,task_revision=ctx.task.revision,state_revision=ctx.snapshot.revision,
        capability_revision=ctx.capabilities.revision,
        snapshot_id=ctx.snapshot.snapshot_id,episode_id=ctx.snapshot.episode_id,
        embodiment_id=ctx.capabilities.embodiment_id,
        grounded_goal=Predicate(name='near',subject='$self',obj='target-1'),
        allowed_targets=('target-1',),allowed_operations=('NAVIGATE_TO',))
    raw=json.loads(step['raw_response'])
    return ctx,binding,raw


def matching_raw():
    _,_,raw=fixture()
    raw['grounded_goal']['obj']='target-1'
    raw['grounded_entities']=['target-1']
    raw['actions'][0]['targets']=['target-1']
    return raw


class IntentAlignmentTests(unittest.TestCase):
    def test_unbound_wrong_target_is_explicitly_not_checked(self):
        ctx,_,raw=fixture()
        candidate=parse_cosmos_candidate(json.dumps(raw),ctx)
        self.assertEqual(candidate.status.value,'ACCEPTED')
        self.assertEqual(candidate.intent_alignment,'NOT_CHECKED')
        self.assertIsNone(candidate.intent_binding_ref)

    def test_bound_wrong_target_rejects_but_matching_target_is_hashed(self):
        ctx,binding,raw=fixture()
        ctx=replace(ctx,intent_binding=binding)
        candidate=parse_cosmos_candidate(json.dumps(raw),ctx)
        self.assertEqual(candidate.status.value,'REJECTED')
        self.assertIn('intent_goal_mismatch',candidate.reasons)
        self.assertIn('intent_target_not_allowed:action-1',candidate.reasons)
        good=parse_cosmos_candidate(json.dumps(matching_raw()),ctx)
        self.assertEqual(good.intent_alignment,'MATCHED')
        self.assertEqual(good.intent_binding_ref,binding.binding_id)
        self.assertEqual(good.intent_binding_sha256,binding.content_sha256())

    def test_correct_goal_cannot_hide_wrong_action_or_extra_detour(self):
        ctx,binding,_=fixture()
        for extra in (False,True):
            raw=matching_raw()
            wrong=dict(id='detour',verb='NAVIGATE_TO',targets=['target-2'],dependencies=[])
            if extra:raw['actions'].append(wrong)
            else:raw['actions']=[wrong]
            raw['grounded_entities'].append('target-2')
            candidate=parse_cosmos_candidate(json.dumps(raw),replace(ctx,intent_binding=binding))
            self.assertEqual(candidate.status.value,'REJECTED')
            self.assertIn('intent_target_not_allowed:detour',candidate.reasons)

    def test_wrong_predicate_subject_and_negation_reject(self):
        ctx,binding,_=fixture()
        for field,value in (('name','holding'),('subject','target-1'),('negated',True)):
            raw=matching_raw()
            raw['grounded_goal'][field]=value
            candidate=parse_cosmos_candidate(json.dumps(raw),replace(ctx,intent_binding=binding))
            self.assertEqual(candidate.status.value,'REJECTED')
            self.assertIn('intent_goal_mismatch',candidate.reasons)
        raw=matching_raw()
        raw['grounded_goal']['negated']='false'
        self.assertIn('invalid_goal_negation',parse_cosmos_candidate(json.dumps(raw),ctx).reasons)

    def test_stale_scopes_unresolved_targets_and_unsupported_binding_reject(self):
        ctx,binding,_=fixture()
        for field,value,reason in (
            ('task_id','another','intent_binding_task_id_mismatch'),
            ('task_revision','old','intent_binding_task_revision_mismatch'),
            ('state_revision','old','intent_binding_state_revision_mismatch'),
            ('capability_revision','old','intent_binding_capability_revision_mismatch'),
            ('snapshot_id','other','intent_binding_snapshot_id_mismatch'),
            ('episode_id','other','intent_binding_episode_id_mismatch'),
            ('embodiment_id','other','intent_binding_embodiment_id_mismatch'),
            ('allowed_targets',('target-1','unseen'),'intent_binding_target_unresolved'),
            ('allowed_operations',('NAVIGATE_TO','GRASP'),'intent_binding_operation_unsupported')):
            changed=TaskIntentBinding.model_validate({**binding.model_dump(),field:value})
            candidate=parse_cosmos_candidate(json.dumps(matching_raw()),replace(ctx,intent_binding=changed))
            self.assertEqual(candidate.status.value,'REJECTED')
            self.assertIn(reason,candidate.reasons)

    def test_task_body_and_capability_body_cannot_disagree(self):
        ctx,binding,_=fixture()
        ctx=replace(ctx,task=ctx.task.model_copy(update={'requested_embodiment_id':'other-body'}),
                    intent_binding=binding)
        self.assertIn('intent_context_embodiment_mismatch',
                      parse_cosmos_candidate(json.dumps(matching_raw()),ctx).reasons)

    def test_model_binding_cannot_create_or_override_caller_binding(self):
        ctx,binding,wrong=fixture()
        wrong['intent_binding']={**binding.model_dump(mode='json'),
                                 'grounded_goal':wrong['grounded_goal'],'allowed_targets':['target-2']}
        self.assertEqual(parse_cosmos_candidate(json.dumps(wrong),ctx).intent_alignment,'NOT_CHECKED')
        self.assertEqual(parse_cosmos_candidate(json.dumps(wrong),replace(ctx,intent_binding=binding)).status.value,'REJECTED')

    def test_disallowed_operation_rejects_even_if_capability_supports_it(self):
        ctx,binding,_=fixture()
        ctx=replace(ctx,capabilities=replace(ctx.capabilities,operations=frozenset({'TAKEOFF','NAVIGATE_TO'})),
                    intent_binding=binding)
        raw=matching_raw()
        raw['actions']=[dict(id='other',verb='TAKEOFF',targets=[],dependencies=[])]
        candidate=parse_cosmos_candidate(json.dumps(raw),ctx)
        self.assertIn('intent_operation_not_allowed:other',candidate.reasons)

    def test_allowed_preparation_and_targetless_plan_needs_no_depth(self):
        ctx,binding,_=fixture()
        capabilities=replace(ctx.capabilities,operations=frozenset({'TAKEOFF','NAVIGATE_TO'}))
        binding=TaskIntentBinding.model_validate({**binding.model_dump(),
            'allowed_operations':('TAKEOFF','NAVIGATE_TO')})
        ctx=replace(ctx,capabilities=capabilities,intent_binding=binding)
        raw=matching_raw()
        raw['actions'][0]['dependencies']=['prepare']
        raw['actions'].insert(0,dict(id='prepare',verb='TAKEOFF',targets=[],dependencies=[]))
        self.assertEqual(parse_cosmos_candidate(json.dumps(raw),ctx).intent_alignment,'MATCHED')
        raw['grounded_goal']=dict(name='airborne',subject='$self',obj=None)
        raw['grounded_entities']=[]
        raw['actions']=raw['actions'][:1]
        binding=TaskIntentBinding.model_validate({**binding.model_dump(),
            'grounded_goal':raw['grounded_goal'],'allowed_targets':(),'allowed_operations':('TAKEOFF',)})
        self.assertEqual(parse_cosmos_candidate(json.dumps(raw),replace(ctx,intent_binding=binding)).intent_alignment,'MATCHED')

    def test_prompt_and_cli_keep_binding_separate_from_model_output(self):
        ctx,binding,_=fixture()
        prompt=render_cosmos_prompt(replace(ctx,intent_binding=binding))
        payload=json.loads(prompt.split('C01/C02/C03 input follows:\n',1)[1])
        self.assertEqual(payload['intent_binding'],binding.model_dump(mode='json'))
        path=Path(__file__).resolve().parents[1]/'scripts/rrm_cosmos_reason2.py'
        spec=importlib.util.spec_from_file_location('intent_cli',path)
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        step=specification()['cases'][0]['steps'][0]
        loaded=module.load_context_payload({**step['context'],'intent_binding':binding.model_dump(mode='json')})
        self.assertEqual(loaded.intent_binding,binding)

    def test_binding_shape_requires_provenance_and_unique_scope(self):
        _,binding,_=fixture()
        for change in ({'source_ref':''},{'allowed_targets':('target-1','target-1')},
                       {'allowed_operations':()},{'allowed_targets':('$self',)}, {'unexpected':True}):
            with self.assertRaises(ValueError):
                TaskIntentBinding.model_validate({**binding.model_dump(),**change})

    def test_binding_goal_is_deeply_immutable_and_not_aliased_to_caller(self):
        _,binding,_=fixture()
        original=Predicate(name='near',subject='$self',obj='target-1')
        changed=TaskIntentBinding.model_validate({**binding.model_dump(),'grounded_goal':original})
        stamp=changed.content_sha256()
        original.obj='target-2'
        self.assertEqual(changed.grounded_goal.obj,'target-1')
        self.assertEqual(changed.content_sha256(),stamp)
        with self.assertRaises(ValueError):changed.grounded_goal.obj='target-2'
        with self.assertRaises(ValueError):
            TaskIntentBinding.model_validate({**binding.model_dump(),
                'grounded_goal':dict(name='near',subject='$self',obj='target-2')})

    def test_matching_goal_requires_an_authored_goal_producing_action(self):
        ctx,binding,_=fixture()
        binding=TaskIntentBinding.model_validate({**binding.model_dump(),
            'allowed_operations':('TAKEOFF','NAVIGATE_TO')})
        ctx=replace(ctx,capabilities=replace(ctx.capabilities,operations=frozenset({'TAKEOFF','NAVIGATE_TO'})),
                    intent_binding=binding)
        raw=matching_raw()
        raw['actions']=[dict(id='prepare-only',verb='TAKEOFF',targets=[],dependencies=[])]
        candidate=parse_cosmos_candidate(json.dumps(raw),ctx)
        self.assertIn('intent_goal_not_supported_by_plan_effects',candidate.reasons)


if __name__=='__main__':unittest.main()

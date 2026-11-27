"""Offline architecture/authority guardrails; no runtime or deployment tests."""
import copy
import importlib.util
import json
from pathlib import Path
import unittest
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'RRM/evidence/M00-E1C'
spec=importlib.util.spec_from_file_location('design',ROOT/'RRM/scripts/m00e1c_design_check.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class DesignTests(unittest.TestCase):
 def setUp(self):self.model=json.loads((E/'M00E1C_DESIGN_MODEL.json').read_text())
 def test_both_policy_projections_acyclic(self):
  r=m.check_model(self.model);self.assertTrue(r['design_consistent']);self.assertFalse(r['execution_authorized']);self.assertFalse(r['implementation_accepted']);self.assertEqual({p['policy'] for p in r['projections']},{'P0','P1'})
 def test_circular_p1_ready_dependency_rejected(self):
  self.model['dependencies'].append(dict(source='READY',target='P1PERMIT',policy='P1',meaning='INVALID'))
  with self.assertRaises(ValueError):m.check_model(self.model)
 def test_missing_fresh_p1_gate_rejected(self):
  self.model['dependencies']=[r for r in self.model['dependencies'] if not(r['source']=='FRESH' and r['target']=='P1PERMIT')]
  with self.assertRaises(ValueError):m.check_model(self.model)
 def test_p0_implicitly_gated_rejected(self):
  self.model['dependencies'].append(dict(source='FRESH',target='P0PERMIT',policy='P0',meaning='INVALID'))
  with self.assertRaises(ValueError):m.check_model(self.model)
 def test_unknown_component_and_duplicate_edge_rejected(self):
  for mode in ['unknown','duplicate']:
   d=copy.deepcopy(self.model)
   if mode=='unknown':d['dependencies'].append(dict(source='MOTOR',target='GUARD',policy='BOTH'))
   else:d['dependencies'].append(copy.deepcopy(d['dependencies'][0]))
   with self.subTest(mode=mode):
    with self.assertRaises(ValueError):m.check_model(d)
 def test_missing_phase_or_transaction_rejected(self):
  for source,target in [('SCENE','PLAY'),('TX','READBACK')]:
   d=copy.deepcopy(self.model);d['dependencies']=[r for r in d['dependencies'] if not(r['source']==source and r['target']==target)]
   with self.subTest(edge=(source,target)):
    with self.assertRaises(ValueError):m.check_model(d)
 def test_false_design_authority_rejected(self):
  for field in ['construction_authorized','deployment_authorized','active_verification_authorized','tier_b_authorized']:
   d=copy.deepcopy(self.model);d[field]=True
   with self.subTest(field=field):
    with self.assertRaises(ValueError):m.check_model(d)
 def test_missing_control_rejected(self):
  self.model['controls'].pop()
  with self.assertRaises(ValueError):m.check_model(self.model)
class ManifestDecisionTests(unittest.TestCase):
 def test_null_manifest_never_execution_complete(self):
  d=json.loads((E/'M00E1C_MANIFEST_PROPOSAL.json').read_text());r=m.check_manifest(d)
  self.assertEqual(len(r['missing_target_field_groups']),17);self.assertFalse(r['complete_for_execution']);self.assertFalse(r['execution_authorized'])
 def test_invented_target_value_rejected(self):
  d=json.loads((E/'M00E1C_MANIFEST_PROPOSAL.json').read_text());d['required_target_fields']['workflow_uid_pool_host_boot_owner']='guessed-workflow'
  with self.assertRaises(ValueError):m.check_manifest(d)
 def test_reference_fact_cannot_become_target_attestation(self):
  d=json.loads((E/'M00E1C_MANIFEST_PROPOSAL.json').read_text());d['known_reference_records'][0]['status']='VERIFIED_TARGET'
  with self.assertRaises(ValueError):m.check_manifest(d)
 def test_pending_decision_and_delivery_remain_blank(self):
  d=json.loads((E/'M00E1C_PENDING_DECISION.json').read_text());r=m.check_pending(d);self.assertIsNone(r['selected_option']);self.assertFalse(r['execution_authorized'])
 def test_forged_approval_or_delivery_rejected(self):
  orig=json.loads((E/'M00E1C_PENDING_DECISION.json').read_text())
  for field,value in [('selected_option','APPROVE DESIGN FOR FURTHER ENGINEERING'),('pd_signature','invented'),('external_owner_requests_sent',1),('activation_authorized',True)]:
   d=copy.deepcopy(orig);d[field]=value
   with self.subTest(field=field):
    with self.assertRaises(ValueError):m.check_pending(d)
if __name__=='__main__':unittest.main()

"""Retained raw timestamp replay and fail-closed document controls; offline only."""
import copy
import csv
from decimal import Decimal
import importlib.util
import json
from pathlib import Path
import unittest
ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('replay',ROOT/'RRM/scripts/m00e1a_analyze.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class ReplayTests(unittest.TestCase):
    def setUp(self):self.doc=json.loads(m.SOURCE.read_text())
    def test_exact_retained_timing(self):
        r=m.analyze(self.doc)
        self.assertEqual({x['measurement_id']:x['seconds'] for x in r['measurements']},{'L11':'19.276162179','L03':'74.228771713'})
    def test_reconnections_are_not_trials(self):
        result=m.analyze(self.doc)
        self.assertEqual(result['heartbeat_messages_in_retained_log'],4)
        self.assertEqual(result['startup_episodes'],1)
        self.doc['original_ros_logs']['logs'][1]['text']+='\n[INFO] [1795999999.000000000] [later]: CON: Got HEARTBEAT'
        self.assertEqual(m.analyze(self.doc)['measurements'][0]['seconds'],'19.276162179')
    def test_missing_deadline_rejected(self):
        self.doc['original_ros_logs']['logs'][0]['text']=''
        with self.assertRaises(ValueError):m.analyze(self.doc)
    def test_duplicate_deadline_rejected(self):
        self.doc['original_ros_logs']['logs'][0]['text']+='\n'+self.doc['original_ros_logs']['logs'][0]['text']
        with self.assertRaises(ValueError):m.analyze(self.doc)
    def test_missing_graph_rejected(self):
        g=self.doc['filtered_logs'][0];g['records']=[r for r in g['records'] if 'PX4 node at' not in r['text']]
        with self.assertRaises(ValueError):m.analyze(self.doc)
    def test_cross_boot_rejected(self):
        self.doc['isaac_processes']['boot_id']='other-boot'
        with self.assertRaises(ValueError):m.analyze(self.doc)
    def test_malformed_native_timestamps_rejected(self):
        for parser,value in [(m.receipt_time,'2026-11-27T18:42:09.695Z log'),(m.producer_time,'[INFO] [not-a-time] log')]:
            with self.subTest(value=value):
                with self.assertRaises(ValueError):parser(value)
    def test_cross_domain_episode_boot_and_negative_rejected(self):
        a=dict(time=Decimal('10'),domain='STEADY',episode='episode',boot='boot')
        for key,value in [('domain','UTC'),('episode','other'),('boot','other'),('time',Decimal('9')),('time',Decimal('Infinity'))]:
            with self.subTest(key=key,value=value):
                b=dict(a);b[key]=value
                with self.assertRaises(ValueError):m.elapsed(a,b)
    def test_no_false_acquisition_or_authorization(self):
        r=m.analyze(self.doc)
        self.assertFalse(r['execution_authorized']);self.assertFalse(r['acquisition_readiness_verified'])
        self.assertEqual(r['new_startup_trials'],0);self.assertEqual(r['requirement_verdict'],'NOT_ASSESSED')
class GovernanceTests(unittest.TestCase):
    def test_all_original_trials_and_fields_preserved(self):
        def read(p):
            with (ROOT/p).open() as f:return list(csv.DictReader(f))
        old=read('RRM/verification/reports/M00-E1/M00E1_TRIAL_MATRIX.csv');new=read('RRM/verification/reports/M00-E1A/M00E1A_FINAL_TRIAL_MATRIX.csv')
        self.assertEqual(len(new),24)
        for a,b in zip(old,new):self.assertTrue(all(b[k]==v for k,v in a.items()))
        self.assertEqual([r['trial_id'] for r in new if r['initial_set_role']=='MINIMAL_INITIAL_PAIR_CHECKPOINT_PROPOSED'],['M00E2-TB-001','M00E2-TB-002'])
    def test_unresolved_prerequisites_have_owners_and_closure(self):
        with (ROOT/'RRM/evidence/M00-E1A/M00E1A_PREREQUISITE_REGISTER.csv').open() as f:r=list(csv.DictReader(f))
        self.assertEqual(len(r),20)
        for x in r:
            self.assertTrue(x['observed_evidence'] and x['evidence_source'])
            if x['status']!='SATISFIED':self.assertTrue(x['owner'] and x['closure_condition'] and x['active_verification_authority_needed'])
        self.assertEqual({x['prerequisite_id'] for x in r if x['status']=='SATISFIED'},{'CHK17','CHK18','CHK20'})
    def test_decision_blank_no_authority(self):
        d=json.loads((ROOT/'RRM/evidence/M00-E1A/pending-decision.json').read_text())
        self.assertEqual(d['recommendation'],'NOT READY — TECHNICAL');self.assertFalse(d['execution_authorized'])
        for field,value in d.items():
            if field.startswith('pd_') or field.startswith('approved_'):self.assertIsNone(value)
if __name__=='__main__':unittest.main()

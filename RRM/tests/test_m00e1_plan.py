"""Offline design and no-inferred-authority checks, not simulator tests."""
import copy
import csv
import importlib.util
import io
import json
from collections import Counter, defaultdict
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('m00e1_plan', ROOT/'RRM/scripts/m00e1_plan.py')
plan = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plan)

class DesignTests(unittest.TestCase):
    def test_deterministic_frozen_schedule(self):
        self.assertEqual(plan.render(), plan.render())
        self.assertEqual(plan.REPORT.read_text(), plan.render())
        self.assertEqual(list(csv.DictReader(io.StringIO(plan.render()))), plan.schedule())

    def test_balance_and_counts(self):
        rows = plan.schedule()
        self.assertEqual(len(rows), 24)
        cells = Counter((x['launch_policy'], x['cache_condition']) for x in rows)
        self.assertEqual(len(cells), 4)
        self.assertEqual(set(cells.values()), {6})
        self.assertEqual([x['order'] for x in rows], list(map(str, range(1,25))))

    def test_unique_ids_and_evidence_paths(self):
        for field in ['trial_id', 'evidence_directory']:
            self.assertEqual(len({x[field] for x in plan.schedule()}),24)
        for x in plan.schedule():
            self.assertIn(x['trial_id'], x['evidence_directory'])
            self.assertIn('<AUTH_ID>', x['evidence_directory'])

    def test_matched_pairs_and_block_repetitions(self):
        pairs = defaultdict(list)
        for x in plan.schedule(): pairs[x['matched_pair']].append(x)
        self.assertEqual(len(pairs),12)
        for pair in pairs.values():
            self.assertEqual(len(pair),2)
            self.assertEqual({x['pair_position'] for x in pair},{'1','2'})
            self.assertEqual(len({x['launch_policy'] for x in pair}),2)
            for field in ['cache_condition','cell_repetition','paired_scene_seed','configuration_id','environment_id']:
                self.assertEqual(len({x[field] for x in pair}),1)
        for cache in {x['cache_condition'] for x in plan.schedule()}:
            for policy in {x['launch_policy'] for x in plan.schedule()}:
                self.assertEqual({x['cell_repetition'] for x in plan.schedule() if x['cache_condition']==cache and x['launch_policy']==policy},set(map(str,range(1,7))))

    def test_every_trial_exploratory_and_unexecuted(self):
        for x in plan.schedule():
            self.assertEqual(x['execution_status'],'NOT_EXECUTED')
            self.assertEqual(x['authorization'],'SEPARATE_PD_AUTHORIZATION_REQUIRED')
            self.assertEqual(x['requirement_verdict'],'NOT_ASSESSED_EXPLORATORY')
            for field in ['objective','required_observations','expected_discrimination','failure_retention']:
                self.assertTrue(x[field])
            self.assertIn('no replacement',x['failure_retention'])

    def test_complete_hypotheses_operations_and_preconditions(self):
        for x in plan.schedule():
            self.assertEqual(set(x['hypotheses'].split(';')),{'HD%02d'%n for n in range(1,9)})
            self.assertEqual(set(x['required_instrumentation'].split(';')),{'I%02d'%n for n in range(1,7)})
            self.assertEqual(set(x['active_operation_ids'].split(';')),{'OP%02d'%n for n in range(1,8)})
            self.assertEqual(set(x['required_preconditions'].split(';')),{'CHK%02d'%n for n in range(1,19)})

class PendingDecisionTests(unittest.TestCase):
    def setUp(self):
        self.d=json.loads((ROOT/'RRM/evidence/M00-E1/M00E1_PENDING_DECISION.json').read_text())
    def test_pending_never_authorizes(self):
        self.assertFalse(plan.prepared_decision_valid(self.d)['execution_authorized'])
    def test_reject_inferred_approval_or_signature(self):
        for field,value in [('pd_decision','APPROVE'),('pd_signature','someone'),('package_state','READY'),('execution_authorized',True),('requested_maximum_startups',25),('approval_reference','made-up'),('approved_environment','existing-workflow')]:
            with self.subTest(field=field):
                d=copy.deepcopy(self.d);d[field]=value
                with self.assertRaises(ValueError):plan.prepared_decision_valid(d)
    def test_missing_authority_flag_rejected(self):
        del self.d['execution_authorized']
        with self.assertRaises(ValueError):plan.prepared_decision_valid(self.d)

if __name__=='__main__': unittest.main()

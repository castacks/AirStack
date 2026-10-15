"""Qualification of authored shadow checking ablations, not robot performance."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from rrm.cosmos_reason2 import parse_cosmos_candidate
from rrm.shadow_robustness import (ARMS, _context, canonical, evaluate, run_campaign,
                                   verify_campaign)
from rrm.shadow_robustness_fixtures import specification


class ShadowRobustnessTests(unittest.TestCase):
    def test_every_case_step_and_arm_has_identical_inputs(self):
        corpus=specification()
        original=copy.deepcopy(corpus)
        report=evaluate(corpus)
        self.assertEqual(corpus,original)
        self.assertEqual(len(corpus['cases']),24)
        self.assertEqual(len(report['input_pairing']),27)
        self.assertEqual(len(report['arms']),5)
        for arm in ARMS:
            data=report['arms'][arm.name]
            self.assertEqual(data['audited_steps'],27)
            self.assertEqual(data['audited_cases'],24)
            self.assertEqual([r['input_sha256'] for r in data['rows']],
                             [r[2] for r in report['input_pairing']])
        self.assertFalse(report['performance_claim_authorized'])
        self.assertFalse(report['execution_dispatch'])
        self.assertFalse(report['closed_loop_counterfactual'])

    def test_independent_labels_expose_failures_not_just_wins(self):
        report=evaluate(specification())
        expected={'context_and_effect':(1,0,0,6),'raw_identity_ablation':(5,0,0,6),
                  'status_effect_ablation':(1,7,1,5),'both_ablated':(5,11,1,5),
                  'bound_context_and_effect':(0,0,0,6)}
        for name,(invalid,false_claims,unsupported,true_claims) in expected.items():
            with self.subTest(arm=name):
                data=report['arms'][name]
                for key,numerator,denominator in (
                    ('invalid_proposal_acceptance',invalid,11),
                    ('false_completion_claims',false_claims,15),
                    ('unsupported_completion_claims',unsupported,6),
                    ('true_goal_claim_coverage',true_claims,6)):
                    self.assertEqual(data[key]['numerator'],numerator)
                    self.assertEqual(data[key]['denominator'],denominator)
                self.assertEqual(data['unknown_proposal_labels'],1)
                self.assertEqual(data['observed_effect_unknown'],8)
                self.assertEqual(data['valid_proposal_refusal']['numerator'],0)
                self.assertEqual(data['valid_proposal_refusal']['denominator'],15)

    def test_common_stop_permission_deadline_are_never_ablated(self):
        report=evaluate(specification())
        cases={'permission-denied':'COMMON_PERMISSION_DENIED','stop-active':'COMMON_STOP',
               'late-proposal':'COMMON_DEADLINE'}
        for arm in report['arms'].values():
            for row in arm['rows']:
                if row['case_id'] in cases:
                    self.assertEqual(row['proposal_status'],cases[row['case_id']])
                    self.assertFalse(row['proposal_accepted'])
                    self.assertFalse(row['completion_claim'])
                self.assertFalse(row['execution_dispatch'])

    def test_missing_evidence_and_wrong_valid_target_stay_visible(self):
        report=evaluate(specification())
        for name,data in report['arms'].items():
            missing=next(r for r in data['rows'] if r['case_id']=='missing-proposal')
            self.assertEqual(missing['proposal_status'],'MISSING_PROPOSAL')
            self.assertIsNone(missing['adjudication']['actual_goal'])
            wrong=next(r for r in data['rows'] if r['case_id']=='wrong-valid-target')
            self.assertEqual(wrong['proposal_accepted'],name!='bound_context_and_effect',
                             'unbound intent gap must remain scored, bound variant must reject')
            if name in ('context_and_effect','raw_identity_ablation'):
                self.assertFalse(wrong['completion_claim'],'check task effect, not model goal')

    def test_ablation_does_not_mutate_production_parser(self):
        original=parse_cosmos_candidate.__globals__['_fresh_entities']
        case=next(c for c in specification()['cases'] if c['case_id']=='contradictory-existence')
        step=case['steps'][0]
        self.assertEqual(parse_cosmos_candidate(step['raw_response'],_context(step['context'])).status.value,'REJECTED')
        evaluate(specification())
        self.assertIs(parse_cosmos_candidate.__globals__['_fresh_entities'],original)
        self.assertEqual(parse_cosmos_candidate(step['raw_response'],_context(step['context'])).status.value,'REJECTED')

    def test_corpus_changes_or_case_selection_are_not_comparable(self):
        for mode in ('case','label','backend','budget'):
            corpus=specification()
            if mode=='case':corpus['cases'].pop()
            elif mode=='label':corpus['cases'][0]['steps'][0]['adjudication']['actual_goal']=False
            elif mode=='backend':corpus['identities']['backend']='another'
            else:corpus['budgets']['max_steps_per_case']=9
            with self.assertRaisesRegex(ValueError,'modified_corpus'):
                evaluate(corpus)

    def test_export_reconstruction_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)/'campaign'
            report=run_campaign(output)
            self.assertTrue(verify_campaign(output)['valid'])
            self.assertEqual(json.loads((output/'report.json').read_bytes()),json.loads(canonical(report)))
            with self.assertRaises(FileExistsError):run_campaign(output)

    def test_tampered_and_rehashed_report_or_corpus_are_rejected(self):
        for mode in ('bytes','report','corpus','source','inventory'):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as folder:
                output=Path(folder)/'campaign'
                run_campaign(output)
                manifest=json.loads((output/'manifest.json').read_text())
                if mode=='bytes':
                    (output/'report.json').write_text('{}')
                elif mode=='source':
                    manifest['source']['rrm/shadow_robustness.py']='0'*64
                elif mode=='inventory':
                    (output/'extra.json').write_text('{}')
                else:
                    name='report.json' if mode=='report' else 'corpus.json'
                    value=json.loads((output/name).read_text())
                    if mode=='report':value['arms']['context_and_effect']['accepted_proposals']=0
                    else:value['cases'].pop()
                    (output/name).write_bytes(canonical(value))
                    manifest['artifacts'][name]=hashlib.sha256((output/name).read_bytes()).hexdigest()
                (output/'manifest.json').write_bytes(canonical(manifest))
                with self.assertRaises(ValueError):verify_campaign(output)


if __name__=='__main__':
    unittest.main()

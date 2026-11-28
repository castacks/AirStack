"""Synthetic offline protocol cases; fixtures are not received organizational evidence."""
import importlib.util
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location('intake', Path(__file__).resolve().parents[1] /
                                              'scripts/gov001b_intake_check.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def example(stage=6):
    history = [{'state': state, 'reference': 'EXAMPLE_ONLY'} for state in MODULE.STATES[:stage+1]]
    record = dict(request_id='EXAMPLE_REQUEST', state=history[-1]['state'], decision_ids='D2;D4',
                  prerequisite_ids='CHK06', delivery_authorization_reference='EXAMPLE_AUTHORITY',
                  send_reference='EXAMPLE_SEND', evidence_id='EXAMPLE_EVIDENCE',
                  source_organization_or_owner='EXAMPLE_ORGANIZATION', source_authority_verification='VERIFIED',
                  date_received='2026-11-27', original_document_id='EXAMPLE_DOCUMENT',
                  revision_or_configuration_id='EXAMPLE_REVISION', applicable_environment='EXAMPLE_ENV',
                  subsystem='EXAMPLE_SUBSYSTEM', evidence_classification='SYNTHETIC_TEST_ONLY',
                  artifact_reference='EXAMPLE_CUSTODY', checksum_status='VERIFIED', response_reference='EXAMPLE_REPLY',
                  integrity_sha256='0'*64, artifact_byte_count='1', provenance_reference='EXAMPLE_PROVENANCE',
                  technical_review_reference='EXAMPLE_TECH_REVIEW', reviewer='EXAMPLE_REVIEWER',
                  review_disposition='ACCEPTED_FOR_STATED_CLAIM', acceptance_reference='EXAMPLE_ACCEPTANCE',
                  acceptance_independence_reference='EXAMPLE_INDEPENDENCE', accepted_claim='EXAMPLE_LIMITED_CLAIM',
                  remaining_limitations='SYNTHETIC ONLY, no real claim')
    return record, history


class IntakeTests(unittest.TestCase):
    def test_complete_synthetic_chain_grants_no_authority(self):
        result = MODULE.check_record(*example())
        self.assertFalse(result['operational_authority_conferred'])
        self.assertFalse(result['authenticity_verified_by_checker'])

    def test_cannot_skip_delivery_or_receipt(self):
        record, history = example()
        with self.assertRaises(ValueError):
            MODULE.check_record(record, [history[0], history[-1]])

    def test_unverified_owner_cannot_be_accepted(self):
        record, history = example()
        record['source_authority_verification'] = 'OWNER UNVERIFIED'
        with self.assertRaises(ValueError):
            MODULE.check_record(record, history)

    def test_authentic_but_insufficient_is_not_accepted(self):
        record, history = example()
        record['review_disposition'] = 'INSUFFICIENT_ISOLATION_SCOPE'
        with self.assertRaises(ValueError):
            MODULE.check_record(record, history)

    def test_rejection_after_receipt_needs_no_fictional_provenance_pass(self):
        record, history = example(3)
        record.update(state='REJECTED OR INSUFFICIENT', source_authority_verification='UNVERIFIED',
                      provenance_reference='', review_disposition='REJECTED_OR_INSUFFICIENT',
                      remaining_limitations='Missing delegation')
        for key in ['acceptance_reference', 'accepted_claim', 'acceptance_independence_reference']:
            record[key] = ''
        history.append(dict(state=record['state'], reference='EXAMPLE_REJECTION'))
        self.assertTrue(MODULE.check_record(record, history)['metadata_consistent'])

    def test_rejected_revision_cannot_be_accepted_in_place(self):
        record, history = example()
        history.insert(-1, dict(state='REJECTED OR INSUFFICIENT', reference='EXAMPLE_REJECT'))
        with self.assertRaises(ValueError):
            MODULE.check_record(record, history)

    def test_restricted_receipt_can_replace_undisclosed_original_hash(self):
        record, history = example()
        record.update(checksum_status='VERIFIED_RESTRICTED_RECEIPT', integrity_sha256='',
                      restricted_integrity_reference='EXAMPLE_CUSTODIAN_VERIFICATION')
        self.assertTrue(MODULE.check_record(record, history)['metadata_consistent'])

    def test_restricted_access_alone_is_not_integrity_verification(self):
        record, history = example()
        record.update(checksum_status='VERIFIED_RESTRICTED_RECEIPT', integrity_sha256='')
        with self.assertRaises(ValueError):
            MODULE.check_record(record, history)

    def test_no_independence_no_acceptance(self):
        record, history = example()
        record['acceptance_independence_reference'] = ''
        with self.assertRaises(ValueError):
            MODULE.check_record(record, history)

    def test_boolean_is_not_a_document_identifier(self):
        record, history = example()
        record['original_document_id'] = True
        with self.assertRaises(ValueError):
            MODULE.check_record(record, history)


if __name__ == '__main__':
    unittest.main()

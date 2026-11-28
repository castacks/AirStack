"""Pure metadata checks for the proposed intake protocol; no runtime or delivery APIs."""
from datetime import date
import re

STATES = ('REQUEST PREPARED', 'DELIVERY AUTHORIZED', 'REQUEST SENT', 'RESPONSE RECEIVED',
          'PROVENANCE VERIFIED', 'TECHNICALLY REVIEWED', 'ACCEPTED', 'REJECTED OR INSUFFICIENT')
RECEIPT_FIELDS = ('evidence_id', 'source_organization_or_owner', 'source_authority_verification',
                  'date_received', 'original_document_id', 'revision_or_configuration_id',
                  'applicable_environment', 'subsystem', 'evidence_classification',
                  'artifact_reference', 'checksum_status', 'response_reference')


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def supplied(value):
    return isinstance(value, str) and bool(value.strip())


def check_record(record, history):
    """Validate declared records, not their authenticity or actual authority."""
    require(isinstance(history, list) and bool(history), 'Missing history')
    for event in history:
        require(event.get('state') in STATES and supplied(event.get('reference')), 'Invalid history event')
    sequence = [event['state'] for event in history]
    state = record.get('state')
    require(state == sequence[-1], 'Record/history state mismatch')
    negative = state == STATES[7]
    if negative:
        positive = sequence[:-1]
        require(4 <= len(positive) <= 6, 'Rejection must follow actual receipt/provenance/technical review')
        require(positive == list(STATES[:len(positive)]), 'Skipped or reordered history')
        require(supplied(record.get('reviewer')) and supplied(record.get('technical_review_reference'))
                and record.get('review_disposition') == 'REJECTED_OR_INSUFFICIENT'
                and supplied(record.get('remaining_limitations')), 'Missing rejection reason/reviewer')
        require(not record.get('acceptance_reference') and not record.get('accepted_claim'),
                'Rejected material falsely accepted')
    else:
        require(state in STATES[:7] and sequence == list(STATES[:len(sequence)]),
                'Skipped, duplicated or reordered states')
    stage = len(sequence) - 1 if not negative else len(sequence) - 2
    require(supplied(record.get('request_id')) and supplied(record.get('decision_ids'))
            and supplied(record.get('prerequisite_ids')), 'Missing request trace')
    if stage >= 1:
        require(supplied(record.get('delivery_authorization_reference')), 'Missing delivery authority')
    else:
        require(not record.get('delivery_authorization_reference'), 'Unperformed delivery authorization')
    if stage >= 2:
        require(supplied(record.get('send_reference')), 'Missing send evidence')
    else:
        require(not record.get('send_reference'), 'Unperformed send')
    if stage >= 3:
        require(all(supplied(record.get(field)) for field in RECEIPT_FIELDS), 'Incomplete received metadata')
        date.fromisoformat(record['date_received'])
    else:
        require(all(not record.get(field) for field in RECEIPT_FIELDS), 'Evidence falsely marked received')
        require(not record.get('integrity_sha256') and not record.get('artifact_byte_count'),
                'Artifact identity without receipt')
    if stage >= 4:
        require(record['source_authority_verification'] == 'VERIFIED'
                and supplied(record.get('provenance_reference')) and supplied(record.get('reviewer')),
                'Provenance or authority not verified')
        status = record['checksum_status']
        if status == 'VERIFIED':
            require(isinstance(record.get('integrity_sha256'), str)
                    and re.fullmatch('[0-9a-f]{64}', record['integrity_sha256']) is not None,
                    'Invalid verified digest')
            count = record.get('artifact_byte_count')
            require(isinstance(count, str) and count.isdecimal(), 'Missing byte count')
        elif status == 'VERIFIED_RESTRICTED_RECEIPT':
            require(supplied(record.get('restricted_integrity_reference'))
                    and not record.get('integrity_sha256'), 'Missing restricted integrity receipt')
        else:
            raise ValueError('Integrity unverified')
    else:
        require(not record.get('provenance_reference'), 'Provenance reference before verification')
        require(record.get('source_authority_verification') != 'VERIFIED', 'Authority verification inferred early')
    if stage >= 5:
        require(supplied(record.get('technical_review_reference')) and supplied(record.get('reviewer')),
                'Technical review missing')
    elif not negative:
        require(not record.get('technical_review_reference'), 'Technical review inferred early')
    if not negative and stage >= 5:
        require(record.get('review_disposition') in ('TECHNICALLY_SUFFICIENT', 'ACCEPTED_FOR_STATED_CLAIM'),
                'Technical adequacy absent')
    if state == 'ACCEPTED':
        require(record.get('review_disposition') == 'ACCEPTED_FOR_STATED_CLAIM'
                and supplied(record.get('acceptance_reference'))
                and supplied(record.get('acceptance_independence_reference'))
                and supplied(record.get('accepted_claim'))
                and supplied(record.get('remaining_limitations')), 'Acceptance/independence/claim missing')
    else:
        require(not record.get('acceptance_reference') and not record.get('accepted_claim')
                and not record.get('acceptance_independence_reference'), 'Acceptance inferred too early')
    return {'metadata_consistent': True, 'state': state, 'authenticity_verified_by_checker': False,
            'operational_authority_conferred': False}

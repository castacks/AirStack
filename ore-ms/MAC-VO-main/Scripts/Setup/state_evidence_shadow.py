"""Opt-in advisory metadata only: raw availability intersected with state evidence."""
from Scripts.Setup.shared_arbiter_replay import advisory
from Scripts.Setup.predecessor_state import validate_predecessor

CAMERAS = ('rgb', 'thermal')


def state_evidence_advisory(*, rows, pair, journals, scope_id, clock_id,
                            deadline_ns, root_initializations=None):
    """Recompute evidence on the same received child; never accept a cached pass.

    Malformed global context raises ValueError with no advisory output. Missing
    or invalid per-expert state evidence removes that expert's eligibility.
    Roots must be supplied explicitly; no journal discovery/backfill occurs here.
    Scope and clock labels are caller attestations, not authentication.
    """
    if (not isinstance(scope_id, str) or not scope_id.strip() or
            not isinstance(clock_id, str) or not clock_id.strip() or
            type(deadline_ns) is not int or deadline_ns < 0 or
            not isinstance(rows, list) or type(pair) is not int or
            not 0 <= pair < len(rows)-1 or
            not isinstance(journals, dict) or set(journals) != set(CAMERAS)):
        raise ValueError('valid scope, clock, deadline, target and both journals required')
    roots = {} if root_initializations is None else root_initializations
    if not isinstance(roots, dict) or not set(roots) <= set(CAMERAS):
        raise ValueError('explicit roots must be keyed by camera')
    try:
        events = {}
        receipts = []
        for camera in CAMERAS:
            journal = journals[camera]
            if not isinstance(journal, list):
                raise ValueError('camera journal must be a list')
            previous_sequence, previous_receipt = -1, -1
            for index, item in enumerate(journal):
                sequence, receipt, event = item['sequence'], item['received_ns'], item['event']
                if (item['scope_id'] != scope_id or item['camera'] != camera or
                        item['clock_id'] != clock_id or type(sequence) is not int or
                        sequence <= previous_sequence or type(receipt) is not int or
                        not previous_receipt <= receipt <= deadline_ns or receipt < 0 or
                        type(event['sequence']) is not int or event['sequence'] != index or
                        event['modality'] != camera):
                    raise ValueError('invalid journal namespace, order, identity or deadline')
                previous_sequence, previous_receipt = sequence, receipt
                receipts.append((sequence, receipt))
            events[camera] = [item['event'] for item in journal]
        receipts.sort()
        if (len({sequence for sequence, _ in receipts}) != len(receipts) or
                any(a[1] > b[1] for a, b in zip(receipts, receipts[1:]))):
            raise ValueError('shared controller journal identities or receipts conflict')
        raw = advisory(rows, pair, events)
    except (KeyError, TypeError, IndexError) as error:
        raise ValueError('malformed journal or target context') from error

    eligible, evidence = {}, {}
    for camera in CAMERAS:
        if not raw['availability'][camera]:
            evidence[camera] = {'status': 'NOT_CHECKED', 'reason': 'raw_unavailable',
                                'physical_motion_admitted': False}
            eligible[camera] = False
            continue
        worker_sequence = raw['evidence'][camera]['motion_event_sequence']
        children = [item for item in journals[camera]
                    if item['event']['sequence'] == worker_sequence]
        if len(children) != 1:
            raise ValueError('raw child must bind uniquely to its journal')
        child = children[0]
        root = roots.get(camera) if child['event']['from_idx'] == 0 else None
        result = validate_predecessor(scope_id=scope_id, camera=camera,
            clock_id=clock_id, deadline_ns=deadline_ns, child=child,
            observed_events=journals[camera], root_initialization=root)
        evidence[camera] = result
        eligible[camera] = result['status'] == 'PASS_PREDECESSOR_EVIDENCE'
    action = 'RGB' if eligible['rgb'] else 'THERMAL' if eligible['thermal'] else 'ABSTAIN'
    reason = ('both_evidence_eligible_rgb_preference' if all(eligible.values()) else
              'only_rgb_evidence_eligible' if eligible['rgb'] else
              'only_thermal_evidence_eligible' if eligible['thermal'] else
              'no_evidence_eligible_expert')
    return {'schema_version': 1, 'pair_ordinal': pair, 'scope_id': scope_id,
            'clock_id': clock_id, 'deadline_ns': deadline_ns,
            'action': action, 'reason': reason, 'raw_action': raw['action'],
            'raw_availability': raw['availability'], 'evidence_eligible': eligible,
            'predecessor_evidence': evidence,
            'observed_event_counts': raw['observed_event_counts'],
            'shadow_only': True, 'physical_motion_admitted': False,
            'reference_or_loss_used': False, 'production_defaults_changed': False}

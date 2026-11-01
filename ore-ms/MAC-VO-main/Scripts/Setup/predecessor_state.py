"""Opt-in graph predecessor evidence check; never physical motion admission.

Envelope scope/clock labels and root documentation are caller attestations.
Exact stored pose values bind a snapshot; no interpolation or pose repair.
"""
from Scripts.Setup.arbiter_motion_output import pose_matrix


def _integer(value):
    if type(value) is not int or value < 0:
        raise ValueError('nonnegative exact integer required')
    return value


def _text(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('nonempty identifier required')
    return value


def validate_predecessor(*, scope_id, camera, clock_id, deadline_ns, child,
                         observed_events, root_initialization=None):
    """Check state evidence at a chosen child first-write in a received prefix.

    Envelopes: scope_id, camera, clock_id, sequence (controller journal ordinal),
    received_ns (controller clock), event (original worker event).
    Root evidence uses the same envelope with event.kind=graph_root_initialized,
    frame_idx=0, time_ns, pose_camera_map_xyzw, documented=True, evidence_ref.
    It must be supplied explicitly; no initialization is inferred from a pose.
    It may be an exactly bound journal entry or a separate noncolliding envelope.
    """
    out = {'schema_version': 1, 'status': 'REJECT_PREDECESSOR_EVIDENCE',
           'reason': 'invalid_input', 'scope_id': scope_id, 'camera': camera,
           'physical_motion_admitted': False, 'root_evidence_used': False,
           'source': None}

    def reject(reason):
        out['reason'] = reason
        return out

    try:
        _text(scope_id); _text(clock_id); _integer(deadline_ns)
        if camera not in ('rgb', 'thermal') or not isinstance(observed_events, list):
            raise ValueError('camera and event prefix required')
        previous_sequence, previous_received = -1, -1
        for index, item in enumerate(observed_events):
            if item['scope_id'] != scope_id or item['camera'] != camera:
                return reject('journal_scope_mismatch')
            if item['clock_id'] != clock_id:
                return reject('journal_clock_mismatch')
            sequence = _integer(item['sequence']); received = _integer(item['received_ns'])
            if sequence <= previous_sequence or received < previous_received:
                return reject('invalid_journal_order')
            if received > deadline_ns:
                return reject('journal_after_deadline')
            event = item['event']
            if _integer(event['sequence']) != index:
                return reject('invalid_worker_sequence')
            previous_sequence, previous_received = sequence, received
        if not isinstance(child, dict):
            return reject('missing_child')
        _integer(child['sequence']); _integer(child['received_ns'])
        matches = [e for e in observed_events if e['sequence'] == child['sequence']]
        if len(matches) != 1 or matches[0] != child:
            return reject('child_not_bound_to_prefix')
        motion = child['event']
        if (motion['kind'] != 'motion_result_written' or
                motion['first_write_for_submission'] is not True or
                _integer(motion['write_ordinal_for_submission']) != 1):
            return reject('child_not_first_write')
        if motion['stage'] != 'pair_processing' or motion['original_need_interp'] is not False:
            return reject('unsupported_child_state')
        parent = _integer(motion['from_idx']); target = _integer(motion['frame_idx'])
        first = _integer(motion['time0_ns']); last = _integer(motion['time1_ns'])
        job = _integer(motion['submission_id'])
        if target <= parent or last <= first:
            return reject('invalid_child_interval')
        snapshot = motion['parent_pose_camera_map_xyzw']
        pose_matrix(snapshot)
        submissions = [e for e in observed_events if e['sequence'] < child['sequence']
                       and e['event']['kind'] == 'optimization_submitted'
                       and e['event']['submission_id'] == job]
        fields = ('from_idx', 'frame_idx', 'time0_ns', 'time1_ns')
        for submission in submissions:
            for key in (*fields, 'submission_id'):
                _integer(submission['event'][key])
        if (len(submissions) != 1 or
                any(submissions[0]['event'][k] != motion[k] for k in fields) or
                submissions[0]['event']['stage'] != 'pair_processing'):
            return reject('child_submission_mismatch')
        firsts = [e for e in observed_events if e['event']['kind'] == 'motion_result_written'
                  and e['event']['submission_id'] == job
                  and e['event']['first_write_for_submission'] is True]
        if len(firsts) != 1:
            return reject('ambiguous_child_first_write')
        earlier = [e for e in observed_events if e['sequence'] < child['sequence']]
        health = [e for e in earlier if e['event']['kind'] == 'pair_end'
                  and e['event'].get('new_frame_idx') == parent]
        writes = [e for e in earlier if e['event']['kind'] == 'motion_result_written'
                  and e['event']['frame_idx'] == parent]
        source = {'child_timeline_sequence': child['sequence'],
                  'child_event_sequence': motion['sequence'], 'parent_frame_idx': parent,
                  'parent_time_ns': first, 'parent_health_sequence': None,
                  'parent_write_sequence': None, 'initialization_evidence_ref': None}
        if parent == 0:
            if health or writes:
                return reject('conflicting_root_state')
            if root_initialization is None:
                return reject('missing_root_initialization')
            root = root_initialization; event = root['event']
            bound_roots = [e for e in observed_events if e['sequence'] == root['sequence']]
            if bound_roots and (len(bound_roots) != 1 or bound_roots[0] != root):
                return reject('root_journal_sequence_conflict')
            journal_roots = [e for e in observed_events
                             if e['event']['kind'] == 'graph_root_initialized']
            if journal_roots and (len(journal_roots) != 1 or journal_roots[0] != root):
                return reject('ambiguous_root_initialization')
            if root['scope_id'] != scope_id or root['camera'] != camera:
                return reject('root_scope_mismatch')
            if root['clock_id'] != clock_id:
                return reject('root_clock_mismatch')
            if (_integer(root['sequence']) >= child['sequence'] or
                    _integer(root['received_ns']) > child['received_ns']):
                return reject('root_not_received_before_child')
            if (event['kind'] != 'graph_root_initialized' or
                    event['documented'] is not True):
                return reject('undocumented_root_initialization')
            _text(event['evidence_ref'])
            if type(event['frame_idx']) is not int or event['frame_idx'] != 0 or type(event['time_ns']) is not int or event['time_ns'] != first:
                return reject('root_identity_mismatch')
            pose_matrix(event['pose_camera_map_xyzw'])
            if event['pose_camera_map_xyzw'] != snapshot:
                return reject('root_pose_mismatch')
            source['initialization_evidence_ref'] = event['evidence_ref']
            out.update(root_evidence_used=True, reason='documented_root_initialization')
        else:
            if len(health) != 1:
                return reject('missing_parent_health_at_write' if not health else 'ambiguous_parent_health')
            end = health[0]['event']
            if (type(end['new_frame_idx']) is not int or type(end['time1_ns']) is not int or
                    end['time1_ns'] != first or type(end.get('time0_ns')) is not int or
                    not 0 <= end['time0_ns'] < first):
                return reject('parent_health_identity_mismatch')
            if (type(end.get('original_need_interp')) is not bool or
                    'execution_error' not in end or end.get('stage') != 'pair_processing'):
                return reject('incomplete_parent_health')
            if end['original_need_interp'] or end['execution_error'] is not None:
                return reject('unsupported_parent_health')
            if not writes:
                return reject('missing_parent_write_at_child')
            if any(type(e['event']['time1_ns']) is not int or e['event']['time1_ns'] != first for e in writes):
                return reject('parent_write_identity_mismatch')
            for write in writes:
                for key in ('submission_id', 'from_idx', 'frame_idx', 'time0_ns', 'time1_ns', 'write_ordinal_for_submission'):
                    _integer(write['event'][key])
            initial = [e for e in writes if e['event']['first_write_for_submission'] is True]
            if len(initial) != 1:
                return reject('ambiguous_parent_first_write')
            initial_event = initial[0]['event']; latest = writes[-1]['event']
            if (_integer(initial_event['write_ordinal_for_submission']) != 1 or
                    any(e['event']['stage'] != 'pair_processing' or
                        e['event']['original_need_interp'] is not False for e in writes)):
                return reject('unsupported_parent_write')
            parent_submissions = [e for e in earlier if e['sequence'] < initial[0]['sequence']
                                  and e['event']['kind'] == 'optimization_submitted'
                                  and e['event']['submission_id'] == initial_event['submission_id']]
            for submission in parent_submissions:
                for key in (*fields, 'submission_id'):
                    _integer(submission['event'][key])
            if (len(parent_submissions) != 1 or
                    any(parent_submissions[0]['event'][k] != initial_event[k] for k in fields) or
                    parent_submissions[0]['event']['stage'] != 'pair_processing'):
                return reject('parent_submission_mismatch')
            if [e['event']['write_ordinal_for_submission'] for e in writes] != list(range(1, len(writes)+1)):
                return reject('invalid_parent_write_order')
            identity = ('submission_id', 'from_idx', 'frame_idx', 'time0_ns', 'time1_ns')
            if any(any(e['event'][k] != initial_event[k] for k in identity) for e in writes):
                return reject('conflicting_parent_writes')
            pose_matrix(latest['committed_pose_camera_map_xyzw'])
            if latest['committed_pose_camera_map_xyzw'] != snapshot:
                return reject('parent_pose_mismatch')
            source.update(parent_health_sequence=health[0]['sequence'],
                          parent_write_sequence=writes[-1]['sequence'])
            out['reason'] = 'supported_parent_with_observed_pose'
        out.update(status='PASS_PREDECESSOR_EVIDENCE', source=source)
        return out
    except (KeyError, TypeError, ValueError, OverflowError):
        return reject('invalid_input')

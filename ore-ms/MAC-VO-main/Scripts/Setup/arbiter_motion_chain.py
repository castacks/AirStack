"""Compose a unique exact-interval path from causally received admission outputs.

No interpolation or motion model is introduced. Caller-attested frame evidence
is inherited, never promoted to physical validation. This is not an expert gate.
"""
from collections import Counter, defaultdict
import copy

from Scripts.Setup.arbiter_motion_output import interval, multiply, rigid
from Scripts.Setup.pair_arbiter_events import integer


def compose(entries, *, target_interval_ns, target_frame, sensor_clock_id, controller_clock_id, deadline_ns):
    """Query controller-received admission outputs without modifying them.

    Each entry has id (unique nonempty string), received_ns, clock_id, output,
    sensor_clock_id and sensor_clock_evidence_ref (caller-attested acquisition
    clock mapping). Exact numeric endpoints alone do not establish a shared clock.
    `received_ns` is when the complete admission output became available to
    this controller, not when a worker began computing. Output is from admit().
    Paths must remain wholly within the target interval, use one target frame
    and clock domain, and contain original (non-reference-transported) motions.
    Different paths, even numerically identical ones, are ambiguous. Path count
    is saturated at two. Duplicate eligible IDs are malformed input.
    """
    first, last = interval(target_interval_ns)
    integer(deadline_ns)
    if deadline_ns < 0 or not isinstance(target_frame, str) or not target_frame.strip() or not isinstance(controller_clock_id, str) or not controller_clock_id.strip() or not isinstance(sensor_clock_id, str) or not sensor_clock_id.strip():
        raise ValueError('target frame, sensor/controller clocks and nonnegative deadline required')
    graph = defaultdict(list)
    excluded = Counter()
    seen = set()
    for entry in entries:
        if entry['clock_id'] != controller_clock_id:
            excluded['controller_clock_mismatch'] += 1
            continue
        received = integer(entry['received_ns'])
        if received < 0:
            raise ValueError('nonnegative controller receipt required')
        # Do not inspect a future payload, including its ID or geometry.
        if received > deadline_ns:
            continue
        identity = entry['id']
        if not isinstance(identity, str) or not identity.strip() or identity in seen:
            raise ValueError('unique nonempty received entry ID required')
        seen.add(identity)
        out = entry['output']
        if out['controller_clock_id'] != controller_clock_id:
            excluded['admission_clock_mismatch'] += 1
            continue
        made = integer(out['deadline_ns'])
        if made < 0 or made > received:
            raise ValueError('admission decision cannot follow its controller receipt')
        if out['action'] == 'ABSTAIN':
            if out['delta_target'] is not None:
                raise ValueError('abstention must have null motion')
            excluded['admission_abstained'] += 1
            continue
        if out['action'] not in ('RGB', 'THERMAL'):
            excluded['unsupported_admission_action'] += 1
            continue
        if entry.get('sensor_clock_id') != sensor_clock_id:
            excluded['sensor_clock_mismatch'] += 1
            continue
        clock_evidence = entry.get('sensor_clock_evidence_ref')
        if not isinstance(clock_evidence, str) or not clock_evidence.strip():
            excluded['unverified_sensor_clock'] += 1
            continue
        if out['target_frame'] != target_frame:
            excluded['target_frame_mismatch'] += 1
            continue
        a, b = interval(out['sensor_interval_ns'])
        if a < first or b > last:
            excluded['outside_requested_interval'] += 1
            continue
        if out.get('reference_transport_used') is not False:
            excluded['reference_transport_not_excluded'] += 1
            continue
        source = out.get('source')
        if not isinstance(source, dict) or source.get('camera') != out['action'].lower() or not isinstance(source.get('frame_evidence_ref'), str) or not source['frame_evidence_ref'].strip():
            excluded['missing_source_evidence'] += 1
            continue
        if interval(source['native_interval_ns']) != [a, b]:
            excluded['source_interval_mismatch'] += 1
            continue
        source_receipt = integer(source['received_ns'])
        if not 0 <= source_receipt <= made:
            raise ValueError('source receipt must precede admission decision')
        try:
            matrix = rigid(out['delta_target'])
        except (ValueError, OverflowError):
            excluded['invalid_geometry'] += 1
            continue
        graph[a].append({'end': b, 'id': identity, 'matrix': matrix,
                         'received_ns': received, 'source': source, 'sensor_clock_evidence_ref': clock_evidence})
    # Dynamic programming over a forward DAG avoids enumerating all paths.
    counts = {last: 1}
    for node in sorted(set(graph) | {last}, reverse=True):
        if node != last:
            counts[node] = min(2, sum(counts.get(edge['end'], 0) for edge in graph[node]))
    paths = counts.get(first, 0)
    result = {'schema_version': 1, 'action': 'ABSTAIN', 'delta_target': None,
              'sensor_interval_ns': [first, last], 'target_frame': target_frame,
              'sensor_clock_id': sensor_clock_id,
              'controller_clock_id': controller_clock_id, 'deadline_ns': deadline_ns,
              'reason': 'no_exact_chain' if paths == 0 else 'ambiguous_exact_chains',
              'path_count_capped_at_two': paths, 'sources': [],
              'excluded_received_entries': dict(sorted(excluded.items())),
              'reference_transport_used': False, 'physical_accuracy_validated': False}
    if paths != 1:
        return result
    matrix = [[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]
    node = first
    while node != last:
        edge = next(e for e in graph[node] if counts.get(e['end'], 0))
        matrix = multiply(matrix, edge['matrix'])
        result['sources'].append({'entry_id': edge['id'], 'sensor_interval_ns': [node, edge['end']],
                                  'received_ns': edge['received_ns'], 'sensor_clock_evidence_ref': edge['sensor_clock_evidence_ref'],
                                  'admission_source': copy.deepcopy(edge['source'])})
        node = edge['end']
    try:
        rigid(matrix)
    except (ValueError, OverflowError):
        result.update(reason='invalid_composed_geometry', sources=[])
        return result
    result.update(action='COMPOSED', delta_target=matrix, reason='unique_exact_chain')
    return result

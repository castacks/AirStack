"""Pure source-row mapping for fixed, open-loop native gap engineering replays."""
import copy


def select_frames(rows, decisions, camera):
    if camera not in ('rgb', 'thermal') or len(rows) < 4:
        raise ValueError('expected known camera and guarded window')
    if [d['pair'] for d in decisions] != list(range(len(rows)-1)):
        raise ValueError('request schedule must contain every ordered source pair exactly once')
    for d in decisions:
        requests = d['requests']
        if len(requests) != len(set(requests)) or not set(requests) <= {'rgb', 'thermal'}:
            raise ValueError('invalid or duplicate camera request')
    selected = [0] + [d['pair']+1 for d in decisions if camera in d['requests']]
    if not {0, 1, 2, len(rows)-2, len(rows)-1} <= set(selected):
        raise ValueError('initial, bootstrap and trailing guard frames must remain requested')
    times = [r[camera+'_left']['time_ns'] for r in rows]
    if any(type(t) is not int for t in times) or any(b <= a for a,b in zip(times,times[1:])):
        raise ValueError('source acquisition times must strictly increase')
    return selected


def mapped_manifest(original, source_indices, camera):
    rows = original['pairs']
    if (not source_indices or source_indices[0] != 0 or
        any(type(i) is not int or not 0 <= i < len(rows) for i in source_indices) or
        any(b <= a for a,b in zip(source_indices,source_indices[1:]))):
        raise ValueError('source indices must begin at zero and strictly increase within manifest')
    result = copy.deepcopy(original)
    result['pairs'] = [copy.deepcopy(rows[i]) for i in source_indices]
    intervals = [{'native_pair': k, 'source_start': a, 'source_end': b,
                  'skipped_source_frames': b-a-1, 'is_adjacent_source_pair': b-a == 1,
                  'sensor_interval_ns': [rows[i][camera+'_left']['time_ns'] for i in (a,b)]}
                 for k,(a,b) in enumerate(zip(source_indices,source_indices[1:]))]
    mapping = {'camera': camera, 'source_indices': list(source_indices), 'intervals': intervals,
               'source_frame_count':len(rows), 'native_frame_count':len(source_indices),
               'schedule_scope':'fixed dense-derived open-loop; not sparse-adaptive controller'}
    return result, mapping

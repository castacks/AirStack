"""Inactive phase-preserving encounter compactor for a future protocol version.

This pure projection reads no files and changes no observations. Caller-supplied
source hashes must come from the saved-run verifier. It never truncates later
encounter phases to accommodate earlier verbose objects.
"""
import copy
import json
import math
import re

VERSION = 'phase_table_encounter_context_v1'
DEFAULT_BYTE_BUDGET = 24_000
COUNT_GROUPS = ('noise', 'delay', 'worker_confirmed_noise', 'worker_confirmed_delay')
STATES = ('active', 'inactive', 'unknown')
COLUMNS = ('entity', 'phase', 'clock', 'observation_status', 'frame_count',
           'worker_confirmed_frames', 'worker_confirmation_unknown_frames',
           'first_sample_sim_s', 'last_sample_sim_s',
           *(group+'_'+state for group in COUNT_GROUPS for state in STATES),
           'first_sample_since_schedule_start_s', 'last_sample_since_schedule_start_s')
LIMITATIONS = [
    'Table rows preserve sampled first/last observations and counts, not continuous intervals or exposure durations.',
    'capture is the source-frame capture clock; bridge_available is first observed bridge availability, not exact worker consumption.',
    'Worker-confirmed counts are the exact-joined subset of bridge-observed frames; consumed-only frames outside this population are separate.',
    'A not_recorded phase has no original bucket; its values remain null, not invented zero exposure.',
    'Schedule-relative times are simple subtraction of a caller-verified schedule anchor, never takeoff or an inferred clock.',
    'Geometric phases, short-gap GT interpolation and sampled exposure are diagnostic evidence, not camera visibility or attack causality.',
]


def encoded_bytes(value):
    return len(json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode())


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _number(raw, path, invalid, integer=False):
    if raw is None: return None
    if (integer and (type(raw) is not int or raw < 0)) or (not integer and not _finite(raw)):
        invalid.append(path); return None
    return raw


def _scalars(value, path, invalid):
    if not isinstance(value, dict): return {}
    result = {}
    for key, item in sorted(value.items()):
        if not isinstance(key, str) or len(key) > 128:
            raise ValueError('Encounter metadata key exceeds bounded scalar schema')
        if item is None or isinstance(item, bool): result[key] = item
        elif isinstance(item, str) and len(item) <= 500: result[key] = item
        elif _finite(item): result[key] = item
        else: invalid.append(path+'.'+key)
    return result


def _bucket(entity, phase, clock, raw, source_path, anchor, invalid):
    if raw is None:
        return [entity, phase, clock, 'not_recorded', *([None]*(len(COLUMNS)-4))]
    if not isinstance(raw, dict):
        invalid.append(source_path)
        return [entity, phase, clock, 'invalid_source_bucket', *([None]*(len(COLUMNS)-4))]
    before = len(invalid)
    counts = [_number(raw.get(key), source_path+'.'+key, invalid, integer=True)
              for key in ('frame_count', 'worker_confirmed_frames', 'worker_confirmation_unknown_frames')]
    times = [_number(raw.get(key), source_path+'.'+key, invalid)
             for key in ('first_sample_sim_s', 'last_sample_sim_s')]
    if all(value is not None for value in times) and times[1] < times[0]:
        invalid.append(source_path+'.reversed_sample_times'); times = [None, None]
    groups = []
    for name in COUNT_GROUPS:
        values = raw.get(name)
        for state in STATES:
            groups.append(_number(values.get(state) if isinstance(values, dict) else None,
                                  source_path+'.'+name+'.'+state, invalid, integer=True))
    relative = [stamp-anchor if stamp is not None and anchor is not None else None for stamp in times]
    return [entity, phase, clock, 'recorded' if len(invalid) == before else 'recorded_with_invalid_fields',
            *counts, *times, *groups, *relative]


def compact_encounter(analysis, *, evidence_id, source, schedule_start_sim_s=None,
                      byte_budget=DEFAULT_BYTE_BUDGET):
    """Keep every entity/phase/clock row or reject an insufficient byte budget.

    source requires verified scenario identity plus configuration/trace/geometry
    hashes. Missing verification returns unavailable; hashes are not fabricated.
    Missing source buckets stay explicitly unknown. No source object is mutated.
    """
    if type(byte_budget) is not int or byte_budget <= 0: raise ValueError('byte_budget must be a positive integer')
    if not isinstance(evidence_id, str) or not re.fullmatch(r'r[0-9]{3,}(?:_p[0-9]{2,})?_(?:clean|perturbed)', evidence_id):
        raise ValueError('Use a recorded evidence ID, not a path')
    if schedule_start_sim_s is not None and (not _finite(schedule_start_sim_s) or schedule_start_sim_s < 0):
        raise ValueError('Schedule anchor must be finite, nonnegative, and verified by the caller')
    base = {'schema': VERSION, 'evidence_id': evidence_id, 'status': 'unavailable',
            'source': {}, 'limitations': LIMITATIONS}
    valid_source = isinstance(source, dict) and source.get('scenario_hash_verified') is True
    hashes = ('configuration_hash', 'trace_sha256', 'geometry_sha256')
    valid_source = valid_source and all(isinstance(source.get(k), str) and re.fullmatch('[0-9a-f]{64}', source[k]) for k in hashes)
    if not valid_source:
        base['reason'] = 'Missing verified scenario identity or saved trace/geometry hashes'
    elif not isinstance(analysis, dict) or analysis.get('status') != 'complete':
        base['source'] = {key: source[key] for key in hashes} | {'scenario_hash_verified': True}
        base['reason'] = analysis.get('status', 'missing_analysis_status') if isinstance(analysis, dict) else 'missing_analysis'
    else:
        return _complete(analysis, evidence_id, source, schedule_start_sim_s, byte_budget)
    if encoded_bytes(base) > byte_budget: raise ValueError('Encounter unavailable record exceeds byte budget; nothing silently omitted')
    return base


def _complete(analysis, evidence_id, source, anchor, budget):
    invalid = []; entities = []; rows = []
    rows.append(_bucket('overall', 'all_observed_frames', 'capture', analysis.get('overall'),
                        'overall', anchor, invalid))
    for kind, key, phases in (('turn', 'turns', ('approach', 'turn', 'departing')),
                              ('obstacle', 'obstacles', ('approach', 'passing', 'departing'))):
        values = analysis.get(key, [])
        if not isinstance(values, list) or len(values) > 64:
            raise ValueError('Encounter entities exceed the bounded input schema')
        seen = set()
        for value in values:
            index = value.get('index') if isinstance(value, dict) else None
            if type(index) is not int or index < 0 or index in seen:
                raise ValueError('Each encounter needs a unique nonnegative integer index')
            seen.add(index); identity = f'{kind}:{index}'; path = f'{key}[{index}]'
            entity = {'entity': identity}
            fields = ('route_start_m', 'route_end_m', 'signed_heading_change_deg', 'direction', 'vertex_count') if kind == 'turn' else ('route_s_m',)
            entity.update(_scalars({k: value[k] for k in fields if k in value}, path, invalid))
            if kind == 'obstacle':
                geometry = {}
                for name in ('center_m', 'size_m'):
                    vector = value.get(name)
                    if isinstance(vector, list) and len(vector) == 3 and all(_finite(x) for x in vector): geometry[name] = copy.deepcopy(vector)
                    elif vector is not None: invalid.append(path+'.'+name)
                if _finite(value.get('yaw_rad')): geometry['yaw_rad'] = value['yaw_rad']
                if geometry: entity['geometry_detail'] = geometry
            entities.append(entity)
            for source_clock, clock in (('at_capture', 'capture'), ('when_available', 'bridge_available')):
                grouped = value.get(source_clock)
                if grouped is not None and not isinstance(grouped, dict): invalid.append(path+'.'+source_clock)
                for phase in phases:
                    raw = grouped.get(phase) if isinstance(grouped, dict) else None
                    rows.append(_bucket(identity, phase, clock, raw, path+'.'+source_clock+'.'+phase, anchor, invalid))
                if isinstance(grouped, dict) and set(grouped)-set(phases):
                    raise ValueError('Unknown encounter phases cannot be silently dropped')
    value = {'schema': VERSION, 'evidence_id': evidence_id, 'status': 'complete',
             'source': {key: source[key] for key in ('configuration_hash', 'trace_sha256', 'geometry_sha256')} |
                       {'scenario_hash_verified': True},
             'source_analysis_schema': analysis.get('schema'),
             'schedule_start_sim_s': anchor,
             'schedule_anchor_provenance': 'Caller-verified source schedule clock' if anchor is not None else 'unavailable; no relative times inferred',
             'thresholds': _scalars(analysis.get('thresholds'), 'thresholds', invalid),
             'gaps': _scalars(analysis.get('gaps'), 'gaps', invalid),
             'issues': _scalars(analysis.get('issues'), 'issues', invalid),
             'entities': entities, 'columns': list(COLUMNS), 'rows': rows,
             'invalid_source_fields': invalid, 'limitations': LIMITATIONS,
             'retention': {'byte_budget': budget, 'all_entity_phase_clock_rows_retained': True,
                           'no_observation_counts_or_times_aggregated_or_truncated': True,
                           'full_original_analysis_preserved': True, 'geometry_detail_omitted': False}}
    if encoded_bytes(value) <= budget: return value
    for entity in entities: entity.pop('geometry_detail', None)
    value['retention']['geometry_detail_omitted'] = True
    if encoded_bytes(value) <= budget: return value
    raise ValueError(f'Encounter core requires {encoded_bytes(value)} bytes, exceeding byte_budget={budget}; '
                     'all phase counts, times, source references and limits must remain. Increase budget or stop explicitly.')

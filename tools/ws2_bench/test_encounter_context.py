import copy

import pytest

from encounter_context import compact_encounter, encoded_bytes, COLUMNS


SOURCE = {'configuration_hash': 'a'*64, 'trace_sha256': 'b'*64,
          'geometry_sha256': 'c'*64, 'scenario_hash_verified': True}


def bucket(first=10., active=2):
    return {'frame_count': 5, 'worker_confirmed_frames': 4, 'worker_confirmation_unknown_frames': 1,
            'first_sample_sim_s': first, 'last_sample_sim_s': first+.4,
            'noise': {'active': active, 'inactive': 5-active, 'unknown': 0},
            'delay': {'active': 0, 'inactive': 5, 'unknown': 0},
            'worker_confirmed_noise': {'active': min(active, 1), 'inactive': 4-min(active, 1), 'unknown': 0},
            'worker_confirmed_delay': {'active': 0, 'inactive': 4, 'unknown': 0}}


def analysis(obstacles=2):
    values = {'schema': 'encounter_evidence_v2', 'status': 'complete',
              'thresholds': {'turn_zone_padding_m': .5, 'max_pose_gap_s': .5},
              'gaps': {'maximum_gt_gap_s': .12}, 'issues': {}, 'overall': bucket(),
              'turns': [], 'obstacles': []}
    for kind, phases, count in (('turns', ('approach', 'turn', 'departing'), 2),
                                 ('obstacles', ('approach', 'passing', 'departing'), obstacles)):
        for index in range(count):
            value = {'index': index, 'route_s_m': float(index), 'route_start_m': float(index),
                     'route_end_m': index+1., 'direction': 'left', 'signed_heading_change_deg': 90.,
                     'center_m': [1., 2., 3.], 'size_m': [.5, .6, 2.4], 'yaw_rad': .1}
            for clock in ('at_capture', 'when_available'):
                value[clock] = {phase: bucket(10.+index, active=number) for number, phase in enumerate(phases)}
            values[kind].append(value)
    return values


def row_dicts(value):
    return [dict(zip(value['columns'], row)) for row in value['rows']]


def test_every_late_turn_and_obstacle_phase_survives_without_reducing_counts():
    original = analysis(12); before = copy.deepcopy(original)
    value = compact_encounter(original, evidence_id='r001_perturbed', source=SOURCE, schedule_start_sim_s=2.)
    assert len(value['rows']) == 1+(12+2)*6
    assert encoded_bytes(value) <= 24000
    assert value['source'] == SOURCE
    rows = row_dicts(value)
    for entity in value['entities']:
        kind, index = entity['entity'].split(':'); index = int(index)
        source = original['turns' if kind == 'turn' else 'obstacles'][index]
        for row in [r for r in rows if r['entity'] == entity['entity']]:
            clock = 'at_capture' if row['clock'] == 'capture' else 'when_available'
            raw = source[clock][row['phase']]
            assert row['noise_active'] == raw['noise']['active']
            assert row['worker_confirmed_noise_active'] == raw['worker_confirmed_noise']['active']
            assert row['first_sample_sim_s'] == raw['first_sample_sim_s']
            assert row['first_sample_since_schedule_start_s'] == raw['first_sample_sim_s']-2.
    assert original == before
    assert all('duration' not in key for key in COLUMNS)


def test_missing_phase_is_not_zero_and_invalid_values_are_explicit():
    raw = analysis(0)
    del raw['turns'][1]['when_available']['departing']
    raw['turns'][1]['at_capture']['turn']['noise']['active'] = float('nan')
    value = compact_encounter(raw, evidence_id='r002_clean', source=SOURCE)
    missing = next(r for r in row_dicts(value) if r['entity'] == 'turn:1' and r['clock'] == 'bridge_available' and r['phase'] == 'departing')
    assert missing['observation_status'] == 'not_recorded'
    assert missing['frame_count'] is None and missing['noise_active'] is None
    invalid = next(r for r in row_dicts(value) if r['entity'] == 'turn:1' and r['clock'] == 'capture' and r['phase'] == 'turn')
    assert invalid['noise_active'] is None and invalid['observation_status'] == 'recorded_with_invalid_fields'
    assert 'turns[1].at_capture.turn.noise.active' in value['invalid_source_fields']
    assert all(r['first_sample_since_schedule_start_s'] is None for r in row_dicts(value))
    assert encoded_bytes(value) > 0


def test_budget_discards_only_redundant_geometry_then_fails_explicitly():
    raw = analysis(8)
    full = compact_encounter(raw, evidence_id='r001_perturbed', source=SOURCE)
    limited = compact_encounter(raw, evidence_id='r001_perturbed', source=SOURCE, byte_budget=encoded_bytes(full)-100)
    assert limited['retention']['geometry_detail_omitted'] is True
    assert limited['rows'] == full['rows'] and limited['thresholds'] == full['thresholds']
    assert all('geometry_detail' not in x for x in limited['entities'])
    assert encoded_bytes(limited) <= encoded_bytes(full)-100
    with pytest.raises(ValueError, match='all phase counts, times, source references'):
        compact_encounter(raw, evidence_id='r001_perturbed', source=SOURCE, byte_budget=1000)


def test_missing_verified_source_and_unavailable_analysis_never_fabricate_phase_data():
    unavailable = compact_encounter(analysis(), evidence_id='r001_clean', source={})
    assert unavailable['status'] == 'unavailable' and 'rows' not in unavailable
    unavailable = compact_encounter({'status': 'unavailable_clock_regression'}, evidence_id='r001_clean', source=SOURCE)
    assert unavailable['reason'] == 'unavailable_clock_regression' and 'rows' not in unavailable
    for anchor in (float('nan'), float('inf'), -1., True):
        with pytest.raises(ValueError):
            compact_encounter(analysis(), evidence_id='r001_clean', source=SOURCE, schedule_start_sim_s=anchor)
    with pytest.raises(ValueError): compact_encounter(analysis(), evidence_id='/etc/passwd', source=SOURCE)

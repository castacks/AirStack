import copy

import pytest

from agent_context import _present
from encounter_context import VERSION, encoded_bytes
from encounter_context_versioned import (summarize, verified_schedule_anchor, LEGACY_FIELDS,
                                         SCHEDULE_CLOCK, detail_levels)
from test_encounter_context import SOURCE, analysis, row_dicts


def observation(anchor=2., sequence=10, stamp=12.):
    return {'worker': {'run_id': 'known-run', 'sequence': sequence, 'sim_stamp': stamp,
                       'consumed_input': {'schema_version': 1, 'sequence': sequence,
                         'sim_stamp': stamp, 'disturbance_revision': 1,
                         'sensor_disturbance': {'revision': 1, 'schedule_clock': SCHEDULE_CLOCK,
                                               'schedule_start_sim_s': anchor}}}}


def test_missing_version_preserves_historical_projection_including_its_limits():
    raw = analysis(12)
    value = summarize(raw, evidence_id='r001_clean', source=SOURCE)
    assert value == _present(raw, LEGACY_FIELDS)
    assert 'verified_schedule_anchor' not in value
    assert detail_levels(None) == ((4, 4, 20), (2, 2, 10), (0, 0, 4))
    assert detail_levels(VERSION) == ((2, 2, 20), (1, 1, 10), (0, 0, 4))


def test_explicit_new_version_preserves_all_phases_and_verified_clock():
    raw = analysis(12); original = copy.deepcopy(raw)
    rows = [observation(None), observation(), observation(sequence=11, stamp=13.)]
    value = summarize(raw, evidence_id='r001_perturbed', source=SOURCE,
                      version=VERSION, rows=rows, run_id='known-run')
    assert value['status'] == 'complete' and len(value['rows']) == 85
    assert value['verified_schedule_anchor']['status'] == 'verified'
    assert value['verified_schedule_anchor']['unique_source_frame_count'] == 2
    assert value['schedule_start_sim_s'] == 2.
    for row in row_dicts(value):
        assert row['first_sample_since_schedule_start_s'] == row['first_sample_sim_s']-2.
    assert raw == original and encoded_bytes(value) <= 24000


@pytest.mark.parametrize('kind', ['identity', 'clock', 'bool_schema', 'nan', 'conflict', 'revision'])
def test_bad_or_conflicting_direct_metadata_never_invents_relative_times(kind):
    bad = observation(sequence=11, stamp=13.)
    consumed = bad['worker']['consumed_input']
    if kind == 'identity': consumed['sequence'] = 12
    elif kind == 'clock': consumed['sensor_disturbance']['schedule_clock'] = 'wall_seconds'
    elif kind == 'bool_schema': consumed['schema_version'] = True
    elif kind == 'nan': consumed['sensor_disturbance']['schedule_start_sim_s'] = float('nan')
    elif kind == 'revision': consumed['disturbance_revision'] = True
    else: consumed['sensor_disturbance']['schedule_start_sim_s'] = 3.
    value = summarize(analysis(1), evidence_id='r001_clean', source=SOURCE,
                      version=VERSION, rows=[observation(), bad], run_id='known-run')
    assert value['schedule_start_sim_s'] is None
    assert value['verified_schedule_anchor']['status'] == 'unavailable'
    assert all(row['first_sample_since_schedule_start_s'] is None for row in row_dicts(value))


def test_foreign_or_missing_frames_do_not_supply_a_schedule_anchor():
    foreign = observation(); foreign['worker']['run_id'] = 'different-run'
    assert verified_schedule_anchor([foreign], 'known-run')['schedule_start_sim_s'] is None
    value = summarize(analysis(), evidence_id='r001_clean', source=SOURCE, version=VERSION)
    assert value['verified_schedule_anchor']['status'] == 'unavailable'
    assert len(value['rows']) == 25  # Phase counts survive even without a clock anchor.


def test_unknown_version_is_rejected_and_budget_failure_is_explicit():
    for version in ('unknown', True, 1):
        with pytest.raises(ValueError, match='Unsupported encounter_context_version'):
            summarize(analysis(), evidence_id='r001_clean', source=SOURCE, version=version)
    with pytest.raises(ValueError, match='byte budget|byte_budget'):
        summarize(analysis(12), evidence_id='r001_clean', source=SOURCE, version=VERSION,
                  rows=[observation()], run_id='known-run', byte_budget=3000)

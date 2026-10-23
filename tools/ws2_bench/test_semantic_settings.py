import copy
import json

import pytest

from agent_analysis import evidence_for_report
from corridor_space import VERSION, action_key, reference_action
from episode import fingerprint
from semantic_settings import canonical_settings, settings_key
from test_scheduled_report import entry, report


def obstacle(integer=True):
    return {'route_s': 6 if integer else 6., 'lateral_m': 1.35, 'width_m': .6,
            'depth_m': .5, 'height_m': 2 if integer else 2., 'yaw_deg': 0 if integer else 0.}


def test_nested_numeric_keys_preserve_booleans_precision_and_original_bytes():
    source = {'list': [6., 2., -0., True, False, None, '6', .6, {'x': 1.0000000000000002}]}
    before = json.dumps(source); original_hash = fingerprint(source)
    normalized = canonical_settings(source)
    assert normalized['list'][:3] == [6, 2, 0]
    assert all(type(x) is int for x in normalized['list'][:3])
    assert normalized['list'][3] is True and normalized['list'][4] is False
    assert settings_key(True) != settings_key(1)
    assert settings_key('6') != settings_key(6)
    assert settings_key({'x': 1.0000000000000002}) != settings_key({'x': 1.})
    assert json.dumps(source) == before and fingerprint(source) == original_hash
    assert fingerprint(source) != fingerprint(normalized)  # Raw provenance deliberately remains distinct.


@pytest.mark.parametrize('number', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_nested_setting_is_rejected_with_source_field(number):
    with pytest.raises(ValueError, match=r'Nonfinite numeric setting at \$\.obstacles\[0\]\.route_s'):
        settings_key({'obstacles': [{'route_s': number}]})


def test_same_physical_action_deduplicates_without_mutating_saved_actions():
    a = reference_action(VERSION)
    a.update(shape='curve', obstacles=[obstacle()], rgb_noise_stddev=24.,
             noise_schedule={'start_s': 12, 'duration_s': 24, 'period_s': 0})
    b = copy.deepcopy(a)
    b['obstacles'] = [obstacle(False)]
    b['noise_schedule'] = {'start_s': 12., 'duration_s': 24., 'period_s': 0.}
    before = [json.dumps(v) for v in (a, b)]
    assert action_key(a) == action_key(b)
    assert len({action_key(a), action_key(b)}) == 1
    assert [json.dumps(v) for v in (a, b)] == before
    assert fingerprint(a) != fingerprint(b)
    changed = copy.deepcopy(b); changed['obstacles'][0]['route_s'] = 6.1
    assert action_key(changed) != action_key(a)
    changed = copy.deepcopy(b); changed['noise_schedule']['start_s'] = 13.
    assert action_key(changed) != action_key(a)


def test_report_groups_numeric_spellings_as_one_environment_and_repeated_condition():
    history = [entry(1), entry(2)]
    for index, row in enumerate(history):
        row['decision']['condition']['corridor'].update(obstacles=1,
            obstacle_placements=[obstacle(index == 0)])
    before = json.dumps(history)
    result = report(history)
    assert len(result['findings']) == 1
    assert result['findings'][0]['status'] == 'reproduced_condition'
    assert result['findings'][0]['failures'] == 2
    design = evidence_for_report(result)['experimental_design']
    assert len(design['distinct_shared_environments']) == 1
    assert design['distinct_shared_environments'][0]['round_ids'] == ['round_01', 'round_02']
    assert len(design['condition_repetitions']) == 1
    assert design['condition_repetitions'][0]['pair_count'] == 2
    assert json.dumps(history) == before
    # A genuine position change must still create a new environment/condition.
    changed = copy.deepcopy(history)
    changed[1]['decision']['condition']['corridor']['obstacle_placements'][0]['route_s'] = 6.1
    updated = report(changed)
    assert len(updated['findings']) == 2
    assert len(evidence_for_report(updated)['experimental_design']['distinct_shared_environments']) == 2

"""The added extension, RGB comparator and per-record stop cannot be bypassed."""
import pytest
import unweighted_longer as run


def values(candidate, rgb, thermal):
    return {'candidate': candidate, 'native_rgb': rgb, 'native_thermal': thermal}


def test_beating_thermal_without_beating_rgb_fails():
    result = run.gate(values({19: 2.}, {19: 1.}, {19: 3.}), [19])
    assert not result['pass'] and result['checks']['improves_native_thermal_mean']


def test_old_prefix_success_does_not_rescue_extension():
    data = values({0: .1, 19: 4.}, {0: 3., 19: 2.}, {0: 3., 19: 3.})
    result = run.gate(data, [19])
    assert not result['pass'] and result['matched_pair_ids'] == [19]
    assert result['rotation_deg']['candidate']['mean'] == 4.


def test_covariance_free_candidate_cannot_trade_coverage():
    result = run.gate(values({19: .1}, {19: 1., 20: 1.}, {19: 1.}), [19, 20])
    assert not result['pass'] and not result['checks']['no_RGB_coverage_loss']


def test_empty_population_fails():
    assert not run.gate(values({}, {}, {}), list(range(19, 99)))['pass']


def test_both_rotation_metrics_required():
    # Lower mean with one larger tail can still worsen RMSE.
    result = run.gate(values({19: 0., 20: 1.9}, {19: 1., 20: 1.}, {19: 1., 20: 1.}), [19, 20])
    assert result['checks']['improves_native_rgb_mean'] and not result['pass']


@pytest.mark.parametrize('failed', ['day', 'night', 'rain'])
def test_no_recording_can_be_pooled_away(failed):
    recordings = {c: {'primary_extension': {'pass': c != failed}} for c in run.CASES}
    assert run.overall(recordings)['decision'] == 'CLOSE_UNWEIGHTED_ROUTE_NO_RETUNING'


def test_all_three_passes_required_for_full_motion_design():
    recordings = {c: {'primary_extension': {'pass': True}} for c in run.CASES}
    assert run.overall(recordings)['decision'] == 'ADVANCE_TO_SEPARATE_FULL_MOTION_DESIGN'

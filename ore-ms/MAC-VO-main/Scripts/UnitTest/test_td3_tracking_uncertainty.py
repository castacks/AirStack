import numpy as np
import pytest

from Scripts.Setup.td3_tracking_uncertainty import auc, population, retained_mask, summarize_pair, rank_association, postprocessed_flags


def test_empty_retention_and_exact_inbound_join():
    assert retained_mask(np.array([], bool), np.array([], bool)).size == 0
    np.testing.assert_array_equal(retained_mask([True, False, True], [False, True]), [False, False, True])
    with pytest.raises(ValueError):
        retained_mask([True, False], [True, False])


def test_empty_population_is_explicit_and_partitions_are_empty():
    result = population(np.empty((0, 2)), np.empty((0, 3)), np.array([]), np.array([]), np.array([], bool), 90)
    assert result['count'] == 0 and result['raw_trace_px2']['p10_p50_p90'] is None
    assert result['grid_quality_below_native_cap_fraction'] is None
    assert sum(v['count'] for v in result['image_thirds'].values()) == 0
    assert sum(v['count'] for v in result['source_forward_depth_bins_m'].values()) == 0


def test_strata_nan_and_floor_cap_boundaries():
    uv = np.column_stack([np.ones(6), [0, 29, 30, 59, 60, 89]])
    cov = np.array([[.01, .0625, 0], [50, 50, 0], [1, 1, 0], [np.nan, 1, 0], [1, 1, 0], [1, 1, 0]])
    depth = np.array([0, 10, 20, 50, 100, np.nan])
    result = population(uv, cov, depth, depth, np.array([1, 1, 0, 1, 0, 0], bool), 90)
    assert [g['count'] for g in result['image_thirds'].values()] == [2, 2, 2]
    assert sum(g['count'] for g in result['source_forward_depth_bins_m'].values()) == 6
    assert result['source_forward_depth_bins_m']['invalid']['count'] == 2
    assert result['raw_trace_px2']['finite_count'] == 5
    assert result['grid_quality_below_native_cap_fraction'] == pytest.approx(4/5)
    assert result['variance_components_below_floor_fraction'] == pytest.approx(1/11)


def test_auc_ties_single_class_and_nonfinite_scores():
    assert auc([0, 1, 1], [False, True, False]) == pytest.approx(.75)
    assert auc([1, 1], [False, True]) == .5
    assert auc([0, 2, np.nan], [False, True, True]) == 1
    assert auc([1, 2], [False, False]) is None
    assert rank_association([1, 1, 1], [1, 2, 3]) is None
    assert rank_association([1, 2, 3], [3, 2, 1]) == pytest.approx(-1)


def test_pair_counts_keep_failed_empty_selection_visible():
    data = {'height_width': np.array([90, 90]), 'time0_ns': np.array(10),
            'outlier_keep_inbound': np.array([], bool)}
    for prefix, count in [('grid', 3), ('selected', 0)]:
        data.update({prefix+'_uv0': np.ones((count, 2)), prefix+'_raw_flow_cov': np.ones((count, 3)),
                     prefix+'_depth0': np.ones(count), prefix+'_depth1': np.ones(count),
                     prefix+'_inbound': np.ones(count, bool)})
    result = summarize_pair(data)
    assert result['counts'] == {'grid': 3, 'selected': 0, 'inbound': 0, 'retained': 0}
    assert result['tracking_loss'] and result['retained']['raw_trace_px2']['p10_p50_p90'] is None


def test_native_boundary_flags_do_not_erase_raw_loss_labels():
    loss = np.ones(19, bool)
    flags = postprocessed_flags(loss)
    assert loss.sum() == 19 and flags.sum() == 15
    np.testing.assert_array_equal(flags[:2], [False, False])
    np.testing.assert_array_equal(flags[-2:], [False, False])
    assert flags[2:-2].all()

"""CoverageTracker — the polygon completion gate (deviation 4)."""
import numpy as np
import pytest

from raven_nav.coverage import CoverageTracker, polygon_area_xy

SQUARE = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])


def test_polygon_area():
    assert polygon_area_xy(SQUARE) == pytest.approx(100.0)
    assert polygon_area_xy(None) == 0.0
    assert polygon_area_xy(np.array([[0.0, 0.0], [1.0, 1.0]])) == 0.0


def test_stamping_points_marks_cells():
    t = CoverageTracker(cell_size_m=1.0)
    t.stamp_points(np.array([[0.5, 0.5], [0.6, 0.6], [3.2, 4.7]]))
    assert t.cells == {(0, 0), (3, 4)}


def test_cells_set_is_the_live_internal_set_not_a_copy():
    """FrontierBehavior's anti-revisit term reads this every tick, so it is
    deliberately an alias — the node documents the read-only contract."""
    t = CoverageTracker(cell_size_m=1.0)
    t.stamp_points(np.array([[0.5, 0.5]]))
    s = t.cells_set
    assert s == {(0, 0)}
    assert s is t.cells                     # no per-tick copy
    t.stamp_points(np.array([[3.2, 4.7]]))
    assert s == {(0, 0), (3, 4)}            # the alias sees later stamps


def test_cells_set_is_empty_before_anything_is_seen():
    assert CoverageTracker().cells_set == set()


def test_cell_centers_are_cell_middles():
    t = CoverageTracker(cell_size_m=2.0)
    t.stamp_points(np.array([[0.5, 0.5]]))
    assert np.allclose(t.cell_centers_xy(), [[1.0, 1.0]])


def test_coverage_fraction_of_a_known_square():
    t = CoverageTracker(cell_size_m=1.0)
    xs, ys = np.meshgrid(np.arange(0.5, 5.5), np.arange(0.5, 10.5))
    t.stamp_points(np.stack([xs.ravel(), ys.ravel()], axis=1))
    assert t.coverage_fraction(SQUARE) == pytest.approx(0.5, abs=0.01)


def test_cells_outside_the_polygon_do_not_count():
    t = CoverageTracker(cell_size_m=1.0)
    t.stamp_points(np.array([[100.0, 100.0]] * 50))
    assert t.coverage_fraction(SQUARE) == 0.0


def test_fraction_is_capped_at_one():
    t = CoverageTracker(cell_size_m=1.0)
    xs, ys = np.meshgrid(np.arange(-5.5, 15.5), np.arange(-5.5, 15.5))
    t.stamp_points(np.stack([xs.ravel(), ys.ravel()], axis=1))
    assert t.coverage_fraction(SQUARE) <= 1.0


def test_no_polygon_means_no_coverage_number():
    t = CoverageTracker()
    t.stamp_points(np.array([[1.0, 1.0]]))
    assert t.coverage_fraction(None) == 0.0


def test_raycast_paints_the_line_but_stops_short_of_the_frontier():
    t = CoverageTracker(cell_size_m=1.0, raycast_range_m=100.0,
                        raycast_min_step_m=0.0)
    assert t.stamp_raycast(np.array([0.0, 0.0]), np.array([[20.0, 0.0]]))
    xs = sorted(c[0] for c in t.cells)
    assert xs[0] == 0
    # pullback is 5 m: the line ends at x=15, the frontier cell at 20 stays
    # unobserved so it is still a frontier next tick.
    assert max(xs) == 15
    assert set(range(0, 16)) <= {c[0] for c in t.cells}


def test_raycast_is_rate_limited_by_travel():
    t = CoverageTracker(cell_size_m=1.0, raycast_min_step_m=5.0)
    assert t.stamp_raycast(np.array([0.0, 0.0]), np.array([[20.0, 0.0]]))
    assert not t.stamp_raycast(np.array([1.0, 0.0]), np.array([[20.0, 0.0]]))
    assert t.stamp_raycast(np.array([10.0, 0.0]), np.array([[30.0, 0.0]]))


def test_raycast_respects_the_range_cap():
    t = CoverageTracker(cell_size_m=1.0, raycast_range_m=10.0,
                        raycast_min_step_m=0.0)
    t.stamp_raycast(np.array([0.0, 0.0]), np.array([[100.0, 0.0]]))
    assert max(c[0] for c in t.cells) < 6      # 10 m range - 5 m pullback


def test_close_frontiers_paint_nothing():
    t = CoverageTracker(cell_size_m=1.0, raycast_min_step_m=0.0)
    assert not t.stamp_raycast(np.array([0.0, 0.0]), np.array([[2.0, 0.0]]))


def test_milestones_fire_once_each_in_order():
    t = CoverageTracker()
    assert list(t.new_milestones(0.05)) == []
    assert list(t.new_milestones(0.34)) == [10, 20, 30]
    assert list(t.new_milestones(0.34)) == []
    assert list(t.new_milestones(0.51)) == [40, 50]


def test_packed_grid_shape_and_bits():
    t = CoverageTracker(cell_size_m=0.5)
    t.stamp_points(np.array([[0.1, 0.1], [1.1, 0.1]]))
    res, ox, oy, w, h, data = t.packed_grid()
    assert res == 0.5 and w == 3 and h == 1
    assert ox == pytest.approx(0.0) and oy == pytest.approx(0.0)
    bits = np.unpackbits(np.frombuffer(data, dtype=np.uint8))[:w * h]
    assert bits.tolist() == [1, 0, 1]


def test_packed_grid_when_nothing_seen():
    assert CoverageTracker().packed_grid()[3:5] == (0, 0)


def test_negative_cells_shift_the_origin():
    t = CoverageTracker(cell_size_m=1.0)
    t.stamp_points(np.array([[-2.5, -3.5]]))
    _res, ox, oy, w, h, _d = t.packed_grid()
    assert (ox, oy, w, h) == (-3.0, -4.0, 1, 1)


def test_cell_centre_sampling_undercounts_a_grid_misaligned_polygon():
    """Coverage tests each cell's CENTRE against the polygon, and the even-odd
    test is asymmetric on the boundary (a centre on the left edge counts, one
    on the right edge does not). So a small polygon whose cells straddle the
    edge cannot read 100%: nine 2 m cells stamped by nine poses over this 6x6 m
    square report 4/9 of the area. Harmless at mission scale (0.5 m cells over
    a 250 m plate), but it decides what the integration harness can assert.
    """
    poly = np.array([[-3.0, -3.0], [3.0, -3.0], [3.0, 3.0], [-3.0, 3.0]])
    coarse = CoverageTracker(cell_size_m=2.0)
    xs, ys = np.meshgrid([-2.0, 0.0, 2.0], [-2.0, 0.0, 2.0])
    pts = np.stack([xs.ravel(), ys.ravel()], axis=1)
    coarse.stamp_points(pts)
    assert len(coarse.cells) == 9                       # cells -1, 0, 1
    assert coarse.coverage_fraction(poly) == pytest.approx(16.0 / 36.0, abs=1e-6)

    # 1 m cells with the same nine poses keep every centre strictly inside.
    fine = CoverageTracker(cell_size_m=1.0)
    fine.stamp_points(np.stack(
        [m.ravel() for m in np.meshgrid([-2.5, -0.5, 2.5], [-2.5, -0.5, 2.5])],
        axis=1))
    assert fine.coverage_fraction(poly) == pytest.approx(9.0 / 36.0, abs=1e-6)

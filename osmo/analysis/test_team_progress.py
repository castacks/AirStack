import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from team_progress import (
    TOTAL_RUNS_PER_BASELINE,
    assign_targets,
    credit_target_circle,
    geometry_from_markdown,
    markdown,
    point_in_polygon,
    resample_window,
    route_length,
)


def test_point_in_polygon_includes_boundary():
    square = [(0, 0), (10, 0), (10, 10), (0, 10)]
    assert point_in_polygon((5, 5), square)
    assert point_in_polygon((0, 5), square)
    assert not point_in_polygon((11, 5), square)


def test_fixed_sector_assignment_and_empty_robot_route():
    gt = {1: (2, 0), 2: (12, 0)}
    sectors = {
        1: [(-1, -1), (5, -1), (5, 1), (-1, 1)],
        2: [(9, -1), (15, -1), (15, 1), (9, 1)],
        3: [(19, -1), (25, -1), (25, 1), (19, 1)],
    }
    assignment = assign_targets(gt, sectors, [1, 2, 3], shared=False)
    assert assignment == {1: 1, 2: 2}
    total, per_robot = route_length(
        {1: (0, 0), 2: (10, 0), 3: (20, 0)}, gt, gt, assignment, 1)
    assert total == 4.0
    assert per_robot[3] == 0.0


def test_path_resampling():
    trajectory = resample_window([(10, 0, 0), (20, 20, 0)], 10, budget=10, step=1)
    assert np.allclose(trajectory[-1], [10, 20, 0])


def test_one_detector_circle_credits_every_gt_person_inside_it():
    gt = {1: (0, 0), 2: (5, 0), 3: (12.01, 0)}
    hits = {}
    credit_target_circle(gt, hits, (0, 0), radius=12.0, rel_t=8.0, robot=4)
    assert set(hits) == {1, 2}
    assert hits[1][:2] == (8.0, 4)


def test_average_table_shows_final_matrix_denominator():
    assert TOTAL_RUNS_PER_BASELINE == 48
    rendered = markdown([], [{
        "method": "Frontier", "runs": 7, "progress": 0.1,
        "progress_auc": 0.1, "actual_path_m": 1000,
        "ideal_all_path_m": 500, "ppl": 0.05,
    }])
    assert "| Frontier | 7/48 |" in rendered
    assert "Completed / total runs" in rendered


def test_geometry_cache_reads_actual_path_column(tmp_path):
    table = tmp_path / "old.md"
    table.write_text(
        "| Scene | Method | Run folder | GT | Visited | Progress | AUC | Actual team path | Ideal | PPL |\n"
        "| Fire / Suburban L1 | Frontier | `run/iter` | 49 | 14 | .2 | .1 | 5.59 km | 1.71 km | .02 |\n")
    assert geometry_from_markdown(table) == {
        ("Fire / Suburban L1", "Frontier"): 5590.0,
    }

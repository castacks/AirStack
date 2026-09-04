"""Regression tests for the deterministic urban-fire corridor."""

from disaster import fire_corridor as fc


def test_collapse_budget_skips_permanent_firebreaks():
    levels = {
        0: ("F5", 0.0, 10_000.0),  # oldest, but omitted from the manifest
        1: ("F5", 0.0, 9_000.0),
        2: ("F5", 0.0, 8_000.0),
        3: ("F5", 0.0, 7_000.0),
    }

    fc.apply_collapse(levels, [], n_f6=1, n_collapse=2, exclude={0})

    assert levels[0][0] == "F5"
    assert levels[1][0] == "F6"
    assert levels[2][0] == "F5c"
    assert levels[3][0] == "F5c"

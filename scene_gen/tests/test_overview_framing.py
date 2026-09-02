#!/usr/bin/env python3
"""test_overview_framing.py — the offline gate for the two overview-camera
defects, and for the per-block capture family.

    python3 -m pytest -q scene_gen/tests/test_overview_framing.py

Pure arithmetic: `plan_overviews` / `plan_block_shots` are camera SOLVES, and
whether a 1 km plate fits in frame is a trigonometry question, not something
that needs a renderer to answer. Both bugs below shipped precisely because
nobody could check them without a GPU.

DEFECT 1 — WRONG CENTRE. `plan_overviews` hardcoded `(0.0, 0.0)`. Any cell
whose content is not centred on the stage origin therefore got an overview of
the wrong ground; on the shipped cropped cells the offset was the crop-window
centre, up to 255 m.

DEFECT 2 — HEIGHT SOLVED FOR THE WRONG AXIS. The `0.95 * span` standoff was
derived from the HORIZONTAL aperture alone. Captures are 1280x720 and
`place_camera` authors only `horizontalAperture`, so the vertical field is
narrower by the frame aspect and a SQUARE plate cannot fit vertically. At
0.95*span the vertical ground coverage is roughly 0.62*span — the shipped
`city_top` was framing about the middle 60 % of the cell and calling it the
city.
"""
import math
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import baseline_captures as bc  # noqa: E402


def _ground_coverage(height_m, focal_mm, aperture_mm, aspect):
    """(width, height) of ground visible from a plumb camera."""
    half_h = math.atan(aperture_mm / (2.0 * focal_mm))
    half_v = math.atan(math.tan(half_h) * aspect[1] / aspect[0])
    return (2.0 * height_m * math.tan(half_h),
            2.0 * height_m * math.tan(half_v))


@pytest.mark.parametrize("span", [500.0, 800.0, 1000.0, 1500.0])
def test_top_down_actually_fits_the_whole_plate(span):
    h = bc.overview_height_m(span)
    w_cov, h_cov = _ground_coverage(h, bc.OVERVIEW_FOCAL_MM,
                                    bc.OVERVIEW_APERTURE_MM,
                                    bc.OVERVIEW_ASPECT)
    assert w_cov >= span, "horizontal coverage %.0f < span %.0f" % (w_cov, span)
    assert h_cov >= span, (
        "vertical coverage %.0f < span %.0f — this is the shipped bug: the "
        "0.95*span rule fits the horizontal axis only" % (h_cov, span))


def test_the_old_rule_would_have_failed_this_gate():
    """Proves the gate is not vacuous. If this ever passes, the test above is
    no longer measuring anything."""
    span = 1000.0
    _w, h_cov = _ground_coverage(span * bc.OVERVIEW_TOP_FRAC,
                                 bc.OVERVIEW_FOCAL_MM,
                                 bc.OVERVIEW_APERTURE_MM, bc.OVERVIEW_ASPECT)
    assert h_cov < span, (
        "the old 0.95*span rule now fits a square plate vertically — either "
        "the aspect or the aperture constant changed and this gate is stale")


def test_overview_is_centred_on_the_cell_not_the_origin():
    centre = (-180.0, 180.0)
    shots = {s.name: s for s in bc.plan_overviews(1000.0, centre=centre)}
    top = shots["overviews/city_top"]
    assert (round(top.eye[0], 3), round(top.eye[1], 3)) == centre
    assert (round(top.target[0], 3), round(top.target[1], 3)) == centre
    # the obliques must orbit the same centre, or a corner shot looks past
    # the cell entirely
    for tag in ("corner_ne", "corner_sw"):
        s = shots["overviews/city_%s" % tag]
        assert (round(s.target[0], 3), round(s.target[1], 3)) == centre


def test_default_centre_is_still_the_origin():
    """A cell that IS centred on the origin must be unchanged in x/y — the
    centre argument is additive, not a behaviour change for existing scenes."""
    shots = {s.name: s for s in bc.plan_overviews(1000.0)}
    top = shots["overviews/city_top"]
    assert (top.eye[0], top.eye[1]) == (0.0, 0.0)


# --------------------------------------------------------------------------
# the per-block family
# --------------------------------------------------------------------------

def test_block_family_shoots_every_block():
    blocks = [((-100.0, -50.0, 100.0, 50.0), "midrise"),
              ((120.0, -50.0, 200.0, 50.0), "tower"),
              ((-100.0, 80.0, 100.0, 180.0), "rowhouse")]
    shots = bc.plan_block_shots(blocks)
    assert len(shots) == len(blocks), (
        "the districts/ family shoots BURNING CLUSTERS, which on a "
        "one-corridor fire is three frames for the whole cell; blocks/ has to "
        "cover every block")
    names = [s.name for s in shots]
    assert all(n.startswith("blocks/b") for n in names)
    assert any("midrise" in n for n in names)


def test_block_shots_are_plumb_and_framed_on_the_block():
    blocks = [((-100.0, -50.0, 100.0, 50.0), "midrise")]
    s = bc.plan_block_shots(blocks)[0]
    assert (s.eye[0], s.eye[1]) == (s.target[0], s.target[1]), "not plumb"
    assert s.eye[0] == 0.0 and s.eye[1] == 0.0
    w_cov, h_cov = _ground_coverage(s.eye[2], s.focal_mm,
                                    bc.OVERVIEW_APERTURE_MM,
                                    bc.OVERVIEW_ASPECT)
    assert h_cov >= 200.0, "block long side does not fit the frame"


def test_block_shots_accept_the_shapes_callers_actually_have():
    """Bare rects, (rect, name) pairs and 5-tuples all appear in the codebase
    depending on whether `_typology_of` was consulted."""
    for blocks in ([(-10.0, -10.0, 10.0, 10.0)],
                   [((-10.0, -10.0, 10.0, 10.0), "tower")],
                   [(-10.0, -10.0, 10.0, 10.0, "tower")]):
        assert len(bc.plan_block_shots(blocks)) == 1


def test_blocks_are_in_the_plan_and_names_stay_unique():
    blocks = [((-100.0, -50.0, 100.0, 50.0), "midrise"),
              ((120.0, -50.0, 200.0, 50.0), "tower")]
    fams = bc.build_capture_plan(1000.0, [], [], centre=(-180.0, 180.0),
                                 blocks=blocks)
    assert fams["blocks"] and len(fams["blocks"]) == 2
    names = [s.name for s in fams["_all"]]
    assert len(names) == len(set(names)), "duplicate shot names"


def test_no_blocks_is_not_an_error():
    """Every other disaster's launcher calls this without a block list."""
    fams = bc.build_capture_plan(1000.0, [], [])
    assert fams["blocks"] == []

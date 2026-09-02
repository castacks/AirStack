#!/usr/bin/env python3
"""test_aec_burn.py -- do the pure-python helpers in `disaster/aec_burn.py`
compute the right geometry, off the plan a rowhouse fire needs?

    python3 scene_gen/tests/test_aec_burn.py
    pytest -q scene_gen/tests/test_aec_burn.py

WHY THIS EXISTS
---------------
`disaster/aec_burn.py` reads a named AEC brownstone row's own prims (units,
window islands, wall planes, storey levels) and turns that into a soot-layer
plan and an interior char pass -- see the module docstring for the asset
shape and the reasoning. Everything USD-shaped in that module (measuring off
a stage, authoring meshes and materials) needs Isaac Sim's `pxr`, which is
NOT on this host and is out of scope here. What IS in scope, and checked in
this file with hand-built inputs (no stage, no `pxr`, no `python3.sh`):

  * `_islands` -- the union-find that turns raw window-frame/glass boxes on
    one facade side into openings: overlapping boxes merge, disjoint boxes
    stay apart, anything under `WIN_MIN_M` in either axis is trim and is
    dropped, and the output is sorted;
  * `default_units` -- which units burn when the caller does not say: always
    a contiguous run inside `1..n`;
  * `_storey_of` -- the storey a world Z falls in, given the level list, with
    its `-0.05` m tolerance at each boundary;
  * `_wall_dist` / `_face_side` -- which of the row's four canvas sides
    (S/E/N/W) a given point or face normal belongs to, and how far inward
    from that side's line a point sits -- the exact geometry `canvas_uv` and
    the interior/exterior wall-inner-face split in `author_row` depend on;
  * `canvas_uv` -- world point -> (u, v) on the unwrapped soot canvas,
    including the roof case that reads the plume DOWN from the eaves instead
    of off the wall's own top;
  * `_is_interior` -- which parts of a burning unit are between the wall
    planes (and so get charred) versus in a plane, beyond it, or above the
    roof deck (and so are always exterior).

`aec_burn` imports `soot_plume`, `urban_fire`, `gac_fire`, `wall_overlay`,
`soot_bake` at module import time (package-relative with a bare-import
fallback); none of those pull `pxr` at module level, so
`from disaster import aec_burn` (or the bare `import aec_burn` this file also
supports) works unmodified on a plain host `python3` -- verified before
writing this file, so no `pxr` stub is installed here.
"""

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

try:                                      # package import (scene_gen on path)
    from disaster import aec_burn as ab
except ImportError:                       # bare-script import
    sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..", "disaster")))
    import aec_burn as ab                 # noqa: E402


# ---------------------------------------------------------------------------
# _islands
# ---------------------------------------------------------------------------
def test_islands_overlapping_boxes_union():
    # a "frame" box and a smaller "glass" box nested inside it, overlapping
    # in both u and z -> exactly one island, extent is the UNION of the two
    boxes = [(0.0, 1.0, 0.0, 1.5), (0.05, 0.95, 0.05, 1.45)]
    out = ab._islands(boxes)
    assert out == [(0.0, 1.0, 0.0, 1.5)], out

    # two boxes that only overlap once padded -> still one island
    boxes2 = [(0.0, 1.0, 0.0, 1.0), (0.9, 2.0, 0.0, 1.0)]
    out2 = ab._islands(boxes2)
    assert len(out2) == 1
    assert out2[0] == (0.0, 2.0, 0.0, 1.0), out2


def test_islands_disjoint_boxes_stay_separate():
    boxes = [(0.0, 1.0, 0.0, 1.0), (5.0, 6.0, 0.0, 1.0)]
    out = ab._islands(boxes)
    assert len(out) == 2
    assert (0.0, 1.0, 0.0, 1.0) in out
    assert (5.0, 6.0, 0.0, 1.0) in out


def test_islands_drops_undersized_extent():
    assert ab.WIN_MIN_M == 0.3
    # narrower than WIN_MIN_M in u -> dropped
    narrow = [(0.0, 0.1, 0.0, 1.0)]
    assert ab._islands(narrow) == []
    # shorter than WIN_MIN_M in z -> dropped
    short = [(0.0, 1.0, 0.0, 0.1)]
    assert ab._islands(short) == []
    # exactly WIN_MIN_M in both axes -> kept (>=, not >)
    exact = [(0.0, 0.3, 0.0, 0.3)]
    assert ab._islands(exact) == [(0.0, 0.3, 0.0, 0.3)]


def test_islands_output_sorted():
    # fed out of order -> comes back sorted ascending
    boxes = [(5.0, 6.0, 0.0, 1.0), (0.0, 1.0, 0.0, 1.0), (2.5, 3.5, 0.0, 1.0)]
    out = ab._islands(boxes)
    assert out == sorted(out)
    assert len(out) == 3


# ---------------------------------------------------------------------------
# default_units
# ---------------------------------------------------------------------------
def test_default_units_exact_values():
    assert ab.default_units(1) == (1,)
    assert ab.default_units(2) == (1, 2)
    assert ab.default_units(5) == (3, 4)
    assert ab.default_units(8) == (4, 5)
    assert ab.default_units(12) == (6, 7)


def test_default_units_always_contiguous_within_range():
    for n in range(1, 41):
        units = ab.default_units(n)
        assert 1 <= min(units) and max(units) <= n, (n, units)
        assert list(units) == list(range(min(units), max(units) + 1)), (n, units)


# ---------------------------------------------------------------------------
# _storey_of
# ---------------------------------------------------------------------------
def test_storey_of():
    levels = [-0.6, 2.5, 5.9, 9.2]
    assert ab._storey_of(levels, -0.3) == 0
    # just under the -0.05 m tolerance band below level[1] (2.5): storey 0
    assert ab._storey_of(levels, 2.44) == 0
    # AT the tolerance boundary (2.5 - 0.05 = 2.45) the storey has already
    # ticked over to 1
    assert ab._storey_of(levels, 2.45) == 1
    assert ab._storey_of(levels, 2.5) == 1
    assert ab._storey_of(levels, 7.0) == 2
    assert ab._storey_of(levels, 12.0) == 3


# ---------------------------------------------------------------------------
# _wall_dist / _face_side
# ---------------------------------------------------------------------------
def _mass():
    return {"cx": 0.0, "cy": 0.0, "W": 20.0, "D": 13.0, "yaw": 0.0}


def test_face_side_primary_walls():
    m = _mass()
    # perp=0: walls face +-x, so W/E are primary
    assert ab._face_side(m, (-1.0, 0.0, 0.0), (-9.0, 0.0), 0) == "W"
    assert ab._face_side(m, (1.0, 0.0, 0.0), (9.0, 0.0), 0) == "E"


def test_face_side_end_walls():
    m = _mass()
    # normal runs along the row (|n_y| > 0.7) and sits within end_tol of the
    # S line (y = -D/2 = -6.5; end_tol default 0.6) -> "S"
    assert ab._face_side(m, (0.0, -1.0, 0.0), (0.0, -6.4), 0) == "S"
    # same normal orientation but near the N line -> "N"
    assert ab._face_side(m, (0.0, 1.0, 0.0), (0.0, 6.3), 0) == "N"


def test_face_side_end_normal_far_from_either_end_falls_back_to_primary():
    m = _mass()
    # normal along the row but NOT near an end (mid-box) -> falls back to the
    # nearer of W/E, by x sign
    assert ab._face_side(m, (0.0, -1.0, 0.0), (-5.0, 0.0), 0) == "W"
    assert ab._face_side(m, (0.0, -1.0, 0.0), (5.0, 0.0), 0) == "E"


def test_wall_dist():
    m = _mass()
    # on the line: 0; moving inward (toward +x from the W line): positive
    assert ab._wall_dist(m, "W", -10.0, 0.0) == 0.0
    assert ab._wall_dist(m, "W", -7.0, 0.0) == 3.0
    assert ab._wall_dist(m, "S", 0.0, -6.5) == 0.0


# ---------------------------------------------------------------------------
# canvas_uv
# ---------------------------------------------------------------------------
def _skin():
    # perimeter 2*(W+D) = 2*(20+13) = 66; offsets per
    # soot_plume.perimeter_offsets: S=0, E=W, N=W+D, W=2W+D
    return {"per": 66.0, "H": 14.0, "z0": -0.6,
            "offsets": {"S": 0.0, "E": 20.0, "N": 33.0, "W": 53.0}}


def test_canvas_uv_wall_points():
    sk = _skin()
    m = _mass()
    # W wall, at its S-corner end (x=-10, y=-6.5, z=z0): u_W = D/2 - ly
    # = 6.5 - (-6.5) = 13 -> (53 + 13) / 66 = 1.0; v = (z0 - z0)/H = 0
    uv = ab.canvas_uv(sk, m, "W", np.asarray([[-10.0, -6.5, -0.6]]))
    assert np.allclose(uv[0], [1.0, 0.0]), uv[0]

    # W wall, at its N-corner end and roof height: u_W = D/2 - 6.5 = 0
    # -> 53/66; v = (13.4 - (-0.6)) / 14 = 1.0
    uv2 = ab.canvas_uv(sk, m, "W", np.asarray([[-10.0, 6.5, 13.4]]))
    assert np.allclose(uv2[0], [53.0 / 66.0, 1.0]), uv2[0]


def test_canvas_uv_v_clamps_above_top():
    sk = _skin()
    m = _mass()
    uv = ab.canvas_uv(sk, m, "W", np.asarray([[-10.0, 6.5, 500.0]]))
    assert uv[0, 1] == 1.0


def test_canvas_uv_v_clamps_below_z0():
    sk = _skin()
    m = _mass()
    uv = ab.canvas_uv(sk, m, "W", np.asarray([[-10.0, 6.5, -50.0]]))
    assert uv[0, 1] == 0.0


def test_roof_uv_is_parapet_distance_on_the_strip():
    sk = _skin()
    m = dict(_mass())
    m["deck_z"] = 11.9
    m["top"] = 13.5
    pt = np.asarray([[-9.0, 0.0, 12.0]])
    # u is the wall's own u: side_u(m, "W", -9, 0)
    u_expected = ab.spl.side_u(m, "W", -9.0, 0.0)
    off = sk["offsets"]["W"]
    # inward distance from the W line at x=-10 is |(-9) - (-10)| = 1 m,
    # so v = 1 - 1 / ROOF_REACH_M (row 0 of the strip is the parapet)
    d = ab._wall_dist(m, "W", -9.0, 0.0)
    assert d == 1.0
    v_expected = 1.0 - d / ab.ROOF_REACH_M

    uv = ab.roof_uv(sk, m, "W", pt)
    assert abs(uv[0, 0] - (off + u_expected) / sk["per"]) < 1e-9
    assert abs(uv[0, 1] - v_expected) < 1e-9

    # the wall u for the SAME point matches (u is shared with the wall)
    uv_flat = ab.canvas_uv(sk, m, "W", pt)
    assert abs(uv[0, 0] - uv_flat[0, 0]) < 1e-12
    # at and beyond the reach the strip's v clamps to 0; on the line it is 1
    far = ab.roof_uv(sk, m, "W", np.asarray([[-10.0 + ab.ROOF_REACH_M + 2.0, 0.0, 12.0]]))
    assert far[0, 1] == 0.0
    on = ab.roof_uv(sk, m, "W", np.asarray([[-10.0, 0.0, 12.0]]))
    assert on[0, 1] == 1.0


def test_roof_canvas_decays_inward_and_keeps_the_eaves_band():
    rgba = np.zeros((40, 8, 4), dtype=np.float32)
    rgba[..., 3] = 0.5
    rgba[0, :, 3] = 0.9          # the parapet row is darker
    rgba[:, :, :3] = 0.03
    sk = {"rgba": rgba, "ppm": 2.0}   # EAVES_BAND_M * 2 px/m = 2 band rows
    out = ab.roof_canvas(sk, rows=10)
    assert out.shape == (10, 8, 4)
    assert abs(out[0, 0, 3] - 0.9) < 1e-6          # row 0 = parapet, undecayed
    assert out[-1, 0, 3] < out[0, 0, 3]              # decays inward
    import math
    d_last = ab.ROOF_REACH_M
    src = rgba[(10 - 1) % 2, 0, 3]
    assert abs(out[-1, 0, 3] - src * math.exp(-d_last / ab.ROOF_DECAY_M)) < 1e-6
    assert np.allclose(out[..., :3], 0.03)


# ---------------------------------------------------------------------------
# _is_interior
# ---------------------------------------------------------------------------
def _unit():
    return {"plane_lo": -6.9, "plane_hi": 6.3, "cx": 0.0, "cy": 0.0}


def _part(cat, x0, x1, z0, z1, y0=-1.0, y1=1.0):
    return {"cat": cat, "bbox": (x0, y0, z0, x1, y1, z1)}


def test_is_interior_category_rule():
    # Floors is in INTERIOR_CATS -> interior regardless of extent, as long as
    # it is under the deck
    p = _part("Floors", -6.8, 6.2, 2.0, 2.5)
    assert ab._is_interior(p, _unit(), 0, 11.9) is True


def test_is_interior_windows_never_char():
    # Windows is in NEVER_CHAR -> always False, even spanning the interior
    p = _part("Windows", -6.8, 6.2, 2.0, 2.5)
    assert ab._is_interior(p, _unit(), 0, 11.9) is False


def test_is_interior_doors_position_test():
    unit = _unit()
    # a front door sitting IN the front wall plane -> exterior (the plane
    # tolerance test fails: its centre is not clear of the plane by
    # PLANE_TOL_M)
    front_door = _part("Doors", -7.1, -6.8, 2.0, 2.5)
    assert ab._is_interior(front_door, unit, 0, 11.9) is False

    # an interior door well inside both planes -> interior
    interior_door = _part("Doors", 0.0, 0.1, 3.0, 5.0)
    assert ab._is_interior(interior_door, unit, 0, 11.9) is True


def test_is_interior_touching_plane_is_exterior():
    unit = _unit()
    # a lighting fixture that touches/crosses the front wall plane -> exterior
    p = _part("Lighting", -7.2, -6.9, 2.0, 2.5)
    assert ab._is_interior(p, unit, 0, 11.9) is False


def test_is_interior_above_deck_is_exterior():
    unit = _unit()
    # roof plant, well above the deck -> always exterior
    p = _part("Mechanical", -1.0, 1.0, 12.0, 13.0)
    assert ab._is_interior(p, unit, 0, 11.9) is False


def test_is_interior_within_plane_tolerance_counts_as_interior():
    unit = _unit()
    # spans exactly plane_lo..plane_hi (within PLANE_TOL_M of each plane by
    # construction) and is not a category/never-char part -> interior
    assert ab.PLANE_TOL_M == 0.3
    p = _part("Structural_Framing", -6.9, 6.3, 2.0, 2.5)
    assert ab._is_interior(p, unit, 0, 11.9) is True


if __name__ == "__main__":
    ok, failed = 0, []
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("ok  " + name)
                ok += 1
            except AssertionError as e:
                print("FAIL " + name + ": " + str(e))
                failed.append(name)
    print("{0} passed, {1} failed".format(ok, len(failed)))
    if failed:
        sys.exit(1)

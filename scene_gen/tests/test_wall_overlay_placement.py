#!/usr/bin/env python3
"""test_wall_overlay_placement.py — is the soot overlay WHERE THE WALL IS?

    python3 scene_gen/tests/test_wall_overlay_placement.py
    pytest -q scene_gen/tests/test_wall_overlay_placement.py

WHY THIS EXISTS
---------------
The mask-driven scorch overlay (`disaster/wall_overlay.py`) was reported three
times running as being in the WRONG PLACE on the building — "the actual
transparent part isn't at the correct locations for these buildings" (user,
2026-08-29). Two rounds of review checked the mask's own internals and found
them self-consistent, which is exactly the failure mode this test is for: the
mask can be perfect in array space and still be pasted onto the wrong patch of
wall, and NOTHING in the mask's own unit tests can see that.

So this test does not look at the mask at all. It compares TWO INDEPENDENT
COMPUTATIONS of the same physical rectangle:

  A. where the overlay quad is authored — `urban_fire._wall_run_frame` gives
     `(fr, L)` and `wall_overlay.author_quad` spans `u = 0 .. L` on that
     frame, so its world corners are `quake_flow._b_face_pt(fr, u, v, out)`;

  B. where the wall actually is — the world bounding box of the very
     elements that run is made of, derived from each piece's own
     `_piece_frame` origin and width.

If those disagree, the overlay is on the wrong part of the building, and the
difference tells you by how much and in which direction. A half-module offset
means the frame is pivoted at a piece's centre rather than its left end; a
whole-run offset means the wrong element was taken as `left`.

Runs HOST-SIDE with no USD, no Kit and no Isaac Sim: `build_building` and
`describe` are pure Python over placement dicts, which is what makes this
cheap enough to run on every change.
"""

import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from detail import urban_building as ub          # noqa: E402
from disaster import quake_flow as qf            # noqa: E402
from disaster import urban_fire as uf            # noqa: E402

# A run's authored rectangle and its true wall rectangle should agree to well
# under a module. 0.35 m is tighter than the narrowest kit module (~1.2 m) and
# loose enough for the frame borrowing its depth from the leftmost piece.
TOL_M = 0.35

# Real bench column positions, so an offset that only appears away from the
# origin cannot hide: the first version of the ModernCity launcher centred on
# a WORLD bbox and worked perfectly at x=0 while cancelling the transform
# everywhere else.
POSITIONS = ((0.0, 0.0), (-277.2, 0.0), (184.8, 0.0))


def _axis(fr):
    """Unit vector along the wall, from the run frame's yaw."""
    return math.cos(fr[2]), math.sin(fr[2])


def _project(px, py, fr):
    """Signed distance of a world point along the run axis from `fr`'s origin.

    BOTH SPANS MUST BE PROJECTED ONTO THE SAME AXIS. The first version of this
    test compared `e["lx"]` against the frame's world origin and "failed" every
    run by exactly the building's own x — `lx`/`ly` are LOCAL (that is what the
    `l` is), and a north-facing run additionally travels along -X, so a raw
    coordinate comparison is wrong twice over. Projecting removes both
    problems and makes the check frame-agnostic.
    """
    ax, ay = _axis(fr)
    return (px - fr[0]) * ax + (py - fr[1]) * ay


def _run_world_span(run, fr):
    """(min, max) of the run's own pieces along the run axis, in metres.

    Each piece is pivoted at its LEFT END on the wall line (`_piece_frame`'s
    own docstring) and extends `fr_e[3]` along its own yaw, so its two ends
    are the points to project.
    """
    lo = hi = None
    for e in run:
        fe = qf._piece_frame(e)
        if fe is None:
            continue
        ex, ey = math.cos(fe[2]), math.sin(fe[2])
        for (px, py) in ((fe[0], fe[1]),
                         (fe[0] + ex * fe[3], fe[1] + ey * fe[3])):
            t = _project(px, py, fr)
            lo = t if lo is None else min(lo, t)
            hi = t if hi is None else max(hi, t)
    return lo, hi


def check(verbose=True):
    bad = []
    n_runs = 0
    rng = random.Random(20260829)
    for style in sorted(ub.STYLES.keys()):
        for (X, Y) in POSITIONS:
            try:
                pls = ub.build_building(style, X, Y, 0.0, rng)
                info = qf.describe(style, pls, X, Y, 0.0)
            except Exception as exc:
                bad.append("{0}: describe failed: {1}".format(style, exc))
                continue
            if info["type"] == "rc_glass":
                continue                   # curtain wall takes no overlay
            for mtag, m in info["masses"].items():
                by_side = {}
                for e in info["elements"]:
                    if e["mass"] == mtag and e["role"] == "wall":
                        by_side.setdefault(e["side"], []).append(e)
                for side, run in by_side.items():
                    if len(run) < 2:
                        continue
                    fr, L = uf._wall_run_frame(run, side)
                    if fr is None:
                        continue
                    n_runs += 1
                    lo, hi = _run_world_span(run, fr)
                    if lo is None:
                        continue
                    # the quad spans u = 0 .. L on this same frame, so in
                    # projected coordinates it is exactly 0 .. L
                    q_lo, q_hi = 0.0, L
                    d_lo, d_hi = abs(q_lo - lo), abs(q_hi - hi)
                    if d_lo > TOL_M or d_hi > TOL_M:
                        bad.append(
                            "{0} @({1:.0f},{2:.0f}) {3}/{4}: overlay spans "
                            "{5:.2f}..{6:.2f} along the wall but the wall's "
                            "own pieces span {7:.2f}..{8:.2f} -- off by "
                            "{9:.2f} m at the start, {10:.2f} m at the end "
                            "({11} piece(s))".format(
                                style, X, Y, mtag, side, q_lo, q_hi, lo, hi,
                                d_lo, d_hi, len(run)))
    if verbose:
        print("[overlay_placement] {0} wall run(s) checked across {1} style(s) "
              "at {2} position(s)".format(n_runs, len(ub.STYLES),
                                          len(POSITIONS)))
        print("[overlay_placement] {0}".format(
            "ok" if not bad else "FAILED — {0} run(s) misplaced".format(len(bad))))
        for b in bad[:12]:
            print("  " + b)
        if len(bad) > 12:
            print("  ... and {0} more".format(len(bad) - 12))
    return bad


def test_overlay_lands_on_its_wall():
    assert check(verbose=True) == []


if __name__ == "__main__":
    raise SystemExit(1 if check() else 0)

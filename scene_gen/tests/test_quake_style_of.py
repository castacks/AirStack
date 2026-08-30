#!/usr/bin/env python3
"""test_quake_style_of.py — does `quake.style_of` parse EVERY baked level,
and does heap clearance use a measured reach when the record has one?

    python3 scene_gen/tests/test_quake_style_of.py
    pytest -q scene_gen/tests/test_quake_style_of.py

WHY THIS EXISTS
---------------
`style_of` used to parse ONLY `bld_<style>_DG<n>.usd` (`rpartition("_DG")`).
The foundation family — `bld_<style>_SETTLE.usd` / `_TILT.usd` / `_OV.usd`,
baked by the SAME launcher for the SAME styles (`ARCH_GRADES`'s own default
already includes them) — has no numeral to split on and fell straight
through to `(None, None)`.

That is not just a missed grade label. `_mono_pass` skips anything
`style_of` resolves a style for (`if style or category != "house": continue`
— a resolved style means "a kit archetype `assemble` already handled",
never a monolith) and it runs AFTER `assemble`'s FOUNDATION pass, over the
SAME `placements` list, re-reading each `p["usd"]` — which the foundation
pass has by then already swapped to `bld_<style>_TILT.usd` etc. for
buildings assigned SETTLE/TILT/OV. With the old parser that swapped
placement's style comes back `None`, so `_mono_pass` did not skip it: it
re-scored the SAME building and could call `_tilt_prim` on it a second
time. `_tilt_prim` composes its rotation onto whatever local transform is
already there (`local = XformCache().GetLocalTransformation(prim)[0]; tr =
Gf.Transform(local * M)`), so this is a genuine COMPOUNDING double-lean, not
a relabelling — a building assigned a mild TILT could come out leaning at
roughly double the intended angle, sunk twice, with two ground-response
authorings round it. Recognising the foundation family in `style_of` is
what makes `_mono_pass`'s existing guard actually catch it.

Separately, `_clear_under_heaps` drew every heap's per-side reach from
`heap_reach_sides` (a formula off height + a stable per-building coin flip
for the second fall side) even when nothing on the record objects — there
was no way for an upstream pass that actually planned the pile geometry to
tell heap clearance where it really reached. `_heap_reach_for` is the pure
seam that prefers `r["reach_m"]` (a `{side: float}` dict) and `r["fall_sides"]`
when a `records` entry carries them, and falls back to the draw otherwise.

Both fixes are pure Python (dict/string work, no pxr, no stage), which is
what lets this run on the host the way `test_quake_heap_clearance.py` does
— `disaster.quake`'s pxr imports are lazy and per-function specifically so
its pure halves stay host-testable.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import quake as q                  # noqa: E402


# ---------------------------------------------------------------------------
def test_module_is_importable_without_pxr():
    for name in ("style_of", "_heap_reach_for", "heap_reach_sides",
                "heap_reach_m"):
        assert hasattr(q, name), name


# ---------------------------------------------------------------------------
# (a) style_of — DG levels, variants, and the foundation family
def test_dg_level():
    assert q.style_of("/x/bld_office_wide_DG0.usd") == ("office_wide", "DG0")
    assert q.style_of("bld_office_DG5.usd") == ("office", "DG5")


def test_dg_level_with_variant_suffix():
    assert q.style_of("/x/bld_office_wide_DG3_v2.usd") == ("office_wide", "DG3")


def test_foundation_family_settle_tilt_ov():
    """The bug: these three used to come back `(None, None)`."""
    assert q.style_of("bld_office_wide_TILT.usd") == ("office_wide", "TILT")
    assert q.style_of("bld_office_wide_SETTLE.usd") == ("office_wide", "SETTLE")
    assert q.style_of("bld_apartment_tall_OV.usd") == ("apartment_tall", "OV")


def test_foundation_family_with_variant_suffix():
    assert q.style_of("bld_office_wide_TILT_v3.usd") == ("office_wide", "TILT")
    assert q.style_of("bld_apartment_tall_OV_v1.usd") == ("apartment_tall", "OV")


def test_garbage_and_wrong_extension_are_unresolved():
    assert q.style_of("") == (None, None)
    assert q.style_of("bld_office_DG0.usdc") == (None, None)      # not ".usd"
    assert q.style_of("not_a_bld_prefix_DG3.usd") == (None, None)
    assert q.style_of("Muyang/DownTown/Assets/BG_Building_D.usd") == (None, None)


def test_mono_pass_guard_now_skips_a_tilt_swapped_kit_building():
    """The concrete double-transform check: `_mono_pass`'s own gate is
    `if style or category != "house": continue`. Before this fix, a
    TILT-swapped placement's `style_of(...)` was falsy and the guard did
    NOT skip it (re-scoring, and potentially re-tilting, a building
    `assemble`'s foundation pass already handled). After the fix, the
    guard correctly treats it as "already handled"."""
    style, _level = q.style_of("bld_office_wide_TILT.usd")
    should_skip = bool(style) or False   # category == "house" in this scenario
    assert should_skip, "a TILT-swapped kit building must be skipped by _mono_pass"


# ---------------------------------------------------------------------------
# (b) `_heap_reach_for` — the manifest-record path, and the fallback
def test_heap_reach_uses_the_record_when_present():
    """A record with a full `reach_m` dict is used verbatim, not redrawn —
    same values every call, unlike `heap_reach_sides`'s hashed pick."""
    r = {"reach_m": {"S": 12.3, "E": 9.0, "N": 3.1, "W": 3.1},
        "fall_sides": ["S", "E"]}
    got = q._heap_reach_for(r, "rc", "DG5", 24.0)
    assert got == {"S": 12.3, "E": 9.0, "N": 3.1, "W": 3.1}


def test_heap_reach_fills_a_missing_side_from_fall_sides():
    """A record whose `reach_m` does not cover every side fills the gap
    from `heap_reach_m`, using `fall_sides` to pick fall vs. blind for
    exactly that side — not a flat default."""
    H, grade, btype = 20.0, "DG5", "rc"
    r = {"reach_m": {"S": 12.3}, "fall_sides": ["S", "E"]}
    got = q._heap_reach_for(r, btype, grade, H)
    assert got["S"] == 12.3                                     # verbatim
    assert abs(got["E"] - q.heap_reach_m(btype, grade, H, fall_side=True)) < 1e-9
    assert abs(got["N"] - q.heap_reach_m(btype, grade, H, fall_side=False)) < 1e-9
    assert abs(got["W"] - q.heap_reach_m(btype, grade, H, fall_side=False)) < 1e-9


def test_heap_reach_defaults_fall_side_to_s_when_record_has_no_opinion():
    H, grade, btype = 20.0, "DG5", "rc"
    r = {"reach_m": {"S": 8.0}}                 # no "fall_sides" key at all
    got = q._heap_reach_for(r, btype, grade, H)
    assert got["S"] == 8.0
    assert abs(got["E"] - q.heap_reach_m(btype, grade, H, fall_side=False)) < 1e-9


def test_heap_reach_falls_back_to_the_draw_when_absent():
    """No `reach_m` at all (every kit archetype today): identical to the
    pre-existing `heap_reach_sides` draw, prim-path hash included."""
    H, grade, btype, prim = 20.0, "DG5", "rc", "/World/b_7"
    r = {}
    assert q._heap_reach_for(r, btype, grade, H, prim) == \
        q.heap_reach_sides(btype, grade, H, prim)


def test_heap_reach_falls_back_when_reach_m_is_empty_or_wrong_type():
    H, grade, btype, prim = 20.0, "DG5", "rc", "/World/b_8"
    for bad in ({}, None, [1, 2, 3], "nope"):
        r = {"reach_m": bad}
        assert q._heap_reach_for(r, btype, grade, H, prim) == \
            q.heap_reach_sides(btype, grade, H, prim)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)

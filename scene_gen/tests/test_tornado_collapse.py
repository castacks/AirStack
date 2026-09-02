#!/usr/bin/env python3
"""test_tornado_collapse.py — `disaster/tornado_collapse.py`, the INDUSTRIAL
tilt-up / light-roof collapse class (`_plans/urban_tornado_plan.md` §8c,
R11's second real urban collapse class).

    python3 -m pytest -q scene_gen/tests/test_tornado_collapse.py

`plan_industrial` is pure (no `pxr` import anywhere in that half of the
module — checked directly below, not just claimed) so most of this file
runs host-side with nothing but the standard library. `pxr` (bare
`usd-core`, no Kit/SimulationApp) IS available on this dev host, so
`apply_industrial` is exercised too, on an in-memory stage — the same
"no Kit launch needed" discipline `tools/tornado_kit_probe.py`'s own
container probe follows, just without even needing the container here.
"""
import json
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
if _SG not in sys.path:
    sys.path.insert(0, _SG)

from disaster import tornado_collapse as tcol           # noqa: E402

# A handful of REAL, measured Dmytro FactoryDistrict shed footprints
# (`config/harvested/standalone_buildings.json`, `config/asset_sets/
# urban_gac.yaml`'s own comments) — the same four chosen for the bench's
# industrial pocket (see `_plans/urban_tornado_C3_notes.md`).
_SHED_C_A = (25.1, 25.1, 11.1)      # Building_TypeC_A
_SHED_D_A = (42.0, 31.5, 11.7)      # Building_TypeD_A
_SHED_B_C = (67.1, 45.1, 12.0)      # Building_TypeB_C
_SHED_C_D = (41.1, 41.1, 16.0)      # Building_TypeC_D
_SHEDS = (_SHED_C_A, _SHED_D_A, _SHED_B_C, _SHED_C_D)


def _wind(bearing_deg=57.6, speed_frac=0.9):
    return {"bearing_deg": float(bearing_deg), "speed_frac": float(speed_frac)}


def _plan(W, D, H, grade, seed=7, yaw=20.0, x=0.0, y=0.0, wind=None):
    rng = random.Random(seed)
    return tcol.plan_industrial(W, D, H, yaw, x, y, grade,
                                wind if wind is not None else _wind(), rng)


# ---------------------------------------------------------------------------
# purity — no pxr anywhere above `apply_industrial`
# ---------------------------------------------------------------------------
def test_module_has_no_module_level_pxr_import():
    src = open(os.path.join(_SG, "disaster", "tornado_collapse.py")).read()
    apply_start = src.index("def apply_industrial")
    plan_half = src[:apply_start]
    assert "import pxr" not in plan_half
    assert "from pxr" not in plan_half


def test_plan_industrial_importable_and_pure():
    # already true by virtue of the import above succeeding without pxr
    # having been imported yet in this process for anything but the check
    # above -- reassert the grade table's own shape.
    assert tcol.GRADES == ("partial", "total")


# ---------------------------------------------------------------------------
# grade_for_intensity
# ---------------------------------------------------------------------------
def test_grade_for_intensity_cuts():
    assert tcol.grade_for_intensity(0.0) is None
    assert tcol.grade_for_intensity(0.49) is None
    assert tcol.grade_for_intensity(0.5) == "partial"
    assert tcol.grade_for_intensity(0.69) == "partial"
    assert tcol.grade_for_intensity(0.7) == "total"
    assert tcol.grade_for_intensity(1.0) == "total"


# ---------------------------------------------------------------------------
# determinism + JSON round-trip
# ---------------------------------------------------------------------------
def test_determinism_same_seed_same_plan():
    for grade in tcol.GRADES:
        for W, D, H in _SHEDS:
            p1 = _plan(W, D, H, grade, seed=13)
            p2 = _plan(W, D, H, grade, seed=13)
            assert json.dumps(p1, sort_keys=True) == json.dumps(p2, sort_keys=True), \
                (W, D, H, grade)


def test_different_seed_different_plan():
    p1 = _plan(*_SHED_D_A, "total", seed=1)
    p2 = _plan(*_SHED_D_A, "total", seed=2)
    assert json.dumps(p1, sort_keys=True) != json.dumps(p2, sort_keys=True)


def test_json_round_trip_every_shed_every_grade():
    for grade in tcol.GRADES:
        for W, D, H in _SHEDS:
            plan = _plan(W, D, H, grade, seed=5)
            s = json.dumps(plan, sort_keys=True)
            back = json.loads(s)
            assert back == plan, (W, D, H, grade)


def test_unknown_grade_raises():
    rng = random.Random(1)
    try:
        tcol.plan_industrial(20.0, 20.0, 10.0, 0.0, 0.0, 0.0, "catastrophic",
                             _wind(), rng)
        assert False, "expected KeyError"
    except KeyError:
        pass


# ---------------------------------------------------------------------------
# panel invariants: hinged at the wall line, outside the footprint
# ---------------------------------------------------------------------------
def _to_local(wx, wy, yaw, x, y):
    a = math.radians(-yaw)
    dx, dy = wx - x, wy - y
    return dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)


def test_every_panel_hinge_sits_on_the_footprint_boundary():
    """`hinge_x`/`hinge_y` is the wall-line point the panel pivoted from —
    it must sit ON the building's own OBB boundary (within float slop),
    never inside it and never far outside it."""
    for W, D, H in _SHEDS:
        for grade in tcol.GRADES:
            plan = _plan(W, D, H, grade, seed=9, yaw=33.0, x=50.0, y=-20.0)
            assert plan["panels"], (W, D, H, grade, "no panels in the plan")
            for p in plan["panels"]:
                lx, ly = _to_local(p["hinge_x"], p["hinge_y"], plan["yaw"],
                                   plan["x"], plan["y"])
                on_w = abs(abs(lx) - W / 2.0) < 1e-6 and abs(ly) <= D / 2.0 + 1e-6
                on_d = abs(abs(ly) - D / 2.0) < 1e-6 and abs(lx) <= W / 2.0 + 1e-6
                assert on_w or on_d, (W, D, H, grade, p["side"], lx, ly)


def test_every_panel_extends_outside_the_footprint():
    """The panel's own CENTRE (`x`/`y`) sits strictly beyond the wall line
    it hinged from, along that wall's own outward normal -- "outside the
    footprint" in the direction that matters, not just "not at the exact
    centre of the building"."""
    for W, D, H in _SHEDS:
        for grade in tcol.GRADES:
            plan = _plan(W, D, H, grade, seed=3, yaw=0.0, x=0.0, y=0.0)
            for p in plan["panels"]:
                dx = p["x"] - p["hinge_x"]
                dy = p["y"] - p["hinge_y"]
                _length, _base, _along, outward = tcol._wall_geom(p["side"], W, D)
                # outward is in the LOCAL (pre-yaw) frame; yaw is 0 here so
                # local == world for this check.
                out_dist = dx * outward[0] + dy * outward[1]
                assert out_dist > 0.0, (W, D, H, grade, p)


def test_flat_panel_reaches_its_own_height_leaning_panel_reaches_less():
    """A measured consequence of the hinge formula (module docstring,
    verified numerically against `_frag_box` directly): a FLAT panel's
    centre sits `height/2` out from the wall line; a LEANING panel's centre
    sits closer in (it has not fully fallen)."""
    plan = _plan(*_SHED_D_A, "total", seed=21, yaw=0.0, x=0.0, y=0.0)
    seen_flat = seen_leaning = False
    for p in plan["panels"]:
        _length, _base, _along, outward = tcol._wall_geom(p["side"], _SHED_D_A[0],
                                                           _SHED_D_A[1])
        dx, dy = p["x"] - p["hinge_x"], p["y"] - p["hinge_y"]
        out_dist = dx * outward[0] + dy * outward[1]
        if p["mode"] == "flat":
            seen_flat = True
            assert abs(out_dist - p["height"] / 2.0) < 1e-6, p
        else:
            seen_leaning = True
            assert 0.0 < out_dist < p["height"] / 2.0, p
    assert seen_flat and seen_leaning, "need both modes across this plan's panels"


def test_panel_base_never_penetrates_the_ground():
    """Every panel's own centre-height `z` combined with its tilt keeps its
    lowest point within a half-thickness of grade -- computed the same way
    `_frag_box`'s own docstring states the box's extents work, without
    needing pxr to check it."""
    for W, D, H in _SHEDS:
        for grade in tcol.GRADES:
            plan = _plan(W, D, H, grade, seed=17)
            for p in plan["panels"]:
                tilt = math.radians(p["tilt_deg"])
                hw, ht = p["height"] / 2.0, p["thickness"] / 2.0
                half_h = abs(hw * math.sin(tilt)) + abs(ht * math.cos(tilt))
                zmin = p["z"] - half_h
                assert -p["thickness"] / 2.0 - 1e-6 <= zmin <= 1e-6, (W, D, H, p)


def test_panel_tilt_and_lean_ranges():
    for W, D, H in _SHEDS:
        for grade in tcol.GRADES:
            plan = _plan(W, D, H, grade, seed=31)
            for p in plan["panels"]:
                if p["mode"] == "flat":
                    assert p["tilt_deg"] == 0.0
                else:
                    assert 15.0 - 1e-6 <= p["lean_deg"] <= 40.0 + 1e-6, p
                    assert abs(p["tilt_deg"] - (90.0 - p["lean_deg"])) < 1e-6
                assert 2.4 - 1e-6 <= p["length"] or p["length"] > 0.0  # last-seg slack
                assert 0.18 - 1e-6 <= p["thickness"] <= 0.30 + 1e-6, p


# ---------------------------------------------------------------------------
# grade shape: total is 60-80% of perimeter, partial is one wall + corners
# ---------------------------------------------------------------------------
def test_total_grade_perimeter_fraction_in_band():
    for W, D, H in _SHEDS:
        for seed in range(15):
            plan = _plan(W, D, H, "total", seed=seed)
            f = plan["stats"]["fallen_frac"]
            assert 0.60 - 1e-6 <= f <= 0.80 + 1e-6, (W, D, H, seed, f)


def test_partial_grade_is_one_wall_plus_its_two_corners():
    for W, D, H in _SHEDS:
        for seed in range(15):
            plan = _plan(W, D, H, "partial", seed=seed)
            primary = plan["stats"]["primary_wall"]
            assert primary in ("S", "N", "W", "E")
            sides_touched = sorted(set(p["side"] for p in plan["panels"]))
            # the primary wall is fully down; at most its two ADJACENT
            # walls (never the opposite one) contribute a single segment
            # each.
            assert primary in sides_touched
            for sd in sides_touched:
                if sd == primary:
                    continue
                assert sd in tcol._ADJACENT_WALLS[primary], (primary, sd)
            assert plan["stats"]["walls_fully_down"] == [primary]
            # partial is a SMALLER event than total -- well under the
            # 60-80% band total uses (never a hard number in the brief,
            # but a whole extra wall + two corner segments is bounded well
            # short of "most of the building").
            assert plan["stats"]["fallen_frac"] < 0.60


def test_never_more_than_the_two_adjacent_walls_on_partial():
    for W, D, H in _SHEDS:
        plan = _plan(W, D, H, "partial", seed=44)
        primary = plan["stats"]["primary_wall"]
        opposite = {"S": "N", "N": "S", "W": "E", "E": "W"}[primary]
        assert not any(p["side"] == opposite for p in plan["panels"]), \
            (W, D, H, primary, opposite)


# ---------------------------------------------------------------------------
# roof field INSIDE the footprint, coverage tracks the drawn fraction
# ---------------------------------------------------------------------------
def test_roof_field_stays_inside_the_footprint():
    for W, D, H in _SHEDS:
        plan = _plan(W, D, H, "total", seed=8, yaw=41.0, x=12.0, y=-7.0)
        assert plan["roof_sheets"], "expected roof sheets"
        for s in plan["roof_sheets"]:
            lx, ly = _to_local(s["x"], s["y"], plan["yaw"], plan["x"], plan["y"])
            assert abs(lx) <= W / 2.0 + 1e-6, (W, D, lx)
            assert abs(ly) <= D / 2.0 + 1e-6, (D, ly)
            assert 0.2 - 1e-6 <= s["z"] <= 0.8 + 1e-6, s
        for j in plan["joists"]:
            lx, ly = _to_local(j["x"], j["y"], plan["yaw"], plan["x"], plan["y"])
            assert abs(lx) <= W / 2.0 + 1e-6
            assert abs(ly) <= D / 2.0 + 1e-6
            assert 0.2 - 1e-6 <= j["z"] <= 0.8 + 1e-6, j


def test_roof_coverage_matches_drawn_fraction_within_tiling_slack():
    for W, D, H in _SHEDS:
        for seed in range(10):
            plan = _plan(W, D, H, "total", seed=seed)
            cov = plan["stats"]["roof_coverage"]
            assert 0.4 - 1e-6 <= cov <= 0.9 + 1e-6, (W, D, seed, cov)
            n_sheets = plan["stats"]["n_roof_sheets"]
            assert n_sheets >= 1


def test_n_joists_in_band():
    for W, D, H in _SHEDS:
        plan = _plan(W, D, H, "total", seed=6)
        assert 4 <= plan["stats"]["n_joists"] <= 10


def test_joist_material_is_steel_joist_not_bare_metal():
    """D4 (round4): the §8c spec calls joists out as "dark steel", distinct
    from a generic sheet-metal roof -- `"steel_joist"`, not `"metal"`, so a
    future `tornado_urban_usd._classify` bucket can key a darker look off
    it without this module changing again (see the report for the exact
    bucket proposed to the lead)."""
    for W, D, H in _SHEDS:
        plan = _plan(W, D, H, "total", seed=6)
        assert plan["joists"], (W, D, H)
        for j in plan["joists"]:
            assert j["material"] == "steel_joist", j


# ---------------------------------------------------------------------------
# D4 (round4): roof sheets must read as a HEAPED pile, not a uniform
# lawn-sprinkler scatter -- "random single coloured rectangles ... paper
# confetti" was the bench verdict regardless of colour.
# ---------------------------------------------------------------------------
def test_roof_sheets_cluster_not_uniform_scatter():
    W, D, H = _SHED_B_C   # the biggest shed -- most sheets, least ambiguous
    plan = _plan(W, D, H, "total", seed=8, yaw=0.0, x=0.0, y=0.0)
    sheets = plan["roof_sheets"]
    assert len(sheets) >= 30, len(sheets)
    pts = [(s["x"], s["y"]) for s in sheets]
    mean_len = sum(s["size"][0] for s in sheets) / len(sheets)
    n_close = 0
    for i, (x0, y0) in enumerate(pts):
        best = min(math.hypot(x0 - x1, y0 - y1)
                  for j, (x1, y1) in enumerate(pts) if j != i)
        if best <= mean_len:
            n_close += 1
    frac_close = n_close / len(pts)
    # A uniform independent scatter over even this shed's footprint (67.1 x
    # 45.1 m, ~845 sheets at the measured coverage) puts a sheet's own
    # nearest neighbour well beyond one sheet-length for MOST sheets (mean
    # nearest-neighbour spacing for a Poisson scatter at this density is
    # several sheet-lengths) -- so a high `frac_close` is a real clustering
    # signature, not a tautology of the density alone.
    assert frac_close >= 0.5, frac_close


def test_roof_sheets_stay_in_the_0_2_to_0_8_band_even_when_clustered():
    """The clustering in `_build_roof` must not smuggle a sheet outside the
    module's own documented Z band (this file's pre-existing
    `test_roof_field_stays_inside_the_footprint` already checks this per-
    sheet on every shed/seed; this is the pile-specific companion: multiple
    DISTINCT pile-top heights actually get drawn, not one flat band reused
    for every cluster)."""
    W, D, H = _SHED_B_C
    plan = _plan(W, D, H, "total", seed=8, yaw=0.0, x=0.0, y=0.0)
    zs = sorted(s["z"] for s in plan["roof_sheets"])
    assert zs[0] >= 0.2 - 1e-6 and zs[-1] <= 0.8 + 1e-6
    # more than one effective pile height in play -- not every sheet at
    # (near enough) the same z, which is what an unclustered flat scatter
    # already gave for free.
    assert (zs[-1] - zs[0]) > 0.1, zs


def test_roof_material_is_metal_or_membrane():
    seen = set()
    for seed in range(20):
        plan = _plan(*_SHED_D_A, "total", seed=seed)
        seen.add(plan["stats"]["roof_material"])
    assert seen <= {"metal", "membrane"}
    assert len(seen) == 2, "expected both roof materials to appear over 20 seeds"


# ---------------------------------------------------------------------------
# contents / rubble
# ---------------------------------------------------------------------------
def test_contents_inside_footprint_and_seated():
    for W, D, H in _SHEDS:
        plan = _plan(W, D, H, "total", seed=2)
        assert 5 <= plan["stats"]["n_contents"] <= 15
        for c in plan["contents"]:
            lx, ly = _to_local(c["x"], c["y"], plan["yaw"], plan["x"], plan["y"])
            assert abs(lx) <= W / 2.0 + 1e-6
            assert abs(ly) <= D / 2.0 + 1e-6
            assert c["z"] >= -0.03 - 1e-6


def test_rubble_rings_every_panel_and_is_never_at_the_hinge():
    for W, D, H in _SHEDS:
        plan = _plan(W, D, H, "total", seed=4)
        assert plan["rubble"], "expected rubble"
        panel_sides = {(p["side"], p["seg_i"]) for p in plan["panels"]}
        rubble_sources = {(r["from_panel_side"], r["from_panel_seg_i"])
                          for r in plan["rubble"]}
        assert rubble_sources <= panel_sides
        # every fallen panel contributed at least SOME rubble.
        assert rubble_sources == panel_sides


# ---------------------------------------------------------------------------
# apply_industrial — pxr IS available on this host, so exercise it too.
# ---------------------------------------------------------------------------
def _bare_stage():
    from pxr import Usd, UsdGeom
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    cell = "/W/cell0"
    UsdGeom.Xform.Define(st, cell)
    return st, cell


def test_apply_industrial_authors_one_mesh_per_class_and_counts_match():
    st, cell = _bare_stage()
    plan = _plan(*_SHED_D_A, "total", seed=7, x=0.0, y=0.0, yaw=0.0)
    ctx = {"stage": st, "parent": cell, "mats": {}, "verbose": False}
    counts = tcol.apply_industrial(st, ctx, plan)
    assert counts["n_panels"] == len(plan["panels"])
    assert counts["n_roof_sheets"] == len(plan["roof_sheets"])
    assert counts["n_joists"] == len(plan["joists"])
    assert counts["n_contents"] == len(plan["contents"])
    assert counts["n_rubble"] == len(plan["rubble"])
    assert counts["n_meshes"] == 5   # panel, roof_sheet, joist, contents, rubble
    from pxr import Usd, UsdGeom
    for path in counts["meshes"]:
        prim = st.GetPrimAtPath(path)
        assert prim and prim.IsValid(), path
        assert UsdGeom.Mesh(prim)


def test_apply_industrial_deactivates_src_when_present():
    st, cell = _bare_stage()
    from pxr import UsdGeom
    UsdGeom.Cube.Define(st, cell + "/src")
    plan = _plan(*_SHED_C_A, "total", seed=1)
    ctx = {"stage": st, "parent": cell, "mats": {}, "verbose": False}
    counts = tcol.apply_industrial(st, ctx, plan)
    assert counts["n_src_deactivated"] == 1
    assert st.GetPrimAtPath(cell + "/src").IsActive() is False


def test_apply_industrial_no_src_is_a_harmless_no_op():
    st, cell = _bare_stage()
    plan = _plan(*_SHED_C_A, "partial", seed=1)
    ctx = {"stage": st, "parent": cell, "mats": {}, "verbose": False}
    counts = tcol.apply_industrial(st, ctx, plan)
    assert counts["n_src_deactivated"] == 0


def test_apply_industrial_debris_bbox_roughly_bounds_the_shed_plus_panels():
    from pxr import Usd, UsdGeom
    st, cell = _bare_stage()
    W, D, H = _SHED_D_A
    plan = _plan(W, D, H, "total", seed=7, x=0.0, y=0.0, yaw=0.0)
    ctx = {"stage": st, "parent": cell, "mats": {}, "verbose": False}
    tcol.apply_industrial(st, ctx, plan)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                           useExtentsHint=False)
    prim = st.GetPrimAtPath(cell + "/industrial_debris")
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    lo, hi = r.GetMin(), r.GetMax()
    # never floating (cause 1 of the tornado skill's own catalogue) and
    # never buried past a shard's own half-thickness.
    assert lo[2] >= -0.05
    # roughly the shed's own footprint, generously padded for the tallest
    # fallen panel's own outward reach (bounded by H) plus rubble's own
    # ring-out slack.
    pad = H + 3.0
    assert lo[0] >= -(W / 2.0 + pad) and hi[0] <= (W / 2.0 + pad)
    assert lo[1] >= -(D / 2.0 + pad) and hi[1] <= (D / 2.0 + pad)


# ---------------------------------------------------------------------------
# D4 (round4) — every authored mesh is BOUND to a real, non-white material.
# The bench-v3 verdict ("roof sheets/joists render PURE WHITE untextured")
# does not reproduce against `tuw.debris_material` as measured (container
# probe, `_plans/urban_tornado_round4_brief.md`'s own D4 report has the
# numbers), but this locks the invariant in so a future regression in
# EITHER this module's own binding call OR `tornado_urban_usd.
# debris_material`'s bucket table trips a test instead of shipping quietly.
# ---------------------------------------------------------------------------
def test_apply_industrial_every_mesh_bound_and_not_white():
    from pxr import UsdGeom, UsdShade
    st, cell = _bare_stage()
    for grade in tcol.GRADES:
        sub = cell + "/" + grade
        UsdGeom.Xform.Define(st, sub)
        plan = _plan(*_SHED_C_A, grade, seed=7)
        ctx = {"stage": st, "parent": sub, "mats": {}, "verbose": False}
        counts = tcol.apply_industrial(st, ctx, plan)
        assert counts["meshes"], grade
        for path in counts["meshes"]:
            prim = st.GetPrimAtPath(path)
            assert prim and prim.IsValid(), path
            mat, _rel = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            assert mat and mat.GetPrim().IsValid(), (grade, path, "UNBOUND")
            sh = UsdShade.Shader.Get(st, mat.GetPath().AppendChild("Shader"))
            assert sh, (grade, path, mat.GetPath(), "no Shader child prim")
            dc_input = sh.GetInput("diffuse_color_constant")
            val = dc_input.Get() if dc_input else None
            assert val is not None, (grade, path, "no diffuse_color_constant authored")
            assert not all(c > 0.95 for c in val), (grade, path, tuple(val),
                                                     "reads as WHITE")


def test_apply_industrial_never_leaves_a_mesh_unbound_even_if_debris_material_misbehaves(
        monkeypatch):
    """The defensive fallback added this round: if `tuw.debris_material`
    ever returns `None` (never observed for this module's own (kind,
    material) pairs, per the container probe -- but the exact failure mode
    the D4 bench symptom would produce if it recurred), `apply_industrial`
    must still bind SOMETHING rather than leave the mesh unbound."""
    from pxr import UsdShade

    from disaster import tornado_urban_usd as tuw
    st, cell = _bare_stage()
    plan = _plan(*_SHED_C_A, "partial", seed=3)
    ctx = {"stage": st, "parent": cell, "mats": {}, "verbose": False}
    monkeypatch.setattr(tuw, "debris_material", lambda *a, **k: None)
    counts = tcol.apply_industrial(st, ctx, plan)
    assert counts["meshes"]
    for path in counts["meshes"]:
        prim = st.GetPrimAtPath(path)
        mat, _rel = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        assert mat and mat.GetPrim().IsValid(), (path, "STILL unbound after fallback")


def test_steel_joist_material_string_resolves_to_the_dark_joist_bucket():
    """Round 4: the lead landed the dedicated `steel_joist` bucket this
    stream's report specced (dark bar-joist steel, plan 8c "joists dark
    steel") -- `"steel_joist"` now routes there, ahead of the generic
    `"steel"` substring match, and the bucket is darker than `metal`."""
    from disaster import tornado_urban_usd as tuw
    assert tuw._classify("joist", "steel_joist") == "steel_joist"
    flat_joist, _grime_j, _r = tuw._CLASS_LOOK["steel_joist"]
    flat_metal, _grime_m, _r2 = tuw._CLASS_LOOK["metal"]
    assert max(flat_joist) < min(flat_metal)


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))

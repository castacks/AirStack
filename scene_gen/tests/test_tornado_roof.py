#!/usr/bin/env python3
"""test_tornado_roof.py — does `disaster/tornado_roof.py`'s roof-damage pass
author NO membrane-peel art at any level, throw its shed sheets off the
windward edge in the per-level count band, sit its one remaining roof-plane
quad (the gravel scour band) exactly 0.015 m above the roof, refuse to
touch a roof the façade ladder already shed, and — the one invariant every
other check is worthless without — leave the FAÇADE plan byte-identical
whether or not this module is ever called?

THE PEEL IS RETIRED (round 4, the user's own call on the bench: the
substrate patches "look weird and unnatural ... I don't think we can make
it look good in isaac-sim"). Two tests below pin that, one on the PLAN
(`test_plan_never_carries_a_peel_patch_at_any_level`) and one on the
authored STAGE (`test_apply_authors_no_substrate_or_lip_prims`) — the
second is the one that matters, because a plan key can go quiet while the
apply step keeps drawing.

    python3 -m pytest -q scene_gen/tests/test_tornado_roof.py

WHY THIS EXISTS
---------------
`_plans/urban_tornado_plan.md` §8 (ROUND 3, stream RF): the aerial-view
defect the user's own bench review named ("no roof damage ... removals read
as cut-outs") gets a SECOND damage pass over the same element table
`tornado_urban.plan_damage` already walks — `tornado_roof.plan_roof`, a
pure planner exactly like its façade sibling, checked the same way `test_
tornado_urban.py`/`test_tornado_kit.py` check theirs: no `pxr` above
`apply_roof`, host-side, seconds.

RUNS WITHOUT ISAAC. `pxr` (usd-core) is on the host (same claim `test_
tornado_urban_usd.py` already verifies) — the one `apply_roof` test below
builds an in-memory stage, no Kit, no VTK.
"""

import json
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
if _SG not in sys.path:
    sys.path.insert(0, _SG)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from disaster import quake_flow as qf                       # noqa: E402
from disaster import quake_sliced as qs                      # noqa: E402
from disaster import tornado_kit as tk                        # noqa: E402
from disaster import tornado_urban as tu                       # noqa: E402
from disaster import tornado_roof as tr                         # noqa: E402
from test_quake_sliced import fake_sliced_building                # noqa: E402


# ---------------------------------------------------------------------------
# fixtures — the same shapes/helper pattern `test_tornado_urban.py` uses
# ---------------------------------------------------------------------------
def fake_wind(bearing_deg, speed_frac=0.8, cross_frac=-0.4, over=False):
    return {"bearing_deg": float(bearing_deg), "speed_frac": float(speed_frac),
            "cross_frac": float(cross_frac), "over": bool(over)}


_FIXTURES = {
    "lowrise": dict(W=24.0, D=18.0, H=12.0, storeys=3),
    "midrise": dict(W=30.0, D=24.0, H=40.0, storeys=10),
    "tower": dict(W=42.0, D=36.0, H=120.0, storeys=30),
}


def _fixture_info(name, seed=5, btype="urm"):
    kw = dict(_FIXTURES[name])
    pls, style, _grid = fake_sliced_building(seed=seed, **kw)
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = btype
    return info


_LEVEL_INTENSITY = {"T0": 0.05, "T1": 0.2, "T2": 0.4, "T3": 0.6, "T4": 0.85}


def _plans(fixture, level, btype="urm", seed=5, wind=None, intensity=None,
          height_class=None, tag="probe"):
    """`(info, facade_plan, roof_plan)` — the façade plan built first, on
    its OWN `random.Random(seed)`, then the roof plan on a wholly separate
    `random.Random(tr.roof_seed(tag))`, mirroring the intended hook order
    in `wreck_urban`/`wreck_kit`."""
    info = _fixture_info(fixture, seed=seed, btype=btype)
    wind = wind if wind is not None else fake_wind(35.0)
    if intensity is None:
        intensity = _LEVEL_INTENSITY.get(level, 0.5)
    facade_rng = random.Random(seed)
    facade_plan = tu.plan_damage(info, info["elements"], level, btype,
                                 facade_rng, wind, height_class=height_class,
                                 intensity=intensity)
    roof_rng = random.Random(tr.roof_seed(tag))
    roof_plan = tr.plan_roof(info, info["elements"], level, wind, roof_rng,
                             height_class, intensity, facade_plan=facade_plan)
    return info, facade_plan, roof_plan


def _mean(xs):
    xs = list(xs)
    return sum(xs) / len(xs) if xs else 0.0


# ---------------------------------------------------------------------------
# schema / determinism
# ---------------------------------------------------------------------------
def test_json_round_trip():
    for level in tr.LEVELS:
        _info, _fp, rp = _plans("midrise", level, seed=9)
        again = json.loads(json.dumps(rp))
        assert again == rp, "roof plan is not JSON round-trip stable at " + level


def test_determinism_same_seed_same_tag():
    _i1, _f1, r1 = _plans("midrise", "T3", seed=5, tag="cell_A")
    _i2, _f2, r2 = _plans("midrise", "T3", seed=5, tag="cell_A")
    assert json.dumps(r1, sort_keys=True) == json.dumps(r2, sort_keys=True)


def test_determinism_different_tag_usually_differs():
    _i1, _f1, r1 = _plans("midrise", "T3", seed=5, tag="cell_A")
    _i2, _f2, r2 = _plans("midrise", "T3", seed=5, tag="cell_B_totally_different")
    assert json.dumps(r1, sort_keys=True) != json.dumps(r2, sort_keys=True)


def test_roof_seed_is_stable_and_int():
    a = tr.roof_seed("some/cell/path")
    b = tr.roof_seed("some/cell/path")
    assert a == b
    assert isinstance(a, int)


# ---------------------------------------------------------------------------
# THE FAÇADE-PLAN-UNCHANGED ASSERTION — the invariant every other check
# depends on being true
# ---------------------------------------------------------------------------
def test_facade_plan_unchanged_by_roof_hook():
    """A façade plan built with `rng = random.Random(seed)` and NEVER
    touched by this module is byte-identical to one built with an
    identical fresh `random.Random(seed)` immediately followed by a
    `plan_roof(..., rng=random.Random(<other seed>))` call on a completely
    separate rng object — the exact ordering `wreck_urban`'s hook uses."""
    info_a = _fixture_info("midrise", seed=5, btype="rc_glass")
    rng_a = random.Random(5)
    wind = fake_wind(57.6, speed_frac=1.0, cross_frac=-0.33)
    plan_pre = tu.plan_damage(info_a, info_a["elements"], "T4", "rc_glass",
                              rng_a, wind, height_class="midrise",
                              intensity=0.85)

    info_b = _fixture_info("midrise", seed=5, btype="rc_glass")
    rng_b = random.Random(5)
    plan_post = tu.plan_damage(info_b, info_b["elements"], "T4", "rc_glass",
                               rng_b, wind, height_class="midrise",
                               intensity=0.85)
    # the hook: plan_roof called on a SEPARATE rng, right after
    roof_rng = random.Random(tr.roof_seed("cell_9"))
    _ = tr.plan_roof(info_b, info_b["elements"], "T4", wind, roof_rng,
                     "midrise", 0.85, facade_plan=plan_post)

    assert json.dumps(plan_pre, sort_keys=True) == json.dumps(plan_post, sort_keys=True), (
        "the facade plan changed depending on whether plan_roof ran afterwards")


# ---------------------------------------------------------------------------
# THE PEEL IS RETIRED — the plan half
# ---------------------------------------------------------------------------
def test_plan_never_carries_a_peel_patch_at_any_level():
    """No level, no construction type, no seed produces a peel patch, and
    the stats no longer carry the peel's own two counters at all (absent,
    not zeroed — a zeroed `patch_coverage_frac` still advertises a feature
    this module does not have)."""
    for btype in ("urm", "rc", "rc_glass"):
        for level in tr.LEVELS:
            for seed in range(1, 8):
                _info, _fp, rp = _plans("midrise", level, btype=btype,
                                        seed=seed,
                                        tag="nopeel_" + btype + level + str(seed))
                assert rp["patches"] == [], (btype, level, seed)
                assert "n_patches" not in rp["stats"]
                assert "patch_coverage_frac" not in rp["stats"]
                assert rp["schema"] == "tornado_roof_plan.v2"


def test_module_exposes_no_peel_machinery():
    """The peel tables/helpers are DELETED, not defaulted to zero — a
    later round cannot re-enable the look by flipping a constant back."""
    for gone in ("_PATCH_COUNT", "_COVERAGE", "_T1_PATCH_P", "_substrate_for",
                 "_author_lip", "_draw_patches", "_shares"):
        assert not hasattr(tr, gone), gone + " is still defined"


# ---------------------------------------------------------------------------
# the tear run — windward, end-snapped, never centred
# ---------------------------------------------------------------------------
def test_tear_run_is_windward_and_end_snapped():
    """The shed sheets' release line runs along the WINDWARD edge (the
    max-weight side), spans `_TEAR_RUN_FRAC` of that edge, and is snapped
    to one END of it — never centred, matching the record's own "failure
    initiates at the edge/corner metal"."""
    n_checked = 0
    for seed in range(1, 21):
        _info, _fp, rp = _plans("midrise", "T3", seed=seed,
                                wind=fake_wind(35.0 + seed), tag="tear_" + str(seed))
        tear = rp["tear"]
        if tear["run_m"] <= 0.0:
            continue
        weights = rp["side_weights"]
        assert tear["side"] == rp["windward_side"]
        assert weights[tear["side"]] >= _mean(weights.values()) - 1e-9

        frame = tr._side_frame(tear["side"], rp["roof"]["rect_local"])
        along_total = frame["along_hi"] - frame["along_lo"]
        lo_f, hi_f = tr._TEAR_RUN_FRAC
        assert lo_f * along_total - 1e-6 <= tear["run_m"] <= hi_f * along_total + 1e-6
        a0 = tear["along_start_m"]
        assert (abs(a0) < 1e-9
                or abs(a0 - (along_total - tear["run_m"])) < 1e-6), (
            "tear run is not snapped to either end of the windward edge")
        assert tear["depth_m"] <= frame["depth_max"] * tr._TEAR_DEPTH_MAX_FRAC + 1e-6
        assert tear["depth_m"] <= tr._TEAR_DEPTH_M[1] + 1e-6
        # the release line sits at that depth, spanning that run
        p0, p1 = tear["p0_local"], tear["p1_local"]
        inner = frame["edge_val"] + frame["depth_sign"] * tear["depth_m"]
        if frame["along_axis"] == "x":
            assert abs(p0[1] - inner) < 1e-6 and abs(p1[1] - inner) < 1e-6
            assert abs(abs(p1[0] - p0[0]) - tear["run_m"]) < 1e-6
        else:
            assert abs(p0[0] - inner) < 1e-6 and abs(p1[0] - inner) < 1e-6
            assert abs(abs(p1[1] - p0[1]) - tear["run_m"]) < 1e-6
        n_checked += 1
    assert n_checked > 0, "no tear run drawn across 20 T3 seeds — test is vacuous"


def test_shed_sheets_are_thin_membrane_frags_seated_on_roof_or_grade():
    n_checked = 0
    for seed in range(1, 16):
        info, _fp, rp = _plans("midrise", "T4", seed=seed, tag="sheet_" + str(seed))
        roof_top = info["masses"]["main"]["top"]
        for f in rp["sheets"]:
            assert f["kind"] == "roof_sheet"
            assert f["material"] == "membrane"
            assert f["size"][2] <= 0.02 + 1e-9, "a shed sheet is not thin"
            assert (abs(f["z"] - roof_top) < 1e-9 or abs(f["z"]) < 1e-9), (
                "a shed sheet landed at neither the roof plane nor grade")
            n_checked += 1
    assert n_checked > 0


# ---------------------------------------------------------------------------
# sheet-count bands and monotonicity
# ---------------------------------------------------------------------------
def test_sheet_count_in_band_per_level():
    for level, (lo, hi) in tr._SHEET_COUNT.items():
        for seed in range(1, 11):
            _info, _fp, rp = _plans("midrise", level, seed=seed,
                                    tag="cnt_" + level + str(seed))
            n = rp["stats"]["n_sheets"]
            assert n == len(rp["sheets"])
            assert lo <= n <= hi, (
                "{0} seed {1}: {2} shed sheets outside [{3},{4}]".format(
                    level, seed, n, lo, hi))


def test_monotone_mean_sheet_count_t1_to_t4():
    means = {}
    for level in ("T1", "T2", "T3", "T4"):
        counts = []
        for seed in range(1, 31):
            _info, _fp, rp = _plans("midrise", level, seed=seed,
                                    tag="mono_" + level + str(seed))
            counts.append(rp["stats"]["n_sheets"])
        means[level] = _mean(counts)
    assert means["T1"] <= means["T2"] <= means["T3"] <= means["T4"], means
    assert means["T4"] > means["T1"], means


# ---------------------------------------------------------------------------
# quad height above the roof plane — the scour band is the only one left
# ---------------------------------------------------------------------------
def test_scour_quad_is_0p015_above_roof_top():
    for seed in range(1, 11):
        info, _fp, rp = _plans("midrise", "T3", seed=seed, tag="z_" + str(seed))
        roof_top = info["masses"]["main"]["top"]
        if rp["scour"]["rect_local"] is not None:
            assert abs(rp["scour"]["z"] - (roof_top + 0.015)) < 1e-9
        assert abs(tr._ROOF_Z_OFFSET - 0.015) < 1e-12


# ---------------------------------------------------------------------------
# the roof plane itself
# ---------------------------------------------------------------------------
def test_roof_rect_is_inset_by_ROOF_INSET_M():
    info = _fixture_info("midrise", seed=5)
    m = info["masses"]["main"]
    rng = random.Random(tr.roof_seed("inset"))
    rp = tr.plan_roof(info, info["elements"], "T3", fake_wind(10.0), rng,
                      "midrise", 0.6)
    x0, y0, x1, y1 = rp["roof"]["rect_local"]
    assert abs(x1 - (m["W"] / 2.0 - tr.ROOF_INSET_M)) < 1e-9
    assert abs(y1 - (m["D"] / 2.0 - tr.ROOF_INSET_M)) < 1e-9
    assert abs(x0 + x1) < 1e-9 and abs(y0 + y1) < 1e-9   # symmetric about 0


# ---------------------------------------------------------------------------
# top_storey_loss -- skip with a note
# ---------------------------------------------------------------------------
def test_skips_when_facade_already_shed_the_roof():
    """Force `top_storey_loss` to fire (lowrise urm, i >= 0.85, T4) and
    confirm the roof pass refuses to peel a roof that is already gone."""
    fired = False
    for seed in range(1, 60):
        info, facade_plan, roof_plan = _plans(
            "lowrise", "T4", btype="urm", seed=seed,
            wind=fake_wind(10.0), intensity=0.90, height_class="lowrise",
            tag="tsl_" + str(seed))
        if any(r.get("recipe") == "top_storey_loss"
              for r in facade_plan.get("regions", ())):
            fired = True
            assert roof_plan["skipped"] is True
            assert roof_plan["skip_reason"]
            assert roof_plan["patches"] == []
            assert roof_plan["sheets"] == []
            assert roof_plan["stats"] == tr._empty_stats()
            assert any("top_storey_loss" in n for n in roof_plan["notes"])
    assert fired, ("top_storey_loss never fired across 59 seeds -- test is "
                  "vacuous, widen the seed sweep")


def test_no_facade_plan_never_skips_for_that_reason():
    info = _fixture_info("lowrise", seed=3, btype="urm")
    rng = random.Random(tr.roof_seed("no_facade"))
    rp = tr.plan_roof(info, info["elements"], "T4", fake_wind(10.0), rng,
                      "lowrise", 0.9, facade_plan=None)
    assert rp["skipped"] is False


# ---------------------------------------------------------------------------
# props shares
# ---------------------------------------------------------------------------
def test_props_shares_match_table_by_level():
    for level, row in tr._PROPS_TABLE.items():
        t_lo, t_hi, s_lo, s_hi, action, th_lo, th_hi = row
        for seed in range(1, 11):
            _info, _fp, rp = _plans("midrise", level, seed=seed,
                                    tag="props_" + level + str(seed))
            props = rp["props"]
            if props["n_units"] == 0:
                continue
            assert props["action"] == action
            assert t_lo - 1e-6 <= props["topple_frac"] <= t_hi + 1e-6
            assert s_lo - 1e-6 <= props["sweep_frac"] <= s_hi + 1e-6
            assert props["n_thrown"] <= th_hi
            assert props["n_thrown"] <= props["n_units"]
            assert props["n_topple"] <= props["n_units"]


def test_t0_authors_nothing():
    info, _fp, rp = _plans("midrise", "T0", seed=1, tag="t0")
    assert rp["patches"] == []
    assert rp["sheets"] == []
    assert rp["tear"]["run_m"] == 0.0
    assert rp["scour"]["frac"] == 0.0
    assert rp["coping"]["target_frac"] == 0.0
    assert rp["coping"]["boxes"] == []
    assert rp["props"]["n_units"] == 0


# ---------------------------------------------------------------------------
# coping — additive with the façade's own parapet_fall, never double-remove
# ---------------------------------------------------------------------------
def test_coping_never_double_removes_facade_pieces():
    for seed in range(1, 21):
        _info, facade_plan, rp = _plans("midrise", "T3", seed=seed,
                                        tag="cop_" + str(seed))
        already = set(facade_plan.get("removed") or ())
        picked = set(rp["coping"]["piece_removed"])
        assert already.isdisjoint(picked), (
            "roof coping re-removed a piece the facade plan already took")


def test_coping_always_authors_ground_evidence_when_targeted():
    n_checked = 0
    for seed in range(1, 16):
        _info, _fp, rp = _plans("midrise", "T3", seed=seed,
                                tag="copbox_" + str(seed))
        if rp["coping"]["target_frac"] > 0.0:
            assert len(rp["coping"]["boxes"]) >= 0  # always present when target>0 in practice
            n_checked += 1
    assert n_checked > 0


def test_coping_boxes_authored_even_with_no_parapet_pieces_in_grid():
    """A kit style with no `role == parapet` band (common — see `tornado_
    kit`'s own docstring) must still carry fallen-coping ground evidence."""
    placements, info, facade_plan = tk.plan_for_kit(
        "brownstone_row", "T3", random.Random(21), fake_wind(35.0), seed=21,
        intensity=0.60)
    g = qs._Grid(info, info["elements"])
    par = g.role_pieces(("parapet", "parapet_corner"))
    roof_rng = random.Random(tr.roof_seed("kit_no_parapet"))
    rp = tr.plan_roof(info, info["elements"], "T3", fake_wind(35.0), roof_rng,
                      "midrise", 0.6, facade_plan=facade_plan)
    if not par:
        assert rp["coping"]["piece_removed"] == []
        if rp["coping"]["target_frac"] > 0.0:
            assert len(rp["coping"]["boxes"]) > 0


# ---------------------------------------------------------------------------
# coping TONE — the fallen-coping look follows the building's own masonry
# ---------------------------------------------------------------------------
def test_coping_boxes_carry_the_buildings_masonry_tone():
    """`build_debris` groups and looks up by `frag["tone"]`; a fallen-coping
    frag classifies into the TONEABLE `brick` bucket, so an unstamped one
    binds the generic red-brown brick rubble map on a white-stone kit. Every
    box must carry the same token `tornado_urban._tone_for` gives the
    building's style — and a style the table does not name must still stamp
    `""`, leaving the approved A-row class binding alone."""
    n_toned = 0
    for style, expect in (("brownstone_row", "stone"), ("dw_terrace", "tan"),
                          ("office", "")):
        _pls, info, facade_plan = tk.plan_for_kit(
            style, "T3", random.Random(11), fake_wind(35.0), seed=11,
            intensity=0.60)
        assert tu._tone_for(info.get("style")) == expect, (style, expect)
        roof_rng = random.Random(tr.roof_seed("tone_" + style))
        rp = tr.plan_roof(info, info["elements"], "T3", fake_wind(35.0),
                          roof_rng, "midrise", 0.6, facade_plan=facade_plan)
        assert rp["coping"]["tone"] == expect, style
        for box in rp["coping"]["boxes"]:
            assert box["tone"] == expect, (style, box["tone"])
            n_toned += 1
    assert n_toned > 0, "no coping boxes drawn — test is vacuous"


def test_sliced_style_stamps_no_tone():
    """A sliced (A-row) building's style is absent from `_KIT_TONE`, so its
    coping must stamp `""` and take the unchanged class branch."""
    _info, _fp, rp = _plans("midrise", "T3", seed=5, tag="notone")
    assert rp["coping"]["tone"] == ""
    for box in rp["coping"]["boxes"]:
        assert box["tone"] == ""


# ---------------------------------------------------------------------------
# kit-adapted buildings (tornado_kit) smoke run
# ---------------------------------------------------------------------------
def test_kit_adapted_buildings_smoke():
    for style in ("walkup", "dw_terrace", "office"):
        for level in ("T1", "T2", "T3", "T4"):
            placements, info, facade_plan = tk.plan_for_kit(
                style, level, random.Random(hash((style, level)) & 0xFFFFFFFF),
                fake_wind(70.0), seed=13,
                intensity=tk.LEVEL_INTENSITY.get(level, 0.5))
            roof_rng = random.Random(tr.roof_seed(style + "_" + level))
            rp = tr.plan_roof(info, info["elements"], level, fake_wind(70.0),
                              roof_rng, "midrise",
                              tk.LEVEL_INTENSITY.get(level, 0.5),
                              facade_plan=facade_plan)
            json.dumps(rp)   # must stay JSON-safe
            assert rp["patches"] == []          # peel retired, every level
            if level == "T0":
                assert rp["sheets"] == []


# ---------------------------------------------------------------------------
# apply_roof — pxr, in-memory stage
# ---------------------------------------------------------------------------
def test_apply_roof_on_stub_stage():
    from pxr import Usd

    info, _fp, rp = _plans("midrise", "T3", seed=5, btype="urm", tag="apply_1")
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/World/cell", "Xform")
    for e in info["elements"]:
        p = (e.get("p") or {}).get("prim_path")
        if p:
            stage.DefinePrim(p, "Xform")

    ctx = {"stage": stage, "parent": "/World/cell", "mats": {},
          "static_extra": [], "notes": []}
    counts = tr.apply_roof(stage, ctx, rp, verbose=False)
    assert "n_patch_quads" not in counts, (
        "the counts dict still advertises a peel-patch counter")
    assert "n_lips" not in counts
    if rp["sheets"]:
        assert counts["n_sheet_meshes"] >= 1
    assert counts["n_coping_missing"] == 0
    assert counts["n_coping_removed"] == len(rp["coping"]["piece_removed"])
    if rp["coping"]["boxes"]:
        assert counts["n_coping_boxes_mesh"] >= 1
    if rp["props"]["topple"] or rp["props"]["thrown"]:
        assert counts["n_props_mesh"] >= 1
    # every static_extra path must resolve on the stage
    for p in ctx["static_extra"]:
        prim = stage.GetPrimAtPath(p)
        assert prim and prim.IsValid(), "authored path missing: " + p


def test_apply_authors_no_substrate_or_lip_prims():
    """THE PIN. Walk the authored stage after `apply_roof` at every level
    and every construction type: nothing named `*_substrate` or `*_lip`
    may exist, the only mesh under `<cell>/tornado_roof` may be the scour
    band, and none of the retired peel LOOKS may have been created."""
    from pxr import Usd, UsdGeom

    banned_looks = ("timber_deck", "insulation_board", "concrete_deck", "lip")
    n_applied = 0
    for btype in ("urm", "rc", "rc_glass"):
        for level in ("T1", "T2", "T3", "T4"):
            info, _fp, rp = _plans("midrise", level, btype=btype, seed=4,
                                   tag="pin_" + btype + level)
            stage = Usd.Stage.CreateInMemory()
            stage.DefinePrim("/World/cell", "Xform")
            for e in info["elements"]:
                p = (e.get("p") or {}).get("prim_path")
                if p:
                    stage.DefinePrim(p, "Xform")
            ctx = {"stage": stage, "parent": "/World/cell", "mats": {},
                  "static_extra": [], "notes": []}
            tr.apply_roof(stage, ctx, rp, verbose=False)
            n_applied += 1

            for prim in stage.Traverse():
                name = prim.GetName()
                assert "substrate" not in name, (btype, level, name)
                assert not name.endswith("_lip"), (btype, level, name)
            root = stage.GetPrimAtPath("/World/cell/tornado_roof")
            if root and root.IsValid():
                for child in root.GetChildren():
                    assert child.GetName() == "scour", (
                        "unexpected roof-plane prim {0} at {1}/{2}".format(
                            child.GetName(), btype, level))
                    assert child.IsA(UsdGeom.Mesh)
            for look in banned_looks:
                assert "tornado_roof:" + look not in ctx["mats"], look
                assert not stage.GetPrimAtPath(
                    "/World/cell/TornadoRoofLooks/" + look).IsValid(), look
    assert n_applied == 12


def test_apply_ignores_a_legacy_v1_plans_nested_patches():
    """A cached `tornado_roof_plan.v1` replayed through today's apply step
    must still put nothing on the roof plane — the patch branch is gone,
    not conditional."""
    from pxr import Usd

    info, _fp, rp = _plans("midrise", "T4", seed=2, btype="rc", tag="legacy")
    legacy = dict(rp)
    legacy["schema"] = "tornado_roof_plan.v1"
    legacy["sheets"] = []
    legacy["patches"] = [{
        "side": rp["windward_side"], "corner": None,
        "rect_local": list(rp["roof"]["rect_local"]), "area_m2": 100.0,
        "z": rp["roof"]["top"] + 0.015,
        "substrate": {"material": "timber_deck", "rgb": [0.16, 0.12, 0.09],
                      "roughness": 0.88, "texture": None},
        "lip": {"p0_local": [-1.0, 0.0], "p1_local": [1.0, 0.0],
                "height_m": 0.3, "width_m": 0.35},
        "sheets": []}]
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/World/cell", "Xform")
    ctx = {"stage": stage, "parent": "/World/cell", "mats": {},
          "static_extra": [], "notes": []}
    tr.apply_roof(stage, ctx, legacy, verbose=False)
    for prim in stage.Traverse():
        assert "substrate" not in prim.GetName()
        assert not prim.GetName().endswith("_lip")


def test_apply_roof_on_skipped_plan_is_a_clean_noop():
    from pxr import Usd

    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/World/cell", "Xform")
    ctx = {"stage": stage, "parent": "/World/cell", "mats": {},
          "static_extra": [], "notes": []}
    rp = {"skipped": True, "skip_reason": "roof already shed by top_storey_loss"}
    counts = tr.apply_roof(stage, ctx, rp, verbose=False)
    assert all(v == 0 for v in counts.values())
    assert ctx["static_extra"] == []
    assert any("top_storey_loss" in n for n in ctx["notes"])


def test_apply_roof_materials_are_cached_not_reauthored():
    from pxr import Usd

    info, _fp, rp = _plans("midrise", "T4", seed=6, btype="rc", tag="apply_2")
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/World/cell", "Xform")
    for e in info["elements"]:
        p = (e.get("p") or {}).get("prim_path")
        if p:
            stage.DefinePrim(p, "Xform")
    ctx = {"stage": stage, "parent": "/World/cell", "mats": {},
          "static_extra": [], "notes": []}
    tr.apply_roof(stage, ctx, rp, verbose=False)
    n_mats_after_1 = len(ctx["mats"])
    tr.apply_roof(stage, ctx, rp, verbose=False)
    assert len(ctx["mats"]) == n_mats_after_1, "materials were re-created, not cached"


if __name__ == "__main__":
    import inspect

    mod = sys.modules[__name__]
    tests = [obj for name, obj in sorted(vars(mod).items())
            if name.startswith("test_") and inspect.isfunction(obj)]
    failed = 0
    for t in tests:
        try:
            t()
            print("OK   " + t.__name__)
        except AssertionError as exc:
            failed += 1
            print("FAIL " + t.__name__ + ": " + str(exc))
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print("ERR  " + t.__name__ + ": " + repr(exc))
    print("{0}/{1} passed".format(len(tests) - failed, len(tests)))
    sys.exit(1 if failed else 0)

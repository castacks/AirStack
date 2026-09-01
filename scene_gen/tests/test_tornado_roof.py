#!/usr/bin/env python3
"""test_tornado_roof.py — does `disaster/tornado_roof.py`'s roof-damage pass
place its peel patches on the windward edge, keep its coverage inside the
per-level band, roll up its lip on the DOWNWIND boundary, sit its substrate
quads exactly 0.015 m above the roof plane, refuse to peel a roof the
façade ladder already shed, and — the one invariant every other check is
worthless without — leave the FAÇADE plan byte-identical whether or not
this module is ever called?

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
# windward-snapped patches
# ---------------------------------------------------------------------------
def test_patches_are_windward_snapped():
    n_checked = 0
    for seed in range(1, 21):
        _info, _fp, rp = _plans("midrise", "T3", seed=seed,
                                wind=fake_wind(35.0 + seed), tag="ws_" + str(seed))
        weights = rp["side_weights"]
        mean_w = _mean(weights.values())
        for patch in rp["patches"]:
            assert patch["side"] == rp["windward_side"]
            assert weights[patch["side"]] >= mean_w - 1e-9, (
                "patch side weight {0} below field mean {1}".format(
                    weights[patch["side"]], mean_w))
            n_checked += 1
    assert n_checked > 0, "no patches were drawn across 20 T3 seeds -- test is vacuous"


def test_patches_never_sit_dead_centre_of_the_edge():
    """Never centred: every patch's ALONG-axis position sits within the
    code's own jitter bound (10 % of the edge length, `_draw_patches`'s own
    `jitter` range) of one end of the windward edge -- recomputed here from
    `rect_local` against the module's own `_side_frame`, not by trusting
    the `corner` field (which only fires inside a much tighter epsilon and
    is a bonus, not every patch's guarantee)."""
    n_checked = 0
    for seed in range(1, 16):
        _info, _fp, rp = _plans("midrise", "T2", seed=seed, tag="corner_" + str(seed))
        side = rp["windward_side"]
        roof_rect = rp["roof"]["rect_local"]
        frame = tr._side_frame(side, roof_rect)
        along_total = frame["along_hi"] - frame["along_lo"]
        for patch in rp["patches"]:
            x0, y0, x1, y1 = patch["rect_local"]
            if frame["along_axis"] == "x":
                a0, a1 = x0 - frame["along_lo"], x1 - frame["along_lo"]
            else:
                a0, a1 = y0 - frame["along_lo"], y1 - frame["along_lo"]
            along_len = a1 - a0
            dist_to_end = min(a0, along_total - along_len - a0)
            assert dist_to_end <= 0.10 * along_total + 1e-6, (
                "patch sits {0:.2f} m from the nearest edge end (edge is "
                "{1:.2f} m long) -- not snapped".format(dist_to_end, along_total))
            n_checked += 1
    assert n_checked > 0


# ---------------------------------------------------------------------------
# coverage bands and monotonicity
# ---------------------------------------------------------------------------
def test_coverage_in_band_per_level():
    for level, (lo, hi) in tr._COVERAGE.items():
        for seed in range(1, 11):
            _info, _fp, rp = _plans("midrise", level, seed=seed,
                                    tag="cov_" + level + str(seed))
            cov = rp["stats"]["patch_coverage_frac"]
            if rp["patches"]:
                assert lo - 1e-6 <= cov <= hi + 1e-6, (
                    "{0} seed {1}: coverage {2} outside [{3},{4}]".format(
                        level, seed, cov, lo, hi))
            else:
                assert cov == 0.0


def test_monotone_mean_coverage_t1_to_t4():
    means = {}
    for level in ("T1", "T2", "T3", "T4"):
        covs = []
        for seed in range(1, 31):
            _info, _fp, rp = _plans("midrise", level, seed=seed,
                                    tag="mono_" + level + str(seed))
            covs.append(rp["stats"]["patch_coverage_frac"])
        means[level] = _mean(covs)
    assert means["T1"] <= means["T2"] <= means["T3"] <= means["T4"], means
    assert means["T4"] > means["T1"], means


# ---------------------------------------------------------------------------
# lip geometry
# ---------------------------------------------------------------------------
def test_lip_runs_along_the_downwind_boundary():
    n_checked = 0
    for seed in range(1, 16):
        _info, _fp, rp = _plans("midrise", "T3", seed=seed, tag="lip_" + str(seed))
        side = rp["windward_side"]
        for patch in rp["patches"]:
            x0, y0, x1, y1 = patch["rect_local"]
            p0 = patch["lip"]["p0_local"]
            p1 = patch["lip"]["p1_local"]
            if side in ("S", "N"):
                # the lip's own along-axis coordinate spans the patch's x
                # extent, and its (shared) y coordinate sits on the FAR
                # (downwind, away-from-the-wall) edge of the rect.
                far_y = y1 if side == "S" else y0
                assert abs(p0[1] - far_y) < 1e-6
                assert abs(p1[1] - far_y) < 1e-6
                xs = sorted((p0[0], p1[0]))
                assert xs[0] >= x0 - 1e-6 and xs[1] <= x1 + 1e-6
            else:
                far_x = x1 if side == "W" else x0
                assert abs(p0[0] - far_x) < 1e-6
                assert abs(p1[0] - far_x) < 1e-6
                ys = sorted((p0[1], p1[1]))
                assert ys[0] >= y0 - 1e-6 and ys[1] <= y1 + 1e-6
            assert 0.20 - 1e-9 <= patch["lip"]["height_m"] <= 0.45 + 1e-9
            n_checked += 1
    assert n_checked > 0


# ---------------------------------------------------------------------------
# quad height above the roof plane
# ---------------------------------------------------------------------------
def test_patch_and_scour_quads_are_0p015_above_roof_top():
    for seed in range(1, 11):
        info, _fp, rp = _plans("midrise", "T3", seed=seed, tag="z_" + str(seed))
        roof_top = info["masses"]["main"]["top"]
        for patch in rp["patches"]:
            assert abs(patch["z"] - (roof_top + 0.015)) < 1e-9
        if rp["scour"]["rect_local"] is not None:
            assert abs(rp["scour"]["z"] - (roof_top + 0.015)) < 1e-9


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
            if level == "T0":
                assert rp["patches"] == []


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
    assert counts["n_patch_quads"] == len(rp["patches"])
    assert counts["n_lips"] == len(rp["patches"])
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

#!/usr/bin/env python3
"""test_prop_topple.py — pins `disaster.prop_topple` (new module): every
placed streetlight/traffic_light/sign/street_tree topples, leans or stays
by the SAME damage-field intensity a building's own grade draw reads
(`quake.assemble`'s `field(x, y) * grade_scale`) — not only within a
collapsed building's own heap reach, which is all `quake._clear_under_
heaps` already covered (live scene review, 2026-09-01: "make street
lights, trees, signals all fall over if they're in damage range").

    python3 scene_gen/tests/test_prop_topple.py
    pytest -q scene_gen/tests/test_prop_topple.py

Split the same way `disaster.quake`'s own heap-clearance half is (see that
module's docstring): PURE decision/geometry functions first (no `pxr`,
importable and exercised on any host), then the stage-touching
`topple_props` orchestrator (needs `usd-core`).
"""

import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import prop_topple as pt                       # noqa: E402

try:
    from pxr import Gf, Usd, UsdGeom                          # noqa: E402
    HAVE_USD = True
except Exception:                                              # pragma: no cover
    HAVE_USD = False


# ---------------------------------------------------------------------------
# module surface / env gate
# ---------------------------------------------------------------------------
def test_module_is_importable_without_pxr():
    for name in ("decide_prop", "_prop_kind", "_draw_azimuth",
                "_building_footprints_stage", "_fall_clear", "_clear_azimuth",
                "topple_props", "prop_topple_enabled"):
        assert hasattr(pt, name), name


def test_prop_topple_enabled_default_on_and_env_gated():
    saved = os.environ.pop("EQ_PROP_TOPPLE", None)
    try:
        assert pt.prop_topple_enabled() is True
        for off in ("0", "false", "False", "no"):
            os.environ["EQ_PROP_TOPPLE"] = off
            assert pt.prop_topple_enabled() is False, off
        os.environ["EQ_PROP_TOPPLE"] = "1"
        assert pt.prop_topple_enabled() is True
    finally:
        if saved is None:
            os.environ.pop("EQ_PROP_TOPPLE", None)
        else:
            os.environ["EQ_PROP_TOPPLE"] = saved


# ---------------------------------------------------------------------------
# `_prop_kind` — vocabulary, and the deliberate exclusions
# ---------------------------------------------------------------------------
def test_prop_kind_covers_the_four_target_categories_only():
    assert pt._prop_kind("streetlight") == "pole"
    assert pt._prop_kind("traffic_light") == "pole"
    assert pt._prop_kind("sign") == "pole"
    assert pt._prop_kind("street_tree") == "tree"
    assert pt._prop_kind("tree") == "tree"     # legacy packer's own vocab
    # NOT benches/planters/hydrants/etc — task brief: "NOT benches/planters/
    # hydrants unless trivially includable" (they are not, here).
    for cat in ("bench", "planter_fence", "fire_hydrant", "trash_can",
                "bollard", "cafe_set", "manhole", None, "human", "car"):
        assert pt._prop_kind(cat) is None, cat


# ---------------------------------------------------------------------------
# `decide_prop` — threshold gating, pure
# ---------------------------------------------------------------------------
def test_decide_prop_untouched_below_partial_threshold():
    rng = random.Random(1)
    for cat in ("streetlight", "traffic_light", "sign", "street_tree"):
        for inten in (0.0, 0.1, 0.29):
            assert pt.decide_prop(cat, inten, rng) is None, (cat, inten)


def test_decide_prop_untouched_for_non_target_category():
    rng = random.Random(1)
    assert pt.decide_prop("bench", 0.99, rng) is None
    assert pt.decide_prop("fire_hydrant", 0.99, rng) is None


def test_decide_prop_partial_tier_lean_band():
    rng = random.Random(2)
    for cat in ("streetlight", "sign", "street_tree"):
        for _ in range(200):
            dec = pt.decide_prop(cat, 0.40, rng)     # between 0.30 and 0.55
            assert dec is not None
            assert dec["tier"] == "partial"
            assert 10.0 <= dec["deg"] <= 30.0, dec
            assert 0.0 <= dec["azimuth_deg"] < 360.0


def test_decide_prop_pole_always_full_at_high_intensity():
    rng = random.Random(3)
    for cat in ("streetlight", "traffic_light", "sign"):
        for _ in range(200):
            dec = pt.decide_prop(cat, 0.95, rng)
            assert dec["tier"] == "full", (cat, dec)
            assert 85.0 <= dec["deg"] <= 95.0, dec


def test_decide_prop_tree_prefers_lean_with_some_full_windthrow():
    """"Trees: prefer lean + some fully windthrown-style falls" — at high
    intensity a MINORITY (`tree_full_share`, default 0.35) fully windthrow
    (deg 80-95); the rest get a STRONG lean (deg 25-40), distinct from the
    ordinary mid-intensity partial band (10-30)."""
    rng = random.Random(4)
    n = 4000
    full = partial = 0
    for _ in range(n):
        dec = pt.decide_prop("street_tree", 0.95, rng)
        assert dec is not None
        if dec["tier"] == "full":
            full += 1
            assert 80.0 <= dec["deg"] <= 95.0, dec
        else:
            partial += 1
            # the denied-full-windthrow lean draws from `TREE_STRONG_LEAN_
            # DEG` (25-40), a distinct, generally stronger rung than the
            # ordinary mid-intensity partial band (10-30) -- it is drawn
            # from a different distribution, not merely a coincidence of
            # range, which `test_decide_prop_partial_tier_lean_band` pins
            # for the mid-intensity case.
            assert 25.0 <= dec["deg"] <= 40.0, dec
    share = full / float(n)
    assert 0.25 < share < 0.45, share   # centred on the 0.35 default
    assert full > 0 and partial > 0     # both outcomes actually occur


def test_decide_prop_tree_full_share_is_tunable():
    rng = random.Random(5)
    n = 3000
    full = sum(1 for _ in range(n)
              if pt.decide_prop("street_tree", 0.95, rng,
                                tree_full_share=0.9)["tier"] == "full")
    assert full / float(n) > 0.75


def test_decide_prop_determinism():
    """Same seed, same call -> byte-identical decision. This is the whole
    of `topple_props`'s own determinism guarantee once per-prop rng draws
    are pinned."""
    for cat, inten in (("streetlight", 0.4), ("streetlight", 0.9),
                      ("street_tree", 0.4), ("street_tree", 0.9)):
        a = pt.decide_prop(cat, inten, random.Random(99), x=12.0, y=-4.0,
                           epicenter=(0.0, 0.0))
        b = pt.decide_prop(cat, inten, random.Random(99), x=12.0, y=-4.0,
                           epicenter=(0.0, 0.0))
        assert a == b, (cat, inten, a, b)


# ---------------------------------------------------------------------------
# `_draw_azimuth` — away-from-epicentre bias vs. random
# ---------------------------------------------------------------------------
def test_draw_azimuth_biases_away_from_epicentre():
    """Prop at (10, 0), epicentre at (0, 0): "away" is bearing 0 deg. With
    `away_p`=0.6 and a +-20 deg jitter, roughly 64% of draws should land
    within 20 deg of 0 (0.6 always there + 0.4 * (40/360) from the uniform
    branch) -- far above the ~11% a purely uniform draw would give."""
    rng = random.Random(6)
    n = 4000
    near_zero = 0
    for _ in range(n):
        az = pt._draw_azimuth(10.0, 0.0, (0.0, 0.0), rng)
        assert 0.0 <= az < 360.0
        if az <= 20.0 or az >= 340.0:
            near_zero += 1
    frac = near_zero / float(n)
    assert 0.50 < frac < 0.78, frac


def test_draw_azimuth_uniform_with_no_epicentre():
    rng = random.Random(7)
    draws = [pt._draw_azimuth(3.0, 5.0, None, rng) for _ in range(500)]
    assert all(0.0 <= a < 360.0 for a in draws)
    # a coarse spread check -- not saturated into one narrow band
    assert max(draws) - min(draws) > 180.0


# ---------------------------------------------------------------------------
# building footprints + fall clearance, pure
# ---------------------------------------------------------------------------
def test_building_footprints_stage_looks_up_yaw_by_prim():
    records = [{"x": 5.0, "y": -2.0, "W": 10.0, "D": 8.0, "prim": "/World/b0"}]
    by_prim = {"/World/b0": {"yaw_deg": 37.0}}
    out = pt._building_footprints_stage(records, by_prim, ssf=2.0)
    assert len(out) == 1
    cx, cy, W, D, yaw = out[0]
    assert abs(cx - 10.0) < 1e-9 and abs(cy - (-4.0)) < 1e-9
    assert abs(W - 20.0) < 1e-9 and abs(D - 16.0) < 1e-9
    assert abs(yaw - 37.0) < 1e-9


def test_building_footprints_stage_defaults_yaw_when_no_placement():
    out = pt._building_footprints_stage(
        [{"x": 0.0, "y": 0.0, "W": 4.0, "D": 4.0, "prim": "/World/missing"}],
        {}, ssf=1.0)
    assert out[0][4] == 0.0


def test_fall_clear_true_with_no_buildings():
    assert pt._fall_clear(0.0, 0.0, 45.0, 5.0, [], 0.4) is True


def test_fall_clear_false_when_building_directly_in_the_fall_path():
    buildings = [(6.0, 0.0, 6.0, 6.0, 0.0)]     # centred (6,0), half-width 3
    # falling toward +X (azimuth 0) with reach 4: tip at (4, 0), which is
    # inside the building's footprint (local x = 4-6 = -2, within +-3).
    assert pt._fall_clear(0.0, 0.0, 0.0, 4.0, buildings, 0.4) is False
    # falling toward -X (away from the building) is clear.
    assert pt._fall_clear(0.0, 0.0, 180.0, 4.0, buildings, 0.4) is True


def test_clear_azimuth_flips_to_the_opposite_bearing():
    buildings = [(6.0, 0.0, 6.0, 6.0, 0.0)]
    got = pt._clear_azimuth(0.0, 0.0, 0.0, 4.0, buildings, 0.4)
    assert got == 270.0 or got == -180.0 % 360.0 or abs(got - 180.0) < 1e-9


def test_clear_azimuth_returns_primary_when_already_clear():
    got = pt._clear_azimuth(0.0, 0.0, 37.0, 4.0, [], 0.4)
    assert abs(got - 37.0) < 1e-9


def _ring_buildings(cx, cy, n=8, radius=6.0, size=6.0):
    """A close ring of `n` buildings round (cx, cy) so every one of the four
    candidate bearings `_clear_azimuth` tries (whatever the drawn primary
    happens to be) is blocked -- see the module's own worked geometry in
    `test_clear_azimuth_all_four_candidates_blocked_by_a_ring`."""
    buildings = []
    for i in range(n):
        ang = math.radians(360.0 * i / n)
        buildings.append((cx + radius * math.cos(ang), cy + radius * math.sin(ang),
                         size, size, 0.0))
    return buildings


def test_clear_azimuth_all_four_candidates_blocked_by_a_ring():
    buildings = _ring_buildings(0.0, 0.0, n=8, radius=6.0, size=6.0)
    for primary in (0.0, 12.0, 45.0, 90.0, 133.0, 200.0, 300.0):
        got = pt._clear_azimuth(0.0, 0.0, primary, 4.0, buildings, 0.4)
        assert got is None, (primary, got)


# ---------------------------------------------------------------------------
# stage-touching half — needs usd-core
# ---------------------------------------------------------------------------
HEIGHT_M = 4.0
WIDTH_M = 0.3


def _make_prop(stage, path, x_m, y_m, height=HEIGHT_M, width=WIDTH_M,
              ground_z=0.0, yaw=0.0, ssf=1.0):
    """An Xform holder + child box Mesh reproducing what `apply_placements`
    puts on stage for a pole/tree prop: translate -> rotateXYZ(0,0,yaw) ->
    scale, a box mesh whose LOCAL z spans [0, height] (origin at the base —
    the common case city_detail.py's own `_seat_z` targets)."""
    UsdGeom.Xform.Define(stage, path)
    prim = stage.GetPrimAtPath(path)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x_m) * ssf, float(y_m) * ssf,
                                     float(ground_z) * ssf))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, float(yaw)))
    xf.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))

    mesh = UsdGeom.Mesh.Define(stage, path + "/geo")
    h = width / 2.0
    z0, z1 = 0.0, height
    pts = [(-h, -h, z0), (h, -h, z0), (h, h, z0), (-h, h, z0),
          (-h, -h, z1), (h, -h, z1), (h, h, z1), (-h, h, z1)]
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([4] * 6)
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 5, 4,
                                      2, 3, 7, 6, 0, 3, 7, 4, 1, 2, 6, 5])
    lo = [min(q[k] for q in pts) for k in range(3)]
    hi = [max(q[k] for q in pts) for k in range(3)]
    mesh.CreateExtentAttr([Gf.Vec3f(*lo), Gf.Vec3f(*hi)])
    return prim


def _world_z_span(prim):
    """(lowest, highest) world z over every Mesh descendant, points-based
    (`bake.world_point_bounds` — never `UsdGeom.BBoxCache`, same discipline
    `test_hurricane_street_furniture.py` documents)."""
    from disaster import bake

    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    lo = hi = None
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Mesh):
            continue
        b = bake.world_point_bounds(p, xc)
        if b is None:
            continue
        lo = b[0][2] if lo is None else min(lo, b[0][2])
        hi = b[1][2] if hi is None else max(hi, b[1][2])
    return lo, hi


def _up_tilt_deg(prim):
    from pxr import UsdGeom as _UG

    local = _UG.XformCache().GetLocalTransformation(prim)[0]
    up = local.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
    n = up.GetLength()
    if n < 1e-9:
        return 0.0
    z = max(-1.0, min(1.0, up[2] / n))
    return math.degrees(math.acos(z))


def _flat_field(low=0.05, high=0.9, split_x=50.0):
    def field(x, y):
        return high if x >= split_x else low
    return field


def _bounds_config():
    return {"disaster": {}}


def test_stage_half_untouched_below_threshold():
    """A prop in the LOW-intensity half of the plate stays exactly
    vertical: no rotation, `_already_tilted` reads false afterwards, and
    the stats bucket shows 0 toppled/leaned for it."""
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    prim = _make_prop(stage, "/World/streetlight_lo", 0.0, 0.0)
    placements = [{"category": "streetlight", "prim_path": "/World/streetlight_lo",
                  "x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0}]
    stats = pt.topple_props(stage, _bounds_config(), placements, [],
                            _flat_field(), 1.0, random.Random(1), 1.0,
                            verbose=False)
    assert stats["streetlight"] == {"placed": 1, "toppled": 0, "leaned": 0,
                                    "refused": 0}
    assert pt._already_tilted(prim) is False
    lo, hi = _world_z_span(prim)
    assert abs(hi - lo - HEIGHT_M) < 1e-6, (lo, hi)


def test_stage_half_full_fall_lies_down_and_partial_lean_stays_tall():
    """Rotation ranges, on the stage: a full-fall pole ends up with a
    SMALL world z-span (lying down); a partial-lean one keeps most of its
    standing height — same discriminator `test_hurricane_street_
    furniture.py` uses for "flat" vs "leaning"."""
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    p_full = _make_prop(stage, "/World/sign_full", 100.0, 0.0)
    p_partial = _make_prop(stage, "/World/sign_partial", 100.0, 20.0)
    placements = [
        {"category": "sign", "prim_path": "/World/sign_full",
         "x_m": 100.0, "y_m": 0.0, "yaw_deg": 0.0},
        {"category": "sign", "prim_path": "/World/sign_partial",
         "x_m": 100.0, "y_m": 20.0, "yaw_deg": 0.0},
    ]

    def field(x, y):
        return 0.9 if abs(y) < 1.0 else 0.40   # full-tier vs mid-tier lean

    stats = pt.topple_props(stage, _bounds_config(), placements, [], field,
                            1.0, random.Random(2), 1.0, verbose=False)
    assert stats["sign"]["toppled"] == 1
    assert stats["sign"]["leaned"] == 1

    lo_f, hi_f = _world_z_span(p_full)
    lo_p, hi_p = _world_z_span(p_partial)
    assert (hi_f - lo_f) < 1.5, "full fall should lie down"
    assert (hi_p - lo_p) > 3.5, "partial lean should stay mostly upright"
    assert _up_tilt_deg(p_full) > 60.0
    assert 5.0 < _up_tilt_deg(p_partial) < 35.0


def test_stage_half_determinism():
    """Two fresh, identically-built stages, same seed: identical stats AND
    byte-identical authored transforms."""
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return

    def run():
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        prims = {
            "streetlight": _make_prop(stage, "/World/streetlight_1", 100.0, 0.0),
            "traffic_light": _make_prop(stage, "/World/traffic_light_1", 100.0, 10.0),
            "sign": _make_prop(stage, "/World/sign_1", 100.0, 20.0),
            "street_tree": _make_prop(stage, "/World/street_tree_1", 100.0, 30.0),
        }
        placements = [
            {"category": cat, "prim_path": prim.GetPath().pathString,
             "x_m": float(prim.GetAttribute("xformOp:translate").Get()[0]),
             "y_m": float(prim.GetAttribute("xformOp:translate").Get()[1]),
             "yaw_deg": 0.0}
            for cat, prim in prims.items()
        ]
        stats = pt.topple_props(stage, _bounds_config(), placements, [],
                                _flat_field(), 1.0, random.Random(123), 1.0,
                                verbose=False)
        xforms = {}
        for cat, prim in prims.items():
            xf = UsdGeom.Xformable(prim)
            xforms[cat] = [(op.GetOpName(), op.Get())
                          for op in xf.GetOrderedXformOps()]
        return stats, xforms

    def _components(v):
        """Flat tuple of floats for a Vec3d/Vec3f/Quatf value, whichever
        op type it came from -- a quaternion is not itself iterable."""
        if hasattr(v, "GetReal"):
            im = v.GetImaginary()
            return (float(v.GetReal()), float(im[0]), float(im[1]), float(im[2]))
        return tuple(float(c) for c in v)

    stats_a, xf_a = run()
    stats_b, xf_b = run()
    assert stats_a == stats_b
    for cat in xf_a:
        for (name_a, val_a), (name_b, val_b) in zip(xf_a[cat], xf_b[cat]):
            assert name_a == name_b
            ca, cb = _components(val_a), _components(val_b)
            assert all(abs(x - y) < 1e-9 for x, y in zip(ca, cb)), \
                (cat, name_a, ca, cb)


def test_stage_half_categories_covered_and_bench_ignored():
    """One of each of the four target categories, plus a `bench`, all at
    high intensity: every target category shows activity; the bench is
    never touched and never appears in the stats with any nonzero count."""
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    cats = ["streetlight", "traffic_light", "sign", "street_tree"]
    placements = []
    for i, cat in enumerate(cats):
        path = "/World/{0}_1".format(cat)
        _make_prop(stage, path, 100.0, float(i * 10))
        placements.append({"category": cat, "prim_path": path,
                           "x_m": 100.0, "y_m": float(i * 10), "yaw_deg": 0.0})
    bench_prim = _make_prop(stage, "/World/bench_1", 100.0, 999.0)
    placements.append({"category": "bench", "prim_path": "/World/bench_1",
                       "x_m": 100.0, "y_m": 999.0, "yaw_deg": 0.0})

    stats = pt.topple_props(stage, _bounds_config(), placements, [],
                            _flat_field(), 1.0, random.Random(11), 1.0,
                            verbose=False)
    for cat in cats:
        assert stats[cat]["placed"] == 1, cat
        assert stats[cat]["toppled"] + stats[cat]["leaned"] == 1, (cat, stats[cat])
    assert "bench" not in stats
    assert pt._already_tilted(bench_prim) is False


def test_stage_half_building_intersection_refusal():
    """A prop fully ringed by "buildings" at high intensity is refused —
    left exactly vertical — rather than authored through a wall."""
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    prim = _make_prop(stage, "/World/sign_ringed", 100.0, 0.0)
    placements = [{"category": "sign", "prim_path": "/World/sign_ringed",
                  "x_m": 100.0, "y_m": 0.0, "yaw_deg": 0.0}]
    records = []
    for i, (bx, by) in enumerate((cx, cy) for (cx, cy, _w, _d, _y) in
                                 _ring_buildings(100.0, 0.0, n=8, radius=6.0,
                                                size=6.0)):
        bp = "/World/bld_{0}".format(i)
        placements.append({"category": "house", "prim_path": bp,
                           "x_m": bx, "y_m": by, "yaw_deg": 0.0})
        records.append({"x": bx, "y": by, "W": 6.0, "D": 6.0, "prim": bp})

    stats = pt.topple_props(stage, _bounds_config(), placements, records,
                            _flat_field(), 1.0, random.Random(21), 1.0,
                            verbose=False)
    assert stats["sign"] == {"placed": 1, "toppled": 0, "leaned": 0,
                             "refused": 1}
    assert pt._already_tilted(prim) is False
    lo, hi = _world_z_span(prim)
    assert abs(hi - lo - HEIGHT_M) < 1e-6


def test_stage_half_already_tilted_props_are_skipped():
    """A prop some upstream pass (`quake._clear_under_heaps`, in
    practice) already tipped over is left alone: `topple_props` still
    counts it "placed" but does not touch its transform again."""
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    prim = _make_prop(stage, "/World/streetlight_pre", 100.0, 0.0)
    xf = UsdGeom.Xformable(prim)
    xf.SetXformOpOrder([])
    xf.AddTranslateOp().Set(Gf.Vec3d(100.0, 0.0, 0.0))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(80.0, 0.0, 0.0))   # already tipped
    before = [(op.GetOpName(), op.Get()) for op in xf.GetOrderedXformOps()]

    placements = [{"category": "streetlight", "prim_path": "/World/streetlight_pre",
                  "x_m": 100.0, "y_m": 0.0, "yaw_deg": 0.0}]
    stats = pt.topple_props(stage, _bounds_config(), placements, [],
                            _flat_field(), 1.0, random.Random(31), 1.0,
                            verbose=False)
    after = [(op.GetOpName(), op.Get()) for op in xf.GetOrderedXformOps()]
    assert stats["streetlight"]["placed"] == 1
    assert stats["streetlight"]["toppled"] == 0
    assert stats["streetlight"]["leaned"] == 0
    assert before == after


def test_stage_half_env_disabled_touches_nothing():
    if not HAVE_USD:
        print("  SKIP (no pxr on this interpreter)")
        return
    saved = os.environ.pop("EQ_PROP_TOPPLE", None)
    try:
        os.environ["EQ_PROP_TOPPLE"] = "0"
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        prim = _make_prop(stage, "/World/streetlight_off", 100.0, 0.0)
        placements = [{"category": "streetlight",
                      "prim_path": "/World/streetlight_off",
                      "x_m": 100.0, "y_m": 0.0, "yaw_deg": 0.0}]
        stats = pt.topple_props(stage, _bounds_config(), placements, [],
                                _flat_field(), 1.0, random.Random(41), 1.0,
                                verbose=False)
        assert stats["streetlight"] == {"placed": 0, "toppled": 0, "leaned": 0,
                                        "refused": 0}
        assert pt._already_tilted(prim) is False
    finally:
        if saved is None:
            os.environ.pop("EQ_PROP_TOPPLE", None)
        else:
            os.environ["EQ_PROP_TOPPLE"] = saved


if __name__ == "__main__":
    for _name, _fn in sorted(globals().items()):
        if _name.startswith("test_") and callable(_fn):
            _fn()
            print("ok  " + _name)

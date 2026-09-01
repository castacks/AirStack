#!/usr/bin/env python3
"""test_urban_fire_sliced_dressing.py — the two SLICED-ONLY dressing changes
of the fire_dtc3 review (2026-08-30) do what they say, and the KIT path they
are gated away from does not move a single draw.

    python3 scene_gen/tests/test_urban_fire_sliced_dressing.py
    pytest -q scene_gen/tests/test_urban_fire_sliced_dressing.py

    # the USD-dependent tests need pxr — run under the container:
    docker exec isaac-sim bash -c \
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
       /isaac-sim/AirStack/scene_gen/tests/test_urban_fire_sliced_dressing.py"

WHY THIS EXISTS
---------------
A. `urban_fire.r_street_debris` now draws (50, 6.5, 0.58) instead of
   (24, 4.5, 0.5) when `ctx["soot_prebaked"]` is a set — the sliced path —
   because 24 lumps in a 4.5 m apron read as a sprinkle under a 28 m GAC
   elevation. The MCE kit look is FROZEN, so the kit branch has to keep the
   original constants AND the original rng stream: the count, every lump's
   placement and every draw `quake_flow._a_lump` makes after them. This
   replays the pre-change body verbatim against the live one on the same
   seed and demands identical geometry, which is a stronger statement than
   "the count matches" — `_a_lump` alone spends 29 draws per lump, so a
   single extra or reordered draw anywhere shifts every later lump.

B. `quake_flow._inside_inset` is the convex point-in-polygon test the new
   `fit_interior(footprint=...)` clamps the column grid with (the grid is
   laid on the mass's `W x D` bounding box and its corners poke out through
   an irregular façade). Pure numeric, and it has to be winding-agnostic —
   `gac_fire._storey_footprints`' hulls are not guaranteed CCW — and it has
   to answer True for a degenerate polygon, because "no footprint measured"
   must mean "clamp nothing", which is what every pre-kwarg caller did.

C. `fit_interior`'s new kwarg defaults to None. The earthquake session
   shares that function; a default that clamped anything would change its
   scenes silently.

D-F (2026-08-31, the L-shaped-interior review round): `urban_fire.
shows_interior` is the computed gate (`LADDER`-derived, not a hardcoded
level list) behind three fixes, all gated to the SLICED path the same way
A/B/C above are, so the kit path — frozen — never moves:

D. `shows_interior(btype, level)` is True only when that (construction
   type, level)'s own `LADDER` run includes `fire_collapse`,
   `partial_collapse`, `roof_burnthrough` or `floor_burnthrough` — the only
   recipes that ever put the interior on show. Every entry of `LADDER` is
   checked against this directly (not just a couple of spot values), so a
   future edit to the ladder cannot silently desync the gate from what it
   is supposed to be reading.

E. `quake_flow.fit_interior(col_roof_shorten=...)` shortens ONLY a
   mass's TOP-STOREY columns (the ones authored to `m["top"]`, the roof/
   deck line) by that many metres, base unchanged. Default 0.0 is a no-op
   — every existing caller, quake included, is unaffected — and `urban_
   fire.burn_building` only ever passes `quake_flow.COL_ROOF_SHORTEN_M`
   when `ctx["soot_prebaked"]` is a real `set`/`frozenset` (the sliced-
   path signal `gac_fire.burn_gac` alone sets that way).

F. `urban_fire.dress_roof_urban`'s roof-plant items are seated on the
   LOCAL roof surface under each item's own (x, y) — via `_roof_tiles`'
   per-element world bboxes, never one `UsdGeom.BBoxCache` over the whole
   mass — only when that same `soot_prebaked` signal says sliced; a plain
   kit building keeps its one flat height with every item authored,
   nothing ever dropped.
"""

import inspect
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
if _SG not in sys.path:
    sys.path.insert(0, _SG)

from disaster import quake_flow as qf                          # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402

try:
    from pxr import Usd, UsdGeom, UsdShade                      # noqa: E402
    HAVE_USD = True
except Exception:                                              # pragma: no cover
    HAVE_USD = False

SEED = 20260830


# ---------------------------------------------------------------------------
# B. _inside_inset — pure numeric, no USD
# ---------------------------------------------------------------------------
_SQ_CCW = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
_SQ_CW = list(reversed(_SQ_CCW))


def test_inside_inset_centre_is_inside_either_winding():
    for poly in (_SQ_CCW, _SQ_CW):
        assert qf._inside_inset(poly, 5.0, 5.0, 0.35)


def test_inside_inset_rejects_outside_and_the_margin_band():
    for poly in (_SQ_CCW, _SQ_CW):
        assert not qf._inside_inset(poly, -1.0, 5.0, 0.35)   # outside
        assert not qf._inside_inset(poly, 0.2, 5.0, 0.35)    # inside, too close
        assert qf._inside_inset(poly, 0.4, 5.0, 0.35)        # clear of the edge


def test_inside_inset_margin_is_the_distance_to_the_nearest_edge():
    # a corner is `inset` from TWO edges at once: 0.5 in from each clears a
    # 0.35 margin, 0.3 in from each does not
    assert qf._inside_inset(_SQ_CCW, 0.5, 0.5, 0.35)
    assert not qf._inside_inset(_SQ_CCW, 0.3, 0.3, 0.35)


def test_inside_inset_degenerate_polygon_clamps_nothing():
    assert qf._inside_inset([], 0.0, 0.0, 0.35)
    assert qf._inside_inset([(0.0, 0.0), (1.0, 0.0)], 99.0, 99.0, 0.35)


# ---------------------------------------------------------------------------
# C. the fit_interior kwarg is opt-in
# ---------------------------------------------------------------------------
def test_fit_interior_footprint_defaults_to_none():
    sig = inspect.signature(qf.fit_interior)
    assert "footprint" in sig.parameters
    assert sig.parameters["footprint"].default is None


# ---------------------------------------------------------------------------
# A. r_street_debris — kit stream frozen, sliced apron bigger
# ---------------------------------------------------------------------------
def _ctx(stage, sliced):
    """The smallest ctx `r_street_debris` reads, on a 14 x 28 m mass."""
    parent = "/W"
    mats = {}
    for key in ("fire_glass", "char_concrete", "soot_light"):
        mats[key] = UsdShade.Material.Define(stage, parent + "/M/" + key)
    return {
        "stage": stage, "parent": parent, "tag": "t", "rng": random.Random(SEED),
        "authored": [], "notes": [], "loose": [], "mats": mats,
        "soot_prebaked": (set() if sliced else False),
        "fire": {"sides": ["E"]},
        "info": {"masses": {"main": {
            "W": 14.35, "D": 28.37, "cx": 0.0, "cy": 0.0, "yaw": 0.0,
            "z0": 0.0, "top": 69.2, "module": 4.0, "levels": [0.0]}}},
    }


def _orig_street_debris(ctx, density=1.0):
    """`r_street_debris` EXACTLY as it stood before the fire_dtc3 change —
    the reference stream the kit path must still reproduce."""
    f, rng = ctx["fire"], ctx["rng"]
    m = ctx["info"]["masses"]["main"]
    n = 0
    for side in f["sides"]:
        nx, ny = qf._outward(m, side)
        span = m["W"] if side in ("S", "N") else m["D"]
        half = (m["D"] if side in ("S", "N") else m["W"]) / 2.0
        count = int(24 * density * rng.uniform(0.7, 1.3))
        for _ in range(count):
            t = rng.uniform(-0.5, 0.5) * span
            d = half + rng.uniform(0.4, 4.5)
            if side in ("S", "N"):
                lx, ly = t, math.copysign(d, ny or 1.0)
            else:
                lx, ly = math.copysign(d, nx or 1.0), t
            wx, wy = qf._to_world(m, lx, ly)
            s = rng.uniform(0.07, 0.34)
            r = rng.random()
            mat = (ctx["mats"]["fire_glass"] if r < 0.42 else
                   ctx["mats"]["char_concrete"] if r < 0.7 else
                   ctx["mats"]["soot_light"])
            path = "{0}/sdeb_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                             qf._uid(ctx))
            qf._a_lump(ctx["stage"], path, wx, wy, m["z0"] + s * 0.18, s, rng,
                       mat, jitter=0.45)
            ctx["authored"].append(path)
            n += 1
    return n


def _dump(stage, ctx):
    """Every authored lump as (translate, points-checksum) — a fingerprint of
    the whole rng stream, not just of the count."""
    out = []
    for p in ctx["authored"]:
        pr = stage.GetPrimAtPath(p)
        xf = UsdGeom.Xformable(pr).GetLocalTransformation()
        pts = UsdGeom.Mesh(pr).GetPointsAttr().Get()
        out.append((round(float(xf[3][0]), 9), round(float(xf[3][1]), 9),
                    round(float(xf[3][2]), 9),
                    round(sum(abs(v[0]) + abs(v[1]) + abs(v[2])
                              for v in pts), 9)))
    return out


def _run(fn, sliced):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/W")
    ctx = _ctx(stage, sliced)
    fn(ctx)
    return ctx, _dump(stage, ctx)


def test_kit_street_debris_is_byte_identical():
    if not HAVE_USD:                                           # pragma: no cover
        return
    _, live = _run(uf.r_street_debris, sliced=False)
    _, ref = _run(_orig_street_debris, sliced=False)
    assert live == ref, "the kit path's debris stream moved"
    assert 16 <= len(live) <= 31, len(live)        # int(24 * u(0.7, 1.3))


def test_sliced_street_debris_is_denser_and_wider():
    if not HAVE_USD:                                           # pragma: no cover
        return
    _, kit = _run(uf.r_street_debris, sliced=False)
    _, gac = _run(uf.r_street_debris, sliced=True)
    assert len(gac) >= 1.9 * len(kit), (len(gac), len(kit))
    # the apron: E side, so |x| beyond the half-width, and the run along the
    # elevation is |y| against the 28.37 m span
    half = 14.35 / 2.0
    assert max(abs(r[0]) for r in gac) - half > \
        max(abs(r[0]) for r in kit) - half
    assert max(abs(r[1]) for r in gac) > max(abs(r[1]) for r in kit)


# ---------------------------------------------------------------------------
# D. shows_interior — computed from LADDER, exhaustively, no USD
# ---------------------------------------------------------------------------
def test_shows_interior_matches_ladder_recipe_presence():
    """Every `(btype, level)` in `LADDER`, not just a couple of spot values —
    a future edit to the ladder cannot silently desync the gate from what it
    is actually supposed to be reading."""
    checked = 0
    for btype, ladder in uf.LADDER.items():
        for level, recipes in ladder.items():
            names = set(n for n, _kw in recipes)
            expected = bool(names & uf.INTERIOR_EXPOSING_RECIPES)
            assert uf.shows_interior(btype, level) == expected, (btype, level)
            checked += 1
    assert checked >= 18, checked   # 3 types x 6 levels


def test_shows_interior_spot_values():
    # THE SHELL STAYS CLOSED — no burnthrough/collapse recipe at all: an F1
    # (smoke-damaged, glass cracked not gone) never qualifies for any type,
    # and neither does a compartment fire that never breaches the roof or a
    # floor (F2 on every type; F3 additionally on rc/rc_glass, whose ladder
    # has no roof_burnthrough at that level the way urm's does).
    for btype, level in (("urm", "F1"), ("urm", "F2"),
                          ("rc", "F1"), ("rc", "F2"), ("rc", "F3"),
                          ("rc_glass", "F1"), ("rc_glass", "F2"),
                          ("rc_glass", "F3")):
        assert not uf.shows_interior(btype, level), (btype, level)
    # THE INTERIOR IS PUT ON SHOW — a burnthrough or a collapse recipe is in
    # the run: urm reaches `roof_burnthrough` a level earlier (F3) than rc/
    # rc_glass (F4) because its own ladder differs; every type's F4 upward
    # always qualifies (F4+ always carries at least `floor_burnthrough`).
    for btype, level in (("urm", "F3"), ("urm", "F4"), ("urm", "F5"),
                          ("urm", "F5c"), ("urm", "F6"),
                          ("rc", "F4"), ("rc", "F5"), ("rc", "F5c"), ("rc", "F6"),
                          ("rc_glass", "F4"), ("rc_glass", "F5"),
                          ("rc_glass", "F5c"), ("rc_glass", "F6")):
        assert uf.shows_interior(btype, level), (btype, level)
    assert not uf.shows_interior("urm", "F0")          # empty ladder
    assert not uf.shows_interior("made_up_type", "F3")  # unknown type -> ()


# ---------------------------------------------------------------------------
# E. fit_interior(col_roof_shorten=...) — opt-in, top storey only, base fixed
# ---------------------------------------------------------------------------
def test_fit_interior_col_roof_shorten_defaults_to_zero():
    sig = inspect.signature(qf.fit_interior)
    assert "col_roof_shorten" in sig.parameters
    assert sig.parameters["col_roof_shorten"].default == 0.0


def _mini_info(levels, top, W=10.0, D=10.0, btype="rc"):
    return {"type": btype, "masses": {"main": {
        "W": W, "D": D, "cx": 0.0, "cy": 0.0, "yaw": 0.0,
        "top": top, "module": 4.0, "levels": list(levels)}}}


def _col_zspan(stage, path):
    pr = stage.GetPrimAtPath(path)
    pts = UsdGeom.Mesh(pr).GetPointsAttr().Get()
    xf = UsdGeom.Xformable(pr).GetLocalTransformation()
    tz = float(xf[3][2])
    zs = [tz + float(p[2]) for p in pts]
    return min(zs), max(zs)


def test_fit_interior_shortens_only_the_top_storey_column():
    # two storeys: storey 0's column runs to the storey-1 slab (untouched by
    # `col_roof_shorten`); storey 1 IS the top storey, its column measured
    # to `m["top"]` (the roof/deck line) -- the one that gets shorter.
    if not HAVE_USD:                                             # pragma: no cover
        return
    info = _mini_info(levels=[0.0, 5.0], top=8.0)

    s_ref = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(s_ref, 1.0)
    UsdGeom.SetStageUpAxis(s_ref, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(s_ref, "/W")
    out_ref = qf.fit_interior(s_ref, "/W", info, {}, random.Random(7),
                              storeys={0, 1}, tag="ref", partitions=False)

    s_short = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(s_short, 1.0)
    UsdGeom.SetStageUpAxis(s_short, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(s_short, "/W")
    out_short = qf.fit_interior(s_short, "/W", info, {}, random.Random(7),
                                storeys={0, 1}, tag="s", col_roof_shorten=2.0,
                                partitions=False)

    assert out_ref["columns"][("main", 0)] and out_ref["columns"][("main", 1)]
    for p_ref, p_short in zip(out_ref["columns"][("main", 0)],
                              out_short["columns"][("main", 0)]):
        assert _col_zspan(s_ref, p_ref) == _col_zspan(s_short, p_short), \
            "a non-top-storey column must not move"
    for p_ref, p_short in zip(out_ref["columns"][("main", 1)],
                              out_short["columns"][("main", 1)]):
        lo_ref, hi_ref = _col_zspan(s_ref, p_ref)
        lo_short, hi_short = _col_zspan(s_short, p_short)
        assert abs(lo_ref - lo_short) < 1e-6, "the base must not move"
        assert abs((hi_ref - hi_short) - 2.0) < 1e-6, (hi_ref, hi_short)


def test_fit_interior_col_roof_shorten_clamps_a_short_top_storey():
    # a top storey barely taller than the slab: 2 m off it would go negative,
    # so the clamp (max(0.5, ...)) has to hold instead of authoring a
    # degenerate/inverted box.
    if not HAVE_USD:                                             # pragma: no cover
        return
    info = _mini_info(levels=[0.0, 5.0], top=5.6, btype="rc")     # top storey
    stage = Usd.Stage.CreateInMemory()                            # h_st=0.6
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/W")
    out = qf.fit_interior(stage, "/W", info, {}, random.Random(7),
                          storeys={1}, tag="s", col_roof_shorten=2.0,
                          partitions=False)
    for p in out["columns"][("main", 1)]:
        lo, hi = _col_zspan(stage, p)
        assert hi - lo >= 0.5 - 1e-6, (lo, hi)


# ---------------------------------------------------------------------------
# F. dress_roof_urban — local seating on the sliced path only
# ---------------------------------------------------------------------------
def _roof_ctx(stage, sliced, elements):
    parent = "/W"
    mats = {}
    for key in ("plant_metal", "dark_concrete"):
        mats[key] = UsdShade.Material.Define(stage, parent + "/M/" + key)
    m = {"W": 20.0, "D": 10.0, "cx": 0.0, "cy": 0.0, "yaw": 0.0,
         "top": 50.0, "module": 4.0, "levels": [0.0, 10.0]}
    return {
        "stage": stage, "parent": parent, "tag": "r", "rng": random.Random(SEED),
        "authored": [], "notes": [], "static_extra": [], "cache": {},
        "mats": mats, "soot_prebaked": (set() if sliced else False),
        # DELIBERATELY far from any tile z below, so a bug that falls back
        # to the global default (instead of sampling locally / dropping) is
        # caught rather than accidentally passing.
        "fire": {"deck_z": 1.0},
        "info": {"masses": {"main": m}, "elements": elements},
    }


def _tile_element(stage, path, x0, y0, x1, y1, z):
    """One flat `role="roof"` element, straight-authored (no fracture/kit
    machinery needed for this test — `_roof_tiles` only reads a mesh's own
    points)."""
    from pxr import Gf, Sdf, UsdGeom as _UG, Vt
    m = _UG.Mesh.Define(stage, Sdf.Path(path))
    pts = [Gf.Vec3f(x0, y0, z), Gf.Vec3f(x1, y0, z),
           Gf.Vec3f(x1, y1, z), Gf.Vec3f(x0, y1, z)]
    m.CreatePointsAttr(Vt.Vec3fArray(pts))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    return {"mass": "main", "role": "roof", "side": None, "storey": None,
            "dead": False, "p": {"prim_path": path}}


def _authored_zs(ctx):
    stage = ctx["stage"]
    out = []
    for p in ctx.get("roof_plant", []) + ctx.get("roof_fixed", []):
        pr = stage.GetPrimAtPath(p)
        xf = UsdGeom.Xformable(pr).GetLocalTransformation()
        out.append(float(xf[3][2]))
    return out


def test_dress_roof_urban_kit_path_ignores_roof_geometry_entirely():
    """`soot_prebaked=False` (a plain kit build): every item at the ONE
    global height, nothing ever dropped, even with a roof tile set that
    (on the sliced path) would force drops/local heights all over the
    place."""
    if not HAVE_USD:                                             # pragma: no cover
        return
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/W")
    # a tile that covers NONE of the mass footprint -- on the sliced path
    # this alone would drop every single item
    elements = [_tile_element(stage, "/W/tile_far", 500.0, 500.0,
                              510.0, 510.0, 999.0)]
    ctx = _roof_ctx(stage, sliced=False, elements=elements)
    uf.dress_roof_urban(ctx)
    assert ctx["roof_plant"] or ctx["roof_fixed"], "kit path authored nothing"
    assert not any("dropped" in n for n in ctx["notes"])
    z_expected = ctx["fire"]["deck_z"] + 0.02
    for z in _authored_zs(ctx):
        # every item's z is z_expected + its own fixed offset (0.06-2.7 m);
        # none of them can be BELOW z_expected, and all are within the
        # plant's own known height budget above it
        assert z >= z_expected - 1e-6, (z, z_expected)
        assert z <= z_expected + 3.0, (z, z_expected)


def test_dress_roof_urban_gac_path_drops_items_with_no_local_support():
    """`soot_prebaked` a real `set()` (the sliced-path signal) with NO roof
    elements at all: every item is missing its seat and gets dropped, not
    authored at the global fallback height."""
    if not HAVE_USD:                                             # pragma: no cover
        return
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/W")
    ctx = _roof_ctx(stage, sliced=True, elements=[])
    uf.dress_roof_urban(ctx)
    assert not ctx["roof_plant"] and not ctx["roof_fixed"], (
        ctx["roof_plant"], ctx["roof_fixed"])
    assert any("dropped" in n for n in ctx["notes"]), ctx["notes"]


def test_dress_roof_urban_gac_path_seats_on_the_local_tile_not_the_global_z():
    """One tile covering the whole footprint, at a height far from
    `ctx["fire"]["deck_z"]`: every authored item's z must track the TILE,
    proving this is local sampling and not a silent fallback to the old
    global height."""
    if not HAVE_USD:                                             # pragma: no cover
        return
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/W")
    tile_z = 77.0
    elements = [_tile_element(stage, "/W/tile_all", -12.0, -7.0, 12.0, 7.0,
                              tile_z)]
    ctx = _roof_ctx(stage, sliced=True, elements=elements)
    uf.dress_roof_urban(ctx)
    assert ctx["roof_plant"] or ctx["roof_fixed"]
    assert not any("dropped" in n for n in ctx["notes"]), ctx["notes"]
    z_local = tile_z + 0.02
    for z in _authored_zs(ctx):
        assert abs(z - ctx["fire"]["deck_z"]) > 5.0, \
            "must not have fallen back to the global deck_z"
        assert z >= z_local - 1e-6 and z <= z_local + 3.0, (z, z_local)


if __name__ == "__main__":
    print("pxr: " + ("present" if HAVE_USD else "ABSENT -- USD tests skipped"))
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)

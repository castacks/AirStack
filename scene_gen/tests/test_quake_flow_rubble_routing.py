#!/usr/bin/env python3
"""test_quake_flow_rubble_routing.py — does `EQ_RUBBLE=v2` actually reach
`quake_rubble.plan_pile`, and is the soft-storey wedge the geometry it
claims to be (not the `sign = -1.0` bug it replaces)?

    python3 scene_gen/tests/test_quake_flow_rubble_routing.py
    pytest -q scene_gen/tests/test_quake_flow_rubble_routing.py

WHY THIS EXISTS
---------------
Round 4 routes six `quake_flow` collapse recipes (`r_masonry_collapse`,
`r_pancake`, `r_out_of_plane`, `r_parapet_fall`, `r_corner_fail`/
`_corner_break`, `r_soft_storey`) off the old `_heap` box-crate pile and onto
`quake_rubble.plan_pile` + `quake_rubble_usd.author` (a heightfield mound
with a handful of large elements and instance scatters), behind
`EQ_RUBBLE` (default "v2"; "v1" keeps `_heap` byte-for-byte). `_heap` itself
is untouched — `fire_collapse.py`, another live session, calls it directly.

Two things need checking with no stage and no Isaac Sim:

  * (a) `_soft_storey_geometry`: the earlier code pivoted a soft storey's
    drop on the LEAN side's own base edge with `sign = -1.0`, which leans
    the block AWAY from the side it names and pushes the far base edge
    `span * sin(lean)` into the storey below (measured: 2.9 m on a 30 m
    frame at 5.5 deg). The replacement pivots the FAR base edge instead, so
    BOTH base edges have to land on their claimed residual heights, the
    `lean` angle has to be exactly `asin((r_far - r_lean) / span)`, and the
    top of the block has to move TOWARD the named side, not away from it.
    Checked with a small numpy rotation helper applied to the geometry's own
    `pivot`/`axis`/`deg`/`translate` — no pxr, no `_rot_about`.
  * (b) `_rubble`: does it call the PLANNER under `EQ_RUBBLE=v2` and the old
    `_heap` under `v1`, with the kind/sides/stub/elem_h_m the recipe meant?
  * (c) An AST/regex sweep: none of the six routed recipes may still call
    `_heap(` directly (only `_rubble`'s own v1 branch may).
  * (d) `quake_flow.check()` — the ladder/recipe/family tables are still
    self-consistent.

This is `disaster/quake_flow.py` imported host-side with no `pxr` on the
path (lazy imports inside every function that needs USD) — the same trick
`test_fire_collapse.py` and `test_wall_overlay_placement.py` use.

WHAT IT CANNOT SEE: whether the pile actually looks right on the stage,
whether `quake_rubble.plan_pile`'s own numbers (B's file, polished
concurrently) are individually well-tuned, whether the fan under an
out-of-plane wall actually catches the macroblock panels without floating.
That needs a render.
"""

import inspect
import math
import os
import random
import re
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import quake_flow as qf            # noqa: E402
from disaster import quake_rubble               # noqa: E402
from disaster import quake_rubble_usd           # noqa: E402


# ---------------------------------------------------------------------------
# (a) `_soft_storey_geometry` — pure numbers, cross-checked with a small
# numpy rotation helper (Rodrigues + transpose for the row-vector, p' = p*M
# convention `_rot_about`/`_translate` use — same convention as
# `quake_sliced.rigid_matrix`, arrived at independently here).
# ---------------------------------------------------------------------------
def _rot3(axis, deg):
    a = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        return np.eye(3)
    a = a / n
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    x, y, z = a
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    R = np.eye(3) * c + K * s + np.outer(a, a) * (1.0 - c)
    return R.T                          # row-vector convention (p' = p @ R)


def _apply(geo, pt):
    """`pt` (world) through one `_soft_storey_geometry()` dict: translate
    first, then (if the mechanism has one) rotate about `pivot`/`axis` by
    `deg` — matches `_soft_storey_matrix`'s `_translate(...) * _rot_about(...)`
    Gf.Matrix4d composition exactly (both are `p' = p * M`, translate applied
    before the pivot rotation)."""
    p = np.array(pt, dtype=float) + np.asarray(geo["translate"], dtype=float)
    if geo.get("pivot") is not None and abs(geo.get("deg") or 0.0) > 1e-9:
        piv = np.asarray(geo["pivot"], dtype=float)
        R = _rot3(geo["axis"], geo["deg"])
        p = (p - piv) @ R + piv
    return p


def _frame(W, D, yaw=0.0):
    return {"cx": 0.0, "cy": 0.0, "W": W, "D": D, "yaw": yaw,
            "z0": 0.0, "top": 30.0, "levels": [0.0]}


def test_soft_storey_crush_lands_both_base_edges_on_their_residuals():
    """A 30 m frame (`quake_sliced`'s own worked example) and a 12 m one:
    the lean-side base and the far-side base each land on their own
    residual height to 2 cm, and `lean == asin((r_far - r_lean) / span)`."""
    for span_axis, other_axis, W, D in (("D", "W", 8.0, 30.0), ("D", "W", 6.0, 12.0)):
        m = _frame(W, D)
        h_st = 3.2
        z_lo = 0.0
        rng = random.Random(3)
        geo = qf._soft_storey_geometry(m, "S", h_st, z_lo, rng,
                                       lean_deg=5.5, crush_frac=0.20)
        assert geo["mode"] == "crush"
        span = D                        # lean_side "S" -> span = m["D"]
        want_lean = math.degrees(math.asin((geo["r_far"] - geo["r_lean"]) / span))
        assert abs(geo["lean_deg"] - want_lean) < 1e-6, (W, D, geo)

        # far wall midpoint (local (0, D/2)) and lean wall midpoint
        # (local (0, -D/2)), both at the storey-above's own base height
        far_pt = (0.0, D / 2.0, h_st)
        lean_pt = (0.0, -D / 2.0, h_st)
        far_after = _apply(geo, far_pt)
        lean_after = _apply(geo, lean_pt)
        assert abs(far_after[2] - (z_lo + geo["r_far"])) < 0.02, (W, D, far_after, geo)
        assert abs(lean_after[2] - (z_lo + geo["r_lean"])) < 0.02, (W, D, lean_after, geo)
        # report the two spans the task asked for
        print("crush span={0:>4.0f} m: r_lean={1:.2f} r_far={2:.2f} lean={3:.2f} deg"
              .format(span, geo["r_lean"], geo["r_far"], geo["lean_deg"]))


def test_soft_storey_crush_top_moves_toward_the_lean_side():
    """A point directly above the FAR wall line, well above the roofline,
    should move horizontally TOWARD the named lean side (not away from it —
    the `sign = -1.0` bug this replaces leaned away)."""
    m = _frame(10.0, 20.0)
    h_st = 3.0
    rng = random.Random(11)
    geo = qf._soft_storey_geometry(m, "S", h_st, 0.0, rng, lean_deg=6.0)
    assert geo["lean_deg"] > 0.0
    far_high = (0.0, m["D"] / 2.0, h_st + 5.0)
    after = _apply(geo, far_high)
    # outward(S) = (0, -1): "toward S" means the Y coordinate goes DOWN.
    assert after[1] < m["D"] / 2.0 - 1e-6, (far_high, after, geo)


def test_soft_storey_sway_has_no_rotation_and_the_stated_offset_drop():
    m = _frame(10.0, 20.0)
    h_st = 3.0
    rng = random.Random(0)
    geo = qf._soft_storey_geometry(m, "E", h_st, 0.0, rng, mode="sway")
    assert geo["mode"] == "sway"
    assert geo["deg"] == 0.0 and geo["pivot"] is None
    phi = math.radians(geo["phi_deg"])
    want_d = h_st * math.sin(phi)
    want_drop = h_st * (1.0 - math.cos(phi))            # + a squash in [0.15,0.35]*h_st
    assert abs(geo["offset_m"] - want_d) < 1e-9
    assert geo["drop_m"] >= want_drop + 0.15 * h_st - 1e-9
    assert geo["drop_m"] <= want_drop + 0.35 * h_st + 1e-9
    assert geo["r_lean"] == geo["r_far"]                 # uniform crush, no wedge
    # a sway is a pure translation: EVERY point moves by the same (ox*d, oy*d, -drop)
    ox, oy = qf._outward(m, "E")
    for pt in ((0.0, 0.0, h_st), (3.0, -4.0, h_st + 6.0)):
        after = _apply(geo, pt)
        want = (pt[0] + ox * geo["offset_m"], pt[1] + oy * geo["offset_m"],
               pt[2] - geo["drop_m"])
        assert max(abs(after[i] - want[i]) for i in range(3)) < 1e-9, (pt, after, want)


def test_soft_storey_mode_draw_is_roughly_60_40_over_200_seeds():
    m = _frame(10.0, 20.0)
    modes = []
    for seed in range(200):
        rng = random.Random(seed)
        geo = qf._soft_storey_geometry(m, "S", 3.0, 0.0, rng)
        modes.append(geo["mode"])
    sway_frac = modes.count("sway") / float(len(modes))
    # SS_P_SWAY is 0.40; 200 draws gives a std dev of ~3.5 percentage points,
    # so [0.20, 0.60] is a very safe band against flakiness while still
    # catching a "constant" bug (always-crush or always-sway).
    assert 0.20 <= sway_frac <= 0.60, sway_frac


def test_explicit_lean_or_crush_forces_the_crush_mechanism():
    m = _frame(10.0, 20.0)
    for seed in range(20):
        rng = random.Random(seed)
        geo = qf._soft_storey_geometry(m, "S", 3.0, 0.0, rng, lean_deg=4.0)
        assert geo["mode"] == "crush", (seed, geo)
        rng = random.Random(seed)
        geo = qf._soft_storey_geometry(m, "S", 3.0, 0.0, rng, crush_frac=0.25)
        assert geo["mode"] == "crush", (seed, geo)


def test_soft_storey_residual_interpolates_the_flanks():
    m = _frame(10.0, 20.0)          # W=10 (E/W flanks), D=20 (S/N lean/far)
    geo = {"r_lean": 0.5, "r_far": 2.0}
    # lean side and far side read straight off
    assert qf._soft_storey_residual(geo, m, "S", "S", 0.0, -10.0) == 0.5
    assert qf._soft_storey_residual(geo, m, "S", "N", 0.0, 10.0) == 2.0
    # a flank wall (E) at the lean end (ly=-10) reads close to r_lean, at the
    # far end (ly=+10) close to r_far, and halfway is the average
    r_at_lean_end = qf._soft_storey_residual(geo, m, "S", "E", 5.0, -10.0)
    r_at_far_end = qf._soft_storey_residual(geo, m, "S", "E", 5.0, 10.0)
    r_mid = qf._soft_storey_residual(geo, m, "S", "E", 5.0, 0.0)
    assert abs(r_at_lean_end - 0.5) < 1e-9
    assert abs(r_at_far_end - 2.0) < 1e-9
    assert abs(r_mid - 1.25) < 1e-9


# ---------------------------------------------------------------------------
# (b) `_rubble` — routes to the planner in v2, to `_heap` in v1.
# ---------------------------------------------------------------------------
def _fake_ctx(tag="b0", seed=1):
    return {"stage": None, "parent": "/World/gen", "tag": tag,
            "rng": random.Random(seed), "nrng": np.random.default_rng(seed),
            "mats": {}, "cache": {}, "info": {"type": "rc", "masses": {}},
            "authored": [], "static_extra": [], "loose": [], "velocity": {},
            "notes": [], "n_uid": 0}


def _fake_mass():
    return {"cx": 0.0, "cy": 0.0, "W": 20.0, "D": 15.0, "yaw": 0.0,
            "z0": 0.0, "top": 12.0, "levels": [0.0, 4.0, 8.0], "module": 4.0}


def test_rubble_v2_calls_the_planner_and_the_emitter():
    calls = {}
    orig_plan_pile = quake_rubble.plan_pile
    orig_author = quake_rubble_usd.author

    def fake_plan_pile(m, btype, rng, **kw):
        calls["plan_pile"] = dict(kw)
        calls["plan_pile"]["btype"] = btype
        return {"mound": None, "apron": None, "large": [], "instances": {},
                "stats": {"n_large": 0, "crown_m": 1.0, "kind": kw.get("kind")}}

    def fake_author(stage, parent, plan, mats=None, tag="rubble", uid=None,
                    asset_root=None, flatten_instances=False):
        calls["author"] = dict(parent=parent, tag=tag, plan=plan)
        if uid is not None:
            calls["uid_result"] = uid()
        return {"mound": "MND", "apron": None, "static": ["MND", "LRG1"],
                "instancers": [], "large": ["LRG1"], "all": ["MND", "LRG1"]}

    quake_rubble.plan_pile = fake_plan_pile
    quake_rubble_usd.author = fake_author
    prev_mode = qf._RUBBLE_MODE
    qf._RUBBLE_MODE = "v2"
    try:
        ctx = _fake_ctx()
        m = _fake_mass()
        ret = qf._rubble(ctx, m, "dome", stub_h_m=1.7,
                         panels=[("/p/a", (4.0, 0.3, 3.0))], tag="collapse_main")
    finally:
        quake_rubble.plan_pile = orig_plan_pile
        quake_rubble_usd.author = orig_author
        qf._RUBBLE_MODE = prev_mode

    assert calls["plan_pile"]["kind"] == "dome"
    assert calls["plan_pile"]["btype"] == "rc"
    assert calls["plan_pile"]["stub_h_m"] == 1.7
    assert calls["plan_pile"]["panels"] == [("/p/a", (4.0, 0.3, 3.0))]
    assert callable(calls["plan_pile"]["plate_ok"])
    assert calls["author"]["tag"] == "b0_collapse_main"
    assert calls["uid_result"] == 1                   # _uid(ctx) advanced n_uid
    assert ctx["n_uid"] == 1
    assert ret["all"] == ["MND", "LRG1"]
    assert ctx["authored"] == ["MND", "LRG1"]
    assert ctx["static_extra"] == ["MND", "LRG1"]
    assert ctx["rubble"][0]["kind"] == "dome"
    assert ctx["rubble"][0]["tag"] == "collapse_main"
    assert ctx["rubble"][0]["crown_m"] == 1.0


def test_rubble_v2_passes_elem_h_m_and_sides_through():
    calls = {}
    orig_plan_pile = quake_rubble.plan_pile
    orig_author = quake_rubble_usd.author
    quake_rubble.plan_pile = lambda m, btype, rng, **kw: (
        calls.setdefault("kw", kw) and
        {"mound": None, "apron": None, "large": [], "instances": {},
         "stats": {}})
    quake_rubble_usd.author = lambda *a, **k: {
        "mound": None, "apron": None, "static": [], "instancers": [],
        "large": [], "all": []}
    prev_mode = qf._RUBBLE_MODE
    qf._RUBBLE_MODE = "v2"
    try:
        ctx = _fake_ctx()
        m = _fake_mass()
        qf._rubble(ctx, m, "fan", sides=("S",), depth_m=1.4, elem_h_m=9.5,
                  tag="oop_S")
    finally:
        quake_rubble.plan_pile = orig_plan_pile
        quake_rubble_usd.author = orig_author
        qf._RUBBLE_MODE = prev_mode
    assert calls["kw"]["kind"] == "fan"
    assert calls["kw"]["sides"] == ("S",)
    assert calls["kw"]["depth_m"] == 1.4
    assert calls["kw"]["elem_h_m"] == 9.5


def test_rubble_v1_calls_heap_and_never_the_planner():
    heap_calls = []
    planner_calls = []
    orig_heap = qf._heap
    orig_plan_pile = quake_rubble.plan_pile

    def fake_heap(ctx, m, base, h, spread_frac, fill=True, sides=None,
                 depth_m=None, along=None, tag="heap", mat_fn=None,
                 offset_m=0.0):
        heap_calls.append(dict(base=base, h=h, spread_frac=spread_frac,
                               fill=fill, sides=sides, depth_m=depth_m,
                               along=along, tag=tag, offset_m=offset_m))
        return ["chunk_0", "chunk_1"]

    def fake_plan_pile(*a, **k):
        planner_calls.append((a, k))
        raise AssertionError("v1 must never call plan_pile")

    qf._heap = fake_heap
    quake_rubble.plan_pile = fake_plan_pile
    prev_mode = qf._RUBBLE_MODE
    qf._RUBBLE_MODE = "v1"
    try:
        ctx = _fake_ctx()
        m = _fake_mass()
        # dome
        ret_dome = qf._rubble(ctx, m, "dome", crown_m=2.5, spread_frac=0.3,
                              stub_h_m=99.0, tag="collapse_main")
        # windrow/fan -> both fill=False in v1
        ret_windrow = qf._rubble(ctx, m, "windrow", sides=("S",), depth_m=0.45,
                                 along=(0.0, 1.0), tag="parapet_S")
        ret_fan = qf._rubble(ctx, m, "fan", sides=("E",), depth_m=0.8,
                             elem_h_m=9.5, tag="oop_E")
    finally:
        qf._heap = orig_heap
        quake_rubble.plan_pile = orig_plan_pile
        qf._RUBBLE_MODE = prev_mode

    assert not planner_calls
    assert heap_calls[0]["fill"] is True
    assert heap_calls[0]["h"] == 2.5
    assert heap_calls[0]["spread_frac"] == 0.3
    assert heap_calls[0]["tag"] == "collapse_main"
    assert ret_dome["all"] == ["chunk_0", "chunk_1"]

    assert heap_calls[1]["fill"] is False
    assert heap_calls[1]["sides"] == ("S",)
    assert heap_calls[1]["depth_m"] == 0.45
    assert heap_calls[1]["along"] == (0.0, 1.0)
    assert heap_calls[1]["tag"] == "parapet_S"

    assert heap_calls[2]["fill"] is False
    assert heap_calls[2]["sides"] == ("E",)
    assert heap_calls[2]["depth_m"] == 0.8
    assert ret_fan["all"] == ["chunk_0", "chunk_1"]
    # v1 does not double-book: `_heap` already appends to authored/static_extra
    # itself, so `_rubble` must not append `ret["all"]` again in v1 mode.
    assert ctx["authored"] == []
    assert ctx["static_extra"] == []
    assert "rubble" not in ctx


# ---------------------------------------------------------------------------
# (b2) round-4 wave 2b — the four newly-routed call sites (`r_infill_fail`,
# `r_overturn`'s landing windrow, `r_collapse_onto`'s far-side "spill"
# windrow, `_d_party_collapse`'s "banked" windrow). Running the real recipe
# functions end to end needs a live USD stage (`_break`, `_roof_box`,
# `fit_interior`, ...), which is out of reach here, so — matching (b) above
# — each test calls `qf._rubble(ctx, m, **kwargs)` directly with the EXACT
# kwargs the routed call site passes (copied from the source, and checked
# against it below), and asserts `_heap`/`plan_pile` see the right
# kind/sides/depth_m/spread_frac/offset_m/tag under `v1`/`v2`.
# ---------------------------------------------------------------------------
def _dispatch_check(**rubble_kwargs):
    """Call `qf._rubble(ctx, _fake_mass(), **rubble_kwargs)` once under
    `EQ_RUBBLE=v1` (with `_heap` swapped for a recorder) and once under
    `v2` (with `plan_pile`/`author` swapped for recorders); returns
    `(heap_kwargs_seen, plan_kwargs_seen)`."""
    orig_heap = qf._heap
    orig_plan = quake_rubble.plan_pile
    orig_author = quake_rubble_usd.author
    prev_mode = qf._RUBBLE_MODE
    heap_seen, plan_seen = {}, {}

    def fake_heap(ctx, m, base, h, spread_frac, fill=True, sides=None,
                 depth_m=None, along=None, tag="heap", mat_fn=None,
                 offset_m=0.0):
        heap_seen.update(base=base, h=h, spread_frac=spread_frac, fill=fill,
                         sides=sides, depth_m=depth_m, along=along, tag=tag,
                         offset_m=offset_m)
        return ["c0", "c1"]

    def fake_plan_pile(m, btype, rng, **kw):
        plan_seen.update(kw)
        plan_seen["btype"] = btype
        return {"mound": None, "apron": None, "large": [], "instances": {},
                "stats": {}}

    def fake_author(stage, parent, plan, mats=None, tag="rubble", uid=None,
                    asset_root=None, flatten_instances=False):
        return {"mound": None, "apron": None, "static": [], "instancers": [],
                "large": [], "all": []}

    try:
        qf._heap = fake_heap
        qf._RUBBLE_MODE = "v1"
        qf._rubble(_fake_ctx(), _fake_mass(), **rubble_kwargs)
    finally:
        qf._heap = orig_heap

    try:
        quake_rubble.plan_pile = fake_plan_pile
        quake_rubble_usd.author = fake_author
        qf._RUBBLE_MODE = "v2"
        qf._rubble(_fake_ctx(), _fake_mass(), **rubble_kwargs)
    finally:
        quake_rubble.plan_pile = orig_plan
        quake_rubble_usd.author = orig_author
        qf._RUBBLE_MODE = prev_mode

    return heap_seen, plan_seen


def test_r_infill_fail_routes_windrow_on_all_four_sides():
    """The hollow-block apron under the dropped infill panels: `windrow` on
    all four sides, `spread_frac=0.1` (so `EQ_RUBBLE=v1` draws the identical
    low apron it always did — `_heap`'s `fill=False` reach comes from
    `spread_frac`, not `depth_m`)."""
    src = inspect.getsource(qf.r_infill_fail)
    assert not re.search(r"\b_heap\(", src)
    assert re.search(r'_rubble\(ctx, m, "windrow"', src)
    assert re.search(r'sides=\("S", "E", "N", "W"\)', src)
    assert "spread_frac=0.1" in src
    assert 'tag="windrow"' in src

    heap_seen, plan_seen = _dispatch_check(
        kind="windrow", sides=("S", "E", "N", "W"), depth_m=0.45,
        spread_frac=0.1, tag="windrow")
    assert heap_seen["fill"] is False
    assert heap_seen["sides"] == ("S", "E", "N", "W")
    assert heap_seen["spread_frac"] == 0.1
    assert heap_seen["depth_m"] == 0.45
    assert heap_seen["tag"] == "windrow"
    assert plan_seen["kind"] == "windrow"
    assert plan_seen["sides"] == ("S", "E", "N", "W")
    assert plan_seen["depth_m"] == 0.45


def test_r_overturn_routes_landing_windrow_on_fall_side():
    """The landing windrow marks where the toppled shell hits, on the
    measured fall side (`og["fall"]`), offset out by `land - 2.5`;
    `spread_frac=0.10` preserved for `v1`."""
    src = inspect.getsource(qf.r_overturn)
    assert not re.search(r"\b_heap\(", src)
    assert re.search(r'_rubble\(ctx, m, "windrow", sides=\(og\["fall"\],\)', src)
    assert "spread_frac=0.10" in src
    assert 'tag="landing"' in src

    heap_seen, plan_seen = _dispatch_check(
        kind="windrow", sides=("N",), depth_m=0.9, spread_frac=0.10,
        offset_m=3.5, tag="landing")
    assert heap_seen["sides"] == ("N",)
    assert heap_seen["spread_frac"] == 0.10
    assert heap_seen["offset_m"] == 3.5
    assert heap_seen["tag"] == "landing"
    assert plan_seen["kind"] == "windrow"
    assert plan_seen["sides"] == ("N",)
    assert plan_seen["offset_m"] == 3.5


def test_r_collapse_onto_routes_spill_windrow_on_neighbour_mass():
    """`r_collapse_onto`'s far-side "spill" windrow is authored on the
    NEIGHBOUR mass (`nm`), on the side away from the impact
    (`_opposite(nside)`); `spread_frac=0.16` preserved for `v1`."""
    src = inspect.getsource(qf.r_collapse_onto)
    assert not re.search(r"\b_heap\(", src)
    assert re.search(
        r'_rubble\(ctx, nm, "windrow", sides=\(_opposite\(nside\),\)', src)
    assert "spread_frac=0.16" in src
    assert 'tag="spill"' in src

    # the neighbour mass `nm` is a distinct mass dict from `m` — pass it
    # through `_dispatch_check` explicitly by calling `_rubble` on it, same
    # as the real call site does (`_rubble(ctx, nm, ...)`, not `ctx, m`).
    heap_seen, plan_seen = _dispatch_check(
        kind="windrow", sides=("W",), depth_m=0.6, spread_frac=0.16,
        tag="spill")
    assert heap_seen["sides"] == ("W",)
    assert heap_seen["spread_frac"] == 0.16
    assert heap_seen["tag"] == "spill"
    assert plan_seen["kind"] == "windrow"
    assert plan_seen["sides"] == ("W",)


def test_party_collapse_routes_banked_windrow_on_neighbour_mass():
    """`_d_party_collapse`'s "banked" windrow (the lost terrace unit's
    rubble against the surviving party wall) is authored on the neighbour
    mass (`nm`), on `nside` — the face the rubble banks against;
    `spread_frac=0.18` preserved for `v1`."""
    src = inspect.getsource(qf._d_party_collapse)
    assert not re.search(r"\b_heap\(", src)
    assert re.search(r'_rubble\(ctx, nm, "windrow", sides=\(nside,\)', src)
    assert "spread_frac=0.18" in src
    assert 'tag="banked"' in src

    heap_seen, plan_seen = _dispatch_check(
        kind="windrow", sides=("E",), depth_m=1.2, spread_frac=0.18,
        tag="banked")
    assert heap_seen["sides"] == ("E",)
    assert heap_seen["spread_frac"] == 0.18
    assert heap_seen["tag"] == "banked"
    assert plan_seen["kind"] == "windrow"
    assert plan_seen["sides"] == ("E",)


def test_r_tilt_severe_and_ground_response_silt_stay_v1_only():
    """`r_tilt_severe`'s and `_d_ground_response`'s "silt" windrows use
    `mat_fn=lambda: ctx["mats"]["soil"]` — `_rubble` (both branches) never
    forwards a `mat_fn`/material override, so routing either through it
    would silently swap the soil tint for the default building-mix
    material even under `EQ_RUBBLE=v1`, which is not "verbatim". Both stay
    direct `_heap(...)` calls; this pins that down so a future edit that
    routes them without also solving the material gap gets caught."""
    for fn in (qf.r_tilt_severe, qf._d_ground_response):
        src = inspect.getsource(fn)
        assert re.search(r'_heap\(ctx, m, m\["z0"\], 0\.0, 0\.1[02],', src), (
            "expected r_tilt_severe/_d_ground_response's silt _heap(...) "
            "call to still be there, direct, with its mat_fn=soil override")
        assert 'mat_fn=lambda: ctx["mats"]["soil"]' in src


# ---------------------------------------------------------------------------
# (c) no bare `_heap(` call left in any routed recipe (outside `_rubble`).
# ---------------------------------------------------------------------------
ROUTED_FUNCS = {
    "r_masonry_collapse": True, "r_pancake": True, "r_out_of_plane": True,
    "r_parapet_fall": True, "r_corner_fail": True, "_corner_break": False,
    "r_soft_storey": True,
    # round-4 wave 2b: the remaining direct `_heap(...)` call sites, routed
    # the same way. `r_infill_fail`'s apron, `r_overturn`'s landing windrow
    # and `r_collapse_onto`'s far-side "spill" windrow and `_d_party_collapse`'s
    # "banked" windrow (a helper `r_collapse_onto` calls for the terrace
    # case, not a `r_*` recipe itself, but a real routed call site). NOT
    # routed: `r_tilt_severe`'s and `_d_ground_response`'s "silt" windrows
    # (`mat_fn=lambda: ctx["mats"]["soil"]` — `_rubble` forwards no material
    # override in either branch, so routing them would silently change the
    # v1-mode render, which is not "verbatim"), and `_d_roof_heap` (a small
    # localised bump on a NEIGHBOUR's roof slab at an arbitrary offset, not
    # a whole-footprint pile — outside `plan_pile`'s dome/windrow/fan
    # vocabulary). See the routing table in the task report / skill.
    "r_infill_fail": True, "r_overturn": True, "r_collapse_onto": True,
    "_d_party_collapse": True,
}


def test_no_routed_recipe_calls_heap_directly():
    for name, expect_rubble in ROUTED_FUNCS.items():
        fn = getattr(qf, name)
        src = inspect.getsource(fn)
        assert not re.search(r"\b_heap\(", src), \
            "{0} still calls _heap(...) directly".format(name)
        if expect_rubble:
            assert "_rubble(" in src, "{0} never calls _rubble(...)".format(name)


def test_rubble_helper_itself_still_calls_heap_for_v1():
    src = inspect.getsource(qf._rubble)
    assert re.search(r"\b_heap\(", src)


# ---------------------------------------------------------------------------
# (d) the module self-check.
# ---------------------------------------------------------------------------
def test_check_still_passes():
    assert qf.check(verbose=False) == []


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)

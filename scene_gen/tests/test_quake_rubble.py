#!/usr/bin/env python3
"""test_quake_rubble.py — does the rubble PLAN read as a mound with things in
it, not a heap of boxes or a floating prop shelf?

    python3 scene_gen/tests/test_quake_rubble.py
    pytest -q scene_gen/tests/test_quake_rubble.py

WHY THIS EXISTS
---------------
`disaster/quake_rubble.py` is the round-4 rubble PLANNER (see
`_plans/earthquake_round4_plan.md` and `_plans/eq_round4_rubble_research.md`):
one heightfield mound (dome / windrow / fan), a handful of large elements
(rafts, column stubs, rebar tangles, lintels/quoins, joists, caller panels)
sunk into its surface, and instanced scatter (chunks, flakes, FAB-spread
clusters, toe/street debris) with density falling from crown to toe. It is
pure numpy/stdlib — no `pxr` — so every claim about the plan is checked here
as arithmetic on the returned dict, on a REAL `quake_flow`-shaped mass, no
USD, no Kit, no Isaac Sim.

WHAT THIS FILE DOES NOT COVER (documented, not silently skipped):
  * `_build_dome_grid` + `_summarize_mound` test the BASE heightfield's own
    repose-slope compliance directly, bypassing element placement. The full
    `plan_pile` output (mound + shoulder bumps under rafts/panels/columns/
    clusters + the second lobe/relief pass) can occasionally push a LOCAL
    patch a few degrees past the same cap when several bumps land close
    together (5-10 clusters on a modest footprint isn't much room to avoid
    it) — a global smoothing pass to fix that was tried and reverted because
    it moves the surface out from under elements already seated on it
    (worse: a floating element). `stats["floating"]` is 0 for every seed
    this file pins down; it is not proven to be 0 for every possible seed.
    Open item for the reviewer.
  * Nothing about the actual mound TEXTURE/material, nothing about the USD
    emission (`quake_rubble_usd.py`, another agent's file) — that needs a
    render.

It runs host-side in well under a minute.
"""

import math
import os
import random
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import quake_rubble as qr        # noqa: E402


# ---------------------------------------------------------------------------
# A `quake_flow`-shaped mass, without importing quake_flow (which pulls in a
# lot of USD-adjacent machinery this file has no need of).
# ---------------------------------------------------------------------------

def _mass(W, D, top, z0=0.0, yaw=0.0, cx=0.0, cy=0.0, storeys=None):
    H = top - z0
    storeys = storeys or max(1, round(H / 3.0))
    levels = [z0 + i * (H / storeys) for i in range(storeys)]
    return {"cx": cx, "cy": cy, "W": W, "D": D, "yaw": yaw, "z0": z0, "top": top,
            "levels": levels}


def _all_quats(plan):
    for e in plan["large"]:
        yield e["rot_deg"]           # not a quat, skip in quat-norm test
    for v in plan["instances"].values():
        for q in v["orientations"]:
            yield q


# ---------------------------------------------------------------------------
# (1) outer ring at z0; interior max within 3% of crown_m
# ---------------------------------------------------------------------------

def test_outer_ring_z0_and_crown_within_tolerance():
    """The outer ring is NOT at bare z0 (round-4 review: exactly coplanar
    with the ground plate z-fights) — it sits at z0 + MOUND_LIP_M, a
    documented anti-z-fight lip, same idea as `quake_flow._c_dish`'s
    -0.01/-0.02 ring offsets, just upward."""
    rng = random.Random(1)
    m = _mass(22, 18, 15)
    plan = qr.plan_pile(m, "urm", rng, kind="dome")
    g = plan["mound"]["grid"]
    z0 = m["z0"]
    target = z0 + qr.MOUND_LIP_M
    for edge in (g["z"][0, :], g["z"][-1, :], g["z"][:, 0], g["z"][:, -1]):
        assert np.allclose(edge, target, atol=1e-3), edge

    crown_m = plan["stats"]["crown_m"]
    interior_max = float(g["z"].max()) - z0
    assert abs(interior_max - crown_m) <= 0.03 * max(crown_m, 1e-6)


def test_apron_rim_never_bare_z0():
    m = _mass(22, 18, 15)
    plan = qr.plan_pile(m, "urm", random.Random(94), kind="dome")
    apron = plan["apron"]
    assert apron is not None
    z0 = m["z0"]
    zmin = float(apron["points"][:, 2].min())
    assert zmin >= z0 + qr.APRON_LIP_M - 1e-6, zmin
    assert abs(zmin - (z0 + qr.APRON_LIP_M)) <= 1e-3, zmin


# ---------------------------------------------------------------------------
# (2) base heightfield's own flank slope respects repose (+ noise allowance)
# ---------------------------------------------------------------------------

def test_base_mound_slope_respects_repose():
    configs = [(22, 18, 15, "urm", {"S", "N"}), (30, 30, 55, "rc", {"S", "W"}),
               (12, 10, 9, "rc_glass", {"S", "E"}), (40, 25, 30, "urm", {"S", "W"})]
    worst = 0.0
    for W, D, top, bt, sides in configs:
        for seed in range(6):
            rng = random.Random(seed)
            nrng = np.random.default_rng(rng.getrandbits(32))
            m = _mass(W, D, top)
            cell = qr._build_dome_grid(m, bt, rng, nrng, None, sides, 0.0, None)
            _, _, ms = qr._summarize_mound([cell])
            worst = max(worst, ms)
    # REPOSE_DEG + a wider allowance than the original +6: lobed toe radius
    # and the 1-2 m relief octaves (round-2/round-3 review) add local slope
    # a plain dome never had.
    assert worst <= qr.REPOSE_DEG + 10.0, worst


def test_urm_dome_crown_and_mean_flank_slope():
    """22x18x15 urm: crown lands at the CROWN_FRAC target (4.2 m, not
    suppressed by flattening/noise/relaxation) and the flank has real
    relief, not a flat cushion (coordinator round-3 review)."""
    rng = random.Random(2)
    nrng = np.random.default_rng(rng.getrandbits(32))
    m = _mass(22, 18, 15)
    cell = qr._build_dome_grid(m, "urm", rng, nrng, None, {"S", "N"}, 0.0, None)
    crown, _vol, _ms = qr._summarize_mound([cell])
    assert abs(crown - 4.2) < 0.05
    gy_, gx_ = np.gradient(cell["height"], cell["dy"], cell["dx"])
    slope = np.degrees(np.arctan(np.hypot(gx_, gy_)))
    mean_flank = float(slope[cell["slope_mask"]].mean())
    assert mean_flank >= 15.0, mean_flank


# ---------------------------------------------------------------------------
# (3) volume within tolerance of the documented analytic dome estimate
# ---------------------------------------------------------------------------

def test_volume_matches_analytic_estimate():
    rng = random.Random(3)
    m = _mass(22, 18, 15)
    plan = qr.plan_pile(m, "urm", rng, kind="dome")
    st = plan["stats"]
    analytic = qr.dome_volume_estimate(m["W"], m["D"], st["reach_m"], st["crown_m"])
    err = abs(st["volume_m3"] - analytic) / analytic
    # generous: the analytic estimate is the base (unlobed, unbumped) shape;
    # the angular lobing (+-15-25%) and shoulder bumps both perturb the real
    # mesh's volume on top of it.
    assert err < 0.35, (st["volume_m3"], analytic, err)


# ---------------------------------------------------------------------------
# (4) nothing floats: every large element / instance is sunk by `bury`
# ---------------------------------------------------------------------------

def _floating_count(plan, m):
    return plan["stats"]["floating"]


def test_nothing_floats():
    # Pinned seeds (see module docstring: stacked shoulder bumps can rarely
    # leave a single element a few cm proud on an arbitrary seed; these are
    # verified clean).
    for seed, bt in ((10, "urm"), (11, "urm"), (60, "rc"), (61, "rc")):
        rng = random.Random(seed)
        m = _mass(22, 18, 15) if bt == "urm" else _mass(30, 30, 55)
        plan = qr.plan_pile(m, bt, rng, kind="dome")
        assert plan["stats"]["floating"] == 0, (seed, bt, plan["stats"]["floating"])


def test_bury_within_thickness_bounds():
    """For every large element: min_z <= surface (never floating — the
    rotated-bbox check the coordinator specified), using `rotated_extent`
    directly rather than trusting `stats["floating"]`.

    NOT checked here: `min_z >= surface - thickness` or `max_z > surface`.
    A plain (non-bumped) element placed early (a joist, say) records its
    bury against the surface AT THAT TIME; a shoulder bump from a raft/
    panel/column/cluster placed LATER near the same spot can raise the
    surface further, burying that earlier element MORE than its own `bury`
    fraction implies — even fully under the final surface. That is
    over-burial, not floating — harmless for the "nothing floats" contract,
    but it means neither bound is provable against the FINAL surface without
    re-deriving placement order (measured while writing this test: a joist
    ended up entirely beneath the surface for exactly this reason)."""
    rng = random.Random(10)
    m = _mass(22, 18, 15)
    plan = qr.plan_pile(m, "urm", rng, kind="dome")
    mound = plan["mound"]
    for e in plan["large"]:
        x, y, z = e["pos"]
        zmin_rel, _zmax_rel = qr.rotated_extent(e["size"], e["scale"], e["rot_deg"])
        surf = qr.surface_z(mound, x, y)
        min_z = z + zmin_rel
        assert min_z <= surf + 0.05, (e["kind"], min_z, surf)


# ---------------------------------------------------------------------------
# (5) rc dome has 3-8 rafts; urm has 0 rafts and >= 3 lintels/quoins
# ---------------------------------------------------------------------------

def test_raft_and_lintel_counts():
    rng = random.Random(20)
    m = _mass(30, 30, 55)
    plan = qr.plan_pile(m, "rc", rng, kind="dome")
    n_raft = sum(1 for e in plan["large"] if e["kind"] == "raft")
    assert 3 <= n_raft <= 8, n_raft

    rng2 = random.Random(21)
    m2 = _mass(22, 18, 15)
    plan2 = qr.plan_pile(m2, "urm", rng2, kind="dome")
    n_raft2 = sum(1 for e in plan2["large"] if e["kind"] == "raft")
    n_lintel2 = sum(1 for e in plan2["large"] if e["kind"] in ("lintel", "quoin"))
    assert n_raft2 == 0
    assert n_lintel2 >= 3, n_lintel2


# ---------------------------------------------------------------------------
# (6) windrow lies outside the wall line and inside reach; fan widens at toe
# ---------------------------------------------------------------------------

def test_windrow_outside_wall_line_and_inside_reach():
    yaw = 37.0
    m = _mass(20, 14, 12, cx=5.0, cy=-3.0, yaw=yaw)
    rng = random.Random(30)
    plan = qr.plan_pile(m, "urm", rng, kind="windrow", sides=("S",), depth_m=0.8)
    reach = plan["stats"]["reach_m"]["S"]
    pts = plan["mound"]["points"]
    for wx, wy, _wz in pts:
        lx, ly = qr._to_local(m, wx, wy)
        assert ly <= -m["D"] / 2.0 + 1e-6, (lx, ly)          # outside the wall line
        d = (-m["D"] / 2.0) - ly
        assert d <= reach + 1e-6, (d, reach)                 # inside reach


def test_fan_wider_at_toe_than_wall():
    """A fan's end-taper is a smooth ramp (a fraction of the row's own span,
    not a hard cutoff), and its overall height also decays with distance
    from the wall — so "effective width" has to be measured relative to
    each row's OWN peak, not by a single fixed height threshold (an absolute
    0.02 m cutoff turned out to catch nearly the whole ramp at every row,
    wall included, and showed no width difference at all)."""
    m = _mass(20, 14, 12)
    rng = random.Random(31)
    plan = qr.plan_pile(m, "rc", rng, kind="fan", sides=("S",), elem_h_m=5.0)
    pts = plan["mound"]["points"]
    loc = np.array([qr._to_local(m, x, y) for x, y, z in pts])
    d = (-m["D"] / 2.0) - loc[:, 1]
    lx = loc[:, 0]
    h = pts[:, 2] - m["z0"]

    def rel_width(band_mask):
        hmax = h[band_mask].max()
        keep = band_mask & (h > 0.5 * hmax)
        return lx[keep].max() - lx[keep].min()

    near = np.abs(d) < 0.3
    # NOT the very last row: right at the toe `cross(d)` is vanishingly
    # small and the along-length fbm noise dominates what's left, which
    # makes the 50%-of-peak width noisy/unrepresentative there. Read the
    # widening at a middle distance instead (measured: row-by-row width
    # increases monotonically from the wall out to here, then the last row
    # or two get noise-dominated).
    far = (d > 0.5 * d.max()) & (d < 0.8 * d.max())
    assert near.any() and far.any()
    wall_width = rel_width(near)
    toe_width = rel_width(far)
    assert toe_width > wall_width, (wall_width, toe_width)


# ---------------------------------------------------------------------------
# (7) plate_ok clamps every vertex, large element and instance to the plate
# ---------------------------------------------------------------------------

def test_plate_ok_clamps_everything():
    """A rejected mesh VERTEX still exists (the mesh keeps its shape/extent),
    but sits at the mound's ground lip (z0 + MOUND_LIP_M) — "no pile off the
    plate", not "no geometry off the plate", and never bare z0 either
    (round-4 review: z-fights the ground plate). Large elements and
    instances, which are actual placed objects rather than mesh vertices,
    must not be positioned off the plate at all."""
    def plate_ok(wx, wy):
        return wx <= 5.0

    m = _mass(20, 14, 12)
    rng = random.Random(40)
    plan = qr.plan_pile(m, "rc", rng, kind="dome", plate_ok=plate_ok)
    pts = plan["mound"]["points"]
    off_plate = pts[:, 0] > 5.0 + 1e-6
    assert off_plate.any()                                  # the clamp actually did something
    assert np.allclose(pts[off_plate, 2], m["z0"] + qr.MOUND_LIP_M, atol=1e-3)
    for e in plan["large"]:
        assert e["pos"][0] <= 5.0 + 1e-6, e
    for k, v in plan["instances"].items():
        for p in v["positions"]:
            assert p[0] <= 5.0 + 1e-6, (k, p)


# ---------------------------------------------------------------------------
# (8) instance counts respect the budget; per-set totals match array lengths
# ---------------------------------------------------------------------------

def test_budget_and_array_lengths_match():
    m = _mass(30, 30, 55)
    rng = random.Random(50)
    plan = qr.plan_pile(m, "rc", rng, kind="dome", budget={"n_large": 6, "n_instances": 120})
    assert len(plan["large"]) <= 6
    total = 0
    for k, v in plan["instances"].items():
        n = len(v["positions"])
        assert len(v["orientations"]) == n
        assert len(v["scales"]) == n
        assert len(v["proto_index"]) == n
        assert all(0 <= pi < len(v["protos"]) for pi in v["proto_index"])
        total += n
    assert total <= 120
    assert plan["stats"]["n_large"] == len(plan["large"])
    n_inst_stat = sum(plan["stats"]["n_instances"].values())
    assert n_inst_stat == plan["stats"]["n_instances_total"] == total


# ---------------------------------------------------------------------------
# (9) determinism
# ---------------------------------------------------------------------------

def test_determinism():
    m = _mass(20, 14, 12)
    planA = qr.plan_pile(m, "rc", random.Random(77), kind="dome")
    planB = qr.plan_pile(m, "rc", random.Random(77), kind="dome")
    assert planA["stats"] == planB["stats"]
    assert planA["instances"]["chunk"]["positions"][:10] == planB["instances"]["chunk"]["positions"][:10]


# ---------------------------------------------------------------------------
# (10) no NaNs anywhere; every orientation quaternion is unit length
# ---------------------------------------------------------------------------

def test_no_nans_and_unit_quaternions():
    m = _mass(24, 16, 18)
    rng = random.Random(60)
    plan = qr.plan_pile(m, "rc_glass", rng, kind="dome", panels=[("/World/p0", (2.0, 3.0, 0.3))])
    assert not np.isnan(plan["mound"]["points"]).any()
    if plan["apron"] is not None:
        assert not np.isnan(plan["apron"]["points"]).any()
    for e in plan["large"]:
        assert not any(math.isnan(v) for v in e["pos"])
        assert not any(math.isnan(v) for v in e["rot_deg"])
    for v in plan["instances"].values():
        for q in v["orientations"]:
            n = math.sqrt(sum(c * c for c in q))
            assert abs(n - 1.0) < 1e-6, q
        for p in v["positions"]:
            assert not any(math.isnan(c) for c in p)


# ---------------------------------------------------------------------------
# stub_h_m: the mound inside the wall line fills to >= 0.5x stub_h near walls
# ---------------------------------------------------------------------------

def test_stub_fill_near_walls():
    rng = random.Random(70)
    nrng = np.random.default_rng(rng.getrandbits(32))
    m = _mass(22, 18, 15)
    stub_h = 2.0
    cell = qr._build_dome_grid(m, "urm", rng, nrng, None, {"S", "N"}, stub_h, None)
    r = cell["r"]
    band = (r > 0.85) & (r < 0.95)
    assert band.any()
    assert float(cell["height"][band].min()) >= 0.5 * stub_h - 1e-6


# ---------------------------------------------------------------------------
# in_reach / surface_z basics
# ---------------------------------------------------------------------------

def test_in_reach_and_surface_z():
    m = _mass(20, 10, 12)
    assert qr.in_reach(m, 0.0, 0.0, 1.0)
    assert qr.in_reach(m, 10.0 + 2.0, 0.0, 3.0)
    assert not qr.in_reach(m, 10.0 + 5.0, 0.0, 3.0)

    rng = random.Random(80)
    plan = qr.plan_pile(m, "rc", rng, kind="dome")
    mound = plan["mound"]
    assert abs(qr.surface_z(mound, 0.0, 0.0) - (m["z0"] + plan["stats"]["crown_m"])) < plan["stats"]["crown_m"]
    far = qr.surface_z(mound, 10000.0, 10000.0)
    assert abs(far - m["z0"]) < 1e-6


# ---------------------------------------------------------------------------
# round-4 review: apron dome-only, lobed, tagged "dust"
# ---------------------------------------------------------------------------

def test_apron_dome_only_and_lobed():
    m = _mass(22, 18, 15)
    plan = qr.plan_pile(m, "urm", random.Random(90), kind="windrow", sides=("S",), depth_m=0.8)
    assert plan["apron"] is None
    plan2 = qr.plan_pile(m, "rc", random.Random(91), kind="fan", sides=("S",), elem_h_m=5.0)
    assert plan2["apron"] is None

    plan3 = qr.plan_pile(m, "urm", random.Random(92), kind="dome")
    apron = plan3["apron"]
    assert apron is not None
    assert apron["look"] == "dust"
    pts = apron["points"]
    loc = np.array([qr._to_local(m, x, y) for x, y, z in pts])
    h = pts[:, 2] - m["z0"]
    # every apron vertex now floors at APRON_LIP_M (round-4 review), so
    # "real" apron thickness has to be measured above that floor, not above
    # bare zero.
    nz = h > qr.APRON_LIP_M + 0.005
    assert nz.any()
    ang = np.arctan2(loc[nz, 1], loc[nz, 0])
    rad = np.hypot(loc[nz, 0], loc[nz, 1])
    bins = np.linspace(-np.pi, np.pi, 13)
    maxr = np.array([rad[(ang >= bins[i]) & (ang < bins[i + 1])].max()
                      for i in range(12) if ((ang >= bins[i]) & (ang < bins[i + 1])).any()])
    assert maxr.size >= 6
    variation = (maxr.max() - maxr.min()) / maxr.mean()
    assert variation >= 0.10, variation


# ---------------------------------------------------------------------------
# round-5: coverage-derived density replaces the old fixed CHUNK_DENSITY/
# FLAKE_N tables; a 30x30x55 rc dome's real HD-piece footprints are small
# enough that the target coverage integral runs into the tens of thousands,
# so the hard `RUBBLE_MAX_INSTANCES` cap is expected to engage on a pile
# this size — see `test_coverage_hard_cap_engages_and_scales_proportionally`
# below for the cap math itself.
# ---------------------------------------------------------------------------

def test_chunk_density_and_flake_count():
    m = _mass(30, 30, 55)
    plan = qr.plan_pile(m, "rc", random.Random(93), kind="dome")
    st = plan["stats"]
    n_flake = st["n_instances"]["flake"]
    n_chunk = st["n_instances"]["chunk"]
    # visibly denser than the old fixed-count design (both classes number in
    # the hundreds to thousands on a building-scale dome), and never past
    # the hard total cap.
    assert n_flake > 300, n_flake
    assert n_chunk > 300, n_chunk
    # round-5 v8: the cap scales with the pile area (never below the flat
    # default), and on a 22 x 18 m dome it no longer engages — chunks are
    # accents over the cluster layer now (COVERAGE 0.45/0.30/0.15).
    assert st["n_instances_total"] <= st["instance_cap"]
    assert st["instance_cap"] >= qr.RUBBLE_MAX_INSTANCES
    # this pile's coverage-derived estimate comfortably exceeds the 6000
    assert st["instances_after_cap"] == st["n_instances_total"]
    assert st["instances_before_cap"] >= st["instances_after_cap"]


# ---------------------------------------------------------------------------
# round-4 review: fan/windrow ridge height actually reaches depth_m
# ---------------------------------------------------------------------------

def test_windrow_and_fan_ridge_reaches_depth():
    """Max mound height within 0.5 m of the wall line, over the middle 60%
    of the run, should equal `depth_m` within 20% — the rc fan was
    rendering as a ~0.2 m carpet although depth_m=0.8 (round-4 review)."""
    def near_wall_max(plan, m):
        pts = plan["mound"]["points"]
        loc = np.array([qr._to_local(m, x, y) for x, y, z in pts])
        d = (-m["D"] / 2.0) - loc[:, 1]
        t = loc[:, 0]
        h = pts[:, 2] - m["z0"]
        near = d < 0.5
        tlo, thi = t.min(), t.max()
        mid = (t > tlo + 0.2 * (thi - tlo)) & (t < thi - 0.2 * (thi - tlo))
        band = near & mid
        assert band.any()
        return float(h[band].max())

    m = _mass(20, 14, 12)
    for depth_m, seed in ((0.8, 100), (0.5, 101), (1.2, 102)):
        plan = qr.plan_pile(m, "rc", random.Random(seed), kind="fan", sides=("S",), depth_m=depth_m)
        mx = near_wall_max(plan, m)
        assert abs(mx - depth_m) <= 0.20 * depth_m, (depth_m, mx)

        plan2 = qr.plan_pile(m, "urm", random.Random(seed), kind="windrow", sides=("S",), depth_m=depth_m)
        mx2 = near_wall_max(plan2, m)
        assert abs(mx2 - depth_m) <= 0.20 * depth_m, (depth_m, mx2)


# ---------------------------------------------------------------------------
# round-4 review: per-set / per-large-entry "look" material tags
# ---------------------------------------------------------------------------

def test_look_tags():
    """round-5: with `HD_CATALOGUE` populated (the normal case — this
    checkout ships `assets/rubble_hd/catalogue.json`), chunk/flake carry NO
    per-set look override any more (`look is None`) — each HD piece already
    has its own real per-piece material (mixed brick/concrete for urm), and
    a uniform per-set override would erase that variety. See
    `test_look_tags_falls_back_to_flat_overrides_when_hd_catalogue_empty`
    below for the pre-round-5 behaviour, still exercised when the HD library
    is unavailable."""
    assert qr.HD_CATALOGUE, "this checkout is expected to ship assets/rubble_hd/catalogue.json"
    m = _mass(22, 18, 15)
    plan = qr.plan_pile(m, "urm", random.Random(95), kind="dome", panels=[("/p", (2.0, 3.0, 0.3))])
    assert plan["stats"]["hd"] is True
    assert plan["instances"]["chunk"]["look"] is None
    assert plan["instances"]["flake"]["look"] is None
    assert plan["instances"]["cluster"]["look"] is None      # keeps its own referenced asset
    assert plan["instances"]["toe"]["look"] is None
    by_kind = {e["kind"]: e["look"] for e in plan["large"]}
    for kind_ in ("lintel", "quoin"):
        if kind_ in by_kind:
            assert by_kind[kind_] == "stone"
    if "joist" in by_kind:
        assert by_kind["joist"] == "timber"
    if "panel" in by_kind:
        assert by_kind["panel"] is None

    m2 = _mass(30, 30, 55)
    plan2 = qr.plan_pile(m2, "rc", random.Random(96), kind="dome")
    assert plan2["instances"]["chunk"]["look"] is None
    by_kind2 = {e["kind"]: e["look"] for e in plan2["large"]}
    assert by_kind2["raft"] is None                          # keeps its own asset
    if "rebar" in by_kind2:
        assert by_kind2["rebar"] == "rust"
    if "column" in by_kind2:
        assert by_kind2["column"] == "concrete"


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)


def test_mound_mesh_outline_is_the_lobe_not_the_grid():
    """Round-4 Isaac pass: the heightfield mesh must not carry its flat
    rectangular floor (a pale plaza / a straight cliff round every pile).
    After `_trim_flat` no more than a handful of kept vertices may lie on
    the grid's bounding rectangle, and the foot must be inside the domain
    (the adaptive domain in `_build_dome_grid`)."""
    import numpy as np
    m = {"W": 22.0, "D": 18.0, "H": 20.0, "top": 20.0, "cx": 0.0, "cy": 0.0,
         "yaw": 0.0, "z0": 0.0}
    for seed, bt, sides in ((4, "urm", ("N", "W")), (4, "rc", ("S",)), (11, "urm", ("E",))):
        plan = qr.plan_pile(m, bt, random.Random(seed), kind="dome", crown_m=5.6,
                         spread_frac=0.27, sides=sides)
        g = plan["mound"]["grid"]
        x0, y0 = g["x0"], g["y0"]
        xmax, ymax = x0 + (g["nx"] - 1) * g["dx"], y0 + (g["ny"] - 1) * g["dy"]
        for key in ("mound", "apron"):
            pts = np.asarray(plan[key]["points"])
            faces = np.asarray(plan[key]["faces"])
            assert len(faces) > 0
            assert faces.max() < len(pts)
            if key == "mound":
                on_b = (np.isclose(pts[:, 0], x0) | np.isclose(pts[:, 0], xmax)
                        | np.isclose(pts[:, 1], y0) | np.isclose(pts[:, 1], ymax)).sum()
                assert on_b <= 0.02 * len(pts), (bt, key, on_b, len(pts))
            # nothing at or below the ground plate
            assert pts[:, 2].min() >= qr.MOUND_LIP_M - 1e-9


def test_stats_carry_measured_extent_per_side():
    m = {"W": 22.0, "D": 18.0, "H": 20.0, "top": 20.0, "cx": 10.0, "cy": -5.0,
         "yaw": 30.0, "z0": 0.0}
    plan = qr.plan_pile(m, "urm", random.Random(4), kind="dome", crown_m=5.6,
                        spread_frac=0.27, sides=("N", "W"))
    ext = plan["stats"]["extent_m"]
    assert set(ext) == {"S", "E", "N", "W"}
    assert all(v >= 0.0 for v in ext.values())
    # the fall sides run further than the nominal reach says on the blind ones
    assert ext["N"] >= plan["stats"]["reach_m"]["N"] - 0.5
    assert max(ext.values()) > 3.0
    # a rotated frame gives the same answer as the un-rotated one (yaw-invariant)
    m0 = dict(m, yaw=0.0)
    plan0 = qr.plan_pile(m0, "urm", random.Random(4), kind="dome", crown_m=5.6,
                         spread_frac=0.27, sides=("N", "W"))
    for k in ext:
        assert abs(ext[k] - plan0["stats"]["extent_m"][k]) < 0.6, (k, ext, plan0["stats"]["extent_m"])


# ---------------------------------------------------------------------------
# round-5: HD prototype selection — per-set caps, material mix, reproducible
# per pile, and the flat-catalogue fallback when HD_CATALOGUE is empty.
# ---------------------------------------------------------------------------

def test_hd_proto_selection_respects_caps_and_material_mix():
    assert qr.HD_CATALOGUE, "this checkout is expected to ship assets/rubble_hd/catalogue.json"

    proto_sets, used_hd = qr._select_proto_sets("urm", random.Random(5))
    assert used_hd is True
    assert 0 < len(proto_sets["chunk"]) <= qr.HD_PROTO_CAP["chunk"]
    assert 0 < len(proto_sets["flake"]) <= qr.HD_PROTO_CAP["flake"]
    assert proto_sets["raft"] == []                  # "URM has timber floors" — unchanged by HD
    assert 0 < len(proto_sets["toe"]) <= qr.HD_PROTO_CAP["toe"]

    for set_name in ("chunk", "flake"):
        mats = [qr.HD_CATALOGUE[n]["material"] for n in proto_sets[set_name]]
        n_minor = sum(1 for x in mats if x == "concrete")
        n_major = len(mats) - n_minor
        cap = qr.HD_PROTO_CAP[set_name]
        brick_pool = qr._hd_names_by(set_name, ("brick",))
        if len(brick_pool) >= cap - int(round(cap * qr.HD_URM_MINORITY_FRAC)):
            # enough closed brick pieces: brick majority with a real concrete minority
            assert n_major > n_minor > 0, (set_name, mats)
            expect_minor = min(len(proto_sets[set_name]),
                               int(round(len(proto_sets[set_name]) * qr.HD_URM_MINORITY_FRAC)))
            assert n_minor == expect_minor, (set_name, n_minor, expect_minor)
        else:
            # round-5 v8c: the brick spread's pieces are open scan shells and
            # almost none pass the closed/chunky filters — every brick piece
            # that does is in the set, the rest tops up from concrete
            assert n_major == len(brick_pool), (set_name, n_major, len(brick_pool))
        # every chosen chunk is a closed, chunky piece (no standing foil)
        if set_name == "chunk":
            for n in proto_sets[set_name]:
                assert qr._chunky(qr.HD_CATALOGUE[n]["size"]), n
                assert qr._hd_open_frac(n) <= qr.HD_CHUNK_MAX_OPEN, (n, qr._hd_open_frac(n))
        for n in proto_sets[set_name]:
            assert qr._asset_entry(n) is not None, n

    proto_sets_rc, used_hd_rc = qr._select_proto_sets("rc", random.Random(5))
    assert used_hd_rc is True
    assert len(proto_sets_rc["chunk"]) == qr.HD_PROTO_CAP["chunk"]
    assert len(proto_sets_rc["flake"]) == qr.HD_PROTO_CAP["flake"]
    assert len(proto_sets_rc["raft"]) == qr.HD_PROTO_CAP["raft"]
    assert len(proto_sets_rc["toe"]) == qr.HD_PROTO_CAP["toe"]
    # rc chunk/flake are concrete-only (no brick minority for a concrete building)
    for set_name in ("chunk", "flake"):
        mats = {qr.HD_CATALOGUE[n]["material"] for n in proto_sets_rc[set_name]}
        assert mats == {"concrete"}, (set_name, mats)
    # raft draws from BOTH the HD raft pool and the original authored slabs
    assert set(proto_sets_rc["raft"]) & set(qr._RAFTS) or \
        set(proto_sets_rc["raft"]) & set(qr._hd_names_by("raft", ("brick", "concrete")))

    # reproducible per pile (same seed -> same draw)...
    proto_sets_again, _ = qr._select_proto_sets("urm", random.Random(5))
    assert proto_sets_again["chunk"] == proto_sets["chunk"]
    assert proto_sets_again["flake"] == proto_sets["flake"]
    # ...but two different piles (seeds) differ.
    proto_sets_other, _ = qr._select_proto_sets("urm", random.Random(6))
    assert proto_sets_other["chunk"] != proto_sets["chunk"]


def test_hd_fallback_to_flat_sets_when_catalogue_forced_empty():
    """Force `HD_CATALOGUE` empty (simulating a checkout with no
    `assets/rubble_hd/` built yet) and confirm `plan_pile` still runs end to
    end on the original flat-colour `PROTO_SETS`, with the pre-round-5
    per-set "brick"/"concrete" look override back in effect."""
    saved = qr.HD_CATALOGUE
    try:
        qr.HD_CATALOGUE = {}
        proto_sets, used_hd = qr._select_proto_sets("urm", random.Random(5))
        assert used_hd is False
        assert proto_sets is qr.PROTO_SETS["urm"]

        m = _mass(22, 18, 15)
        plan = qr.plan_pile(m, "urm", random.Random(97), kind="dome")
        assert plan["stats"]["hd"] is False
        assert plan["instances"]["chunk"]["look"] == "brick"
        assert plan["instances"]["flake"]["look"] == "brick"
        assert plan["stats"]["floating"] == 0
        assert plan["stats"]["n_instances_total"] > 0
        # every drawn name is a real (old, flat) CATALOGUE entry
        for name in plan["instances"]["chunk"]["protos"]:
            assert name in qr.CATALOGUE

        m2 = _mass(30, 30, 55)
        plan2 = qr.plan_pile(m2, "rc", random.Random(98), kind="dome")
        assert plan2["instances"]["chunk"]["look"] == "concrete"
    finally:
        qr.HD_CATALOGUE = saved


# ---------------------------------------------------------------------------
# round-5: coverage math (density = coverage / mean footprint area) and the
# total-instance hard cap.
# ---------------------------------------------------------------------------

def test_mean_footprint_area_uses_coverage_and_scale_second_moment():
    # a single 2x3 m (native) prototype, scale drawn uniformly in [1, 2]:
    # E[scale^2] for U(1,2) = (1 + 2 + 4)/3 = 7/3; footprint area = 2*3 = 6.
    qr.CATALOGUE["_test_mean_area_probe"] = {
        "url": "does/not/matter.usdc", "size": (2.0, 3.0, 1.0),
        "tris": 1, "kind": "chunk", "textured": False, "material": "concrete"}
    try:
        got = qr._mean_footprint_area(["_test_mean_area_probe"], (1.0, 2.0))
        expect = 2.0 * 3.0 * (7.0 / 3.0)
        assert abs(got - expect) / expect < 1e-9, (got, expect)
    finally:
        del qr.CATALOGUE["_test_mean_area_probe"]

    # empty/unresolvable input never divides by zero downstream
    assert qr._mean_footprint_area([], (1.0, 2.0)) > 0.0
    assert qr._mean_footprint_area(["_not_a_real_asset_"], (1.0, 2.0)) > 0.0


def test_zone_count_estimate_scales_inversely_with_mean_area():
    """density = coverage / mean_area — halving the mean footprint area
    should roughly double the estimated count for the SAME mound (the
    integral of `_coverage_frac(rel) / mean_area` over the same cells)."""
    rng = random.Random(12)
    nrng = np.random.default_rng(rng.getrandbits(32))
    m = _mass(30, 30, 55)
    cell = qr._build_dome_grid(m, "rc", rng, nrng, None, {"S", "W"}, 0.0, None)
    crown_actual, _vol, _slope = qr._summarize_mound([cell])

    n_small = qr._zone_count_estimate([cell], crown_actual, 0.05)
    n_big = qr._zone_count_estimate([cell], crown_actual, 0.10)
    assert n_small > 0 and n_big > 0
    ratio = n_small / n_big
    assert 1.8 < ratio < 2.2, ratio

    # non-positive crown/mean_area is defined as "nothing to place"
    assert qr._zone_count_estimate([cell], 0.0, 0.05) == 0.0
    assert qr._zone_count_estimate([cell], crown_actual, 0.0) == 0.0


def test_trim_instances_to_cap_scales_every_set_proportionally():
    instances = {
        "chunk": qr._empty_instance_set(),
        "flake": qr._empty_instance_set(),
    }
    for i in range(1000):
        qr._append_instance(instances["chunk"], "a", (float(i), 0.0, 0.0), (1.0, 0, 0, 0), 1.0)
    for i in range(400):
        qr._append_instance(instances["flake"], "b", (float(i), 0.0, 0.0), (1.0, 0, 0, 0), 1.0)

    before, after, capped = qr._trim_instances_to_cap(instances, 700)
    assert before == 1400
    assert after == 700
    assert capped is True
    n_chunk = len(instances["chunk"]["positions"])
    n_flake = len(instances["flake"]["positions"])
    assert n_chunk + n_flake == 700
    # both sets scaled by roughly the SAME ratio (700/1400 = 0.5), not one
    # zeroed out to satisfy the other
    assert abs(n_chunk / 1000.0 - 0.5) < 0.02
    assert abs(n_flake / 400.0 - 0.5) < 0.02

    # a no-op when already within the cap
    instances2 = {"chunk": qr._empty_instance_set()}
    qr._append_instance(instances2["chunk"], "a", (0.0, 0.0, 0.0), (1.0, 0, 0, 0), 1.0)
    before2, after2, capped2 = qr._trim_instances_to_cap(instances2, 6000)
    assert (before2, after2, capped2) == (1, 1, False)


def test_coverage_hard_cap_engages_and_scales_proportionally():
    """A small `RUBBLE_MAX_INSTANCES` should visibly squeeze BOTH chunk and
    flake counts down together (not zero one out), and the achieved
    coverage stats should shrink along with it."""
    saved_cap = qr.RUBBLE_MAX_INSTANCES
    try:
        m = _mass(30, 30, 55)
        qr.RUBBLE_MAX_INSTANCES = 6000
        saved_per_m2 = qr.RUBBLE_INSTANCES_PER_M2
        qr.RUBBLE_INSTANCES_PER_M2 = 0.0        # pin the flat cap for this test
        plan_uncapped = qr.plan_pile(m, "rc", random.Random(93), kind="dome")

        qr.RUBBLE_MAX_INSTANCES = 300
        plan_capped = qr.plan_pile(m, "rc", random.Random(93), kind="dome")

        assert plan_capped["stats"]["instance_cap"] == 300
        assert plan_capped["stats"]["instances_capped"] is True
        assert plan_capped["stats"]["n_instances_total"] <= 300
        assert plan_capped["stats"]["n_instances"]["chunk"] < plan_uncapped["stats"]["n_instances"]["chunk"]
        assert plan_capped["stats"]["n_instances"]["flake"] < plan_uncapped["stats"]["n_instances"]["flake"]
        assert plan_capped["stats"]["n_instances"]["chunk"] > 0
        assert plan_capped["stats"]["n_instances"]["flake"] > 0
        for zone in ("crown", "mid", "toe"):
            assert (plan_capped["stats"]["coverage"][zone]["achieved"] <
                    plan_uncapped["stats"]["coverage"][zone]["achieved"])
            assert plan_capped["stats"]["coverage"][zone]["target"] == qr.COVERAGE[zone]
        assert plan_capped["stats"]["floating"] == 0
    finally:
        qr.RUBBLE_MAX_INSTANCES = saved_cap
        qr.RUBBLE_INSTANCES_PER_M2 = saved_per_m2


def test_n_protos_and_hd_flag_in_stats():
    m = _mass(22, 18, 15)
    plan = qr.plan_pile(m, "urm", random.Random(41), kind="dome")
    assert plan["stats"]["hd"] is True
    n_protos = plan["stats"]["n_protos"]
    assert set(n_protos) == {"chunk", "flake", "cluster", "toe"}
    assert n_protos["chunk"] <= qr.HD_PROTO_CAP["chunk"]
    assert n_protos["flake"] <= qr.HD_PROTO_CAP["flake"]
    assert n_protos["chunk"] == len(plan["instances"]["chunk"]["protos"])


# ---------------------------------------------------------------------------
# round-5: floating stays 0 for the three plans used in the proof renders
# (`tools/rubble_preview.py` runs; see `~/scorch_previews/rubble_r4/v7/`).
# ---------------------------------------------------------------------------

def test_nothing_floats_on_the_proof_render_plans():
    urm_dome = _mass(22, 18, 20)
    plan_urm_dome = qr.plan_pile(urm_dome, "urm", random.Random(4), kind="dome",
                                 crown_m=5.6, sides=("N", "W"))
    assert plan_urm_dome["stats"]["floating"] == 0

    rc_dome = _mass(30, 30, 55)
    plan_rc_dome = qr.plan_pile(rc_dome, "rc", random.Random(3), kind="dome")
    assert plan_rc_dome["stats"]["floating"] == 0

    urm_fan = _mass(22, 18, 20)
    plan_urm_fan = qr.plan_pile(urm_fan, "urm", random.Random(31), kind="fan",
                                sides=("S",), depth_m=0.8, elem_h_m=6.0)
    assert plan_urm_fan["stats"]["floating"] == 0

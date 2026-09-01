#!/usr/bin/env python3
"""test_settle_budget.py — SETTLE_BODY_BUDGET (round 8): does the earthquake
archetype bake's per-row loose set actually get capped for physics, and are
the pieces it cuts still there, seated, and placed the same way every time?

USER, 2026-08-31: "settle/bake the buildings lazily. 15000 bodies seems like
too much, you wanna place some by hand or something." `block_residential`
DG3-5 measured 18,771 loose bodies and a settle that ran over 1.5 h before it
was killed — see `disaster.quake_collapse`'s own "SETTLE BODY BUDGET" section
for the arithmetic (a "block" style's main mass plus several storeys-tall
wings multiplies every per-element/per-storey break population by both the
mass count and each wing's own storey count).

`disaster.quake_collapse.apply_settle_budget` is the fix: rank a row's loose
pieces by a "z-and-size" visual-importance proxy, keep the top `budget` for
physics unchanged, and place everything past it GEOMETRICALLY — seated via
`_deck_support_z` (ground fallback), laid flat with `quake_flow._a_lay_flat`'s
own thin-axis idiom, sunk a few cm, marked static. NEVER via Isaac Sim — every
fixture here is a synthetic in-memory stage, exactly like
`test_quake_collapse.py`'s own `_deck_support_z` tests, so this whole file is
offline and host-testable.

Fixtures use `quake_flow._box` directly (not a hand-rolled box mesh) because
`_a_lay_flat`'s pivot is read off the prim's OWN `xformOp:translate` — `_box`
authors LOCAL points centred on the origin plus that translate op, the same
convention every real fracture/break-authored loose piece in this codebase
uses. A world-baked-points helper (`test_quake_collapse._stage_box`, used
ONLY for testing `_deck_support_z` itself, which reads world points and does
not care about pivots) would silently rotate about the world origin instead
of the piece's own centre if reused here.
"""

import os
import random
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import quake_collapse as qc        # noqa: E402
from disaster import quake_flow as qf             # noqa: E402


def _new_stage(root="/World/b0"):
    from pxr import Sdf, Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, Sdf.Path(root))
    return stage, root


def _slab(stage, root, top_z=5.0, size=40.0, path=None):
    """A wide, flat support surface — `quake_flow._box`'s own construction —
    whose TOP face sits at `top_z`."""
    p = path or (root + "/support_slab")
    qf._box(stage, p, 0.0, 0.0, top_z - 0.15, size, size, 0.3)
    return p


def _upright_plate(stage, root, name, cx, cy, cz, w=2.0, h=1.4, t=0.15):
    """A wall-fragment PLATE standing upright (thin in Y) — the shape
    `_a_lay_flat` is meant to tip over onto its thin axis."""
    path = "{0}/{1}".format(root, name)
    qf._box(stage, path, cx, cy, cz, w, t, h)
    return path


def _flat_crumb(stage, root, name, cx, cy, cz, s=0.3, t=0.08):
    """Already thin-in-Z (litter-like) — `_a_lay_flat` should only tilt/
    yaw-jitter this, never flip it."""
    path = "{0}/{1}".format(root, name)
    qf._box(stage, path, cx, cy, cz, s, s, t)
    return path


def _mixed_row(stage, root, n=10, base_z=5.6, seed=3):
    """`n` loose pieces spread over the slab's footprint at increasing
    heights (so volume x z-height gives every piece a distinct rank) —
    alternating upright plates and flat crumbs, the two shapes
    `_a_lay_flat` treats differently."""
    rng = random.Random(seed)
    paths = []
    for i in range(n):
        cx = rng.uniform(-3.0, 3.0)
        cy = rng.uniform(-3.0, 3.0)
        cz = base_z + i * 1.4
        if i % 2 == 0:
            paths.append(_upright_plate(stage, root, "plate_{0}".format(i),
                                        cx, cy, cz))
        else:
            paths.append(_flat_crumb(stage, root, "crumb_{0}".format(i),
                                     cx, cy, cz))
    return paths


def _world_points_min_z(stage, path):
    """The piece's ACTUAL mesh points, transformed to world — never a
    `BBoxCache` query. This is the exact discipline the project's own bug
    catalogue calls out (a `PointInstancer`'s bbox reports its LARGEST
    instance's envelope, never any individual instance's true position);
    these are plain Meshes so the two would agree here, but the seating
    assertions below use points on purpose rather than reach for the
    bbox shortcut."""
    import numpy as np
    from pxr import UsdGeom
    pr = stage.GetPrimAtPath(path)
    xf = UsdGeom.XformCache()
    M = xf.GetLocalToWorldTransform(pr)
    pts = UsdGeom.Mesh(pr).GetPointsAttr().Get()
    world = [M.Transform(p) for p in pts]
    return float(min(p[2] for p in world))


# ---------------------------------------------------------------------------
# settle_body_budget() — the env reader
# ---------------------------------------------------------------------------
def test_settle_body_budget_env_reader(monkeypatch):
    monkeypatch.delenv(qc.SETTLE_BODY_BUDGET_ENV, raising=False)
    assert qc.settle_body_budget() == qc.SETTLE_BODY_BUDGET_DEFAULT

    monkeypatch.setenv(qc.SETTLE_BODY_BUDGET_ENV, "")
    assert qc.settle_body_budget() == qc.SETTLE_BODY_BUDGET_DEFAULT

    monkeypatch.setenv(qc.SETTLE_BODY_BUDGET_ENV, "1500")
    assert qc.settle_body_budget() == 1500

    monkeypatch.setenv(qc.SETTLE_BODY_BUDGET_ENV, "-1")
    assert qc.settle_body_budget() is None       # unlimited

    monkeypatch.setenv(qc.SETTLE_BODY_BUDGET_ENV, "not-a-number")
    assert qc.settle_body_budget() == qc.SETTLE_BODY_BUDGET_DEFAULT

    assert qc.settle_body_budget(default=42) == 42


# ---------------------------------------------------------------------------
# Budget respected, and the split covers every piece exactly once
# ---------------------------------------------------------------------------
def test_budget_respected_and_split_is_a_partition():
    stage, root = _new_stage()
    _slab(stage, root)
    paths = _mixed_row(stage, root, n=10)

    kept, geometric, report = qc.apply_settle_budget(
        stage, paths, budget=4, root=root, ground_z=0.0,
        rng=random.Random(7))

    assert len(kept) == 4
    assert len(geometric) == 6
    assert len(report) == 6
    # every piece accounted for exactly once
    assert set(kept) | set(geometric) == set(paths)
    assert set(kept) & set(geometric) == set()
    # never both loose AND static
    for p in geometric:
        assert p not in kept


def test_budget_zero_is_all_geometric():
    stage, root = _new_stage()
    _slab(stage, root)
    paths = _mixed_row(stage, root, n=8)

    kept, geometric, report = qc.apply_settle_budget(
        stage, paths, budget=0, root=root, ground_z=0.0,
        rng=random.Random(1))

    assert kept == []
    assert set(geometric) == set(paths)
    assert len(report) == len(paths)


# ---------------------------------------------------------------------------
# No-op / backward compat — a budget covering every piece must not touch the
# stage at all, so a bake at a high-enough budget exports byte-for-byte.
# ---------------------------------------------------------------------------
def test_huge_budget_is_byte_identical_noop():
    stage, root = _new_stage()
    _slab(stage, root)
    paths = _mixed_row(stage, root, n=6)
    before = stage.GetRootLayer().ExportToString()

    kept, geometric, report = qc.apply_settle_budget(
        stage, paths, budget=1_000_000, root=root, ground_z=0.0,
        rng=random.Random(2))

    after = stage.GetRootLayer().ExportToString()
    assert after == before, "a no-op budget must not touch the stage at all"
    assert kept == paths
    assert geometric == []
    assert report == []


def test_budget_none_is_unlimited_noop():
    stage, root = _new_stage()
    _slab(stage, root)
    paths = _mixed_row(stage, root, n=5)
    before = stage.GetRootLayer().ExportToString()

    kept, geometric, report = qc.apply_settle_budget(
        stage, paths, budget=None, root=root, ground_z=0.0,
        rng=random.Random(2))

    assert stage.GetRootLayer().ExportToString() == before
    assert kept == paths and geometric == [] and report == []


def test_budget_exactly_equal_to_count_is_noop():
    """The boundary: `budget == len(paths)` keeps every piece loose (`>=`,
    not `>`, in `apply_settle_budget`'s own guard)."""
    stage, root = _new_stage()
    _slab(stage, root)
    paths = _mixed_row(stage, root, n=7)
    before = stage.GetRootLayer().ExportToString()

    kept, geometric, report = qc.apply_settle_budget(
        stage, paths, budget=len(paths), root=root, ground_z=0.0,
        rng=random.Random(2))

    assert stage.GetRootLayer().ExportToString() == before
    assert kept == paths and geometric == []


# ---------------------------------------------------------------------------
# Geometric pieces are actually seated — POINTS-based, not bbox
# ---------------------------------------------------------------------------
def test_geometric_pieces_seated_on_the_slab_within_tolerance():
    stage, root = _new_stage()
    _slab(stage, root, top_z=5.0)
    paths = _mixed_row(stage, root, n=10, base_z=5.6)

    kept, geometric, report = qc.apply_settle_budget(
        stage, paths, budget=2, root=root, ground_z=0.0,
        rng=random.Random(42))

    assert geometric, "fixture must actually produce over-budget pieces"
    for p in geometric:
        z_min = _world_points_min_z(stage, p)
        # support (~5.0) minus a 2-6 cm sink; a small margin for the plate's
        # own tilt jitter (`_a_lay_flat`'s 3-34 deg tumble can lift ONE
        # corner slightly proud of the lowest point's exact target)
        assert 4.85 <= z_min <= 5.05, (p, z_min)


def test_geometric_piece_falls_back_to_ground_when_nothing_is_under_it():
    stage, root = _new_stage()
    _slab(stage, root, top_z=5.0)
    # far outside the slab's footprint — no support to find
    far = _upright_plate(stage, root, "far_plate", 500.0, 500.0, 8.0)
    paths = [far] + _mixed_row(stage, root, n=3, base_z=5.6)

    kept, geometric, report = qc.apply_settle_budget(
        stage, paths, budget=1, root=root, ground_z=1.5,
        rng=random.Random(5))

    assert far in geometric
    z_min = _world_points_min_z(stage, far)
    assert 1.35 <= z_min <= 1.55, z_min      # ground_z (1.5) minus the sink


# ---------------------------------------------------------------------------
# Determinism
# ---------------------------------------------------------------------------
def test_deterministic_same_seed_same_split_and_placement():
    def _run():
        stage, root = _new_stage()
        _slab(stage, root)
        paths = _mixed_row(stage, root, n=12)
        kept, geometric, report = qc.apply_settle_budget(
            stage, paths, budget=5, root=root, ground_z=0.0,
            rng=random.Random(99))
        z_by_path = {p: _world_points_min_z(stage, p) for p in geometric}
        return set(kept), set(geometric), z_by_path

    kept_a, geo_a, z_a = _run()
    kept_b, geo_b, z_b = _run()
    assert kept_a == kept_b
    assert geo_a == geo_b
    for p in geo_a:
        assert abs(z_a[p] - z_b[p]) < 1e-9, p


def test_ranking_has_no_rng_draw_and_is_stable():
    """`rank_loose_for_settle_budget` measures the stage; it must not touch
    any rng, and must return the same order across repeated calls with no
    stage mutation in between."""
    stage, root = _new_stage()
    _slab(stage, root)
    paths = _mixed_row(stage, root, n=9)
    r1 = qc.rank_loose_for_settle_budget(stage, paths)
    r2 = qc.rank_loose_for_settle_budget(stage, paths)
    assert r1 == r2
    assert set(p for p, _s in r1) == set(paths)
    # descending
    scores = [s for _p, s in r1]
    assert scores == sorted(scores, reverse=True)


# ---------------------------------------------------------------------------
# Robustness: an unresolvable piece keeps its geometry (stays loose) rather
# than being silently dropped.
# ---------------------------------------------------------------------------
def test_unresolvable_piece_stays_loose_rather_than_lost():
    stage, root = _new_stage()
    _slab(stage, root)
    paths = _mixed_row(stage, root, n=4)
    ghost = root + "/does_not_exist"
    all_paths = paths + [ghost]

    kept, geometric, report = qc.apply_settle_budget(
        stage, all_paths, budget=1, root=root, ground_z=0.0,
        rng=random.Random(3))

    assert ghost not in geometric
    assert ghost in kept


# ---------------------------------------------------------------------------
# `root` accepts a per-piece callable, not just a single string (the
# archetype launcher passes a `path -> owning-building-scope` lookup)
# ---------------------------------------------------------------------------
def test_root_accepts_a_callable():
    stage, root = _new_stage()
    _slab(stage, root)
    paths = _mixed_row(stage, root, n=6)

    calls = []

    def _root_fn(p):
        calls.append(p)
        return root

    kept, geometric, report = qc.apply_settle_budget(
        stage, paths, budget=2, root=_root_fn, ground_z=0.0,
        rng=random.Random(8))

    assert geometric, "fixture must produce over-budget pieces"
    assert set(calls) == set(geometric)


# ---------------------------------------------------------------------------
# ROUND 8 FOLLOW-UP — the `_deck_support_candidates` cache
# (`apply_settle_budget` no longer calls `_deck_support_z` against a fresh
# `stage.Traverse()` for every over-budget piece; it builds one candidate
# list per distinct root and reuses it). These tests pin that the cached
# path answers EXACTLY what the original, still-untouched `candidates=None`
# code path answers, and that the cache actually delivers the perf win.
# ---------------------------------------------------------------------------
def _multi_candidate_stage():
    """Several meshes at different heights/footprints/orientations under one
    root — including a wide slab (the "one big quad, fan-triangulated off-
    centre" case `_deck_support_z`'s own docstring calls out) and a raised
    "coping" mesh whose bottom is below a query ceiling but whose top is
    not, so more than one candidate can plausibly matter for a given
    query."""
    stage, root = _new_stage()
    qf._box(stage, root + "/slab_wide", 0.0, 0.0, 4.85, 20.0, 8.0, 0.3)
    qf._box(stage, root + "/slab_narrow", 15.0, 0.0, 2.85, 4.0, 4.0, 0.3)
    qf._box(stage, root + "/coping", -15.0, 0.0, 6.0, 3.0, 3.0, 2.0)
    qf._box(stage, root + "/blocker_above", 0.0, 0.0, 12.0, 20.0, 8.0, 0.3)
    return stage, root


def _sample_queries():
    """(cx, cy, half_w, half_d, z_ceiling) tuples exercising: dead centre of
    the wide slab, its off-centre edge (the fan-triangulation trap), the
    narrow slab, the coping mesh, and a query with nothing under it at
    all."""
    return [
        (0.0, 0.0, 1.0, 1.0, 5.05),
        (8.0, 0.0, 1.0, 1.0, 5.05),
        (15.0, 0.0, 1.0, 1.0, 3.05),
        (-15.0, 0.0, 1.0, 1.0, 7.05),
        (100.0, 100.0, 1.0, 1.0, 5.05),
    ]


def test_deck_support_z_cache_is_byte_identical_to_uncached():
    stage, root = _multi_candidate_stage()
    candidates = qc._deck_support_candidates(stage, root)
    assert len(candidates) >= 3   # slab_wide, slab_narrow, coping all qualify

    for cx, cy, half_w, half_d, z_ceiling in _sample_queries():
        uncached = qc._deck_support_z(stage, root, cx, cy, half_w, half_d,
                                      z_ceiling)
        cached = qc._deck_support_z(stage, root, cx, cy, half_w, half_d,
                                    z_ceiling, candidates=candidates)
        assert uncached == cached, (cx, cy, z_ceiling, uncached, cached)


def test_deck_support_z_cache_is_byte_identical_to_uncached_with_exclude():
    """The SAME comparison, with a non-empty `exclude` — a path-based
    filter applied fresh per query on both paths, never baked into the
    cache (see `_deck_support_candidates`'s own docstring)."""
    stage, root = _multi_candidate_stage()
    candidates = qc._deck_support_candidates(stage, root)
    excl = {root + "/slab_wide"}

    for cx, cy, half_w, half_d, z_ceiling in _sample_queries():
        uncached = qc._deck_support_z(stage, root, cx, cy, half_w, half_d,
                                      z_ceiling, exclude=excl)
        cached = qc._deck_support_z(stage, root, cx, cy, half_w, half_d,
                                    z_ceiling, exclude=excl,
                                    candidates=candidates)
        assert uncached == cached, (cx, cy, z_ceiling, uncached, cached)


def test_deck_support_candidates_drops_degenerate_triangles_safely():
    """A zero-area sliver (two coincident points) must never appear as a
    "hit" via the cache, matching the uncached path's own `ok = ar > 1e-9`
    gate."""
    from pxr import Gf, Sdf, UsdGeom, Vt
    stage, root = _new_stage()
    p = root + "/sliver"
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(p))
    mesh.CreatePointsAttr([Gf.Vec3f(0, 0, 5), Gf.Vec3f(0, 0, 5),
                          Gf.Vec3f(1, 0, 5), Gf.Vec3f(1, 0, 5)])
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    candidates = qc._deck_support_candidates(stage, root)
    # either dropped outright (no triangles survive `ok`), or present with
    # arrays too degenerate to ever register a hit — both are correct;
    # what matters is neither path ever returns a false support here.
    uncached = qc._deck_support_z(stage, root, 0.5, 0.0, 0.4, 0.4, 5.05)
    cached = qc._deck_support_z(stage, root, 0.5, 0.0, 0.4, 0.4, 5.05,
                                candidates=candidates)
    assert uncached is None and cached is None


def _candidate_floor_grid(stage, root, nx, ny, cell, z):
    for i in range(nx):
        for j in range(ny):
            cx = (i - nx / 2.0 + 0.5) * cell
            cy = (j - ny / 2.0 + 0.5) * cell
            qf._box(stage, "{0}/floor_{1}_{2}".format(root, i, j),
                   cx, cy, z - 0.15, cell * 0.98, cell * 0.98, 0.3)


def _big_row(stage, root, n, seed, base_z, spread):
    rng = random.Random(seed)
    paths = []
    for i in range(n):
        cx = rng.uniform(-spread, spread)
        cy = rng.uniform(-spread, spread)
        cz = base_z + rng.uniform(0.0, 3.0)
        if i % 2 == 0:
            paths.append(_upright_plate(stage, root, "bigplate_{0}".format(i),
                                        cx, cy, cz))
        else:
            paths.append(_flat_crumb(stage, root, "bigcrumb_{0}".format(i),
                                     cx, cy, cz))
    return paths


def test_perf_cache_handles_2000_geometric_placements_in_single_digit_seconds():
    """PERF (round 8 follow-up, requested by the coordinator ahead of the
    pod re-bake, targeting this file's own flagged caveat). Before the
    `_deck_support_candidates` cache, `apply_settle_budget` called
    `_deck_support_z` — a fresh `stage.Traverse()` PLUS a fresh per-mesh
    world-transform/fan-triangulation of every candidate — once PER OVER-
    BUDGET PIECE; a manual dry run at a much smaller scale (2,000 total
    prims, 1,200 geometric) measured ~28s BEFORE this cache existed. Here:
    a dedicated 500-mesh floor scope (25 x 20 grid, `nx * ny == 500`) plus
    2,000 loose pieces scattered above it, `budget=0` so EVERY piece is
    resolved via the cache.

    MEASURED on this machine: 1.25s (`uv run --python 3.13`, CPython 3.13,
    single process, `numpy` reference BLAS — no GPU). The `assert dt < 10.0`
    below is deliberately generous (roughly an order of magnitude of
    headroom over the measured figure) so this stays a regression guard
    against the cache being lost, not a flaky timing assertion on a loaded
    CI box.
    """
    import time
    stage, root = _new_stage()
    floor_root = root + "/floor"
    loose_root = root + "/loose"
    _candidate_floor_grid(stage, floor_root, nx=25, ny=20, cell=3.0, z=5.0)
    paths = _big_row(stage, loose_root, n=2000, seed=17, base_z=6.0, spread=27.0)

    t0 = time.perf_counter()
    kept, geometric, report = qc.apply_settle_budget(
        stage, paths, budget=0, root=floor_root, ground_z=0.0,
        rng=random.Random(9))
    dt = time.perf_counter() - t0

    print("\n[perf] 2000 geometric placements against a 500-mesh candidate "
          "scope: {0:.3f}s".format(dt))
    assert kept == []
    assert len(geometric) >= 0.98 * len(paths), (
        "fixture should place nearly every piece over the floor "
        "footprint — got {0}/{1}".format(len(geometric), len(paths)))
    assert dt < 10.0, dt


if __name__ == "__main__":
    import sys as _sys
    _sys.exit(pytest.main([__file__, "-v"]))

#!/usr/bin/env python3
"""test_aec_quake.py — earthquake damage for an AEC brownstone ROW, by name.

    uv run --python 3.13 --with vtk --with usd-core --with numpy --with \
        pytest --with shapely --with scipy --with pyyaml -m pytest -q \
        scene_gen/tests/test_aec_quake.py

USER DIRECTIVE (2026-09-02): "brownstone aec has a new skill file on how to
damage it, we did it for fire but you need to include it for earthquake
too." `disaster/aec_quake.py` ports `disaster/aec_burn.py`'s by-NAME
part-addressing method (de-instance, then address parts by category name
and geometric position) from fire levels to EMS-98 grades — see that
module's own docstring for the full method and the ladder table.

Every real-asset test below runs against the ACTUAL local
`Reference_Brownstone<N>Row.usd` files under `scene_gen/assets/aec/
brownstone/` (no Isaac Sim, no Nucleus — `nucleus_fetch.py`'s own mirror,
already on disk) and is skipped cleanly if that directory is absent (a
fresh checkout without the asset pull). Pure-python tests (`_select_units`,
the ladder table shape) need no asset and always run.
"""
import os
import random
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, _SCENE_GEN)
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..", "tools")))

from disaster import aec_burn as ab             # noqa: E402
from disaster import aec_quake as aq            # noqa: E402
from disaster import quake as q                 # noqa: E402


_ASSET_DIR = os.path.join(_SCENE_GEN, "assets", "aec", "brownstone", "Assets",
                          "Create_Brownstone02")
_ROW5 = os.path.join(_ASSET_DIR, "Reference_Brownstone5Row.usd")
_ROW2 = os.path.join(_ASSET_DIR, "Reference_Brownstone2Row.usd")
_HAVE_ASSET = os.path.exists(_ROW5)
_skip_no_asset = pytest.mark.skipif(
    not _HAVE_ASSET, reason="scene_gen/assets/aec/brownstone/ not present "
                            "on this checkout (asset pull required)")


def _row_stage(asset_path, root_path="/World/row0"):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    prim = stage.DefinePrim(root_path, "Xform")
    prim.GetReferences().AddReference(asset_path)
    return stage, root_path


def _world_z(stage, path):
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(path)
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    return float(xf.GetLocalToWorldTransform(prim).ExtractTranslation()[2])


# ---------------------------------------------------------------------------
# pure-python: the ladder table + `_select_units` (no asset, no stage)
# ---------------------------------------------------------------------------
def test_ladder_covers_dg0_through_dg5_cumulatively():
    for g in ("DG0", "DG1", "DG2", "DG3", "DG4", "DG5"):
        assert g in aq.LADDER
    assert aq.LADDER["DG0"] == {}
    # cumulative: DG5's keys are a superset of DG1's
    assert set(aq.LADDER["DG1"]) <= set(aq.LADDER["DG5"])
    assert aq.LADDER["DG3"]["pile"] == "small"
    assert aq.LADDER["DG4"]["pile"] == "big"
    assert aq.LADDER["DG5"]["pile"] == "huge"
    # DG5 leaves at least one unit standing no matter the row length
    assert aq.LADDER["DG5"]["wall_units"] < 1.0


def test_select_units_tuple_is_an_explicit_count():
    rng = random.Random(0)
    out = aq._select_units(list(range(5)), (1, 1), rng, reserve=0)
    assert len(out) == 1


def test_select_units_float_is_a_fraction():
    rng = random.Random(0)
    out = aq._select_units(list(range(10)), 0.5, rng, reserve=0)
    assert len(out) == 5


def test_select_units_reserve_keeps_at_least_one_unit_untouched():
    """DG5's "gable/party-wall remnant" -- `reserve=1` must never select
    every unit, even when the spec fraction alone would cover the whole
    row, and even repeated over many draws (never just lucky once)."""
    units = list(range(5))
    for seed in range(30):
        rng = random.Random(seed)
        out = aq._select_units(units, 1.0, rng, reserve=1)
        assert len(out) <= 4


def test_select_units_returns_nothing_when_reserve_consumes_the_row():
    rng = random.Random(0)
    out = aq._select_units(list(range(1)), 1.0, rng, reserve=1)
    assert out == []


# ---------------------------------------------------------------------------
# real asset: by-NAME unit enumeration (`aec_burn.measure_row`, reused)
# ---------------------------------------------------------------------------
@_skip_no_asset
def test_measure_row_enumerates_units_by_name_on_the_real_asset():
    stage, root = _row_stage(_ROW5)
    meas = ab.measure_row(stage, root, verbose=False)
    units = meas["units"]
    assert len(units) == 5
    for u in units:
        assert u["name"].startswith("Reference_Brownstone02_")
        assert isinstance(u["idx"], int)
        assert u["inst"] is not None            # the instanceable prim to flip
        assert len(u["meshes"]) > 0
        assert any(m["cat"] == "Windows" for m in u["meshes"])
        assert any(m["cat"].startswith("Roofs") for m in u["meshes"])


@_skip_no_asset
def test_measure_row_on_the_smaller_two_unit_row():
    stage, root = _row_stage(_ROW2)
    meas = ab.measure_row(stage, root, verbose=False)
    assert len(meas["units"]) == 2


# ---------------------------------------------------------------------------
# real asset: DG0 is a byte-identical no-op
# ---------------------------------------------------------------------------
@_skip_no_asset
def test_dg0_is_byte_identical_noop():
    stage, root = _row_stage(_ROW5)
    before = stage.GetRootLayer().ExportToString()
    stats = aq.quake_row(stage, root, grade="DG0", seed=7, verbose=False)
    after = stage.GetRootLayer().ExportToString()
    assert before == after
    assert stats["deinstanced"] == 0
    assert stats["debris"] == []
    assert stats["scars"] == 0
    assert stats["pile"] is None


@_skip_no_asset
def test_unknown_grade_is_also_a_noop():
    stage, root = _row_stage(_ROW5)
    before = stage.GetRootLayer().ExportToString()
    aq.quake_row(stage, root, grade="DG9", seed=7, verbose=False)
    assert stage.GetRootLayer().ExportToString() == before


# ---------------------------------------------------------------------------
# real asset: removal counts grow monotonically with grade (fixed seed,
# same asset -- an empirical measurement, pinned, like every other seeded
# figure in this codebase's own quake test suite)
# ---------------------------------------------------------------------------
@_skip_no_asset
def test_removal_counts_are_monotone_across_grades():
    totals = []
    for grade in ("DG1", "DG2", "DG3", "DG4", "DG5"):
        stage, root = _row_stage(_ROW5)
        stats = aq.quake_row(stage, root, grade=grade, seed=7, verbose=False)
        total = sum(stats["killed"].values())
        totals.append(total)
        assert total > 0, "{0} removed nothing".format(grade)
    assert totals == sorted(totals), totals
    # strictly increasing, not merely non-decreasing -- every grade above
    # DG1 adds at least one NEW removal category this ladder does not share
    # with the grade below it (window voiding at DG2, roof loss at DG3,
    # wall loss at DG4)
    assert len(set(totals)) == len(totals), totals


@_skip_no_asset
def test_dg5_leaves_at_least_one_units_wall_standing():
    stage, root = _row_stage(_ROW5)
    stats = aq.quake_row(stage, root, grade="DG5", seed=7, verbose=False)
    assert 1 <= len(stats["wall_unit_names"]) <= 4
    assert len(set(stats["wall_unit_names"])) == len(stats["wall_unit_names"])


@_skip_no_asset
def test_dg4_removes_exactly_one_units_wall():
    stage, root = _row_stage(_ROW5)
    stats = aq.quake_row(stage, root, grade="DG4", seed=7, verbose=False)
    assert len(stats["wall_unit_names"]) == 1


@_skip_no_asset
def test_dg3_has_no_wall_loss_but_dg3_has_roof_loss():
    stage, root = _row_stage(_ROW5)
    stats = aq.quake_row(stage, root, grade="DG3", seed=7, verbose=False)
    assert stats["wall_unit_names"] == []
    assert 1 <= len(stats["roof_unit_names"]) <= 2


# ---------------------------------------------------------------------------
# real asset: determinism
# ---------------------------------------------------------------------------
@_skip_no_asset
@pytest.mark.parametrize("grade", ["DG1", "DG2", "DG3", "DG4", "DG5"])
def test_same_seed_is_deterministic(grade):
    s1, r1 = _row_stage(_ROW5)
    s2, r2 = _row_stage(_ROW5)
    stats1 = aq.quake_row(s1, r1, grade=grade, seed=42, verbose=False)
    stats2 = aq.quake_row(s2, r2, grade=grade, seed=42, verbose=False)
    for key in ("killed", "debris_n", "debris_kinds", "scars",
               "window_units_voided", "roof_unit_names", "wall_unit_names"):
        assert stats1[key] == stats2[key], key
    if stats1["pile"] is not None:
        assert stats1["pile"]["n"] == stats2["pile"]["n"]


@_skip_no_asset
def test_different_seed_moves_at_least_one_stochastic_count():
    """Not a hash-collision-proof guarantee -- just confirms the seed is
    actually threaded through (a bug that silently ignored `seed` would
    make this fail every time, not just by chance)."""
    s1, r1 = _row_stage(_ROW5)
    s2, r2 = _row_stage(_ROW5)
    stats1 = aq.quake_row(s1, r1, grade="DG3", seed=1, verbose=False)
    stats2 = aq.quake_row(s2, r2, grade="DG3", seed=2, verbose=False)
    assert stats1 != stats2


# ---------------------------------------------------------------------------
# real asset: debris budget is actually respected
# ---------------------------------------------------------------------------
@_skip_no_asset
def test_debris_budget_caps_authored_chunks():
    stage, root = _row_stage(_ROW5)
    uncapped = aq.quake_row(stage, root, grade="DG5", seed=7, verbose=False)
    assert uncapped["debris_n"] > 5, \
        "test asset/seed no longer produces enough debris to prove a cap"

    stage2, root2 = _row_stage(_ROW5)
    capped = aq.quake_row(stage2, root2, grade="DG5", seed=7, verbose=False,
                          debris_budget=5)
    assert capped["debris_n"] == 5
    assert len(capped["debris"]) == 5


@_skip_no_asset
def test_debris_budget_falls_back_to_the_module_default(monkeypatch):
    """`debris_budget=None` (the default) reads `aec_quake.DEBRIS_BUDGET_
    DEFAULT` at CALL time, not at import time -- patched here as an
    attribute (monkeypatch restores it automatically) rather than via
    `AEC_QUAKE_DEBRIS_BUDGET` + a module reload, which would leak a reload
    across the rest of this file's tests if the restore ever misfired."""
    monkeypatch.setattr(aq, "DEBRIS_BUDGET_DEFAULT", 3)
    stage, root = _row_stage(_ROW5)
    stats = aq.quake_row(stage, root, grade="DG5", seed=7, verbose=False)
    assert stats["debris_n"] <= 3


# ---------------------------------------------------------------------------
# real asset: displaced parts are seated at a known reference plane, never
# floating -- every debris chunk this module authors lands at either the
# unit's own GROUND level or its ROOF DECK, both already measured by
# `aec_burn.measure_row` (see `aec_quake`'s own docstring on why a full
# `quake_collapse._deck_support_z` stage query is not needed here).
# ---------------------------------------------------------------------------
@_skip_no_asset
def test_displaced_debris_is_seated_not_floating():
    stage, root = _row_stage(_ROW5)
    meas = ab.measure_row(stage, root, verbose=False)
    ground_z = min(u["bbox"][2] for u in meas["units"])
    deck_z = max(u["deck_z"] for u in meas["units"])

    stats = aq.quake_row(stage, root, grade="DG5", seed=7, verbose=False)
    assert stats["debris"], "no debris authored to check"
    n_checked = 0
    for path in stats["debris"]:
        z = _world_z(stage, path)
        name = path.rsplit("/", 1)[-1]
        if name.startswith("chimney_"):
            assert deck_z - 0.5 <= z <= deck_z + 1.5, (path, z, deck_z)
        else:
            assert ground_z - 0.5 <= z <= ground_z + 1.5, (path, z, ground_z)
        n_checked += 1
    assert n_checked == len(stats["debris"])


# ---------------------------------------------------------------------------
# real asset: de-instancing (the design sketch's "0.0025 s, prims 199 ->
# 1181" measurement) actually happens, and only for THIS row's own units
# ---------------------------------------------------------------------------
@_skip_no_asset
def test_units_are_deinstanced_when_damaged():
    stage, root = _row_stage(_ROW5)
    meas_before = ab.measure_row(stage, root, verbose=False)
    assert all(u["inst"].IsInstanceable() for u in meas_before["units"])

    stats = aq.quake_row(stage, root, grade="DG2", seed=7, verbose=False)
    assert stats["deinstanced"] == 5

    # `measure_row`'s own `inst` search only matches a STILL-instanceable
    # prim (`Usd.Prim.IsInstanceable()`), so a re-measure after de-instancing
    # finds none at all -- `inst` comes back `None` for every unit, which is
    # itself the proof (there is no longer anything left to flip).
    meas_after = ab.measure_row(stage, root, verbose=False)
    assert all(u["inst"] is None for u in meas_after["units"])


# ---------------------------------------------------------------------------
# routing: an AEC placement at DG4 goes through `aec_quake`, not the bare
# `_mono_pass` rigid-lean fallback
# ---------------------------------------------------------------------------
@_skip_no_asset
def test_routing_sends_a_real_aec_row_through_aec_quake(monkeypatch):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    path = "/World/gen/aec_row"
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(_ROW2)

    monkeypatch.setattr(q.qf, "level_for_intensity", lambda *a, **k: "DG4")
    config = {"usds": {"buildings": {"destroyed": []}}}
    records, tally = [], {}
    placements = [dict(
        category="house", prim_path=path, x_m=0.0, y_m=0.0, yaw_deg=0.0,
        usd="airstack://scene_gen/assets/aec/brownstone/Assets/"
            "Create_Brownstone02/Reference_Brownstone2Row.usd")]

    n = q._mono_pass(stage, config, placements, lambda x, y: 1.0, 1.0,
                     random.Random(3), 1.0, records, tally, verbose=False)

    assert n == 1
    assert len(records) == 1
    assert records[0]["grade"] == "AEC_DG4"
    assert tally.get("AEC_DG4") == 1
    # the ladder actually ran on the stage -- not just a relabelled fallback
    assert stage.GetPrimAtPath(path + "/QuakeDecals").IsValid()
    assert stage.GetPrimAtPath(path + "_debris").IsValid()


@_skip_no_asset
def test_routing_grade_dg0_never_calls_the_ladder_but_still_records(monkeypatch):
    """DG0 for an AEC placement takes the SAME "untouched" path `_mono_pass`
    gives any other monolith at DG0 -- `_aec_pass_one` is never even called
    (`aec and grade != "DG0"` guards it) -- yet still gets a `records` entry
    like every other candidate `_mono_pass` measures (parity with the
    non-AEC branches, and with `test_quake_v5_city.py`'s own DG0 case)."""
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    path = "/World/gen/aec_row"
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(_ROW2)

    monkeypatch.setattr(q.qf, "level_for_intensity", lambda *a, **k: "DG0")
    config = {"usds": {"buildings": {"destroyed": []}}}
    records, tally = [], {}
    placements = [dict(
        category="house", prim_path=path, x_m=0.0, y_m=0.0, yaw_deg=0.0,
        usd="airstack://scene_gen/assets/aec/brownstone/Assets/"
            "Create_Brownstone02/Reference_Brownstone2Row.usd")]

    n = q._mono_pass(stage, config, placements, lambda x, y: 1.0, 1.0,
                     random.Random(3), 1.0, records, tally, verbose=False)

    assert n == 1
    assert records[0]["grade"] == "DG0"
    assert not stage.GetPrimAtPath(path + "/QuakeDecals").IsValid()
    assert not stage.GetPrimAtPath(path + "_debris").IsValid()


def test_routing_falls_back_gracefully_on_a_non_aec_shaped_stage():
    """Pins the exact scenario `_aec_pass_one`'s own docstring describes:
    an AEC-shaped USD PATH whose actual stage geometry is not a real AEC
    row (no asset pull needed for this one -- a bare cube, same fixture
    `test_quake_v5_city.py::_author_cube` uses). `aec_quake.quake_row` must
    raise (measure_row finds no named unit meshes), and `_mono_pass` must
    fall all the way through to the generic monolith branches unchanged --
    this is exactly what keeps `test_quake_v5_city.py`'s own
    `test_pristine_pack_gate_now_only_covers_downtowncity` passing."""
    from pxr import Gf, Usd, UsdGeom, Vt

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    path = "/World/gen/fake_aec"
    pts = Vt.Vec3fArray([
        Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, -0.5, -0.5),
        Gf.Vec3f(0.5, 0.5, -0.5), Gf.Vec3f(-0.5, 0.5, -0.5),
        Gf.Vec3f(-0.5, -0.5, 0.5), Gf.Vec3f(0.5, -0.5, 0.5),
        Gf.Vec3f(0.5, 0.5, 0.5), Gf.Vec3f(-0.5, 0.5, 0.5)])
    face_counts = [4, 4, 4, 4, 4, 4]
    face_idx = [0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 5, 4,
               1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr(Vt.IntArray(face_counts))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(face_idx))
    m.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    xf = UsdGeom.Xformable(m)
    xf.AddScaleOp().Set(Gf.Vec3f(20.0, 20.0, 20.0))

    import scene_generator  # noqa: F401  -- proves the module import chain is fine

    def _force_dg4(*_a, **_k):
        return "DG4"

    config = {"usds": {"buildings": {"destroyed": []}}}
    records, tally = [], {}
    placements = [dict(
        category="house", prim_path=path, x_m=0.0, y_m=0.0, yaw_deg=0.0,
        usd="airstack://scene_gen/assets/aec/brownstone/Assets/"
            "Create_Brownstone02/Reference_Brownstone2Row.usd")]

    orig = q.qf.level_for_intensity
    q.qf.level_for_intensity = _force_dg4
    try:
        n = q._mono_pass(stage, config, placements, lambda x, y: 1.0, 1.0,
                         random.Random(3), 1.0, records, tally, verbose=False)
    finally:
        q.qf.level_for_intensity = orig

    assert n == 1
    assert len(records) == 1
    # fell through to the generic fallback: "DG4+tilt", never "AEC_DG4"
    assert records[0]["grade"] == "DG4+tilt"

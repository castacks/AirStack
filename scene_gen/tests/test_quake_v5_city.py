#!/usr/bin/env python3
"""test_quake_v5_city.py — round 6: `urban_quake_v5`'s asset-set semantics
(extends `urban_gac`, `+` append, no bare-key pool replacement) and the
`disaster.quake` decide-layer guarantee that downtowncity never receives an
earthquake damage decision.

    pytest -q scene_gen/tests/test_quake_v5_city.py

Uses pytest's `monkeypatch` fixture throughout (no `tmp_path` needed), so —
like `test_quake_gac_city.py` — this file has no `__main__` runner; invoke it
with pytest.

WHY THIS EXISTS
---------------
User request (2026-08-31, verbatim): "I want you to use the newer layout gen
we have that includes buildings from GAC and MCE. You can place downtown
city env buildings but only undamaged." `urban_quake_v5.yaml` wires this by
extending `urban_gac` (GAC + downtowncity + AEC + the original urban.yaml
kit/MCE/Muyang/Dmytro stock, unchanged) rather than re-curating a subset the
way `urban_quake_v3`/`v4` did — verified here to place BYTE-FOR-BYTE the same
buildings `urban_gac` itself would (`test_v5_building_pools_are_byte_for_
byte_urban_gacs`). The new behaviour is entirely in `disaster/quake.py`:
`PRISTINE_PACKS` / `_is_pristine_pack` makes a downtowncity placement skip
`_mono_pass`'s rigid lean/sink/ruin-swap fallback outright — tested here end
to end on a real (in-memory) USD stage, no Isaac Sim.

ROUND 6c (2026-08-31, later the same day): a MORE SPECIFIC request — the
earthquake TEST/showcase scene — asks for the opposite of round 6 for the
AEC brownstones specifically: "AEC brownstones, GAC, and MCE kit buildings
must all appear damaged. DowntownCity buildings must be SKIPPED for damage —
placed pristine as filler only." `PRISTINE_PACKS` is narrowed to
`("downtowncity/",)` (see its own header comment in `quake.py`) — an AEC
brownstone now falls through to the SAME generic `_mono_pass` fallback a
standalone monolith already got. Every test below that asserted "AEC is
pristine" is updated in place to assert the new behaviour instead; nothing
about the downtowncity guarantee changed, so those assertions are untouched.

FOLLOW-UP (round 6): `config/presets/downtown_earthquake.yaml`'s
ring/typology section now mirrors `downtown_gac.yaml`'s five rings / six
typologies (`lowrise`/`brick_midrise`/`highrise` included), replacing the
three (`rowhouse`/`midrise`/`tower`, `rowhouse` always at mix weight 0.00)
that made downtowncity/AEC structurally unreachable at layout time — see
that file's own `rings:`/`typologies:` comments for the full delta.
`test_v5_layout_places_at_least_one_downtowncity_and_one_aec_building`
(below) proves both pools still place on a REAL region-500 layout, via
`tools/layout_dry_run.run_one` — not the synthetic 3-building stage the
`_mono_pass` tests above use — even though (round 6c) only the downtowncity
half of that placement is still pristine at the DECISION layer.

WHAT THIS CANNOT SEE: whether a pristine downtowncity building, or a now-
damaged AEC brownstone, actually reads right next to a neighbour in a
render.
"""

import os
import random
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..", "tools")))

from disaster import quake as q               # noqa: E402
from disaster import kit_substitute as ks      # noqa: E402
import scene_generator as sg                   # noqa: E402


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
_DTC_USD = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
           "scene_gen/assets/downtowncity/Amar_Tower.usdc")
_AEC_USD = ("airstack://scene_gen/assets/aec/brownstone/Assets/"
           "Create_Brownstone02/Reference_Brownstone2Row.usd")
_GAC_USD = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
           "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/"
           "SM_Building_10.usd")
_MCE_USD = ("Muyang/ModernCityEnvironment/Collected_Building01/"
           "SM_MERGED_BP_MBuilding01.usd")
_KIT_USD = "omniverse://host/archetype/bld_office_DG3.usd"
_MONO_USD = "Muyang/DownTown/Assets/BG_Building_A.usd"
_STANDALONE_USD = ("omniverse://host/scene_gen/assets/standalone/buildings/"
                   "intact/tower/office_tower/office_tower.usdc")


# ---------------------------------------------------------------------------
# `_is_pristine_pack` — the pure pack-recognition helper
# ---------------------------------------------------------------------------
def test_module_exposes_pristine_pack_gate():
    assert hasattr(q, "PRISTINE_PACKS")
    assert hasattr(q, "_is_pristine_pack")
    # round 6c (2026-08-31): narrowed to downtowncity only — see quake.py's
    # own header comment above PRISTINE_PACKS for why AEC was removed.
    assert q.PRISTINE_PACKS == ("downtowncity/",)


def test_is_pristine_pack_recognizes_downtowncity():
    assert q._is_pristine_pack(_DTC_USD)
    # a bare relative path (no scheme prefix at all) must also match — the
    # same suffix/substring discipline `_is_gac` uses
    assert q._is_pristine_pack("scene_gen/assets/downtowncity/Building_11.usdc")


def test_is_pristine_pack_no_longer_recognizes_aec():
    """Round 6c (2026-08-31): `"assets/aec/"` was removed from
    `PRISTINE_PACKS` — the showcase-scene request wants AEC brownstones
    damaged, not skipped. An AEC brownstone now falls through to
    `_mono_pass`'s generic rigid-lean fallback like any other
    non-kit/non-GAC monolith; see `test_pristine_pack_gate_now_only_covers_
    downtowncity` below for the end-to-end proof."""
    assert not q._is_pristine_pack(_AEC_USD)
    assert not q._is_pristine_pack("scene_gen/assets/aec/brownstone/x.usd")


def test_is_pristine_pack_rejects_everything_else():
    assert not q._is_pristine_pack(_GAC_USD)
    assert not q._is_pristine_pack(_MCE_USD)
    assert not q._is_pristine_pack(_KIT_USD)
    assert not q._is_pristine_pack(_MONO_USD)
    assert not q._is_pristine_pack(_STANDALONE_USD)
    assert not q._is_pristine_pack(_AEC_USD)   # round 6c: no longer pristine


def test_pristine_and_gac_pack_recognition_never_overlap():
    """A path cannot be BOTH pristine and GAC — `downtowncity/` /
    `assets/aec/` and `GreatAmericanCity/` never co-occur in a real asset
    path, but this pins the invariant so a future rename cannot quietly
    make one pack shadow the other in `_mono_pass`'s gate ordering."""
    for usd in (_DTC_USD, _AEC_USD, _GAC_USD, _MCE_USD, _KIT_USD, _MONO_USD):
        assert not (q._is_pristine_pack(usd) and q._is_gac(usd))


def test_pristine_and_aec_fallback_packs_are_pack_of_other_not_kit_or_same_art():
    """Sanity-checks the routing assumption both `_mono_pass`'s gate AND its
    generic fallback rely on: downtowncity/AEC are never classified `kit` or
    `same_art` by `kit_substitute.pack_of` (which would route them through a
    DIFFERENT branch of `assemble`'s main loop that never even reaches
    `_mono_pass`) — they only ever reach `_mono_pass`'s call site via the
    `pack_of(usd) == "other"` / not-GAC fallthrough `assemble`'s main loop
    already documents. Round 6c: only downtowncity is then skipped BY the
    gate; the AEC brownstone reaches `_mono_pass`'s body proper."""
    for usd in (_DTC_USD, _AEC_USD):
        assert ks.pack_of(usd) == "other"
        assert not q._is_gac(usd)
    assert q._is_pristine_pack(_DTC_USD)
    assert not q._is_pristine_pack(_AEC_USD)


# ---------------------------------------------------------------------------
# `_mono_pass` end to end, on a real in-memory USD stage — no Isaac Sim
# ---------------------------------------------------------------------------
def _author_cube(stage, path, cx):
    """An 8-point unit-cube Mesh, translated to (cx, 0, 0) and scaled to a
    20 m box — enough for `_mono_dims`'s `UsdGeom.BBoxCache.ComputeWorldBound`
    to measure a real, non-empty footprint. Same construction
    `tests/test_bake_instancer.py::_author_cube_mesh` already uses."""
    from pxr import Gf, UsdGeom, Vt

    pts = Vt.Vec3fArray([
        Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, -0.5, -0.5),
        Gf.Vec3f(0.5, 0.5, -0.5), Gf.Vec3f(-0.5, 0.5, -0.5),
        Gf.Vec3f(-0.5, -0.5, 0.5), Gf.Vec3f(0.5, -0.5, 0.5),
        Gf.Vec3f(0.5, 0.5, 0.5), Gf.Vec3f(-0.5, 0.5, 0.5)])
    face_counts = [4, 4, 4, 4, 4, 4]
    face_idx = [0, 1, 2, 3,  4, 5, 6, 7,  0, 1, 5, 4,
               1, 2, 6, 5,  2, 3, 7, 6,  3, 0, 4, 7]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr(Vt.IntArray(face_counts))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(face_idx))
    m.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    xf = UsdGeom.Xformable(m)
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, 0.0, 0.0))
    xf.AddScaleOp().Set(Gf.Vec3f(20.0, 20.0, 20.0))
    return m.GetPrim()


def _three_placements():
    """A downtowncity building (still gated pristine), an AEC brownstone
    (round 6c: no longer gated — falls through to the SAME `_mono_pass`
    fallback as a plain monolith), and a plain (non-pristine) Muyang
    DownTown monolith — the CONTROL, unaffected by either round."""
    return [
        dict(category="house", prim_path="/World/gen/dtc", x_m=0.0, y_m=0.0,
             yaw_deg=0.0, usd=_DTC_USD),
        dict(category="house", prim_path="/World/gen/aec", x_m=50.0, y_m=0.0,
             yaw_deg=0.0, usd=_AEC_USD),
        dict(category="house", prim_path="/World/gen/mono", x_m=100.0, y_m=0.0,
             yaw_deg=0.0, usd=_MONO_USD),
    ]


@pytest.mark.parametrize("forced_grade", ["DG0", "DG1", "DG2", "DG3", "DG4", "DG5"])
def test_pristine_pack_gate_now_only_covers_downtowncity(
        forced_grade, monkeypatch):
    """Round 6c: `PRISTINE_PACKS` narrowed to `("downtowncity/",)` (see
    `quake.py`'s own header comment above the tuple). downtowncity is still
    skipped BEFORE `_mono_pass` even measures it or draws a grade, no matter
    what grade the field/rng would otherwise have produced — proved here by
    forcing `quake_flow.level_for_intensity` (every candidate that is NOT
    skipped draws this exact grade) across the whole EMS-98 ladder. The AEC
    brownstone now reaches `_mono_pass`'s body exactly like the plain Muyang
    monolith CONTROL — both get measured, both get a grade, both get a
    `records` entry, at every grade including DG4/DG5 where `_mono_pass`
    actually authors a rigid-lean transform (`_tilt_prim`) — proving the
    harness itself exercises the full pass, not a code path that happens to
    no-op."""
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    for p in _three_placements():
        _author_cube(stage, p["prim_path"], p["x_m"])

    monkeypatch.setattr(q.qf, "level_for_intensity",
                        lambda *a, **k: forced_grade)

    config = {"usds": {"buildings": {"destroyed": []}}}
    records, tally = [], {}
    n = q._mono_pass(stage, config, _three_placements(),
                     lambda x, y: 1.0, 1.0, random.Random(0), 1.0,
                     records, tally, verbose=False)

    # downtowncity is the ONLY one still gated out; AEC + the Muyang control
    # both get a decision now.
    assert n == 2
    assert [r["prim"] for r in records] == ["/World/gen/aec", "/World/gen/mono"]
    assert sum(tally.values()) == 2
    # and downtowncity's geometry truly was never touched: still an
    # untransformed 20 m cube at its original placement, no extra xform ops
    prim = stage.GetPrimAtPath("/World/gen/dtc")
    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    assert [op.GetOpName() for op in ops] == \
        ["xformOp:translate", "xformOp:scale"]
    assert ops[0].Get() == __import__("pxr").Gf.Vec3d(0.0, 0.0, 0.0)


def test_mono_pass_docstring_names_the_pristine_gate():
    assert "PRISTINE_PACKS" in q._mono_pass.__doc__


# ---------------------------------------------------------------------------
# eq500_v4 regression (2026-09-01): `/World/stage/generated/house_29_241`
# (`SM_Building_31`, the GAC `highrise`-pool supertall) recorded W=60.3,
# D=142.2, H=302.2 in `quake_buildings.json` — a real, CORRECTLY measured
# 302 m tower (confirmed against `urban_gac.yaml`'s own pre-recorded comment
# for that exact asset), not an axis/unit bug. But the shape of the numbers
# (D at 142 m, close to a plausible "swapped" height) is exactly what an
# axis-confusion bug WOULD look like, so this locks down that `_mono_dims`
# never lets a yaw-driven footprint swap touch H, and that the new
# `_warn_if_oversized` guard actually fires for a building this tall and
# stays quiet for an ordinary one — see `downtown_earthquake.yaml`'s
# `overrides.usds.buildings.highrise`, which now excludes SM_Building_31 and
# its 312 m sibling SM_Building_16 for this preset specifically.
# ---------------------------------------------------------------------------
def _author_box(stage, path, cx, sx, sy, sz, yaw_deg=0.0):
    """Like `_author_cube`, but with an independent per-axis scale (and an
    optional yaw) so a NON-cubic footprint — W != D != H, as every real
    building is — can be measured. `_mono_dims` reads world-space extents,
    so the yaw here is authored as a real `rotateZ` xform op, exactly what
    `apply_placements` writes for a placement's own `yaw_deg`."""
    from pxr import Gf, UsdGeom, Vt

    pts = Vt.Vec3fArray([
        Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, -0.5, -0.5),
        Gf.Vec3f(0.5, 0.5, -0.5), Gf.Vec3f(-0.5, 0.5, -0.5),
        Gf.Vec3f(-0.5, -0.5, 0.5), Gf.Vec3f(0.5, -0.5, 0.5),
        Gf.Vec3f(0.5, 0.5, 0.5), Gf.Vec3f(-0.5, 0.5, 0.5)])
    face_counts = [4, 4, 4, 4, 4, 4]
    face_idx = [0, 1, 2, 3,  4, 5, 6, 7,  0, 1, 5, 4,
               1, 2, 6, 5,  2, 3, 7, 6,  3, 0, 4, 7]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr(Vt.IntArray(face_counts))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(face_idx))
    m.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    xf = UsdGeom.Xformable(m)
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, 0.0, 0.0))
    xf.AddRotateZOp().Set(float(yaw_deg))
    xf.AddScaleOp().Set(Gf.Vec3f(float(sx), float(sy), float(sz)))
    return m.GetPrim()


@pytest.mark.parametrize("yaw_deg", [0.0, 90.0, 180.0, 270.0])
def test_mono_dims_reports_the_same_canonical_wdh_at_every_cardinal_yaw(
        yaw_deg):
    """The exact numbers `quake_buildings.json` recorded for house_29_241
    (SM_Building_31): W=60.3, D=142.2, H=302.2. `_mono_dims`'s docstring
    promises "(W, D, H) of a placed monolith in ITS OWN yaw frame" — the
    world-space bbox at a 90/270 placement yaw already comes out with sx/sy
    swapped (a box rotated a quarter turn measures its depth along world X),
    and the function's own `if 45 < yaw < 135: sx, sy = sy, sx` swaps them
    BACK, undoing the rotation's effect so the returned (W, D) is always the
    asset's own canonical footprint, regardless of which cardinal direction
    the layout yawed it. So a correct measurement must return the SAME
    (W, D, H) at all four cardinal yaws; a real axis/unit bug (a Y-up asset
    measured as Z-up, or a stray extra scale factor) would instead show H
    drifting with yaw, or W/D/H permuted together instead of staying put."""
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    prim = _author_box(stage, "/World/gen/tower", 0.0,
                       60.3, 142.2, 302.2, yaw_deg=yaw_deg)

    dims = q._mono_dims(stage, prim, {"yaw_deg": yaw_deg})
    assert dims is not None
    W, D, H = dims
    assert W == pytest.approx(60.3, abs=0.05)
    assert D == pytest.approx(142.2, abs=0.05)
    assert H == pytest.approx(302.2, abs=0.05)   # NEVER swapped with W or D


# ---------------------------------------------------------------------------
# `_warn_if_oversized` — the round-6 follow-up guard
# ---------------------------------------------------------------------------
def test_warn_if_oversized_fires_above_threshold_and_names_the_asset(capsys):
    q._MONO_HEIGHT_WARNED.clear()
    q._warn_if_oversized("SM_Building_31.usd", 60.3, 142.2, 302.2,
                         "/World/stage/generated/house_29_241")
    out = capsys.readouterr().out
    assert "WARNING" in out
    assert "SM_Building_31.usd" in out
    assert "/World/stage/generated/house_29_241" in out


def test_warn_if_oversized_stays_quiet_at_and_under_the_threshold(capsys):
    q._MONO_HEIGHT_WARNED.clear()
    q._warn_if_oversized("bld_office_DG0.usd", 32.4, 17.8, 43.0, "/World/gen/x")
    q._warn_if_oversized("bld_tower_DG0.usd", 25.6, 20.6,
                         q.MONO_HEIGHT_WARN_M, "/World/gen/y")   # == cap: quiet
    assert capsys.readouterr().out == ""


def test_warn_if_oversized_is_deduped_per_asset_not_per_call(capsys):
    """One loud warning per OVERSIZED USD, not one per placement — a scene
    with many copies of the same too-tall building must not flood the log."""
    q._MONO_HEIGHT_WARNED.clear()
    q._warn_if_oversized("SM_Building_31.usd", 60.3, 142.2, 302.2, "/a")
    q._warn_if_oversized("SM_Building_31.usd", 60.3, 142.2, 302.2, "/b")
    first_two = capsys.readouterr().out
    assert first_two.count("WARNING") == 1

    q._warn_if_oversized("SM_Building_16.usd", 84.5, 56.9, 312.0, "/c")
    third = capsys.readouterr().out
    assert "SM_Building_16.usd" in third


def test_warn_if_oversized_silent_when_verbose_is_false(capsys):
    q._MONO_HEIGHT_WARNED.clear()
    q._warn_if_oversized("SM_Building_31.usd", 60.3, 142.2, 302.2, "/a",
                         verbose=False)
    assert capsys.readouterr().out == ""
    # verbose=False must not itself count as "already warned" — a later,
    # verbose call for the SAME asset still has to print.
    q._warn_if_oversized("SM_Building_31.usd", 60.3, 142.2, 302.2, "/a")
    assert "WARNING" in capsys.readouterr().out


def test_mono_pass_warns_on_an_oversized_monolith_end_to_end(capsys):
    """`_mono_pass` end to end (the same harness `_three_placements` uses,
    real in-memory USD stage, no Isaac Sim): an oversized monolith placed
    alongside the ordinary control both gets a correct, un-corrupted
    `records` entry AND trips the loud warning — proving the guard is wired
    into the actual code path `quake_buildings.json` is built from, not just
    reachable in isolation."""
    from pxr import Usd, UsdGeom

    q._MONO_HEIGHT_WARNED.clear()
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    oversized_usd = "Muyang/DownTown/Assets/BG_Building_Z.usd"
    placements = [
        dict(category="house", prim_path="/World/gen/mono", x_m=0.0, y_m=0.0,
             yaw_deg=0.0, usd=_MONO_USD),
        dict(category="house", prim_path="/World/gen/tower", x_m=100.0,
             y_m=0.0, yaw_deg=180.0, usd=oversized_usd),
    ]
    _author_cube(stage, "/World/gen/mono", 0.0)
    _author_box(stage, "/World/gen/tower", 100.0, 60.3, 142.2, 302.2,
               yaw_deg=180.0)

    records, tally = [], {}
    n = q._mono_pass(stage, {"usds": {"buildings": {"destroyed": []}}},
                     placements, lambda x, y: 0.05, 1.0, random.Random(0),
                     1.0, records, tally, verbose=True)

    assert n == 2
    tower_rec = next(r for r in records if r["prim"] == "/World/gen/tower")
    assert tower_rec["W"] == pytest.approx(60.3, abs=0.1)
    assert tower_rec["D"] == pytest.approx(142.2, abs=0.1)
    assert tower_rec["H"] == pytest.approx(302.2, abs=0.1)

    out = capsys.readouterr().out
    assert "WARNING" in out
    assert "BG_Building_Z.usd" in out
    assert "/World/gen/tower" in out


# ---------------------------------------------------------------------------
# `_same_art_material` — the tagged-match-wins fix (round 6)
# ---------------------------------------------------------------------------
def test_same_art_material_prefers_a_tagged_match_over_an_earlier_untagged_one():
    """Regression test for the bug this round found and fixed: when the SAME
    asset appears in more than one pool — an untagged copy in an ordinary
    typology pool (as `urban.yaml`'s own `intact`/`midrise` carry MBuilding01
    bare) AND a tagged copy in a later, metadata-only pool
    (`urban_quake_v5.yaml`'s `quake_material_tags`) — the function must
    return the TAGGED entry's material, not silently default because the
    untagged copy happened to be scanned first."""
    config = {"usds": {"buildings": {
        "intact": [{"usd": _MCE_USD, "scale": 1.0}],       # untagged, first
        "midrise": [{"usd": _MCE_USD, "scale": 1.0}],      # untagged, first
        "quake_material_tags": [{"usd": _MCE_USD, "material": "urm"}],
    }}}
    assert q._same_art_material(_MCE_USD, config) == "urm"


def test_same_art_material_still_defaults_when_no_match_is_ever_tagged():
    config = {"usds": {"buildings": {
        "intact": [{"usd": _MCE_USD, "scale": 1.0}],
        "midrise": [{"usd": _MCE_USD, "scale": 1.0}],
    }}}
    assert q._same_art_material(_MCE_USD, config) == q._SAME_ART_DEFAULT_TYPE


# ---------------------------------------------------------------------------
# `urban_quake_v5.yaml` — extends + append semantics, byte-for-byte building
# list, and the two intentional `usds.buildings` overrides
# ---------------------------------------------------------------------------
def _resolve(name):
    return sg.resolve_asset_set({"asset_set": name})


def test_v5_extends_urban_gac():
    import yaml
    with open(os.path.join(_HERE, "..", "config", "asset_sets",
                           "urban_quake_v5.yaml")) as fh:
        doc = yaml.safe_load(fh)
    assert doc["extends"] == "urban_gac"


def test_v5_building_pools_are_byte_for_byte_urban_gacs():
    """The whole point of this round's asset-set work: v5 must place EXACTLY
    what `urban_gac` places — no buildings grafted in. Every pool a typology
    can actually draw from (everything except the two intentional
    overrides) must come through `_merge_asset_set`'s `+`-append /
    bare-inherit machinery UNCHANGED."""
    v5 = _resolve("urban_quake_v5")["usds"]["buildings"]
    gac = _resolve("urban_gac")["usds"]["buildings"]
    # `damaged`/`destroyed` are deliberately CLEARED (own test below) —
    # every OTHER pool, the ones a typology can actually draw a building
    # from, must be untouched.
    real_pools = set(gac.keys()) - {"damaged", "destroyed"}
    assert real_pools, "urban_gac resolved with no building pools at all"
    for pool in real_pools:
        assert v5.get(pool) == gac.get(pool), \
            f"pool {pool!r} diverged between urban_quake_v5 and urban_gac"


def test_v5_damaged_and_destroyed_are_cleared():
    """Bare `damaged: []` / `destroyed: []` — a deliberate CLEAR, the same
    two lines `urban_quake.yaml` carries, so no pre-baked ruin ever gets
    packed at layout time regardless of what a future preset's
    damaged_fraction/destroyed_fraction happen to be."""
    v5 = _resolve("urban_quake_v5")["usds"]["buildings"]
    gac = _resolve("urban_gac")["usds"]["buildings"]
    assert v5["damaged"] == []
    assert v5["destroyed"] == []
    # and prove this is actually a CLEAR, not a no-op — urban_gac's own
    # inherited pools are non-empty before v5 clears them
    assert gac["damaged"]
    assert gac["destroyed"]


def test_v5_append_semantics_do_not_replace_an_inherited_pool():
    """The one-character trap every asset set in this chain's header warns
    about: `urban_quake_v5.yaml` must never write a BARE key over a pool
    `urban_gac`/`urban` already owns (that would silently REPLACE it,
    dropping the base's own content). Proven two ways: (1) v5's own YAML
    source declares no bare building-pool key other than the two documented
    overrides; (2) an inherited NON-building pool (greenery — `urban_gac`
    explicitly never touches it, so it must survive from `urban.yaml`
    unchanged) still carries real content, not the single-tree collapse
    this exact trap caused once before (see `urban_gac.yaml`'s own header)."""
    import yaml
    with open(os.path.join(_HERE, "..", "config", "asset_sets",
                           "urban_quake_v5.yaml")) as fh:
        doc = yaml.safe_load(fh)
    bare_building_keys = {k for k in doc["usds"]["buildings"]
                          if not str(k).endswith("+")}
    assert bare_building_keys == {"damaged", "destroyed", "quake_material_tags"}

    v5 = _resolve("urban_quake_v5")["usds"]
    urban = _resolve("urban")["usds"]
    assert len(v5.get("trees") or []) > 1
    assert v5.get("trees") == urban.get("trees")


def test_v5_quake_material_tags_is_metadata_only_never_a_typology_pool():
    """`quake_material_tags` must never collide with a real typology pool
    name — if it did, `districts.typologies.*.pools` could accidentally draw
    houses from it. This does not re-derive the full preset wiring (that is
    what `tools/layout_dry_run.py`'s dry run proves empirically — see
    `_plans/eq_v5_city_dry_run.md`, whose `by_style` table is byte-identical
    between `urban_gac` and `urban_quake_v5` at the same region/seed); it
    pins the cheap, permanent half of that guarantee."""
    v5 = _resolve("urban_quake_v5")["usds"]["buildings"]
    real_pool_names = {"intact", "rowhouse", "midrise", "midrise_v2", "tower",
                       "lowrise", "brick_midrise", "highrise", "damaged",
                       "destroyed"}
    assert "quake_material_tags" not in real_pool_names
    assert v5["quake_material_tags"]        # non-empty, actually present
    for e in v5["quake_material_tags"]:
        assert isinstance(e, dict) and "usd" in e and "material" in e


def test_v5_quake_material_tags_covers_all_three_mce_and_all_31_gac_names():
    v5 = _resolve("urban_quake_v5")["usds"]["buildings"]["quake_material_tags"]
    names = {os.path.splitext(os.path.basename(e["usd"]))[0] for e in v5}
    mce = {"SM_MERGED_BP_MBuilding01", "SM_MERGED_BP_MBuilding02",
          "SM_MERGED_BP_MBuilding05"}
    gac = {f"SM_Building_{n:02d}" for n in range(1, 32)} - {"SM_Building_06"}
    gac.add("SM_Building_06_Small")
    assert mce <= names
    assert gac <= names
    assert len(v5) == 34


# ---------------------------------------------------------------------------
# GAC-no-bake stays kept (existing behaviour, pinned unchanged under v5)
# ---------------------------------------------------------------------------
def test_gac_no_bake_stays_kept():
    """`decide_gac_building` (unmodified by this round) must still keep a
    GAC building's OWN mesh reference — never swap it to an unrelated
    building or a generic ruin — when no per-building bake exists at the
    drawn grade. `_mono_pass` may still apply its generic rigid-lean
    fallback to a kept GAC building (unchanged, documented in
    `urban_quake_v5.yaml`'s own header) — that is a POSE change, not an
    asset swap, and is out of scope for this assertion."""
    rng = random.Random(0)
    for grade in ("DG1", "DG2", "DG3", "DG4", "DG5"):
        out = q.decide_gac_building(_GAC_USD, grade, {}, rng,
                                    x=1.0, y=2.0, yaw_deg=90.0)
        assert out["action"] == "keep"
        assert out["grade"] == grade
        assert "SM_Building_10" in out.get("reason", "")
        # pose rides through untouched regardless of the decision
        assert (out["x"], out["y"], out["yaw_deg"]) == (1.0, 2.0, 90.0)


# ---------------------------------------------------------------------------
# PLACEMENT-LEVEL PIN: a REAL region-500 `urban_quake_v5` layout actually
# contains downtowncity and AEC buildings (round-6 follow-up:
# `downtown_earthquake.yaml`'s ring/typology section now mirrors
# `downtown_gac.yaml`'s — see this file's own header). This builds the FULL
# city (`tools.layout_dry_run.run_one`, the same call `_plans/
# eq_v5_city_dry_run.md` was generated from), so it is slower than every
# other test in this file and the only one that is — no Isaac Sim, still
# pure `usd-core` + `pxr`, same as the rest of this module.
#
# ROUND 6c: only downtowncity is still gated PRISTINE at the decision layer
# — the test name and body below dropped "_both_pristine" and the
# `pristine_aec` assertion accordingly. `_pack_breakdown` (`tools/
# layout_dry_run.py`) now folds an AEC placement into `standalone_other`
# (alongside Muyang DownTown / Dmytro FactoryDistrict / the `assets/
# standalone/` drop) instead of a dedicated `pristine_aec` bucket, because
# `_is_pristine_pack` no longer matches it — see `test_is_pristine_pack_no_
# longer_recognizes_aec` above. This test still proves the LAYOUT places an
# AEC building at this seed; the real per-building proof that it then
# receives a grade (including DG4+tilt / TILT) is `disaster.quake.assemble`
# run directly against this same seed-9 layout, reported separately (not a
# claim this placement-only test needs to re-derive).
# ---------------------------------------------------------------------------
def test_v5_layout_places_at_least_one_downtowncity_and_one_aec_building():
    """SEED 9, not this preset's own committed default (42): WHICH zoning
    nucleus forms (Harris & Ullman's multiple-nuclei model,
    `detail/districts.py`) is randomised per seed, and the AEC `rowhouse`
    typology is drawn at only the `edge` ring's mix — so whether a
    rowhouse-ranked area gets a block AT ALL varies by seed. Measured over
    seeds 1-12 at this exact region/asset-set (`_plans/eq_v5_city_dry_run.
    md`'s own note, from before round 6c renamed the bucket), an AEC block
    was nonzero for 5 of 12 (seed 9 among them; the default seed 42 draws 0
    AEC this region). This pins a SPECIFIC, reproducible seed that shows
    both — a proof the ring/typology wiring works end to end on a real
    layout, not a claim that every seed places an AEC building."""
    import layout_dry_run as ldr    # tools/layout_dry_run.py, path added above

    res = ldr.run_one(500.0, asset_set="urban_quake_v5", seed=9)
    pb = res["pack_breakdown"]

    assert pb.get("pristine_downtowncity", 0) >= 1, \
        f"expected >=1 downtowncity building at seed 9, got {pb}"
    # Round 6c: `pristine_aec` no longer exists as a label at all -- confirm
    # that directly (a regression here would mean `_pack_breakdown` drifted
    # from `_is_pristine_pack` again) -- and that AEC's placements instead
    # show up in the generic `standalone_other` bucket it now shares with
    # Muyang DownTown / Dmytro FactoryDistrict / the `assets/standalone/`
    # drop.
    assert "pristine_aec" not in pb, \
        f"pristine_aec should no longer be produced by _pack_breakdown, got {pb}"
    assert pb.get("standalone_other", 0) >= 1, \
        f"expected >=1 standalone/other-pack building (AEC folds in here " \
        f"now) at seed 9, got {pb}"

    # `pack_breakdown`'s buckets are DEFINED by `_is_pristine_pack` /
    # `kit_substitute.pack_of` / `_is_gac` (`_pack_breakdown`'s own
    # docstring), so this is not circular — it is the SAME classification
    # `_mono_pass`'s gate uses, run here against houses `generate_scene_on_
    # stage` actually placed rather than the hand-authored ones the tests
    # above use. Confirm the total accounts for every placed house with none
    # left unclassified or double-counted.
    assert sum(pb.values()) == res["n_buildings"]
    assert set(pb.keys()) <= {"kit", "mce_same_art", "gac",
                              "pristine_downtowncity", "standalone_other"}

#!/usr/bin/env python3
"""test_roof_plant_seat_probe.py — pytest wrapper over
`tools/roof_plant_seat_probe.py`: does every rooftop-plant cluster in every
baked quake archetype rest on real geometry, checked OFFLINE against the
exported `.usd` on disk (no Isaac Sim, no Kit — bare `pxr` + `numpy`, same as
the probe itself).

WHY THIS IS A SEPARATE FILE, NOT PART OF `test_quake_collapse.py`: that file
tests the AUTHORING code (`quake_collapse.py`) against in-memory synthetic
stages it builds itself; this one tests the ARTEFACT — the actual baked
`.usd` files under `scene_gen/assets/archetype/` — which may or may not
exist on a given machine (they are a multi-GB bake output, not checked into
git) and may be STALE relative to the authoring code (that staleness is
exactly what round 6 found: every record in `archetypes.json` predates the
`deactivate_airborne` integration). A stale bake failing this test is not a
test bug — it is the gate this test exists to be, and the reason a rebake
is queued rather than assumed clean.

SKIPS CLEANLY, ALWAYS, WHEN THE ASSETS ARE ABSENT: `pytest.importorskip`-
style guard at collection time, not a hard failure, so a checkout without
the (multi-GB, gitignored) archetype bake does not fail the suite.

NOT FAST. `_deck_support_z` re-traverses one building's own Mesh prims per
rooftop-plant cluster it checks, deliberately (the geometric check has to be
right, not quick — it is a pre-rebake gate, not a per-commit one). MEASURED:
~8 s for `bld_brownstone_row_DG3.usd` (230 prims, 5 clusters); the full
`scene_gen/assets/archetype/*.usd` directory (162 files at last count) is
tens of minutes, not seconds — run it as its own step before a rebake, not
inline in a quick `pytest -q`. `ROOF_PLANT_PROBE_LIMIT=N` caps the file
count for a fast smoke pass (first N files, alphabetical) without touching
the gate's default (every file) when unset."""
import glob
import os

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
_ARCH_DIR = os.path.join(_SG, "assets", "archetype")
_ARCH_GLOB = os.path.join(_ARCH_DIR, "*.usd")

_FILES = sorted(glob.glob(_ARCH_GLOB))
_LIMIT = os.environ.get("ROOF_PLANT_PROBE_LIMIT", "").strip()
if _LIMIT:
    _FILES = _FILES[:int(_LIMIT)]

pytestmark = pytest.mark.skipif(
    not _FILES,
    reason="no baked quake archetypes under {0} (multi-GB bake output, not "
           "checked in) -- nothing to gate".format(_ARCH_DIR))


def _rp():
    import sys
    sys.path.insert(0, os.path.join(_SG, "tools"))
    import roof_plant_seat_probe as rp
    return rp


@pytest.mark.parametrize("usd_path", _FILES, ids=[os.path.basename(p) for p in _FILES])
def test_every_roof_plant_cluster_rests_on_real_geometry(usd_path):
    """The gate: EVERY rooftop-plant cluster in this archetype must have
    real, upward-facing support within `roof_plant_seat_probe.
    DEFAULT_TOLERANCE_M` of its own resting height. A failure here is a
    floating tank or AC unit exactly like the ones round 6 found by eye in
    `b0_apartment_DG5_obl.png` / `b4_brownstone_row_DG4_obl.png` — this
    test exists so the NEXT one is caught before a render, not after."""
    rp = _rp()
    results = rp.check_archetype(usd_path, verbose=False)
    bad = [r for r in results if not r["ok"]]
    assert not bad, "{0} floating rooftop-plant cluster(s) in {1}:\n{2}".format(
        len(bad), os.path.basename(usd_path),
        "\n".join("  {material} cluster {cluster} @ ({cx:+.1f},{cy:+.1f}): "
                  "base_z={base_z} support_z={support_z} gap={gap}".format(**r)
                  for r in bad))


def test_the_probe_itself_flags_a_synthetic_floater():
    """A probe that never fails is not a gate. Independent of whatever the
    CURRENT archetype bake happens to contain, a synthetic stage with a
    prop over open air must fail `check_archetype`."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade
    rp = _rp()
    qc = rp.qc

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, Sdf.Path("/Baked"))

    def _mat(path):
        # No real shader network needed: `check_archetype` only reads the
        # bound material PRIM's own name (`ComputeBoundMaterial().name`),
        # never its shading network.
        return UsdShade.Material.Define(stage, Sdf.Path(path))

    tank_mat = _mat("/Baked/Looks/tank_dark")

    def _box(path, cx, cy, cz, sx, sy, sz, mat=None):
        hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
        mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
        pts = []
        for dz in (-hz, hz):
            for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
                pts.append(Gf.Vec3f(cx + dx, cy + dy, cz + dz))
        faces, counts = [], []
        for f in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5),
                  (2, 3, 7, 6), (3, 0, 4, 7)):
            faces.extend(f)
            counts.append(4)
        mesh.CreatePointsAttr(pts)
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(faces)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        if mat is not None:
            UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(mat)
        return mesh.GetPrim()

    # a tank, floating (nothing within reach underneath)
    _box("/Baked/merged_tank_dark", 0, 0, 20.0, 1.6, 1.6, 2.0, tank_mat)
    # a real deck, but far below and far away in XY -- must not count
    _box("/Baked/ground", 100, 100, 0.0, 4.0, 4.0, 0.2)

    assert qc._deck_support_z is not None    # imported and reachable
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "synthetic.usd")
        stage.Export(path)
        out = rp.check_archetype(path, verbose=False)
    assert out, "the synthetic tank was not even found as a candidate"
    assert all(not r["ok"] for r in out), \
        "the probe passed a prop with nothing whatsoever underneath it"


def test_export_repair_moves_only_the_floating_cluster(tmp_path):
    """A merged material Mesh may contain both a seated and a floating AC.

    Repairing the whole Mesh would bury the first while seating the second.
    The export repair must discover the disconnected clusters and modify only
    the floating cluster's point range; dry-run must not modify either one.
    """
    import numpy as np
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

    rp = _rp()
    from disaster import bake

    path = str(tmp_path / "merged_roof_plant.usd")
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, Sdf.Path("/Baked"))
    mat = UsdShade.Material.Define(
        stage, Sdf.Path("/Baked/Looks/plant_metal"))

    def box_data(cx, cy, base_z, sx, sy, sz):
        hx, hy = sx / 2.0, sy / 2.0
        z0, z1 = base_z, base_z + sz
        pts = [(cx + dx, cy + dy, z)
               for z in (z0, z1)
               for dx, dy in ((-hx, -hy), (hx, -hy),
                              (hx, hy), (-hx, hy))]
        faces = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
                 (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
        return pts, faces

    # One Mesh, two disconnected boxes more than XY_MERGE_M apart.
    plant = UsdGeom.Mesh.Define(stage, Sdf.Path("/Baked/merged_plant_metal"))
    pts, faces = [], []
    for cx, base in ((-4.0, 0.0), (4.0, 2.0)):
        p, f = box_data(cx, 0.0, base, 1.0, 1.0, 1.0)
        off = len(pts)
        pts.extend(p)
        faces.extend(tuple(off + q for q in face) for face in f)
    plant.CreatePointsAttr([Gf.Vec3f(*q) for q in pts])
    plant.CreateFaceVertexCountsAttr([4] * len(faces))
    plant.CreateFaceVertexIndicesAttr([q for face in faces for q in face])
    plant.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    UsdShade.MaterialBindingAPI(plant.GetPrim()).Bind(mat)

    # Its upper surface is exactly z=0 beneath both objects.
    deck = UsdGeom.Mesh.Define(stage, Sdf.Path("/Baked/deck"))
    p, f = box_data(0.0, 0.0, -0.2, 12.0, 4.0, 0.2)
    deck.CreatePointsAttr([Gf.Vec3f(*q) for q in p])
    deck.CreateFaceVertexCountsAttr([4] * len(f))
    deck.CreateFaceVertexIndicesAttr([q for face in f for q in face])
    deck.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    stage.GetRootLayer().Save()
    stage = None

    before = rp.check_archetype(path, verbose=False)
    assert len(before) == 2
    assert sum(not q["ok"] for q in before) == 1
    dry = bake.reseat_roof_plant_clusters_in_file(
        path, dry_run=True, verbose=False)
    assert len(dry) == 1 and dry[0]["dz"] == -2.0
    assert rp.check_archetype(path, verbose=False) == before

    changed = bake.reseat_roof_plant_clusters_in_file(path, verbose=False)
    assert len(changed) == 1 and changed[0]["dz"] == -2.0
    after = rp.check_archetype(path, verbose=False)
    assert len(after) == 2 and all(q["ok"] for q in after)
    assert sorted(round(q["base_z"], 3) for q in after) == [0.0, 0.0]

    # Independent point check: both disconnected objects now retain the same
    # one-metre height and rest on the deck; neither was deleted or collapsed.
    stage = Usd.Stage.Open(path)
    P, counts, indices = rp._world_points(
        stage.GetPrimAtPath("/Baked/merged_plant_metal"))
    groups = rp._clusters(P, counts, indices)
    assert sorted(round(float(P[g, 2].min()), 3) for g in groups) == [0.0, 0.0]
    assert all(np.isclose(P[g, 2].max() - P[g, 2].min(), 1.0)
               for g in groups)

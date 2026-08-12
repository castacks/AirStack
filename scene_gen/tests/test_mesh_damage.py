"""
Tests for the USD port of the mesh-damage operators.

These need only `pxr` — an in-memory stage with a synthetic box stands in for a
building — so they run on the host with the rest of the suite. No Isaac, no
Nucleus, no assets.

The properties worth guarding are the ones that were easy to get wrong in the
port and would be invisible in a screenshot:

* the world-space round trip through a non-identity transform;
* `lean` and `pancake` leaving the FOOTPRINT alone — a building racks and
  collapses onto its own slab, it does not slide down the street;
* scale-relativity — the same profile on a 15 m house and a 90 m tower should
  do proportionally the same thing, which is why every size in the operators is
  a fraction of the bounding radius rather than a distance in metres;
* determinism, and severity 0 being an exact no-op.
"""

import os
import sys

import numpy as np
import pytest
from pxr import Gf, Usd, UsdGeom

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from disaster import mesh_damage as M  # noqa: E402

# Stages are kept alive for the duration of a test: dropping the Python
# reference invalidates every prim handed out from it, which surfaces as
# "Accessed schema on invalid prim" a long way from the cause.
_STAGES = []


def box(scale=1.0, translate=(0.0, 0.0, 0.0), rot_z=0.0, n=4):
    """A subdivided unit box as a stand-in building. Returns its mesh prims."""
    st = Usd.Stage.CreateInMemory()
    _STAGES.append(st)
    xf = UsdGeom.Xform.Define(st, "/B")
    api = UsdGeom.XformCommonAPI(xf)
    api.SetTranslate(Gf.Vec3d(*translate))
    api.SetRotate(Gf.Vec3f(0.0, 0.0, rot_z))
    api.SetScale(Gf.Vec3f(scale, scale, scale))
    mesh = UsdGeom.Mesh.Define(st, "/B/Mesh")
    mesh.GetPointsAttr().Set([
        Gf.Vec3f(ix / (n - 1) - 0.5, iy / (n - 1) - 0.5, iz / (n - 1))
        for iz in range(n) for ix in range(n) for iy in range(n)])
    mesh.GetFaceVertexCountsAttr().Set([4])
    mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
    return M.mesh_prims(xf.GetPrim())


def test_world_round_trip_under_a_transform():
    """Read world points, write them back unchanged, get the same geometry.

    The adapter is the entire Blender dependency that had to be replaced, and
    a wrong matrix convention here would quietly distort every damaged
    building. USD is row-vector: `world = local @ M[:3,:3] + M[3,:3]`.
    """
    prims = box(scale=2.0, translate=(10.0, -5.0, 3.0), rot_z=37.0)
    before = M.get_points(prims[0]).copy()
    M.set_points(prims[0], before)
    assert np.abs(M.get_points(prims[0]) - before).max() < 1e-9


def test_bounds_reflect_the_world_transform():
    prims = box(scale=2.0, translate=(10.0, -5.0, 3.0))
    b = M.bounds_of(prims)
    assert b is not None
    assert b.height == pytest.approx(2.0, abs=1e-6)   # unit box x scale 2
    assert b.base_z == pytest.approx(3.0, abs=1e-6)   # translated to z=3


@pytest.mark.parametrize("op", ["lean", "pancake"])
def test_footprint_stays_put(op):
    """A building racks or collapses onto its own slab — the base does not move.

    This is the property that makes mesh damage safe to apply after the layout
    is fixed: the geometry fails, the footprint does not migrate.
    """
    prims = box(scale=2.0, translate=(10.0, -5.0, 3.0), rot_z=37.0)
    b = M.bounds_of(prims)
    before = M.get_points(prims[0]).copy()
    if op == "lean":
        M.lean(prims, b, angle_deg=14.0, direction_deg=25.0)
    else:
        M.pancake(prims, b, z_lo=0.1, z_hi=0.35, collapse=0.8, spread=0.0)
    after = M.get_points(prims[0])
    at_base = before[:, 2] < b.base_z + 1e-6
    assert at_base.any()
    assert np.linalg.norm(after[at_base] - before[at_base], axis=1).max() < 1e-6
    assert np.linalg.norm(after - before, axis=1).max() > 1e-3   # did something


def test_profiles_are_scale_relative():
    """The same profile does proportionally the same thing at any size.

    The library spans 15 m houses to 90 m towers, so every size in the
    operators is a fraction of the bounding radius. A metre-valued knob would
    make one preset look right on exactly one asset.
    """
    fracs = []
    for scale in (1.0, 20.0):
        prims = box(scale=scale)
        before = M.get_points(prims[0]).copy()
        M.apply_profile(prims, "earthquake", 0.6, seed=7)
        moved = np.linalg.norm(M.get_points(prims[0]) - before, axis=1).max()
        fracs.append(moved / scale)          # height == scale for a unit box
    assert fracs[0] == pytest.approx(fracs[1], rel=1e-6)


def test_profiles_are_deterministic():
    out = []
    for _ in range(2):
        prims = box()
        M.apply_profile(prims, "tornado", 0.7, seed=11)
        out.append(M.get_points(prims[0]))
    assert np.abs(out[0] - out[1]).max() == 0.0


@pytest.mark.parametrize("disaster,profile", [
    ("earthquake", "structural_collapse"),
    ("explosion", "blast"),
    ("tornado", "wind_shear"),
    ("hurricane", "wind_shear"),
    ("flood", "inundation"),
])
def test_every_disaster_type_maps_to_a_profile(disaster, profile):
    """The taxonomy is fixed; the mapping is the only thing that changes."""
    prims = box()
    before = M.get_points(prims[0]).copy()
    assert M.apply_profile(prims, disaster, 0.8, seed=3) == profile
    assert np.linalg.norm(M.get_points(prims[0]) - before, axis=1).max() > 1e-4


@pytest.mark.parametrize("disaster", ["earthquake", "tornado", "flood", "none"])
def test_zero_severity_is_an_exact_noop(disaster):
    prims = box()
    before = M.get_points(prims[0]).copy()
    M.apply_profile(prims, disaster, 0.0, seed=3)
    assert np.abs(M.get_points(prims[0]) - before).max() == 0.0


def test_flood_barely_deforms_but_earthquake_does():
    """Profiles differ in kind, not just degree — flood is not a small quake.

    `GENERATION.md` says flood causes little structural loss: things float
    away, buildings stay up. That has to show in the geometry.
    """
    def moved(disaster):
        prims = box()
        before = M.get_points(prims[0]).copy()
        M.apply_profile(prims, disaster, 0.8, seed=5)
        return np.linalg.norm(M.get_points(prims[0]) - before, axis=1).max()

    assert moved("flood") < 0.25 * moved("earthquake")


def test_blast_does_not_bury_the_building():
    """A blast throws a building outward; it does not push it into the ground.

    `shockwave` displaces radially from the epicentre, and the blast profile
    seats that epicentre low on a facade — so without a floor clamp every
    vertex below it is driven straight down. Measured on a real brownstone
    before the clamp: the base sank **1.29 m**, which reads as the building
    being swallowed rather than blown apart.
    """
    prims = box(scale=10.0)
    b0 = M.bounds_of(prims)
    M.apply_profile(prims, "explosion", 0.9, seed=13)
    b1 = M.bounds_of(prims)
    sank = b0.base_z - b1.base_z
    assert sank < 0.02 * b0.height, (
        f"base sank {sank:.3f} on a {b0.height:.1f} building — "
        "shockwave needs floor_z=bounds.base_z")


def test_deinstance_opens_nested_instances():
    """Assets ship internally instanced; their meshes cannot take an opinion.

    The AEC brownstone is `/World/Brownstone02_Instanced/...` — 307 meshes
    behind one instance root — and `set_points` raises on an instance proxy.
    Synthetic test geometry never reproduced this, which is why it survived
    until the port met a real asset.
    """
    from pxr import Sdf

    st = Usd.Stage.CreateInMemory()
    _STAGES.append(st)
    src = UsdGeom.Xform.Define(st, "/Proto")
    m = UsdGeom.Mesh.Define(st, "/Proto/Mesh")
    m.GetPointsAttr().Set([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0),
                           Gf.Vec3f(1, 1, 0), Gf.Vec3f(0, 1, 1)])
    inst = UsdGeom.Xform.Define(st, "/Inst").GetPrim()
    inst.GetReferences().AddInternalReference(Sdf.Path("/Proto"))
    inst.SetInstanceable(True)
    assert inst.IsInstance()

    assert M.deinstance(inst) >= 1
    prims = M.mesh_prims(inst)
    assert prims
    pts = M.get_points(prims[0])
    M.set_points(prims[0], pts + 1.0)          # would raise on a proxy
    assert np.abs(M.get_points(prims[0]) - (pts + 1.0)).max() < 1e-5


def test_earthquake_uses_more_than_one_failure_mode():
    """A quake-hit street is mixed, not a row of identical casualties.

    `structural_collapse` used to be one recipe — rack, pancake one storey,
    spall lightly — so every building failed the same way. It now picks among
    six modes, and which one is mostly a function of intensity.
    """
    modes = {M._pick_quake_mode(0.6, np.random.default_rng(i))
             for i in range(200)}
    assert len(modes) >= 4, f"only saw {modes}"


def test_severity_shifts_the_failure_mix_toward_collapse():
    """Low severity leaves buildings standing; high severity brings them down."""
    def share(intensity, wanted):
        picks = [M._pick_quake_mode(intensity, np.random.default_rng(i))
                 for i in range(400)]
        return sum(1 for p in picks if p in wanted) / len(picks)

    standing = {"racking", "spall"}
    collapsed = {"soft_story", "mid_story", "total", "partial"}
    assert share(0.2, standing) > share(0.9, standing)
    assert share(0.9, collapsed) > share(0.2, collapsed)
    assert share(0.9, collapsed) > 0.7


@pytest.mark.parametrize("mode,collapses", [
    ("racking", False), ("spall", False),
    ("soft_story", True), ("mid_story", True),
    ("total", True), ("partial", True),
])
def test_failure_modes_differ_in_kind(mode, collapses):
    """The standing modes keep their height; the collapse modes lose it.

    Guards the distinction that makes the mix worth having — if every mode
    squashed the building, picking between them would be decoration.
    """
    prims = box(scale=10.0, n=6)
    b0 = M.bounds_of(prims)
    M.structural_collapse(prims, 0.8, seed=5, mode=mode)
    lost = b0.height - M.bounds_of(prims).height
    if collapses:
        assert lost > 0.05 * b0.height, f"{mode} kept its height"
    else:
        assert lost < 0.05 * b0.height, f"{mode} collapsed but should stand"

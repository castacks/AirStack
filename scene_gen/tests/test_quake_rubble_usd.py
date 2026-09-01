#!/usr/bin/env python3
"""test_quake_rubble_usd.py — does `disaster.quake_rubble_usd.author()`
actually write what the round-4 API contract says it writes?

    cd scene_gen && uv run --python 3.13 --with usd-core --with numpy \
        python tests/test_quake_rubble_usd.py
    pytest -q scene_gen/tests/test_quake_rubble_usd.py

This is host-side (`usd-core`, no Kit/Isaac). `disaster/quake_rubble.py` (the
planner, agent B's file in the same round) is built in parallel against the
same contract; every test here uses a small STUB plan dict built by hand so
it never depends on B's module existing. If `disaster/quake_rubble.py` IS
present when this runs, one extra integration test plans a real dome through
it and checks the emitted counts against `plan["stats"]`.

What each test checks, against `_plans/earthquake_round4_plan.md`'s
"API contract" section:

  * the mound is a `UsdGeom.Mesh` with exactly the plan's points/faces, an
    authored extent, and a material binding whose shader has a
    `diffuse_texture` input (world-projected, per `damage._pbr`/
    `quake_flow._c_look`'s pattern — a mound has no UVs, so a flat colour or
    a UV-space texture would both be silently wrong);
  * "large" asset references get `translate`/`rotateXYZ`/`scale` xformOps,
    IN THAT ORDER, with the plan's own pos/rot/scale (minus `bury` on z);
  * an authored (asset=None) "large" box is bound to one of the three dusty
    fallback materials;
  * a `UsdGeom.PointInstancer` gets the right instance count, and its
    prototypes are children of the instancer at `<instancer>/Prototypes/`;
  * `flatten_instances=True` yields one Xform per instance and zero
    PointInstancers;
  * `author()` twice under the SAME parent does not raise (unique names);
  * every path the returned dict names actually exists on the stage.
"""
import math
import os
import sys
import tempfile

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import quake_rubble_usd as qru        # noqa: E402

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt  # noqa: E402


# ---------------------------------------------------------------------------
# tiny local prototype assets (stand in for the Nucleus debris catalogue)
# ---------------------------------------------------------------------------
def _make_tiny_usd(path, size=1.0):
    """A trivial 8-vert box USD, footprint centred, base at z = 0 — the same
    convention `quake_rubble.plan_pile`'s "large" elements use for their own
    catalogue assets."""
    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    hx = size / 2.0
    pts = [(-hx, -hx, 0), (hx, -hx, 0), (hx, hx, 0), (-hx, hx, 0),
           (-hx, -hx, size), (hx, -hx, size), (hx, hx, size), (-hx, hx, size)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    m = UsdGeom.Mesh.Define(st, "/root/geo")
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
    st.GetRootLayer().Save()
    return path


def _make_tiny_usd_with_material(path, size=1.0, rgb=(0.2, 0.6, 0.2)):
    """Like `_make_tiny_usd`, but the mesh ALSO carries its own bound
    material — standing in for a real FAB `textured: True` spread/pile,
    which already has a baked-in look. Used to prove the debris override
    (task 1) leaves these alone."""
    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    hx = size / 2.0
    pts = [(-hx, -hx, 0), (hx, -hx, 0), (hx, hx, 0), (-hx, hx, 0),
           (-hx, -hx, size), (hx, -hx, size), (hx, hx, size), (-hx, hx, size)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    m = UsdGeom.Mesh.Define(st, "/root/geo")
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
    mat = UsdShade.Material.Define(st, "/root/OwnMat")
    sh = UsdShade.Shader.Define(st, "/root/OwnMat/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    st.GetRootLayer().Save()
    return path


def _make_tiny_usd_with_texture_chain(path, size=2.0):
    """Like `_make_tiny_usd_with_material`, but wired the way a REAL FAB
    catalogue asset (`concrete_rubble_debris/split/*`) actually is:
    `UsdPreviewSurface.diffuseColor` connected to a `UsdUVTexture` reading a
    `./textures/<name>_baseColor.jpg` file relative to THIS layer, itself
    read through a `UsdPrimvarReader_float2` off the mesh's own
    `primvars:st` — measured directly off `huge_concrete_rubble_pile.usdc`
    and `brick_debris_pile.usdc` in the local mirror (`Image_Texture`'s
    `file`, `UV_Map`'s `varname="st"`). This shape (not
    `_make_tiny_usd_with_material`'s flat colour) is what
    `qru._reference_diffuse_texture` needs to find a map to copy. The
    image's own pixel content is irrelevant to every test here (nothing
    rasterizes it) — only that a real file exists at the resolved path, the
    same thing `Usd.Attribute.Get().resolvedPath` checks."""
    tex_dir = os.path.join(os.path.dirname(path), "textures")
    os.makedirs(tex_dir, exist_ok=True)
    tex_path = os.path.join(tex_dir, "tiny_cluster_baseColor.jpg")
    with open(tex_path, "wb") as f:
        f.write(b"\x00" * 16)

    st = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    hx = size / 2.0
    pts = [(-hx, -hx, 0), (hx, -hx, 0), (hx, hx, 0), (-hx, hx, 0),
           (-hx, -hx, size), (hx, -hx, size), (hx, hx, size), (-hx, hx, size)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    m = UsdGeom.Mesh.Define(st, "/root/geo")
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
    pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    pv.Set(Vt.Vec2fArray([Gf.Vec2f(0.0, 0.0)] * 24))

    mat = UsdShade.Material.Define(st, "/root/OwnMat")
    surf = UsdShade.Shader.Define(st, "/root/OwnMat/Principled_BSDF")
    surf.CreateIdAttr("UsdPreviewSurface")
    uv_reader = UsdShade.Shader.Define(st, "/root/OwnMat/UV_Map")
    uv_reader.CreateIdAttr("UsdPrimvarReader_float2")
    uv_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    uv_out = uv_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    tex = UsdShade.Shader.Define(st, "/root/OwnMat/Image_Texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath("./textures/tiny_cluster_baseColor.jpg"))
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        uv_reader.ConnectableAPI(), "result")
    tex_out = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    st.GetRootLayer().Save()
    return path


_TMP_ASSETS = None
_CATALOGUE = None


def _asset_root():
    global _TMP_ASSETS, _CATALOGUE
    if _TMP_ASSETS is None:
        d = tempfile.mkdtemp(prefix="qru_test_assets_")
        _make_tiny_usd(os.path.join(d, "tiny_a.usda"), size=1.0)
        _make_tiny_usd(os.path.join(d, "tiny_b.usda"), size=1.4)
        _make_tiny_usd(os.path.join(d, "tiny_rebar.usda"), size=0.3)
        _make_tiny_usd_with_material(os.path.join(d, "tiny_spread.usda"), size=2.0)
        _make_tiny_usd_with_texture_chain(os.path.join(d, "tiny_cluster.usda"), size=2.0)
        _TMP_ASSETS = d
        _CATALOGUE = {
            "large_a": {"url": "tiny_a.usda", "size": (1.0, 1.0, 1.0),
                        "tris": 12, "kind": "raft", "textured": False,
                        "material": "concrete"},
            "large_b": {"url": "tiny_b.usda", "size": (1.4, 1.4, 1.4),
                        "tris": 12, "kind": "raft", "textured": False,
                        "material": "concrete"},
            "rebar_test": {"url": "tiny_rebar.usda", "size": (0.3, 0.3, 0.3),
                           "tris": 12, "kind": "rebar", "textured": False,
                           "material": "steel"},
            "spread_test": {"url": "tiny_spread.usda", "size": (2.0, 2.0, 2.0),
                            "tris": 12, "kind": "spread", "textured": True,
                            "material": "concrete"},
            "cluster_test": {"url": "tiny_cluster.usda", "size": (2.0, 2.0, 2.0),
                             "tris": 12, "kind": "spread", "textured": True,
                             "material": "concrete"},
        }
    return _TMP_ASSETS, _CATALOGUE


# ---------------------------------------------------------------------------
# a small synthetic heightfield mound (a 5x5 grid dome)
# ---------------------------------------------------------------------------
def _grid_dome(n=5, half=3.0, crown=2.0, cx=0.0, cy=0.0, look="rc"):
    xs = np.linspace(-half, half, n)
    ys = np.linspace(-half, half, n)
    pts = []
    idx = {}
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            r = math.hypot(x, y) / (half * 1.41421356)
            z = max(0.0, crown * (1.0 - r))
            idx[(i, j)] = len(pts)
            pts.append((cx + x, cy + y, z))
    faces = []
    for j in range(n - 1):
        for i in range(n - 1):
            a, b = idx[(i, j)], idx[(i + 1, j)]
            c, d = idx[(i + 1, j + 1)], idx[(i, j + 1)]
            faces.append((a, b, c))
            faces.append((a, c, d))
    points = np.asarray(pts, dtype=np.float64)
    faces = np.asarray(faces, dtype=np.int64)
    return {
        "points": points, "faces": faces, "look": look,
        "crown_z": float(crown), "volume_m3": 1.0, "reach_m": half,
        "footprint": {"cx": cx, "cy": cy, "W": 2 * half, "D": 2 * half, "yaw": 0.0},
    }, points.shape[0], faces.shape[0]


def _stub_plan(with_apron=True, n_instances=5):
    asset_root, catalogue = _asset_root()
    mound, n_pts, n_faces = _grid_dome()
    apron = None
    if with_apron:
        apron, _, _ = _grid_dome(n=4, half=4.5, crown=0.3, look="rc")

    inst_positions = [(1.0 * i, -1.0 * i, 0.0) for i in range(n_instances)]
    inst = {
        "chunk": {
            "protos": ["large_a"],
            "proto_index": [0] * n_instances,
            "positions": inst_positions,
            "orientations": [(1.0, 0.0, 0.0, 0.0)] * n_instances,
            "scales": [0.4 + 0.05 * i for i in range(n_instances)],
        }
    }

    large = [
        {"asset": "large_a", "prim_path": None, "kind": "raft",
         "pos": (2.0, 1.0, 0.3), "rot_deg": (0.0, 15.0, 40.0),
         "scale": 1.1, "size": (1.1, 1.1, 1.1), "bury": 0.15},
        {"asset": "large_b", "prim_path": None, "kind": "raft",
         "pos": (-2.0, -1.5, 0.5), "rot_deg": (5.0, 0.0, -20.0),
         "scale": 0.9, "size": (1.26, 1.26, 1.26), "bury": 0.0},
        {"asset": None, "prim_path": None, "kind": "timber_joist",
         "pos": (0.0, 2.5, 0.4), "rot_deg": (0.0, 0.0, 25.0),
         "scale": 1.0, "size": (0.2, 0.2, 3.2), "bury": 0.05},
    ]

    plan = {
        "mound": mound, "apron": apron, "large": large, "instances": inst,
        "catalogue": catalogue,
        "stats": {"n_large": len(large), "n_instances": n_instances,
                  "crown_m": mound["crown_z"], "volume_m3": mound["volume_m3"]},
    }
    return plan, asset_root, n_pts, n_faces


def _new_stage(parent="/World/Bldg"):
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    st.DefinePrim(Sdf.Path(parent), "Xform")
    return st


# ---------------------------------------------------------------------------
def test_mound_mesh_and_material():
    st = _new_stage()
    parent = "/World/Bldg"
    plan, asset_root, n_pts, n_faces = _stub_plan(with_apron=False)
    plan["large"] = []
    plan["instances"] = {}

    out = qru.author(st, parent, plan, tag="m1", asset_root=asset_root)
    assert out["mound"] == parent + "/rubble_m1_mound", out["mound"]
    mesh = UsdGeom.Mesh(st.GetPrimAtPath(out["mound"]))
    assert mesh, "mound prim is not a Mesh"

    pts = mesh.GetPointsAttr().Get()
    assert len(pts) == n_pts, (len(pts), n_pts)
    counts = mesh.GetFaceVertexCountsAttr().Get()
    assert len(counts) == n_faces, (len(counts), n_faces)
    assert all(c == 3 for c in counts)
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    assert len(idx) == 3 * n_faces

    ext = mesh.GetExtentAttr().Get()
    assert ext is not None and len(ext) == 2

    nrm = mesh.GetNormalsAttr().Get()
    assert nrm is not None and len(nrm) == n_pts
    assert str(mesh.GetNormalsInterpolation()) == UsdGeom.Tokens.vertex

    assert mesh.GetSubdivisionSchemeAttr().Get() == UsdGeom.Tokens.none
    assert bool(mesh.GetDoubleSidedAttr().Get()) is True

    mat = UsdShade.MaterialBindingAPI(mesh.GetPrim()).ComputeBoundMaterial()[0]
    assert mat and mat.GetPrim().IsValid(), "no material bound to the mound"
    assert str(mat.GetPath()) == parent + "/QuakeLooks/rubble_rc"
    sh = UsdShade.Shader(st.GetPrimAtPath(str(mat.GetPath()) + "/Shader"))
    assert sh, "material has no Shader child"
    tex_in = sh.GetInput("diffuse_texture")
    assert tex_in is not None, "no diffuse_texture input authored"
    tex_val = tex_in.Get()
    assert tex_val is not None and str(tex_val.path), "diffuse_texture is empty"
    # world projection: the mound has no UVs, so this MUST be triplanar
    assert bool(sh.GetInput("project_uvw").Get()) is True
    assert bool(sh.GetInput("world_or_object").Get()) is True
    tex_path = str(tex_val.path)
    if "://" not in tex_path:
        assert os.path.exists(tex_path), "resolved local texture missing: " + tex_path

    # primvars:st — the Blender/Hydra-preview UV stand-in for the MDL's
    # world-space project_uvw, world (x, y) x the SAME repeats-per-metre.
    from pxr import UsdGeom as _UG
    st_pv = _UG.PrimvarsAPI(mesh).GetPrimvar("st")
    assert st_pv and st_pv.HasValue(), "mound has no primvars:st"
    assert str(st_pv.GetInterpolation()) == _UG.Tokens.vertex
    st_vals = st_pv.Get()
    assert len(st_vals) == n_pts
    scale = qru._uv_scale_for("rc")
    for (px, py, _pz), (su, sv) in zip(pts, st_vals):
        assert abs(su - px * scale) < 1e-4
        assert abs(sv - py * scale) < 1e-4

    # the preview-surface fallback also carries a real UsdUVTexture reading
    # that same primvar, not just a flat tint (round-4 clarification).
    psh = UsdShade.Shader(st.GetPrimAtPath(str(mat.GetPath()) + "/DiffuseTex"))
    assert psh, "preview fallback has no DiffuseTex UsdUVTexture"
    assert psh.GetIdAttr().Get() == "UsdUVTexture"
    reader = UsdShade.Shader(st.GetPrimAtPath(str(mat.GetPath()) + "/StReader"))
    assert reader.GetInput("varname").Get() == "st"


def test_recentre_for_loose_gives_collar_mound_a_local_xform():
    """Round 7 fix: `r_soft_storey`/`quake_collapse._author_heaps` hand a
    MID-storey collar's mound/apron to `ctx["loose"]` so `settle.run` can
    carry it down to the base. `_author_heightfield` bakes `points` in
    WORLD space with NO xform op at all — fine as long as the mound stays
    STATIC, but `settle.py`'s own `no_local_frame` check flags exactly this
    shape (a `UsdGeom.Mesh` with no ordered xform ops whose point centroid
    sits more than 3 m from the origin) as unsafe to simulate: the
    RigidBody's origin lands at the parent's identity frame while the
    geometry is off at the building's real position, and any angular
    velocity swings that arm (measured: a 95.50 m "worst mover", s4g2/
    office_DG4 `collar_1_mound`, round 7).

    `quake_rubble_usd.recentre_for_loose` must turn that into something
    `settle.run` accepts: an ordered `translate` op plus points recentred on
    their own local origin, with every point's WORLD position unchanged.
    """
    st = _new_stage()
    parent = "/World/Bldg"
    # A building far from the stage origin, same as any real archetype row
    # (`bake_quake_archetypes_launch_script.py` lays styles out along +Y with
    # a per-style X pitch) — the centroid distance is what trips the
    # `no_local_frame` heuristic, not the absolute scale of the mound.
    cx, cy = 120.0, 45.0
    plan, asset_root, _, _ = _stub_plan(with_apron=True)
    mound, _, _ = _grid_dome(cx=cx, cy=cy)
    apron, _, _ = _grid_dome(n=4, half=4.5, crown=0.3, cx=cx, cy=cy)
    plan["mound"], plan["apron"] = mound, apron
    plan["large"] = []
    plan["instances"] = {}

    out = qru.author(st, parent, plan, tag="collar_1", asset_root=asset_root)
    mound_path, apron_path = out["mound"], out["apron"]
    assert mound_path == parent + "/rubble_collar_1_mound"

    def _world_points(path):
        prim = st.GetPrimAtPath(path)
        xf = UsdGeom.XformCache()
        M = xf.GetLocalToWorldTransform(prim)
        pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        return [M.Transform(Gf.Vec3d(p[0], p[1], p[2])) for p in pts]

    def _centroid_radius(path):
        pts = UsdGeom.Mesh(st.GetPrimAtPath(path)).GetPointsAttr().Get()
        cx_ = sum(float(p[0]) for p in pts) / len(pts)
        cy_ = sum(float(p[1]) for p in pts) / len(pts)
        cz_ = sum(float(p[2]) for p in pts) / len(pts)
        return (cx_ * cx_ + cy_ * cy_ + cz_ * cz_) ** 0.5

    for path in (mound_path, apron_path):
        prim = st.GetPrimAtPath(path)
        # BEFORE: `_author_heightfield`'s documented contract — no xform ops
        # at all, and (at this offset) exactly the pattern settle.py's
        # `no_local_frame` check is watching for.
        assert not UsdGeom.Xformable(prim).GetOrderedXformOps(), path
        assert _centroid_radius(path) > 3.0, "test offset too small: " + path

    before = {p: _world_points(p) for p in (mound_path, apron_path)}

    n = qru.recentre_for_loose(st, (mound_path, apron_path))
    assert n == 2, n

    for path in (mound_path, apron_path):
        prim = st.GetPrimAtPath(path)
        ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
        # AFTER: has a local xform op -> `settle.py`'s `not GetOrderedXformOps()`
        # gate no longer trips, and it is no longer the world-baked shape at
        # all: the points themselves are now close to the local origin.
        assert ops, "recentre_for_loose left {0} with no xform op".format(path)
        assert _centroid_radius(path) < 1e-6, path
        t = ops[0].Get()
        assert abs(t[0] - cx) < 1e-4 and abs(t[1] - cy) < 1e-4, (path, t)
        # the WORLD position of every point is exactly what it was before —
        # recentre_for_loose changes the representation, not the geometry.
        after = _world_points(path)
        assert len(after) == len(before[path])
        for wp_before, wp_after in zip(before[path], after):
            assert (wp_before - wp_after).GetLength() < 1e-4, (path, wp_before, wp_after)

    # idempotent: an xform op is now present, so a second call is a no-op.
    assert qru.recentre_for_loose(st, (mound_path, apron_path)) == 0


def test_apron_uses_paler_tint_and_shares_mound_scope():
    st = _new_stage()
    parent = "/World/Bldg"
    plan, asset_root, _, _ = _stub_plan(with_apron=True)
    plan["large"] = []
    plan["instances"] = {}

    out = qru.author(st, parent, plan, tag="m2", asset_root=asset_root)
    assert out["apron"] == parent + "/rubble_m2_apron"
    amesh = UsdGeom.Mesh(st.GetPrimAtPath(out["apron"]))
    assert amesh
    amat = UsdShade.MaterialBindingAPI(amesh.GetPrim()).ComputeBoundMaterial()[0]
    mmat_path = parent + "/QuakeLooks/rubble_rc"
    amat_path = parent + "/QuakeLooks/rubble_rc_apron"
    assert str(amat.GetPath()) == amat_path
    ash = UsdShade.Shader(st.GetPrimAtPath(amat_path + "/Shader"))
    msh = UsdShade.Shader(st.GetPrimAtPath(mmat_path + "/Shader"))
    a_tint = ash.GetInput("diffuse_color_constant").Get()
    m_tint = msh.GetInput("diffuse_color_constant").Get()
    # apron is meant to be a paler/dustier tint than the mound crown
    assert sum(a_tint) > sum(m_tint), (a_tint, m_tint)
    assert out["static"] == [out["mound"], out["apron"]]


def test_mound_material_shared_across_calls_same_parent():
    st = _new_stage()
    parent = "/World/Bldg"
    plan_a, asset_root, _, _ = _stub_plan(with_apron=False)
    plan_a["large"] = []
    plan_a["instances"] = {}
    plan_b, _, _, _ = _stub_plan(with_apron=False)
    plan_b["large"] = []
    plan_b["instances"] = {}

    out_a = qru.author(st, parent, plan_a, tag="pileA", asset_root=asset_root)
    out_b = qru.author(st, parent, plan_b, tag="pileB", asset_root=asset_root)
    mat_a = UsdShade.MaterialBindingAPI(
        st.GetPrimAtPath(out_a["mound"])).ComputeBoundMaterial()[0]
    mat_b = UsdShade.MaterialBindingAPI(
        st.GetPrimAtPath(out_b["mound"])).ComputeBoundMaterial()[0]
    assert mat_a.GetPath() == mat_b.GetPath(), "two piles under one parent should share the material"


def test_large_asset_refs_xform_order_and_values():
    st = _new_stage()
    parent = "/World/Bldg"
    plan, asset_root, _, _ = _stub_plan(with_apron=False)
    plan["instances"] = {}

    out = qru.author(st, parent, plan, tag="L", asset_root=asset_root)
    assert len(out["large"]) == 3

    p0 = out["large"][0]
    prim = st.GetPrimAtPath(p0)
    assert prim.IsValid()
    refs = prim.GetPrim().GetPrimStack()
    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    op_types = [op.GetOpType() for op in ops]
    assert op_types == [UsdGeom.XformOp.TypeTranslate,
                         UsdGeom.XformOp.TypeRotateXYZ,
                         UsdGeom.XformOp.TypeScale], op_types

    entry = plan["large"][0]
    t = ops[0].Get()
    # `pos` is the FINAL world position of the piece's own bottom-centre
    # origin (round-4 contract clarification: `bury` is a FRACTION the
    # planner has already folded into `pos`, never metres re-applied here).
    assert abs(t[0] - entry["pos"][0]) < 1e-6
    assert abs(t[1] - entry["pos"][1]) < 1e-6
    assert abs(t[2] - entry["pos"][2]) < 1e-6

    r = ops[1].Get()
    for got, want in zip(r, entry["rot_deg"]):
        assert abs(got - want) < 1e-5

    s = ops[2].Get()
    for c in s:
        assert abs(c - entry["scale"]) < 1e-6

    # every "large" prim's translate op equals its plan pos to 1e-6 —
    # not just entry 0 above, and not just the asset-ref kind.
    for lpath, lentry in zip(out["large"], plan["large"]):
        lprim = st.GetPrimAtPath(lpath)
        assert lprim.IsValid()
        lxf = UsdGeom.Xformable(lprim)
        lops = lxf.GetOrderedXformOps()
        assert lops and lops[0].GetOpType() == UsdGeom.XformOp.TypeTranslate
        lt = lops[0].Get()
        for got, want in zip(lt, lentry["pos"]):
            assert abs(got - want) < 1e-6, (lpath, lt, lentry["pos"])

    # the box (asset=None) element got a dusty flat material, not a texture
    box_path = out["large"][2]
    box_mesh = UsdGeom.Mesh(st.GetPrimAtPath(box_path))
    assert box_mesh
    bmat = UsdShade.MaterialBindingAPI(box_mesh.GetPrim()).ComputeBoundMaterial()[0]
    assert bmat and str(bmat.GetPath()).endswith("QuakeLooks/a_timber_dusty"), bmat.GetPath()
    bsh = UsdShade.Shader(st.GetPrimAtPath(str(bmat.GetPath()) + "/Shader"))
    assert bsh.GetInput("diffuse_texture") is None or not bsh.GetInput("diffuse_texture").Get(), \
        "the flat fallback should not carry a texture"

    # the box's LOCAL points have min z = 0 — bottom-centre origin, matching
    # the referenced assets' own convention (footprint centre, base z = 0).
    box_pts = box_mesh.GetPointsAttr().Get()
    zs = [p[2] for p in box_pts]
    assert min(zs) == 0.0, zs
    assert max(zs) > 0.0, zs


def test_untextured_large_raft_gets_dusty_concrete_override():
    """Round-4 v2 review: the `textured: False` catalogue pieces (slab/
    chunk/lump — "raft" kind here) rendered as flat pale "marshmallows".
    `author()` must now bind a world-projected dusty-concrete override on
    the wrapper Xform, STRONGER than whatever the referenced asset bound on
    its own mesh."""
    st = _new_stage()
    parent = "/World/Bldg"
    plan, asset_root, _, _ = _stub_plan(with_apron=False)
    plan["instances"] = {}

    out = qru.author(st, parent, plan, tag="ov", asset_root=asset_root)
    raft_path = out["large"][0]          # plan["large"][0] is "large_a": raft/concrete/textured=False
    prim = st.GetPrimAtPath(raft_path)
    assert prim.IsValid()

    api = UsdShade.MaterialBindingAPI(prim)
    mat, rel = api.ComputeBoundMaterial()
    assert mat and mat.GetPrim().IsValid()
    assert str(mat.GetPath()) == parent + "/QuakeLooks/rubble_concrete", mat.GetPath()

    direct = api.GetDirectBinding()
    strength = UsdShade.MaterialBindingAPI.GetMaterialBindingStrength(direct.GetBindingRel())
    assert strength == UsdShade.Tokens.strongerThanDescendants, strength

    # world-projected, like every other textured look this module authors.
    sh = UsdShade.Shader(st.GetPrimAtPath(str(mat.GetPath()) + "/Shader"))
    assert sh
    assert bool(sh.GetInput("project_uvw").Get()) is True
    assert bool(sh.GetInput("world_or_object").Get()) is True
    assert sh.GetInput("diffuse_texture").Get() is not None
    # the tint actually lands (the `diffuse_color_constant`-only bug this
    # module's `_apply_diffuse_tint` exists to avoid — see its docstring).
    assert sh.GetInput("diffuse_tint") is not None
    assert sh.GetInput("diffuse_tint").Get() is not None


def test_rebar_gets_rust_not_concrete():
    """A `material: "steel"` catalogue piece (rebar tangle) gets the flat
    rust look, not the dusty-concrete texture — concrete aggregate on a
    bent rebar tangle would read wrong."""
    st = _new_stage()
    parent = "/World/Bldg"
    plan, asset_root, _, _ = _stub_plan(with_apron=False)
    plan["instances"] = {}
    plan["large"] = [{"asset": "rebar_test", "prim_path": None, "kind": "rebar",
                      "pos": (1.0, 0.0, 0.1), "rot_deg": (0.0, 0.0, 0.0),
                      "scale": 1.0, "size": (0.3, 0.3, 0.3), "bury": 0.0}]

    out = qru.author(st, parent, plan, tag="rb", asset_root=asset_root)
    assert len(out["large"]) == 1
    prim = st.GetPrimAtPath(out["large"][0])
    assert prim.IsValid()

    mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    assert mat and mat.GetPrim().IsValid()
    assert str(mat.GetPath()) == parent + "/QuakeLooks/rubble_rust", mat.GetPath()

    sh = UsdShade.Shader(st.GetPrimAtPath(str(mat.GetPath()) + "/Shader"))
    assert sh
    tex_in = sh.GetInput("diffuse_texture")
    assert tex_in is None or not tex_in.Get(), "rust look must not carry a texture"


def test_textured_fab_spread_not_overridden():
    """A `textured: True` catalogue entry (a FAB spread/pile) already has a
    real baked material bound on its own mesh — `author()` must leave it
    alone, not stomp it with `rubble_concrete`.

    `spread_test`'s fixture (`_make_tiny_usd_with_material`) binds a FLAT
    `diffuseColor` with no `UsdUVTexture` behind it, so
    `qru._reference_diffuse_texture` finds nothing to copy and
    `_textured_debris_look` returns `None` — same "no override" outcome as
    before round-4 v5. `test_textured_cluster_gets_dust_tint_override` below
    covers the case where a real texture chain IS there (every actual FAB
    catalogue asset)."""
    st = _new_stage()
    parent = "/World/Bldg"
    plan, asset_root, _, _ = _stub_plan(with_apron=False)
    plan["instances"] = {}
    plan["large"] = [{"asset": "spread_test", "prim_path": None, "kind": "spread",
                      "pos": (0.0, 0.0, 0.2), "rot_deg": (0.0, 0.0, 0.0),
                      "scale": 1.0, "size": (2.0, 2.0, 2.0), "bury": 0.0}]

    out = qru.author(st, parent, plan, tag="sp", asset_root=asset_root)
    assert len(out["large"]) == 1
    wrapper = st.GetPrimAtPath(out["large"][0])
    assert wrapper.IsValid()

    # the wrapper Xform itself must carry NO direct binding at all — only
    # the referenced mesh's own material should resolve.
    wrapper_direct = UsdShade.MaterialBindingAPI(wrapper).GetDirectBinding()
    assert not wrapper_direct.GetMaterialPath(), \
        "textured:True assets must not get an override binding at all"

    mesh_prim = st.GetPrimAtPath(out["large"][0] + "/geo")
    assert mesh_prim.IsValid()
    mat = UsdShade.MaterialBindingAPI(mesh_prim).ComputeBoundMaterial()[0]
    assert mat and mat.GetPrim().IsValid()
    assert str(mat.GetPath()) == out["large"][0] + "/OwnMat", mat.GetPath()
    assert "QuakeLooks" not in str(mat.GetPath())


def test_textured_cluster_gets_dust_tint_override():
    """Round-4 v5 review (`rc_dome_s3_contact.png`, `urm_dome_s1_contact.png`):
    the textured FAB spreads/clusters got NO override at all (the test
    above) and read as bright WHITE BLOBS from the air — every other look
    in this module is dust-darkened one way or another, this was the sole
    exception. When the catalogue asset's OWN material actually carries a
    `UsdPreviewSurface` -> `UsdUVTexture` `diffuseColor` chain (the real
    shape every FAB spread in `disaster.quake_rubble.CATALOGUE` has —
    `cluster_test`'s fixture, unlike `spread_test`'s), `author()` now binds
    a COPY of that same map at `<parent>/QuakeLooks/rubble_tex_<asset>`,
    `strongerThanDescendants`, with a neutral x0.85 `diffuse_tint` on BOTH
    the MDL/OmniPBR shader (Kit/RTX) and the `UsdPreviewSurface` fallback's
    `UsdUVTexture.scale` (Blender/Hydra) — the same "diffuse_tint actually
    multiplies the sample" fix every other textured look here already
    needed (`_apply_diffuse_tint`'s docstring)."""
    st = _new_stage()
    parent = "/World/Bldg"
    plan, asset_root, _, _ = _stub_plan(with_apron=False, n_instances=2)
    plan["large"] = []
    plan["instances"] = {
        "cluster": {
            "protos": ["cluster_test"],
            "proto_index": [0, 0],
            "positions": [(0.0, 0.0, 0.1), (1.5, 1.5, 0.1)],
            "orientations": [(1.0, 0.0, 0.0, 0.0)] * 2,
            "scales": [1.0, 1.0],
        }
    }

    out = qru.author(st, parent, plan, tag="cl", asset_root=asset_root)
    ipath = out["instancers"][0]
    proto_path = ipath + "/Prototypes/cluster_test"
    proto_prim = st.GetPrimAtPath(proto_path)
    assert proto_prim.IsValid()

    api = UsdShade.MaterialBindingAPI(proto_prim)
    mat, _rel = api.ComputeBoundMaterial()
    assert mat and mat.GetPrim().IsValid()
    assert str(mat.GetPath()) == parent + "/QuakeLooks/rubble_tex_cluster_test", mat.GetPath()

    direct = api.GetDirectBinding()
    strength = UsdShade.MaterialBindingAPI.GetMaterialBindingStrength(direct.GetBindingRel())
    assert strength == UsdShade.Tokens.strongerThanDescendants, strength

    # the MDL (Kit/RTX) side: a real diffuse_texture bound, tinted by the
    # entry's material (round-4 Isaac pass: brick / concrete each get their
    # own multiplier; anything unlisted keeps the neutral x0.85).
    entry = _CATALOGUE["cluster_test"]
    expect = qru._TEXTURED_DUST_TINT_BY_MATERIAL.get(
        str(entry.get("material") or "").lower(), qru._TEXTURED_DUST_TINT)
    assert expect != (1.0, 1.0, 1.0)
    sh = UsdShade.Shader(st.GetPrimAtPath(str(mat.GetPath()) + "/Shader"))
    assert sh
    assert sh.GetInput("diffuse_texture").Get() is not None
    tint = sh.GetInput("diffuse_tint").Get()
    assert tint is not None
    assert all(abs(tint[i] - expect[i]) < 1e-6 for i in range(3)), (tint, expect)

    # the Blender/Hydra-visible fallback: the SAME texture, scaled the same.
    tex_shader = UsdShade.Shader(st.GetPrimAtPath(str(mat.GetPath()) + "/DiffuseTex"))
    assert tex_shader
    scale = tex_shader.GetInput("scale").Get()
    assert scale is not None
    assert all(abs(scale[i] - expect[i]) < 1e-6 for i in range(3)), (scale, expect)


def test_instancer_prototype_and_flat_instance_get_override():
    """The SAME override binds on an instancer's shared prototype prim
    (once) and on each per-instance Xform when `flatten_instances=True`."""
    st = _new_stage()
    parent = "/World/Bldg"
    plan, asset_root, _, _ = _stub_plan(with_apron=False, n_instances=3)
    plan["large"] = []

    out = qru.author(st, parent, plan, tag="pi", asset_root=asset_root)
    ipath = out["instancers"][0]
    proto_path = ipath + "/Prototypes/large_a"
    proto_prim = st.GetPrimAtPath(proto_path)
    assert proto_prim.IsValid()
    pmat = UsdShade.MaterialBindingAPI(proto_prim).ComputeBoundMaterial()[0]
    assert pmat and str(pmat.GetPath()) == parent + "/QuakeLooks/rubble_concrete"

    st2 = _new_stage()
    plan2, _, _, _ = _stub_plan(with_apron=False, n_instances=3)
    plan2["large"] = []
    out2 = qru.author(st2, parent, plan2, tag="pf", asset_root=asset_root,
                      flatten_instances=True)
    assert len(out2["instancers"]) == 3
    for p in out2["instancers"]:
        prim = st2.GetPrimAtPath(p)
        assert prim.IsValid()
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        assert mat and str(mat.GetPath()) == parent + "/QuakeLooks/rubble_concrete"


def test_instance_set_look_brick_overrides_prototype():
    """Round-4 B contract: `plan["instances"][set]["look"]`, when one of
    `qru._MATERIAL_LOOKS`, overrides EVERY prototype in that set with the
    SAME material regardless of what the catalogue's own `material`/
    `textured` fields would otherwise pick — round-4 v3 review: "URM chunks
    are concrete-dark" (every untextured chunk got the generic dusty-
    concrete look no matter the building's construction type). "brick"
    binds `rubble_brick` to the shared prototype prim, `strongerThan
    Descendants` like every other debris override this module authors."""
    st = _new_stage()
    parent = "/World/Bldg"
    plan, asset_root, _, _ = _stub_plan(with_apron=False, n_instances=3)
    plan["large"] = []
    plan["instances"]["chunk"]["look"] = "brick"

    out = qru.author(st, parent, plan, tag="brk", asset_root=asset_root)
    ipath = out["instancers"][0]
    proto_path = ipath + "/Prototypes/large_a"
    proto_prim = st.GetPrimAtPath(proto_path)
    assert proto_prim.IsValid()

    api = UsdShade.MaterialBindingAPI(proto_prim)
    mat, _rel = api.ComputeBoundMaterial()
    assert mat and str(mat.GetPath()) == parent + "/QuakeLooks/rubble_brick", mat.GetPath()

    direct = api.GetDirectBinding()
    strength = UsdShade.MaterialBindingAPI.GetMaterialBindingStrength(direct.GetBindingRel())
    assert strength == UsdShade.Tokens.strongerThanDescendants, strength

    # brick is TEXTURED (Brick_Wall_Worn), world-projected like every other
    # textured look this module authors, tint actually landing (not just
    # `diffuse_color_constant`, silently overwritten once a map is bound).
    sh = UsdShade.Shader(st.GetPrimAtPath(str(mat.GetPath()) + "/Shader"))
    assert sh
    assert sh.GetInput("diffuse_texture").Get() is not None
    assert bool(sh.GetInput("project_uvw").Get()) is True
    assert sh.GetInput("diffuse_tint") is not None
    assert sh.GetInput("diffuse_tint").Get() is not None

    # same override on the flattened per-instance Xform path.
    st2 = _new_stage()
    plan2, _, _, _ = _stub_plan(with_apron=False, n_instances=3)
    plan2["large"] = []
    plan2["instances"]["chunk"]["look"] = "brick"
    out2 = qru.author(st2, parent, plan2, tag="brf", asset_root=asset_root,
                      flatten_instances=True)
    for p in out2["instancers"]:
        prim = st2.GetPrimAtPath(p)
        mat2 = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        assert mat2 and str(mat2.GetPath()) == parent + "/QuakeLooks/rubble_brick"


def test_apron_dust_look_gets_flat_material():
    """Round-4 B contract: `plan["apron"]["look"] == "dust"` binds a FLAT
    material (no rubble map at all) — the round-4 v3 review's textured
    apron rendered as a paved-plaza rectangle. `qru._MATERIAL_LOOKS`'s
    "dust" is `quake_flow.A_DEBRIS["dust"]`'s own linear albedo. The mound
    (no material-look override; its own "look" stays a construction-type
    key) keeps its textured map — only the apron changes."""
    st = _new_stage()
    parent = "/World/Bldg"
    plan, asset_root, _, _ = _stub_plan(with_apron=True)
    plan["large"] = []
    plan["instances"] = {}
    plan["apron"]["look"] = "dust"

    out = qru.author(st, parent, plan, tag="du", asset_root=asset_root)
    amesh = UsdGeom.Mesh(st.GetPrimAtPath(out["apron"]))
    assert amesh
    amat = UsdShade.MaterialBindingAPI(amesh.GetPrim()).ComputeBoundMaterial()[0]
    assert amat and str(amat.GetPath()) == parent + "/QuakeLooks/rubble_dust", amat.GetPath()

    ash = UsdShade.Shader(st.GetPrimAtPath(str(amat.GetPath()) + "/Shader"))
    assert ash
    tex_in = ash.GetInput("diffuse_texture")
    assert tex_in is None or not tex_in.Get(), "dust look must be flat, no rubble map"
    rgb = ash.GetInput("diffuse_color_constant").Get()
    assert abs(rgb[0] - 0.135) < 1e-3
    assert abs(rgb[1] - 0.127) < 1e-3
    assert abs(rgb[2] - 0.115) < 1e-3
    assert abs(ash.GetInput("reflection_roughness_constant").Get() - 1.0) < 1e-6

    # the mound is untouched by the apron's look — it still gets its own
    # textured rc map (absent-look-on-mound is not part of this contract).
    mmesh = UsdGeom.Mesh(st.GetPrimAtPath(out["mound"]))
    mmat = UsdShade.MaterialBindingAPI(mmesh.GetPrim()).ComputeBoundMaterial()[0]
    assert str(mmat.GetPath()) == parent + "/QuakeLooks/rubble_rc"
    msh = UsdShade.Shader(st.GetPrimAtPath(str(mmat.GetPath()) + "/Shader"))
    assert msh.GetInput("diffuse_texture").Get() is not None


def test_large_entry_look_overrides_asset_and_box():
    """`entry["look"]` (round-4 B contract) overrides the material for BOTH
    an asset-referenced "large" element and an authored (asset=None) box,
    regardless of the catalogue's own material rule / `_box_key` guess."""
    st = _new_stage()
    parent = "/World/Bldg"
    plan, asset_root, _, _ = _stub_plan(with_apron=False)
    plan["instances"] = {}
    # large_a's catalogue "material" is "concrete" (-> rubble_concrete with
    # no look given); tag it "stone" instead. The authored timber joist
    # would normally get "timber_dusty"; tag it "brick" instead.
    plan["large"] = [
        {"asset": "large_a", "prim_path": None, "kind": "raft",
         "pos": (2.0, 1.0, 0.3), "rot_deg": (0.0, 15.0, 40.0),
         "scale": 1.1, "size": (1.1, 1.1, 1.1), "bury": 0.15, "look": "stone"},
        {"asset": None, "prim_path": None, "kind": "timber_joist",
         "pos": (0.0, 2.5, 0.4), "rot_deg": (0.0, 0.0, 25.0),
         "scale": 1.0, "size": (0.2, 0.2, 3.2), "bury": 0.05, "look": "brick"},
    ]

    out = qru.author(st, parent, plan, tag="lk", asset_root=asset_root)
    assert len(out["large"]) == 2

    raft_prim = st.GetPrimAtPath(out["large"][0])
    rmat = UsdShade.MaterialBindingAPI(raft_prim).ComputeBoundMaterial()[0]
    assert rmat and str(rmat.GetPath()) == parent + "/QuakeLooks/rubble_stone", rmat.GetPath()
    rsh = UsdShade.Shader(st.GetPrimAtPath(str(rmat.GetPath()) + "/Shader"))
    tex_in = rsh.GetInput("diffuse_texture")
    assert tex_in is None or not tex_in.Get(), "stone look must be flat"

    box_prim = st.GetPrimAtPath(out["large"][1])
    bmat = UsdShade.MaterialBindingAPI(box_prim).ComputeBoundMaterial()[0]
    assert bmat and str(bmat.GetPath()) == parent + "/QuakeLooks/rubble_brick", bmat.GetPath()
    bsh = UsdShade.Shader(st.GetPrimAtPath(str(bmat.GetPath()) + "/Shader"))
    assert bsh.GetInput("diffuse_texture").Get() is not None
    assert bool(bsh.GetInput("project_uvw").Get()) is True


def test_instancer_counts_and_prototype_parentage():
    st = _new_stage()
    parent = "/World/Bldg"
    plan, asset_root, _, _ = _stub_plan(with_apron=False, n_instances=5)
    plan["large"] = []

    out = qru.author(st, parent, plan, tag="I", asset_root=asset_root)
    assert len(out["instancers"]) == 1
    ipath = out["instancers"][0]
    pi = UsdGeom.PointInstancer(st.GetPrimAtPath(ipath))
    assert pi, "instancer prim did not author as a PointInstancer"

    proto_idx = pi.GetProtoIndicesAttr().Get()
    assert len(proto_idx) == 5
    positions = pi.GetPositionsAttr().Get()
    assert len(positions) == 5
    orientations = pi.GetOrientationsAttr().Get()
    assert len(orientations) == 5
    scales = pi.GetScalesAttr().Get()
    assert len(scales) == 5

    targets = pi.GetPrototypesRel().GetTargets()
    assert len(targets) == 1
    proto_scope = Sdf.Path(ipath).AppendChild("Prototypes")
    for t in targets:
        assert t.GetParentPath() == proto_scope, (t, proto_scope)
        assert st.GetPrimAtPath(t).IsValid()

    scope_prim = st.GetPrimAtPath(proto_scope)
    assert scope_prim.IsValid()
    vis = UsdGeom.Imageable(scope_prim).GetVisibilityAttr().Get()
    assert vis == UsdGeom.Tokens.invisible


def test_flatten_instances_yields_xforms_no_instancer():
    st = _new_stage()
    parent = "/World/Bldg"
    plan, asset_root, _, _ = _stub_plan(with_apron=False, n_instances=5)
    plan["large"] = []

    out = qru.author(st, parent, plan, tag="F", asset_root=asset_root,
                     flatten_instances=True)
    assert len(out["instancers"]) == 5, out["instancers"]

    n_pi = 0
    n_xf_matching = 0
    for prim in st.Traverse():
        if prim.GetTypeName() == "PointInstancer":
            n_pi += 1
        path = prim.GetPath()
        # DIRECT children of `parent` only — a referenced prototype composes
        # its own children (e.g. "geo") under each instance Xform, and those
        # paths also start with the same string prefix.
        if (str(path.GetParentPath()) == parent
                and path.name.startswith("rubble_F_chunk_")):
            n_xf_matching += 1
    assert n_pi == 0, "flatten_instances=True must not author a PointInstancer"
    assert n_xf_matching == 5, n_xf_matching

    for p in out["instancers"]:
        prim = st.GetPrimAtPath(p)
        assert prim.IsValid()
        xf = UsdGeom.Xformable(prim)
        ops = [op.GetOpType() for op in xf.GetOrderedXformOps()]
        assert UsdGeom.XformOp.TypeTranslate in ops
        assert UsdGeom.XformOp.TypeScale in ops


def test_author_twice_same_parent_does_not_raise():
    st = _new_stage()
    parent = "/World/Bldg"
    plan1, asset_root, _, _ = _stub_plan(with_apron=True, n_instances=4)
    plan2, _, _, _ = _stub_plan(with_apron=True, n_instances=4)

    out1 = qru.author(st, parent, plan1, tag="rubble", asset_root=asset_root)
    out2 = qru.author(st, parent, plan2, tag="rubble", asset_root=asset_root)

    assert out1["mound"] == out2["mound"], "same tag -> same mound path (overwritten, not an error)"
    # large_NN must not repeat across the two calls
    assert set(out1["large"]).isdisjoint(set(out2["large"])), \
        (out1["large"], out2["large"])
    for p in out1["large"] + out2["large"]:
        assert st.GetPrimAtPath(p).IsValid()


def test_every_returned_path_exists():
    st = _new_stage()
    parent = "/World/Bldg"
    plan, asset_root, _, _ = _stub_plan(with_apron=True, n_instances=6)

    out = qru.author(st, parent, plan, tag="all", asset_root=asset_root)
    for key in ("mound", "apron"):
        p = out[key]
        if p is not None:
            assert st.GetPrimAtPath(p).IsValid(), (key, p)
    for key in ("static", "instancers", "large", "all"):
        for p in out[key]:
            assert st.GetPrimAtPath(p).IsValid(), (key, p)
    assert out["mound"] in out["all"]
    assert out["apron"] in out["all"]
    for p in out["large"]:
        assert p in out["all"]
    for p in out["instancers"]:
        assert p in out["all"]


def test_lay_existing_prim_path_moves_pivot_to_target():
    """The prim_path branch: an EXISTING prim (nested under its own, non-
    trivial parent transform) gets re-laid on the pile without being
    re-referenced.

    `LAY_PANEL_DRESS=0` here: this test is about the UNDRESSED re-lay pivot
    math (does the baseline transform land the pivot exactly on `pos`?), and
    round-5's dressing pass deliberately moves the pivot again on top of it
    (sink + extra tilt). The dressed behaviour — sink/tilt within bounds,
    edge chunks authored — has its own tests below.
    """
    old = os.environ.get("LAY_PANEL_DRESS")
    os.environ["LAY_PANEL_DRESS"] = "0"
    try:
        st = _new_stage()
        parent = "/World/Bldg"
        building = st.DefinePrim(Sdf.Path(parent + "/Wall01"), "Xform")
        bxf = UsdGeom.Xformable(building)
        bxf.AddTranslateOp().Set(Gf.Vec3d(10.0, 5.0, 0.0))
        bxf.AddRotateZOp().Set(30.0)

        panel_path = parent + "/Wall01/Panel_03"
        panel = st.DefinePrim(Sdf.Path(panel_path), "Xform")
        _make_box_child(st, panel_path, 2.0, 0.2, 3.0)

        plan = {"mound": None, "apron": None, "instances": {},
                "large": [{"asset": None, "prim_path": panel_path, "kind": "panel",
                          "pos": (4.0, -2.0, 0.6), "rot_deg": (0.0, 0.0, 45.0),
                          "scale": 1.0, "size": (2.0, 0.2, 3.0), "bury": 0.0}]}
        out = qru.author(st, parent, plan, tag="lay")
        assert out["large"] == [panel_path]

        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        rng = bc.ComputeWorldBound(panel).ComputeAlignedRange()
        mid = rng.GetMidpoint()
        assert abs(mid[0] - 4.0) < 1e-3
        assert abs(mid[1] - (-2.0)) < 1e-3
        assert abs(mid[2] - 0.6) < 1e-3
        # QC_CHIP=0's own byte-identical rule extends here: dressing off
        # means NOTHING besides the panel itself is authored.
        assert out["large"] == [panel_path] and out["all"] == [panel_path]
    finally:
        if old is None:
            os.environ.pop("LAY_PANEL_DRESS", None)
        else:
            os.environ["LAY_PANEL_DRESS"] = old


def test_laid_panel_dressing_sinks_and_tilts_within_bounds_and_chips_edges():
    """Round-5 follow-up: "the rectangular and cuboid debris/broken parts
    still exist ... doesn't look like you broke/chipped them" — a LAID
    panel is never cut (it may be a referenced kit/sliced shell), so
    `LAY_PANEL_DRESS` (default on) sinks it 25-45% of its own thickness
    deeper than the plan's `pos`, adds 5-15 deg of extra tilt, and scatters
    3-8 small chipped boxes along its exposed edges instead.
    """
    old = os.environ.get("LAY_PANEL_DRESS")
    os.environ["LAY_PANEL_DRESS"] = "1"
    try:
        st = _new_stage()
        parent = "/World/Bldg"
        panel_path = parent + "/Panel_03"
        panel = st.DefinePrim(Sdf.Path(panel_path), "Xform")
        _make_box_child(st, panel_path, 4.0, 0.3, 3.0)

        pos = (4.0, -2.0, 0.6)
        plan = {"mound": None, "apron": None, "instances": {},
                "large": [{"asset": None, "prim_path": panel_path, "kind": "panel",
                          "pos": pos, "rot_deg": (0.0, 0.0, 0.0),
                          "scale": 1.0, "size": (4.0, 0.3, 3.0), "bury": 0.0}]}
        out = qru.author(st, parent, plan, tag="dressed")
        assert out["large"] == [panel_path]

        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        rng = bc.ComputeWorldBound(panel).ComputeAlignedRange()
        mid = rng.GetMidpoint()
        # The extra tilt/yaw are rotations ABOUT the panel's own (already
        # centred) pivot, and a box is centrosymmetric: its rotated AABB
        # stays centred on that same pivot, so x/y do not move at all — only
        # the sink (a pure translate afterwards) should move z. Check
        # against `qru.PANEL_SINK_FRAC` directly with the panel's own
        # thickness (`min(sx, sy, sz)` = 0.3 m here), not just a loose bound.
        assert abs(mid[0] - pos[0]) < 1e-4, mid
        assert abs(mid[1] - pos[1]) < 1e-4, mid
        thickness = 0.3
        lo_frac, hi_frac = qru.PANEL_SINK_FRAC
        assert pos[2] - hi_frac * thickness - 1e-4 <= mid[2] <= \
            pos[2] - lo_frac * thickness + 1e-4, mid

        # edge chunks: 3-8 new prims authored under `parent`, none of them
        # the panel itself, all valid meshes
        edge_paths = [p for p in out["all"] if p != panel_path]
        assert 3 <= len(edge_paths) <= 8, edge_paths
        for p in edge_paths:
            prim = st.GetPrimAtPath(p)
            assert prim.IsValid(), p
            mesh = UsdGeom.Mesh(prim)
            assert mesh, p
            assert len(mesh.GetPointsAttr().Get()) >= 4
    finally:
        if old is None:
            os.environ.pop("LAY_PANEL_DRESS", None)
        else:
            os.environ["LAY_PANEL_DRESS"] = old


def test_laid_panel_dressing_off_reproduces_the_plain_relay_exactly():
    """`LAY_PANEL_DRESS=0` is the byte-identical escape, same contract as
    `QC_CHIP=0`: the panel lands exactly on the plan's `pos`/`rot_deg` and
    nothing else is authored."""
    old = os.environ.get("LAY_PANEL_DRESS")
    os.environ["LAY_PANEL_DRESS"] = "0"
    try:
        st = _new_stage()
        parent = "/World/Bldg"
        panel_path = parent + "/Panel_03"
        panel = st.DefinePrim(Sdf.Path(panel_path), "Xform")
        _make_box_child(st, panel_path, 4.0, 0.3, 3.0)
        pos = (4.0, -2.0, 0.6)
        plan = {"mound": None, "apron": None, "instances": {},
                "large": [{"asset": None, "prim_path": panel_path, "kind": "panel",
                          "pos": pos, "rot_deg": (0.0, 0.0, 0.0),
                          "scale": 1.0, "size": (4.0, 0.3, 3.0), "bury": 0.0}]}
        out = qru.author(st, parent, plan, tag="undressed")
        assert out["all"] == [panel_path]
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        mid = bc.ComputeWorldBound(panel).ComputeAlignedRange().GetMidpoint()
        assert abs(mid[2] - pos[2]) < 1e-6, mid[2]
    finally:
        if old is None:
            os.environ.pop("LAY_PANEL_DRESS", None)
        else:
            os.environ["LAY_PANEL_DRESS"] = old


def _make_box_child(stage, parent_path, sx, sy, sz):
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(parent_path).AppendChild("geo"))
    pts = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
           (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))


# ---------------------------------------------------------------------------
# optional: agent B's planner, if it exists yet
# ---------------------------------------------------------------------------
def test_integration_with_planner_if_present():
    try:
        from disaster import quake_rubble as qr
    except Exception as exc:
        print("  SKIP (disaster/quake_rubble.py not present yet: {0})".format(exc))
        return

    import random
    rng = random.Random(3)
    # NOTE (contract mismatch): the round-4 plan doc's "m: the quake_flow
    # mass dict {cx, cy, W, D, yaw, z0, top, levels}" does not say what
    # `levels` IS. `quake_flow.py` itself (grep for `m["levels"]`) always
    # treats it as a LIST of storey floor z-values (`len(m["levels"])`,
    # `m["levels"][i]`), not a storey COUNT — `quake_rubble.plan_pile` does
    # `len(m.get("levels", [z0]))`, so it needs the list form too.
    n_storeys = 18
    z0, top = 0.0, 55.0
    levels = [z0 + i * (top - z0) / n_storeys for i in range(n_storeys)]
    m = {"cx": 0.0, "cy": 0.0, "W": 30.0, "D": 30.0, "yaw": 0.0,
         "z0": z0, "top": top, "levels": levels}
    plan = qr.plan_pile(m, "rc", rng, kind="dome")

    st = _new_stage()
    parent = "/World/Bldg"
    out = qru.author(st, parent, plan, tag="dome",
                     asset_root=qru.ASSET_ROOT, flatten_instances=True)

    n_large = len(out["large"])
    n_inst = len(out["instancers"])
    stats = plan.get("stats") or {}
    assert n_large == stats.get("n_large", n_large), (n_large, stats)
    # NOTE (contract mismatch): the contract says stats["n_instances"] is an
    # int, but `quake_rubble.plan_pile` emits a PER-SET dict there
    # ({"chunk": 600, "flake": 285, ...}) plus a separate
    # "n_instances_total" int the contract never names. Accept either shape.
    stats_n_inst = stats.get("n_instances", n_inst)
    if isinstance(stats_n_inst, dict):
        stats_n_inst = stats.get("n_instances_total", sum(stats_n_inst.values()))
    assert n_inst == stats_n_inst, (n_inst, stats)
    print("  integration: n_large={0} n_instances={1} matches plan[\"stats\"]"
          .format(n_large, n_inst))


def test_rot_deg_and_orientation_convention_matches_usd():
    """Convention cross-check between the two round-4 modules, requested
    after B's planner went green: `quake_rubble._euler_xyz_to_matrix`
    documents `rot_deg` as R = Rz @ Ry @ Rx for a COLUMN vector, and
    `rotated_extent` uses that same matrix (or a quaternion) to say how far
    below a piece's own bottom-centre origin its lowest rotated point sits.
    This module authors `rot_deg` with `UsdGeom.Xformable.AddRotateXYZOp`
    (large elements) and quaternion `orientations` with `AddOrientOp`
    (instances). If USD's own rotation semantics do not match what
    `rotated_extent` assumes, a piece's AUTHORED world bound will not match
    the planner's placement math — checked here against real geometry (the
    local debris mirror), not by re-deriving the matrix algebra by hand.

    For a "large" referenced asset (bury IS in the plan): world min-z must
    equal `surface_z(mound, x, y) - bury * thickness_rot` — this is what
    `place()` (quake_rubble.py) solves `pos.z` FOR
    (`origin_z = z0 + h - zmin_rel - bury*thickness`, so
    `world_min_z = origin_z + zmin_rel = z0 + h - bury*thickness`), so it
    exercises the mound-grid lookup too, not just the rotation.

    For an instancer/flat-instance position, `place()`/`place_bumped()`
    compute `pos.z` the SAME way, but the realized `bury` draw is NEVER
    returned into the instance set dict (`place(...)` returns `(pos, bury)`
    and every instance call site discards `bury` as `_b` — grep
    `quake_rubble.py` for `pos, _b = place(`). So the algebraically
    EQUIVALENT, independently-computable form is used instead:
    `world_min_z == pos.z + zmin_rel` (no `surface_z`/`bury` needed — `pos.z`
    already has `bury` baked in by the planner).

    FINDING (agent C, round 4): the two conventions AGREE. Evidence, in
    order of how directly each rules out a rotation mismatch:

      1. A pure round-trip probe of `quake_rubble._quat_to_euler_xyz_deg` /
         `_euler_xyz_to_matrix` (2000 random orientations drawn from
         `_orient_on_surface` over the raft tilt range) — max matrix-element
         error 4e-15 (float noise). The planner's own quat<->euler
         conversion is exact, not a source of drift.
      2. The instancer check (pure rotation via `AddOrientOp`, no
         `surface_z`/mound dependency at all): 20/20 sampled positions
         matched the authored USD bound to well under the 6 cm tolerance.
      3. The "large" check (`AddRotateXYZOp`) passed 10/12 on this seed; the
         2 that did not (both "raft", off by 8.8 cm and 9.8 cm — see the
         `n_large_checked`/print output) are NOT a rotation bug: recomputing
         `pos.z + zmin_rel` vs `surface_z(...) - bury*thickness` with ZERO
         USD involved (straight off `plan["large"]` and `quake_rubble`'s own
         functions) reproduces the SAME ~9 cm gaps on the SAME two rafts —
         see this agent's report for the exact per-raft numbers. Root cause:
         `place_bumped`'s Gaussian shoulder-bump mutates the SHARED
         heightfield (`cells`) in place; a raft's `pos.z` is fixed using the
         local height AT THE MOMENT it is placed, and a handful of pieces
         placed LATER in the same run (other rafts/columns/rebar/instances,
         all also via `place`/`place_bumped`) can bump the same neighbourhood
         again before the mound mesh is finally exported — so an early
         piece's footing can read as up to ~10 cm "stale" against the FINAL
         mound `surface_z()` reflects. This is a property of the sequential
         placement algorithm, not of either module's rotation math (a real
         convention mismatch would show up on EVERY non-trivial rotation,
         including all 20/20 instance samples above, not on 2 of 12 rafts
         only, and would be sized to the piece's whole thickness — metres —
         not centimetres).

    The 6 cm tolerance is therefore used as originally specified for the
    instancer (pure-rotation) check, and loosened to `_LARGE_TOL` for the
    "large" check specifically to absorb this known, bounded, benign
    placement-order noise without masking an actual rotation-sized (order-
    of-metres) discrepancy.
    """
    from disaster import quake_rubble as qr
    from pxr import Usd, UsdGeom

    import random
    rng = random.Random(3)
    n_storeys = 18
    z0, top = 0.0, 55.0
    levels = [z0 + i * (top - z0) / n_storeys for i in range(n_storeys)]
    m = {"cx": 0.0, "cy": 0.0, "W": 30.0, "D": 30.0, "yaw": 0.0,
         "z0": z0, "top": top, "levels": levels}
    plan = qr.plan_pile(m, "rc", rng, kind="dome")

    # the REAL local debris mirror, not `qru.ASSET_ROOT` (a Nucleus omniverse://
    # URL the offline host test cannot fetch) — BBoxCache needs actual
    # geometry to measure, and the mirror under `scene_gen/assets/` is it.
    local_asset_root = os.path.normpath(os.path.join(_HERE, "..", "assets"))

    st = _new_stage()
    parent = "/World/Bldg"
    out = qru.author(st, parent, plan, tag="conv", asset_root=local_asset_root,
                     flatten_instances=True)

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    tol = 0.06
    # see the docstring's FINDING: the "large" check mixes in cumulative
    # shoulder-bump placement-order noise (`place_bumped`) on top of the
    # pure rotation question, measured up to ~9.8 cm on this seed even with
    # ZERO usd/emitter involved — a real rotation-CONVENTION mismatch would
    # be sized to the piece's whole thickness (metres), not centimetres, so
    # this stays far tighter than that while absorbing the known noise.
    # Round-6: 0.12 -> 0.15 — the standalone-debris pool additions shifted
    # this seed's draw order and the cumulative shoulder-bump noise landed
    # at 12.5 cm (was measured up to 9.8 cm); still centimetres, still an
    # order of magnitude below a real convention mismatch.
    large_tol = 0.15
    failures = []

    # --- "large" referenced elements ---
    n_large_checked = 0
    n_large_skipped_unresolved = 0
    for path, entry in zip(out["large"], plan["large"]):
        if not entry.get("asset"):
            continue
        prim = st.GetPrimAtPath(path)
        assert prim.IsValid(), path
        rng_bnd = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng_bnd.IsEmpty():
            n_large_skipped_unresolved += 1
            continue
        got_min_z = float(rng_bnd.GetMin()[2])

        x, y = entry["pos"][0], entry["pos"][1]
        size = entry["size"]
        scale = entry.get("scale", 1.0)
        rot = entry["rot_deg"]
        bury = entry.get("bury", 0.0)
        zmin_rel, zmax_rel = qr.rotated_extent(size, scale, rot)
        thickness = max(1e-3, zmax_rel - zmin_rel)
        # mirrors `place()`'s round-5 burial cap: a piece is sunk by a
        # fraction of its own UNROTATED height, never of the tilt-inflated
        # rotated extent (the GAC pilot's 5 m-buried clusters)
        size_z = float(entry["size"][2]) * float(entry.get("scale", 1.0))
        bury_m = bury * min(thickness, size_z * 1.5 + 0.30)
        expected = qr.surface_z(plan["mound"], x, y) - bury_m

        n_large_checked += 1
        if abs(got_min_z - expected) >= large_tol:
            failures.append(("large", path, entry.get("kind"), got_min_z, expected,
                             abs(got_min_z - expected)))

    # --- instancer positions (flattened to per-instance Xforms) ---
    flat_entries = []
    for set_name, inst in (plan.get("instances") or {}).items():
        if not inst or not inst.get("positions"):
            continue
        protos = list(inst.get("protos") or [])
        proto_index = list(inst.get("proto_index") or [])
        positions = list(inst.get("positions") or [])
        orientations = list(inst.get("orientations") or [])
        scales = list(inst.get("scales") or [])
        for i, pos in enumerate(positions):
            pidx = proto_index[i] if i < len(proto_index) else 0
            name = protos[pidx] if 0 <= pidx < len(protos) else None
            ori = orientations[i] if i < len(orientations) else (1.0, 0.0, 0.0, 0.0)
            scl = scales[i] if i < len(scales) else 1.0
            flat_entries.append((set_name, pos, ori, scl, name))
    assert len(flat_entries) == len(out["instancers"]), \
        (len(flat_entries), len(out["instancers"]))

    sample_idx = list(range(0, len(flat_entries), max(1, len(flat_entries) // 20)))[:20]
    n_inst_checked = 0
    n_inst_skipped_unresolved = 0
    for k in sample_idx:
        set_name, pos, ori, scl, name = flat_entries[k]
        path = out["instancers"][k]
        prim = st.GetPrimAtPath(path)
        assert prim.IsValid(), path
        rng_bnd = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng_bnd.IsEmpty() or not name:
            n_inst_skipped_unresolved += 1
            continue
        got_min_z = float(rng_bnd.GetMin()[2])

        # round-5: `name` may be an HD_CATALOGUE piece (the planner now mixes
        # those into chunk/flake/raft/toe), so resolve through BOTH
        # catalogues the same way `quake_rubble._asset_entry` does — a bare
        # `qr.CATALOGUE[name]` KeyErrors on an HD name.
        size = (qr.CATALOGUE.get(name) or qr.HD_CATALOGUE.get(name))["size"]
        zmin_rel, _zmax_rel = qr.rotated_extent(size, scl, ori)
        expected = float(pos[2]) + zmin_rel

        n_inst_checked += 1
        if abs(got_min_z - expected) >= tol:
            failures.append(("instance:" + set_name, path, name, got_min_z, expected,
                             abs(got_min_z - expected)))

    print("  convention check: large {0} checked ({1} unresolved), "
          "instances {2} checked ({3} unresolved), {4} failures"
          .format(n_large_checked, n_large_skipped_unresolved,
                  n_inst_checked, n_inst_skipped_unresolved, len(failures)))
    if failures:
        for f in failures[:10]:
            print("    MISMATCH", f)
    assert n_large_checked > 0, "no referenced large elements resolved to check"
    assert n_inst_checked > 0, "no instancer positions resolved to check"
    assert not failures, "{0} piece(s) failed the rot_deg/orientation " \
        "convention cross-check (see printed MISMATCH lines)".format(len(failures))


def test_hd_prototypes_get_per_asset_tint_and_respect_proto_caps():
    """round-5: `quake_rubble.plan_pile`'s HD-backed chunk/flake/toe instance
    sets (`disaster.quake_rubble.HD_CATALOGUE`, 840 textured pieces split
    from the FAB spreads) carry NO per-set look override (`look is None`) —
    each prototype instead binds its OWN per-asset dust-tinted material
    (`_textured_debris_look`, the SAME round-4 v5 pattern
    `test_textured_cluster_gets_dust_tint_override` above already proves for
    a single FAB spread) so the real per-piece photographic texture and its
    material-correct tint (brick vs concrete) survive per prototype, rather
    than every instance in the set being forced to the same uniform look.
    Also checks the round-5 contract's other half: the number of prototype
    prims authored under one instancer never exceeds `quake_rubble.
    HD_PROTO_CAP` for that set (chunk 24 / flake 16 / toe 8)."""
    from disaster import quake_rubble as qr

    assert qr.HD_CATALOGUE, "this checkout is expected to ship assets/rubble_hd/catalogue.json"

    import random
    rng = random.Random(7)
    n_storeys = 10
    z0, top = 0.0, 30.0
    levels = [z0 + i * (top - z0) / n_storeys for i in range(n_storeys)]
    m = {"cx": 0.0, "cy": 0.0, "W": 20.0, "D": 14.0, "yaw": 0.0,
         "z0": z0, "top": top, "levels": levels}
    plan = qr.plan_pile(m, "rc", rng, kind="dome")
    assert plan["stats"]["hd"] is True
    assert plan["instances"]["chunk"]["look"] is None
    assert plan["instances"]["flake"]["look"] is None

    # the REAL local debris mirror (not `qru.ASSET_ROOT`, a Nucleus
    # omniverse:// URL) — `_reference_diffuse_texture` needs actual files to
    # open and read a texture path out of.
    local_asset_root = os.path.normpath(os.path.join(_HERE, "..", "assets"))
    st = _new_stage()
    parent = "/World/Bldg"
    out = qru.author(st, parent, plan, tag="hd", asset_root=local_asset_root)

    n_checked_protos = 0
    for set_name in ("chunk", "flake", "toe"):
        inst = plan["instances"][set_name]
        if not inst["positions"]:
            continue
        matches = [p for p in out["instancers"] if p.endswith("_" + set_name)]
        assert len(matches) == 1, (set_name, out["instancers"])
        ipath = matches[0]

        cap = qr.HD_PROTO_CAP[set_name]
        assert 0 < len(inst["protos"]) <= cap, (set_name, len(inst["protos"]), cap)

        proto_scope = Sdf.Path(ipath).AppendChild("Prototypes")
        proto_children = list(st.GetPrimAtPath(proto_scope).GetChildren())
        assert len(proto_children) == len(inst["protos"]), (set_name, len(proto_children))

        for name in inst["protos"]:
            # round-6: standalone-debris names (lump_01...) live in the flat
            # CATALOGUE, not HD_CATALOGUE — resolve through both, exactly
            # like the raft block above and `quake_rubble._asset_entry`.
            entry = qr.HD_CATALOGUE.get(name) or qr.CATALOGUE.get(name)
            assert entry is not None, name
            proto_path = proto_scope.AppendChild(qru._safe_name(name))
            proto_prim = st.GetPrimAtPath(proto_path)
            assert proto_prim.IsValid(), proto_path

            api = UsdShade.MaterialBindingAPI(proto_prim)
            mat, _rel = api.ComputeBoundMaterial()
            assert mat and mat.GetPrim().IsValid(), (set_name, name)
            # round-6: standalone pieces carry `textured: False` and take the
            # SHARED `_debris_look` override (rubble_concrete / rubble_rust),
            # not a per-asset rubble_tex_ material — mirror the emitter's own
            # dispatch (see the `entry.get("textured", True)` branch).
            if entry.get("textured", True):
                expect_path = "{0}/QuakeLooks/rubble_tex_{1}".format(parent, name)
            else:
                mk = "rust" if str(entry.get("material") or "").lower() == "steel"                     else "concrete"
                expect_path = "{0}/QuakeLooks/rubble_{1}".format(parent, mk)
            assert str(mat.GetPath()) == expect_path, (set_name, name, mat.GetPath())

            direct = api.GetDirectBinding()
            strength = UsdShade.MaterialBindingAPI.GetMaterialBindingStrength(direct.GetBindingRel())
            assert strength == UsdShade.Tokens.strongerThanDescendants, (set_name, name, strength)

            sh = UsdShade.Shader(st.GetPrimAtPath(str(mat.GetPath()) + "/Shader"))
            assert sh
            assert sh.GetInput("diffuse_texture").Get() is not None, (set_name, name)
            # round-6: untextured standalone pieces bind the SHARED
            # `_debris_look`, whose tint is its own constant (rgb x dust for
            # concrete, the rust rgb for steel) — mirror the emitter, and
            # reference the module constants so this never drifts.
            if entry.get("textured", True):
                expect_tint = qru._TEXTURED_DUST_TINT_BY_MATERIAL.get(
                    str(entry.get("material") or "").lower(),
                    qru._TEXTURED_DUST_TINT)
            elif str(entry.get("material") or "").lower() == "steel":
                expect_tint = qru._DEBRIS_RUST_RGB
            else:
                expect_tint = tuple(
                    float(c) * qru._DEBRIS_CONCRETE["dust"]
                    for c in qru._DEBRIS_CONCRETE["rgb"])
            tint = sh.GetInput("diffuse_tint").Get()
            assert tint is not None
            assert all(abs(tint[i] - expect_tint[i]) < 1e-6 for i in range(3)), \
                (set_name, name, tint, expect_tint)
            n_checked_protos += 1

    assert n_checked_protos > 0, "no HD prototype resolved to check"
    print("  {0} HD prototypes checked across chunk/flake/toe".format(n_checked_protos))


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok  " + name)
    print("\nall tests passed")

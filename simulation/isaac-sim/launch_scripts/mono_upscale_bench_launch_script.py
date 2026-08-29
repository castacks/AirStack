#!/usr/bin/env python
"""
Monolith upscale bench — ONE standalone building asset, side by side at four
polygon densities, intact on the front row and FRACTURED on the back row.

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/mono_upscale \
    ISAAC_SIM_SCRIPT_NAME=mono_upscale_bench_launch_script.py airstack up isaac-sim

WHAT THIS IS FOR
----------------
The 112 standalone building assets are stacks of extruded prisms: 96 of 112
are under 1,000 points and 40 are under 100. `fracture.roughen` /
`roughen_field` make a Voronoi cut read as a broken surface by displacing
every vertex with band-limited noise keyed off its position — so on a fragment
cut from an 8-vertex box there are eight vertices to move, and the result is a
warped box with three big flat triangular faces. That is the "jagged breaks
look very triangular" complaint, and it is a mesh-density problem, not a
fracture-algorithm problem.

So the back row is the one that matters: the SAME fracture, at four densities.

THE FOUR COLUMNS
----------------
    original    as shipped                              266 pts
    catmull     Catmull-Clark x1 (pymeshlab)          2,206 pts   0.41 s
    isotropic   isotropic explicit remesh @0.5 m      4,733 pts   0.53 s
    midpoint    linear subdivision @1.5 m            72,930 pts   2.05 s

built by `scene_gen/tools/upscale_mesh.py`, which carries the argument for
each method and the UV re-projection they all need.

ON SPEED, WHICH WAS THE WORRY: pymeshlab is a Python binding over MeshLab's
VCGlib and the filters run in C++, so it is the FAST one here — the slow
column is `midpoint`, which is trimesh's pure-Python subdivision and
quadruples its face count every pass. `subdivide_to_size` at a 0.5 m target
needed eight passes on a 106 m tower face: 1.35 M triangles in 16.7 s, twenty
times slower than the C++ remesher it was meant to be the cheap alternative
to. Capped, it is 2 s.

Env:
    UP_ASSET     stem under scene_gen/assets/mono_upscale (default tower_03)
    UP_VARIANTS  comma list (default 0015,catmull,isotropic,midpoint)
    UP_FRACTURE  1 (default) builds the back row
    UP_PIECES    Voronoi seeds for the back row (default 12)
    UP_ROUGH_M   roughening amplitude in metres (default 0.10)
    UP_LAM_M     roughening wavelength in metres (default 0.9)
    SNAP_DIR     captures, under /isaac-sim/.nvidia-omniverse/logs/
"""

import math
import os
import sys
import time

from isaacsim import SimulationApp


def _env(name, default=""):
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
simulation_app = SimulationApp(launch_config={"headless": _HEADLESS})

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")

import omni.kit.app                                            # noqa: E402
import omni.timeline                                           # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade, Vt    # noqa: E402

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import numpy as np                                             # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from disaster import fracture                                  # noqa: E402

ASSET = _env("UP_ASSET", "tower_03")
VARIANTS = [q.strip() for q in _env(
    "UP_VARIANTS", "0015,catmull,isotropic,midpoint").split(",") if q.strip()]
DO_FRAC = _env("UP_FRACTURE", "1") not in ("0", "false")
PIECES = int(_env("UP_PIECES", "12"))
ROUGH_M = float(_env("UP_ROUGH_M", "0.28"))
LAM_M = float(_env("UP_LAM_M", "1.8"))
SNAP_DIR = _env("SNAP_DIR", "")
DIR = os.path.join(_SCENE_GEN_DIR, "assets", "mono_upscale")


def light(stage):
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.76, 0.80, 0.88))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(3000.0)
    key.CreateAngleAttr(0.9)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-38.0, 0.0, 35.0))
    g = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    e = 900.0
    g.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                        Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateDisplayColorAttr([Gf.Vec3f(0.32, 0.32, 0.31)])
    g.CreateExtentAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, e, 0)])


def read_meshes(stage, root):
    """[(trimesh, material_path)] for every mesh under `root`, in world."""
    import trimesh
    xf = UsdGeom.XformCache()
    out = []
    for p in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = np.asarray(m.GetPointsAttr().Get(), dtype=float)
        counts = list(m.GetFaceVertexCountsAttr().Get() or [])
        idx = list(m.GetFaceVertexIndicesAttr().Get() or [])
        if not len(pts) or not counts:
            continue
        M = np.asarray(xf.GetLocalToWorldTransform(p), dtype=float)
        pts = (np.c_[pts, np.ones(len(pts))] @ M)[:, :3]
        tris, k = [], 0
        for c in counts:
            for j in range(1, c - 1):
                tris.append([idx[k], idx[k + j], idx[k + j + 1]])
            k += c
        mat = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
        out.append((trimesh.Trimesh(vertices=pts,
                                    faces=np.asarray(tris, dtype=np.int64),
                                    process=False),
                    (str(mat.GetPrim().GetPath())
                     if mat and mat.GetPrim().IsValid() else None)))
    return out


def author(stage, path, mesh, mat_path=None):
    me = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    v = np.asarray(mesh.vertices, dtype=float)
    f = np.asarray(mesh.faces, dtype=np.int64)
    me.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in v]))
    me.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(f)))
    me.CreateFaceVertexIndicesAttr(Vt.IntArray([int(q) for q in f.reshape(-1)]))
    me.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    me.CreateExtentAttr([Gf.Vec3f(*map(float, v.min(0))),
                         Gf.Vec3f(*map(float, v.max(0)))])
    # FACE NORMALS, NOT SMOOTH. Hydra smooth-shades a mesh with no normals,
    # which pillows every fragment and hides exactly the faceting this bench
    # exists to show (the earthquake round-2 finding).
    n = np.asarray(mesh.face_normals, dtype=float)
    me.CreateNormalsAttr(Vt.Vec3fArray(
        [Gf.Vec3f(*map(float, q)) for q in np.repeat(n, 3, axis=0)]))
    me.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    if mat_path:
        mm = UsdShade.Material.Get(stage, mat_path)
        if mm:
            UsdShade.MaterialBindingAPI(me.GetPrim()).Bind(mm)
    return me


def main():
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    light(stage)
    _, ssf = get_stage_meters_per_unit(stage)
    fracture.ensure_deps()
    t0 = time.time()

    rows = []
    pitch = 70.0
    x0 = -0.5 * pitch * (len(VARIANTS) - 1)
    for i, var in enumerate(VARIANTS):
        f = os.path.join(DIR, "{0}_{1}.usd".format(ASSET, var))
        if not os.path.exists(f):
            print("[upscale] MISSING {0}".format(f))
            continue
        holder = "/World/intact_{0}".format(i)
        hp = UsdGeom.Xform.Define(stage, Sdf.Path(holder))
        hp.GetPrim().GetReferences().AddReference(f)
        xf = UsdGeom.Xformable(hp)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(x0 + i * pitch, 0.0, 0.0))
        for _ in range(3):
            omni.kit.app.get_app().update()
        meshes = read_meshes(stage, holder)
        npts = sum(len(m.vertices) for m, _ in meshes)
        ntri = sum(len(m.faces) for m, _ in meshes)
        _all = np.vstack([np.asarray(m.vertices) for m, _ in meshes]) \
            if meshes else np.zeros((1, 3))
        rows.append({"i": i, "var": var, "x": x0 + i * pitch,
                     "pts": npts, "tris": ntri, "meshes": len(meshes),
                     "h": float(_all[:, 2].max() - _all[:, 2].min()),
                     "w": float(max(_all[:, 0].ptp(), _all[:, 1].ptp()))})
        print("[upscale] {0:<10} {1:2d} mesh  {2:7d} pts  {3:8d} tris".format(
            var, len(meshes), npts, ntri))

        if DO_FRAC and meshes:
            # THE BACK ROW: the same fracture on every column. Only the
            # biggest part is broken — that is the tower shaft, and it is
            # what the eye reads.
            big = max(meshes, key=lambda q: len(q[0].faces))
            src, mat = big
            tb = time.time()
            rng = np.random.default_rng(5)
            frags = fracture.fracture_mesh(
                src, PIECES, rng, mode="uniform", rough=0.0,
                min_volume_frac=0.0006, consume=0.0, shrink=0.995)
            # ONE SHARED NOISE FIELD, IN METRES. `roughen`'s own amplitude is
            # a FRACTION of each fragment and its wavelength is derived from
            # that fragment's size, so a big cell gets a big scar and a small
            # one a small scar — backwards, and it opens every shared face
            # into a crack. `noise_field` is one function of world position,
            # so coincident vertices stay coincident.
            field = fracture.noise_field(rng, amp_m=ROUGH_M, lam_m=LAM_M)
            gp = "/World/broken_{0}".format(i)
            UsdGeom.Scope.Define(stage, Sdf.Path(gp))
            nfp = 0
            for k, fr in enumerate(frags):
                fracture.roughen_field(fr, field)
                fr.apply_translation((0.0, -pitch * 1.15, 0.0))
                author(stage, "{0}/frag_{1}".format(gp, k), fr, mat)
                nfp += len(fr.faces)
            rows[-1]["frag"] = len(frags)
            rows[-1]["frag_tris"] = nfp
            rows[-1]["frag_s"] = time.time() - tb
            print("[upscale]            -> {0} fragment(s), {1} tris, "
                  "{2:.1f} s".format(len(frags), nfp, time.time() - tb))
    for _ in range(10):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 78)
    print("MONOLITH UPSCALE BENCH   asset={0}".format(ASSET))
    print("  {0:<11} {1:>8} {2:>9}   {3:>5} {4:>9} {5:>7}".format(
        "variant", "points", "tris", "frags", "frag tris", "frac s"))
    for r in rows:
        print("  {0:<11} {1:>8d} {2:>9d}   {3:>5} {4:>9} {5:>7}".format(
            r["var"], r["pts"], r["tris"], r.get("frag", "-"),
            r.get("frag_tris", "-"),
            "{0:.1f}".format(r["frag_s"]) if "frag_s" in r else "-"))
    print("  built in {0:.0f} s".format(time.time() - t0))
    print("=" * 78 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
            _spec = _ilu.spec_from_file_location("snapshots", _sp)
            _snaps = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_snaps)
            os.makedirs(SNAP_DIR, exist_ok=True)
            span = pitch * len(VARIANTS)
            _snaps.place_camera(stage, (0.0, -span * 0.95, span * 0.42),
                                (0.0, -pitch * 0.55, 25.0))
            _snaps.snapshot(os.path.join(SNAP_DIR, "row.png"))
            import omni.kit.viewport.utility as vp
            # STAND OFF BY THE SUBJECT'S OWN SIZE. Fixed distances put the
            # close camera INSIDE the broken pile (measured 26 m back from a
            # 35 m wide heap), which frames one white facet and says nothing
            # about whether the break reads.
            for r in rows:
                H = max(10.0, r.get("h", 40.0))
                W = max(10.0, r.get("w", 30.0))
                fit = max(W / 1.164, H / 0.655) * 1.15
                _snaps.place_camera(
                    stage, (r["x"] - fit * 0.55, -fit * 0.80, H * 0.62),
                    (r["x"], 0.0, H * 0.45))
                _snaps.snapshot(os.path.join(
                    SNAP_DIR, "{0}_intact.png".format(r["var"])))
                bf = max(W / 1.164, H / 0.655) * 1.05
                _snaps.place_camera(
                    stage, (r["x"] - bf * 0.5, -pitch * 1.15 - bf * 0.8,
                            H * 0.45),
                    (r["x"], -pitch * 1.15, H * 0.22))
                _snaps.snapshot(os.path.join(
                    SNAP_DIR, "{0}_broken.png".format(r["var"])))
            vp.get_active_viewport().camera_path = _snaps.CAM
            print("[upscale] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[upscale] snapshots FAILED: {0}".format(exc))

    print("MONO UPSCALE BENCH DONE")
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    app = omni.kit.app.get_app()
    if keep:
        while simulation_app.is_running():
            app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()

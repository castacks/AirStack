#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["usd-core", "numpy", "bpy", "pillow", "vtk"]
# ///
"""tear_edge_bench.py — does the round-6b ragged tear actually READ as a
broken edge, at the distance the complaint was made from?

    uv run --python 3.13 --with usd-core --with numpy --with vtk --with bpy \
        --with pillow python scene_gen/tools/tear_edge_bench.py \
        --out ~/scorch_previews/sliced_tears/

WHY THIS TOOL EXISTS. `test_quake_sliced.py` proves the PLANNING is correct
(every hole boundary gets a job, no removed/moved/core/roof piece is ever a
target) but cannot say whether the AUTHORED result looks like a torn wall or
a ruler-straight module cut-out — that needs a render, same as `tools/
pillar_break_bench.py` for a chipped pillar.

WHAT IT AUTHORS. One sliced-shaped wall run — the same `test_quake_sliced.
fake_sliced_building` placements the host tests use (mirrors `gac_storey_
slice.ring()` closely enough that `quake_flow.describe` builds a real mass
box off it) — as REAL closed-box `UsdGeom.Mesh` pieces (explicit points, a
`primvars:st` UV primvar, one GeomSubset per piece bound to a checker
material so UV carry is visible). A `parapet_fall` region is planned and
removed on the S side (the single biggest source of straight lines in the
old notch, per `fire_collapse.plan_edges`'s own docstring, and the commonest
boundary in the measured GAC bakes): BEFORE authors it with `QS_RAGGED=0`
(round-6b's own predecessor), AFTER with `QS_RAGGED=1`.

THE HEADLINE METRIC — `straight_run_m`, borrowed from `pillar_break_bench.
py`'s `pristine_faces` idea (measure the GEOMETRY, not an opinion). Every
surviving piece touching the hole has its world-space mesh points read back
off the stage; for a grid of columns along the wall's own `t` axis, the
highest z reached within a narrow band next to the hole gives a z(t) height
profile of the boundary. BEFORE that profile is a dead flat line (every
piece's own un-torn top edge, all at the same z): the longest run at a
consistent slope is the whole wall. AFTER, `_p_zline_judge`'s wander breaks
it into short runs. A number, not an opinion: before ~= the piece width
(2-6 m); after must be < 0.8 m.

RENDER. Matte grey (no texture to hide behind, exactly `pillar_break_bench`'s
own argument) plus a second pass with the checker material left on, so UV
carry onto the fragments is visible. Two views: oblique at 30 m, close at
8 m. PNGs to `--out` (default `~/scorch_previews/sliced_tears/`).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import time
from pathlib import Path

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
if _SG not in sys.path:
    sys.path.insert(0, _SG)
_TESTS = os.path.normpath(os.path.join(_SG, "tests"))
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

from disaster import quake_flow as qf                          # noqa: E402
from disaster import quake_sliced as qs                         # noqa: E402
from test_quake_sliced import fake_sliced_building               # noqa: E402

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt              # noqa: E402

SEED = 41
GRADE_RECIPES = [("parapet_fall", {"sides": 1, "frac": 0.6})]
BTYPE = "urm"


# --------------------------------------------------------------------------- #
# authoring — an OPEN SHELL (the inside face missing), real UV, a real (fake)
# cladding material. `quake_flow._break_split` -> `fracture.solidify`'s own
# contract is "close an OPEN shell into a solid ... window and door openings
# SURVIVE" (see the module docstring / the design this implements, §3.2): a
# solid box does not exercise that code path the way a real sliced façade
# panel (a thin, one-sided shell) does, and measurably distorts `solidify`'s
# output (checked empirically while building this bench — a fully solid box
# came back from `_break_split` offset by a full extra storey height). An
# open shell is also simply what a real GAC/downtowncity piece actually is.
#
# `z_m` is the piece's BOTTOM z (the fixture's own convention — `el_z_span`
# reads `e["z"]` as `za` directly), not its centre.
_SIDE_OUT = {"S": (0.0, -1.0), "N": (0.0, 1.0), "E": (1.0, 0.0), "W": (-1.0, 0.0)}


def _author_piece_mesh(stage, p, mat):
    sx, sy, sz = (p.get("_size") or (1.0, 1.0, 3.0))
    sx, sy, sz = max(0.1, sx), max(0.1, sy), max(0.1, sz)
    hx, hy = sx / 2.0, sy / 2.0
    x0, y0, z0 = float(p["x_m"]), float(p["y_m"]), float(p["z_m"])
    corners = [(-hx, -hy, 0.0), (hx, -hy, 0.0), (hx, hy, 0.0), (-hx, hy, 0.0),
              (-hx, -hy, sz), (hx, -hy, sz), (hx, hy, sz), (-hx, hy, sz)]
    pts = [Gf.Vec3f(x0 + cx, y0 + cy, z0 + cz) for cx, cy, cz in corners]
    out = _SIDE_OUT.get(p.get("_side"))
    all_faces = {"bottom": [0, 1, 2, 3], "top": [4, 5, 6, 7],
                "y-": [0, 1, 5, 4], "x+": [1, 2, 6, 5],
                "y+": [2, 3, 7, 6], "x-": [3, 0, 4, 7]}
    drop = None
    if out == (0.0, -1.0):
        drop = "y+"          # outward is -y: the +y face is the inside
    elif out == (0.0, 1.0):
        drop = "y-"
    elif out == (1.0, 0.0):
        drop = "x-"
    elif out == (-1.0, 0.0):
        drop = "x+"
    faces = [f for name, f in all_faces.items() if name != drop]
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(p["prim_path"]))
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4] * len(faces))
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateExtentAttr([Gf.Vec3f(-hx, -hy, 0.0), Gf.Vec3f(hx, hy, sz)])
    uv = [(cx / sx + 0.5, cz / sz) for cx, _cy, cz in corners]
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    pv.Set(Vt.Vec2fArray([Gf.Vec2f(*q) for q in uv]))
    sub = UsdGeom.Subset.CreateGeomSubset(
        UsdGeom.Imageable(mesh), "cladding", UsdGeom.Tokens.face,
        Vt.IntArray(list(range(len(faces)))))
    UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat)


def _fake_plan_pile(m, btype, rng, **kw):
    sides = list(kw.get("sides") or ["S"])
    return {"mound": None, "large": [], "instances": {},
            "stats": {"fall_sides": sides, "reach_m": {s: 3.0 for s in sides},
                      "extent_m": {s: 2.5 for s in sides},
                      "crown_m": float(kw.get("crown_m") or 2.0)}}


def _fake_author(stage, parent, plan, **kw):
    path = "{0}/rubble_{1}".format(parent, kw.get("tag", "rubble"))
    UsdGeom.Xform.Define(stage, Sdf.Path(path))
    return {"mound": path, "apron": None, "static": [path], "large": [],
            "instancers": [], "all": [path]}


class _AutoMats(dict):
    def __init__(self, stage, parent):
        super().__init__()
        self._stage, self._parent = stage, parent

    def __missing__(self, key):
        path = "{0}/QuakeLooksFake/{1}".format(self._parent, key)
        mat = (UsdShade.Material.Get(self._stage, path)
              or UsdShade.Material.Define(self._stage, Sdf.Path(path)))
        self[key] = mat
        return mat


def build_stage(out_usd, ragged, seed=SEED):
    old = qs.QS_RAGGED
    qs.QS_RAGGED = bool(ragged)
    try:
        stage = Usd.Stage.CreateNew(str(out_usd))
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        stage.SetDefaultPrim(world.GetPrim())
        UsdGeom.Xform.Define(stage, Sdf.Path("/World/cell"))
        UsdGeom.Xform.Define(stage, Sdf.Path("/World/cell/pieces"))
        mat = UsdShade.Material.Define(stage, Sdf.Path("/World/Cladding"))

        pls, style, _grid = fake_sliced_building(seed=seed)
        for p in pls:
            _author_piece_mesh(stage, p, mat)

        info = qf.describe(style, pls, 0.0, 0.0, 0.0)
        info["type"] = BTYPE
        rng = random.Random(seed)
        plan = qs.plan_damage(info, info["elements"], GRADE_RECIPES, BTYPE, rng)

        ctx = {"stage": stage, "parent": "/World/cell", "info": info,
              "rng": rng, "nrng": None, "mats": _AutoMats(stage, "/World/cell"),
              "cache": {}, "tag": "bench", "loose": [], "static_extra": [],
              "velocity": {}, "authored": [], "notes": [],
              "fit": {"slabs": {}, "columns": {}, "partitions": [],
                      "props": {}, "all": []}}
        old_pp, old_au = qs.PLAN_PILE, qs.AUTHOR
        qs.PLAN_PILE, qs.AUTHOR = _fake_plan_pile, _fake_author
        try:
            qs.apply_plan(stage, ctx, plan, verbose=True)
        finally:
            qs.PLAN_PILE, qs.AUTHOR = old_pp, old_au
        stage.GetRootLayer().Save()
        return info, plan, ctx
    finally:
        qs.QS_RAGGED = old


# --------------------------------------------------------------------------- #
# THE METRIC — a real z(t) height profile off the actual authored geometry
# --------------------------------------------------------------------------- #
def _mesh_world_points(stage, path):
    from pxr import UsdGeom as _ug
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid() or not prim.IsActive():
        return None
    xf = _ug.XformCache()
    out = []
    for p in Usd.PrimRange(prim):
        if not p.IsA(_ug.Mesh):
            continue
        m = _ug.Mesh(p)
        pts = m.GetPointsAttr().Get()
        if not pts:
            continue
        M = np.array(xf.GetLocalToWorldTransform(p), dtype=float)
        P = np.asarray([[q[0], q[1], q[2]] for q in pts], dtype=float)
        out.append(P @ M[:3, :3] + M[3, :3])
    if not out:
        return None
    return np.concatenate(out, axis=0)


def height_profile(stage, m, side, t_lo, t_hi, n=140, band_m=0.35):
    """z(t): the highest world z of ANY mesh point within `band_m` of the
    wall's own plane, for `n` columns of `t` across [t_lo, t_hi]. Walks every
    Mesh prim under `/World/cell` once (cheap: a bench-scale scene).

    BENCH FIX: was rooted at `/World/cell/pieces` only. `quake_flow.
    _break_split` (via `fire_collapse._tear_perimeter`) authors a torn
    piece's fragments at `"{parent}/brk_{tag}_{name}/frag_NNN"` -- a SIBLING
    of `pieces`, not a child of it (`quake_flow._break_split`'s own `out =
    "{0}/brk_{1}_{2}".format(ctx["parent"], ...)`) -- while deactivating the
    original whole piece under `pieces`. Scanning `pieces` alone therefore
    saw the torn piece vanish (correctly excluded via `IsActive()`) but
    never saw what replaced it, so `straight_run_m` came back IDENTICAL
    before/after (measured: 7.742 both ways) despite 13 real tears having
    been authored -- a bench blind spot, not a production defect. Root one
    level up so both `pieces/*` and `brk_*/frag_*` are walked.
    """
    root = stage.GetPrimAtPath("/World/cell")
    all_pts = []
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh) or not prim.IsActive():
            continue
        pts = _mesh_world_points(stage, prim.GetPath())
        if pts is not None:
            all_pts.append(pts)
    if not all_pts:
        return np.linspace(t_lo, t_hi, n), np.zeros(n)
    P = np.concatenate(all_pts, axis=0)
    lx = P[:, 0] - m["cx"]
    ly = P[:, 1] - m["cy"]
    if side in ("S", "N"):
        t = lx + m["W"] / 2.0
        d = np.abs(ly - (-m["D"] / 2.0 if side == "S" else m["D"] / 2.0))
    else:
        t = ly + m["D"] / 2.0
        d = np.abs(lx - (-m["W"] / 2.0 if side == "W" else m["W"] / 2.0))
    near = d <= band_m
    ts = np.linspace(t_lo, t_hi, n)
    z = np.full(n, np.nan)
    step = (t_hi - t_lo) / n
    for i, tc in enumerate(ts):
        sel = near & (np.abs(t - tc) <= step)
        if sel.any():
            z[i] = P[sel, 2].max()
    return ts, z


def straight_run_m(ts, z, tol_m=0.03):
    """Longest run of consecutive samples that stay within `tol_m` of the
    straight line fit through the run's own first and last valid point.
    Pure numpy, no opinion: this is the same idea as `pillar_break_bench.
    face_report`'s "pristine face" test, applied to a height profile instead
    of a box face."""
    ok = ~np.isnan(z)
    idx = np.where(ok)[0]
    if len(idx) < 2:
        return 0.0
    best = 0.0
    i0 = 0
    n = len(idx)
    while i0 < n:
        j0 = idx[i0]
        i1 = i0 + 1
        while i1 < n:
            j1 = idx[i1]
            # fit a line through (ts[j0], z[j0])-(ts[j1], z[j1]) and check
            # every sample in between stays within tol_m of it
            span = ts[j1] - ts[j0]
            if span < 1e-9:
                i1 += 1
                continue
            slope = (z[j1] - z[j0]) / span
            mid = idx[i0:i1 + 1]
            pred = z[j0] + slope * (ts[mid] - ts[j0])
            if np.nanmax(np.abs(z[mid] - pred)) > tol_m:
                break
            i1 += 1
        run_len = ts[idx[i1 - 1]] - ts[j0]
        best = max(best, float(run_len))
        i0 += 1
    return best


# --------------------------------------------------------------------------- #
# bpy render (idiom copied from tools/pillar_break_bench.py /
# tools/rubble_preview.py)
# --------------------------------------------------------------------------- #
def _setup_engine(samples, use_gpu):
    import bpy
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = samples
    scene.cycles.use_denoising = True
    device = "CPU"
    if use_gpu:
        prefs = bpy.context.preferences.addons["cycles"].preferences
        for backend in ("OPTIX", "CUDA", "HIP", "METAL", "ONEAPI"):
            try:
                prefs.compute_device_type = backend
                prefs.get_devices()
            except Exception:
                continue
            gpus = [d for d in prefs.devices if d.type == backend]
            if gpus:
                for d in prefs.devices:
                    d.use = d.type == backend
                scene.cycles.device = "GPU"
                device = "GPU/{0} ({1})".format(backend, gpus[0].name)
                break
    if device == "CPU":
        scene.cycles.device = "CPU"
    return device


def _add_lighting(bg=0.55):
    import bpy
    world = bpy.data.worlds.new("vw")
    world.use_nodes = True
    n = world.node_tree.nodes["Background"]
    n.inputs[0].default_value = (bg, bg, bg * 1.05, 1.0)
    n.inputs[1].default_value = 0.9
    bpy.context.scene.world = world

    def sun(name, rot, energy, angle=4.0):
        d = bpy.data.lights.new(name, "SUN")
        d.energy = energy
        d.angle = math.radians(angle)
        o = bpy.data.objects.new(name, d)
        o.rotation_euler = rot
        bpy.context.collection.objects.link(o)

    sun("key", (math.radians(58), 0, math.radians(-35)), 4.0)
    sun("fill", (math.radians(70), 0, math.radians(150)), 1.1)


def _matte_grey():
    import bpy
    m = bpy.data.materials.new("piece")
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (0.55, 0.54, 0.50, 1.0)
    b.inputs["Roughness"].default_value = 0.78
    for o in bpy.data.objects:
        if o.type == "MESH":
            o.data.materials.clear()
            o.data.materials.append(m)


def _place_camera_lookat(location, target, fov_deg=42.0, clip_end=800.0):
    import bpy
    from mathutils import Vector
    cam_data = bpy.data.cameras.new("cam")
    cam_data.angle = math.radians(fov_deg)
    cam_data.clip_start = 0.05
    cam_data.clip_end = clip_end
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.collection.objects.link(cam)
    cam.location = Vector(location)
    cam.rotation_euler = (Vector(target) - Vector(location)).to_track_quat(
        "-Z", "Y").to_euler()
    bpy.context.scene.camera = cam
    return cam


def _render_to(path, w, h):
    import bpy
    s = bpy.context.scene
    s.render.resolution_x, s.render.resolution_y = w, h
    s.render.resolution_percentage = 100
    s.render.image_settings.file_format = "PNG"
    s.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)


def render_stage(usd_path, out_dir, tag, m, side, keep_material, res, samples,
                 cpu, hole_x=None, hole_z=None):
    import bpy
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=str(usd_path), import_materials=keep_material)
    dev = _setup_engine(samples, use_gpu=not cpu)
    print("[bench] rendering on {0}".format(dev))
    _add_lighting()
    if not keep_material:
        _matte_grey()
    # BENCH FIX: a fixed `height_frac` of the WHOLE building's `top` aims
    # the camera at a storey chosen by the mass's overall height, not at
    # the actual hole -- for a `parapet_fall` recipe (this bench's own
    # `GRADE_RECIPES`) the loss is at the roofline, well above a generic
    # 0.35-0.55 fraction on a many-storey mass, so both shots came back
    # framing an untouched mid-height pier run in both before/after (the
    # visual symptom paired with the `straight_run_m` scan-root bug fixed
    # above). `main()` now measures the removed pieces' own world centroid
    # and passes it through as `hole_x`/`hole_z`; fall back to the old
    # whole-wall-centre behaviour only if a caller has none to give.
    cx0 = hole_x if hole_x is not None else m["cx"]
    made = []
    for name, dist, elev, height_frac, fov in (
            ("obl30", 30.0, 22.0, 0.55, 42.0), ("close8", 8.0, 8.0, 0.35, 55.0)):
        az = math.radians(-100.0)
        el = math.radians(elev)
        cz = hole_z if hole_z is not None else m["z0"] + m["top"] * height_frac
        loc = (cx0 + dist * math.cos(el) * math.cos(az),
               (m["cy"] - m["D"] / 2.0 - 3.0) + dist * math.cos(el) * math.sin(az),
               cz + dist * math.sin(el))
        tgt = (cx0, m["cy"] - m["D"] / 2.0, cz)
        _place_camera_lookat(loc, tgt, fov_deg=fov)
        fp = Path(out_dir) / "{0}_{1}.png".format(tag, name)
        _render_to(fp, *res)
        made.append(fp)
        print("  {0} -> {1}".format(name, fp))
    return made


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.expanduser(
        "~/scorch_previews/sliced_tears"))
    ap.add_argument("--res", default="900x900")
    ap.add_argument("--samples", type=int, default=48)
    ap.add_argument("--cpu", action="store_true")
    ap.add_argument("--no-render", action="store_true")
    args = ap.parse_args()

    out = Path(os.path.expanduser(args.out))
    out.mkdir(parents=True, exist_ok=True)
    res = tuple(int(q) for q in args.res.lower().split("x"))

    report = {}
    window = None
    for tag, ragged in (("before", False), ("after", True)):
        usd = out / "{0}.usda".format(tag)
        if usd.exists():
            usd.unlink()
        info, plan, ctx = build_stage(usd, ragged)
        m = info["masses"]["main"]
        if window is None:
            # THE HOLE'S OWN WINDOW, not the whole wall: away from the
            # failure the top edge is identical before/after (nothing there
            # changed), so a whole-wall profile is dominated by the
            # UNCHANGED roofline and both straight_run_m numbers come out
            # the same. Bracket the removed pieces' own along-wall extent
            # (padded one pier width either side) instead.
            by_path = {(e.get("p") or {}).get("prim_path"): e
                      for e in info["elements"]}
            removed_s = [by_path[p] for p in plan["removed"] if p in by_path
                        and by_path[p]["side"] == "S"]
            ts_removed = [e["lx"] + m["W"] / 2.0 for e in removed_s]
            if ts_removed:
                pad = 2.0
                window = (max(0.0, min(ts_removed) - pad),
                         min(m["W"], max(ts_removed) + pad))
            else:
                window = (0.0, m["W"])
            print("[bench] hole window (S, local t): {0}".format(window))
            # BENCH FIX (paired with the `render_stage` one above): aim the
            # camera at the REMOVED pieces' own world centroid, not a fixed
            # fraction of the whole mass height -- see that function's
            # docstring for why the old fixed fraction missed a roofline
            # `parapet_fall` hole entirely in both before/after renders.
            if removed_s:
                hole_x = m["cx"] + sum(e["lx"] for e in removed_s) / len(removed_s)
                hole_z = (sum(e["z"] + e["h"] / 2.0 for e in removed_s)
                         / len(removed_s))
            else:
                hole_x = hole_z = None
            print("[bench] hole centroid: x={0} z={1}".format(hole_x, hole_z))
        stage = Usd.Stage.Open(str(usd))
        ts, z = height_profile(stage, m, "S", window[0], window[1])
        run = straight_run_m(ts, z)
        report[tag] = {
            "n_removed": plan["stats"]["n_removed"],
            "n_tears": plan["stats"].get("n_tears", 0),
            "n_tears_dropped": plan["stats"].get("n_tears_dropped", 0),
            "straight_run_m": round(run, 3),
        }
        print("[bench] {0}: n_removed={1} n_tears={2} straight_run_m={3:.3f}"
              .format(tag, plan["stats"]["n_removed"],
                      plan["stats"].get("n_tears", 0), run))
        if not args.no_render:
            render_stage(usd, out, tag, m, "S", keep_material=False, res=res,
                        samples=args.samples, cpu=args.cpu,
                        hole_x=hole_x, hole_z=hole_z)
            render_stage(usd, out, tag + "_uv", m, "S", keep_material=True,
                        res=res, samples=args.samples, cpu=args.cpu,
                        hole_x=hole_x, hole_z=hole_z)

    (out / "metrics.json").write_text(json.dumps(report, indent=1))
    print()
    print("[bench] SUMMARY: before straight_run_m={0:.3f}  after={1:.3f}  "
          "(want after < 0.8 m)".format(report["before"]["straight_run_m"],
                                       report["after"]["straight_run_m"]))
    print("[bench] metrics -> {0}".format(out / "metrics.json"))


if __name__ == "__main__":
    main()

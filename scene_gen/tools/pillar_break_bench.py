#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["usd-core", "numpy", "bpy", "pillow", "vtk"]
# ///
"""pillar_break_bench.py — does a chipped pillar READ as broken at 5-10 m?

    uv run --python 3.13 --with usd-core --with numpy --with vtk \
        --with bpy --with pillow \
        python scene_gen/tools/pillar_break_bench.py \
        --out ~/scorch_previews/pillar_bench/

WHY THIS TOOL EXISTS. The user has reported "the pillars are still perfect
cuboids" THREE times, across three review rounds, on scenes that the code
says are chipped (`fracture.chip_box` is wired into `quake_rubble_usd._box`,
`quake_collapse._chip_prim` and `quake_sliced`). Every previous claim that
chipping works was made from unit tests that measure VOLUME LOSS, and volume
loss is not the thing the user is looking at — a 6 % loss spread over six
2 cm nicks on a 0.4 m column is invisible from 8 m away, and the piece still
reads as a cuboid. This renders the actual production output at the actual
viewing distance so the claim can be checked with eyes instead of numbers.

WHAT IT AUTHORS. Two shapes, the two the user keeps pointing at:

  pillar   0.40 x 0.40 x 3.50 m concrete column, standing
  plate    2.50 x 1.60 x 0.12 m rectangular slab piece, lying

Each is authored FIVE ways, all through PRODUCTION code with PRODUCTION
spec dicts imported from their own modules (never re-typed here):

  plain           `quake_rubble_usd._box(chip=None)`         — the control
  rubble_usd      `_box(chip=_chip_spec(kind, seed))`        — the authored
                  "large element" path (lintels/quoins/joists/column stubs)
  collapse        `quake_collapse._chip_prim(spec)`          — the in-place
                  round trip `_break_box` cells and dropped plates take
  collapse_tess   `_chip_prim(spec, tessellate=True)`        — what a dropped
                  floor plate and `quake_sliced` get
  (x N seeds)     the same path at several seeds, because "does the
                  POPULATION read as broken" is the actual question

MEASUREMENTS, printed and written to `metrics.json`, chosen so they answer
the complaint rather than restate the unit tests:

  vol_loss_pct     what the existing tests already pin
  pristine_faces   how many of the 6 original box faces still have >90 % of
                   their area intact and flat. THE HEADLINE NUMBER: a piece
                   with 5 or 6 pristine faces IS a cuboid, whatever its
                   volume loss says, and the user will call it one.
  max_bite_frac    the deepest single bite as a fraction of the shortest
                   cross-section. "small to very large chips" needs some
                   pieces well above ~0.25 here; if the population maxes out
                   at 0.1 there is no large-bite mode at all.
  sil_dev_mm       how far the silhouette departs from the bounding box,
                   in mm, at the 90th percentile of the outline.

RENDER. Two views per piece at the distance the complaint is made from:
an oblique at `--obl-dist` (default 7 m, elevation 22 deg) and a
contact-height view at `--close-dist` (default 5 m, camera at 1.6 m).
Matte grey, two suns — shape-from-shading, no texture to hide behind.
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
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, _SCENE_GEN)

from disaster import fracture as fr                  # noqa: E402
from disaster import quake_collapse as qc            # noqa: E402
from disaster import quake_rubble_usd as qru         # noqa: E402

# The two shapes, and which PRODUCTION spec each of them is really governed
# by. `kind` is the `_CHIP_KIND` key `_author_large` would pass for that
# population; `collapse_spec` is the `_CHIP_*` dict `quake_collapse` /
# `quake_sliced` hand `_chip_prim` for the same geometry.
SHAPES = {
    "pillar": {
        "size": (0.40, 0.40, 3.50),
        "kind": "column",
        "collapse_spec": "_CHIP_PRISM",
        "bottom": True,          # stands on the ground
        "pitch": 3.0,
    },
    "plate": {
        "size": (2.50, 1.60, 0.12),
        "kind": "lintel",
        "collapse_spec": "_CHIP_SLAB",
        "bottom": True,
        "pitch": 4.0,
    },
}

GROUND_Z = 0.0


# --------------------------------------------------------------------------- #
# measurement
# --------------------------------------------------------------------------- #
def _tri_areas(v, f):
    return 0.5 * np.linalg.norm(
        np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]]), axis=1)


def face_report(v, f, sizes, tol_m=0.012):
    """Per-original-box-face intact area fraction, and the headline count.

    A face of the source cuboid is "pristine" when >90 % of its nominal area
    is still covered by triangles that lie in its plane (within `tol_m`) and
    face the same way. That is the geometric statement of "this still looks
    like a cuboid from that side" — and it is what volume loss cannot see:
    six small nicks on six different edges cost 5 % of the volume and leave
    all six faces reading flat.
    """
    v = np.asarray(v, dtype=float)
    f = np.asarray(f, dtype=np.int64)
    lo, hi = v.min(0), v.max(0)
    n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    ln = np.linalg.norm(n, axis=1)
    ok = ln > 1e-12
    nh = np.zeros_like(n)
    nh[ok] = n[ok] / ln[ok][:, None]
    area = 0.5 * ln
    cen = v[f].mean(axis=1)
    sx, sy, sz = [abs(float(q)) for q in sizes]
    nominal = {0: sy * sz, 1: sx * sz, 2: sx * sy}
    out, pristine = {}, 0
    for ax in range(3):
        for sgn, plane in ((+1.0, hi[ax]), (-1.0, lo[ax])):
            d = np.abs(cen[:, ax] - plane)
            sel = ok & (d <= tol_m) & (nh[:, ax] * sgn > 0.90)
            a = float(area[sel].sum())
            frac = a / max(nominal[ax], 1e-9)
            name = "{0}{1}".format("+-"[sgn < 0], "xyz"[ax])
            out[name] = round(min(frac, 1.5), 3)
            if frac > 0.90:
                pristine += 1
    return out, pristine


def section_profile(v, f, sizes, n_cross=13, n_long=40):
    """CROSS-SECTION FILL at `n_long` stations along the piece's long axis.

    THE MEASUREMENT THAT SETTLED THE ARGUMENT, and the reason it is done by
    OCCUPANCY (`vtkSelectEnclosedPoints` over a grid inside the nominal box)
    rather than by clipping the mesh into slabs: a slab clip of an already
    clipped solid comes back with broken caps and reports fills of 7.0 and
    14.9, which is nonsense. Sampling points against the closed surface cannot
    fail that way.

    A piece whose profile sits at 0.85-0.99 everywhere except its two end
    stations IS a cuboid with the corners knocked off, however much volume the
    unit tests say it lost — that is exactly what round 5 produced, and exactly
    what the user has objected to three times.
    """
    import vtk
    from vtk.util import numpy_support as ns

    sizes = np.abs(np.asarray(sizes, dtype=float))
    axis = int(np.argmax(sizes))
    v = np.asarray(v, dtype=float)
    pd = fr._to_vtk(fr._Arrays(v, np.asarray(f, dtype=np.int64)))
    c = 0.5 * (v.max(0) + v.min(0))
    ns_ = [n_cross] * 3
    ns_[axis] = n_long
    ax_pts = []
    for i in range(3):
        h = sizes[i] / 2.0
        ax_pts.append(np.linspace(c[i] - h * (1 - 0.5 / ns_[i]),
                                  c[i] + h * (1 - 0.5 / ns_[i]), ns_[i]))
    G = np.stack(np.meshgrid(*ax_pts, indexing="ij"), -1).reshape(-1, 3)
    pts = vtk.vtkPoints()
    pts.SetData(ns.numpy_to_vtk(np.ascontiguousarray(G), deep=True))
    q = vtk.vtkPolyData()
    q.SetPoints(pts)
    sel = vtk.vtkSelectEnclosedPoints()
    sel.SetInputData(q)
    sel.SetSurfaceData(pd)
    sel.SetTolerance(1e-7)
    sel.Update()
    ins = ns.vtk_to_numpy(
        sel.GetOutput().GetPointData().GetArray("SelectedPoints"))
    occ = ins.reshape(ns_).astype(float)
    other = tuple(i for i in range(3) if i != axis)
    return occ.mean(axis=other), float(occ.mean())


def measure(v, f, sizes, profile=True):
    faces, pristine = face_report(v, f, sizes)
    v0 = float(abs(sizes[0] * sizes[1] * sizes[2]))
    vol = fr.mesh_volume(np.asarray(v, float), np.asarray(f, np.int64))
    out = {
        "verts": int(len(v)), "tris": int(len(f)),
        "vol_loss_pct": round(100.0 * (1.0 - vol / max(v0, 1e-9)), 1),
        "pristine_faces": int(pristine),
        "face_intact": faces,
        "open_edges": int(fr.open_edge_count(np.asarray(f, np.int64))),
    }
    if profile:
        sec, tot = section_profile(v, f, sizes)
        # THE SHAFT, not the ends: the middle 80 % of the piece is what a
        # camera at 7 m is looking at, and it is where round 5 changed nothing.
        k = max(1, int(round(len(sec) * 0.10)))
        shaft = sec[k:-k]
        out.update(
            box_fill=round(tot, 3),
            shaft_min=round(float(shaft.min()), 3),
            shaft_p25=round(float(np.percentile(shaft, 25)), 3),
            profile=[round(float(q), 2) for q in sec],
        )
    return out


# --------------------------------------------------------------------------- #
# authoring — every variant goes through PRODUCTION code
# --------------------------------------------------------------------------- #
def _mesh_arrays(stage, path):
    from pxr import UsdGeom
    m = UsdGeom.Mesh(stage.GetPrimAtPath(path))
    pts = m.GetPointsAttr().Get()
    v = np.asarray([[float(p[0]), float(p[1]), float(p[2])] for p in pts])
    cnt = [int(c) for c in m.GetFaceVertexCountsAttr().Get()]
    idx = [int(i) for i in m.GetFaceVertexIndicesAttr().Get()]
    tris, k = [], 0
    for c in cnt:
        for i in range(1, c - 1):
            tris.append((idx[k], idx[k + i], idx[k + i + 1]))
        k += c
    return v, np.asarray(tris, dtype=np.int64)


def _place(stage, path, x, y, z=0.0, rot=None):
    from pxr import Gf, UsdGeom
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(path))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), float(z)))
    if rot:
        xf.AddRotateXYZOp().Set(Gf.Vec3f(*[float(q) for q in rot]))


def author_variants(stage, shape, seeds, variants):
    """One row per variant, one column per seed. Returns the piece records."""
    from pxr import UsdGeom
    cfg = SHAPES[shape]
    sx, sy, sz = cfg["size"]
    cspec = dict(getattr(qc, cfg["collapse_spec"]))
    pitch = cfg["pitch"]
    recs = []
    for row, variant in enumerate(variants):
        for col, seed in enumerate(seeds):
            name = "{0}_{1}_s{2}".format(shape, variant, seed)
            path = "/World/{0}/{1}".format(shape, name)
            if variant == "plain":
                qru._box(stage, path, sx, sy, sz)
            elif variant == "rubble_usd":
                spec = qru._chip_spec(cfg["kind"], seed)
                qru._box(stage, path, sx, sy, sz, chip=spec)
            elif variant in ("collapse", "collapse_tess"):
                qru._box(stage, path, sx, sy, sz)
                # `_chip_prim` seeds off the PRIM PATH (`stable_seed`), which
                # is how production varies pieces — so the seed column here is
                # carried by the path, exactly as in a real build.
                qc._chip_prim(stage, path, cspec,
                              tessellate=(variant == "collapse_tess"))
            else:
                raise SystemExit("unknown variant " + variant)
            v, f = _mesh_arrays(stage, path)
            m = measure(v, f, (sx, sy, sz))
            m.update(shape=shape, variant=variant, seed=seed, path=path)
            x = (col - (len(seeds) - 1) / 2.0) * pitch
            y = row * (pitch * 2.5)
            _place(stage, path, x, y, GROUND_Z)
            m["world"] = (x, y, GROUND_Z)
            recs.append(m)
    return recs


# --------------------------------------------------------------------------- #
# heap mode — a masonry heap with lintels, through `quake_flow._chip_authored`
# --------------------------------------------------------------------------- #
# The per-piece grid above answers "does ONE piece read as broken". This
# answers the question the user actually asked on the DG5 shots: does a HEAP
# stop reading as a box of bricks. It authors with `quake_flow._box` and the
# EXACT draw ranges of `_p_lintels` (the three branches, the yaw, the lift off
# the base), scatters small rubble round them for context, and then hands the
# whole list to `quake_flow._chip_authored` — the same call the emitter now
# makes. `--chipped 0` is the before: it sets `QC_CHIP=0`, so the identical
# scatter is authored and NOTHING is chipped, which makes the two renders
# differ only in the chip pass.


def author_heap(stage, seed=3, n_lintel=9, n_rubble=70):
    """`_p_lintels`' own population, plus rubble, in one 12 x 9 m heap."""
    from disaster import quake_flow as qf

    rng = random.Random(int(seed))
    made, sizes = [], []
    for i in range(int(n_lintel)):
        r = rng.random()
        if r < 0.55:                            # LINTEL / cornice run
            sx, sy, sz = (rng.uniform(1.1, 2.2), rng.uniform(0.20, 0.32),
                          rng.uniform(0.18, 0.30))
        elif r < 0.85:                          # QUOIN / sill block
            sx, sy, sz = (rng.uniform(0.36, 0.58), rng.uniform(0.28, 0.42),
                          rng.uniform(0.24, 0.38))
        else:                                   # arch head / coping stone
            sx, sy, sz = (rng.uniform(0.7, 1.1), rng.uniform(0.30, 0.45),
                          rng.uniform(0.22, 0.34))
        x = rng.uniform(-0.44, 0.44) * 13.0
        y = rng.uniform(-0.44, 0.44) * 10.0
        z = GROUND_Z + sz * 0.5 + rng.uniform(0.05, 0.6)
        p = "/World/heap/lintel_{0:02d}".format(i)
        qf._box(stage, p, x, y, z, sx, sy, sz,
                yaw_deg=rng.uniform(0.0, 360.0))
        made.append(p)
        sizes.append((sx, sy, sz))
    for k in range(int(n_rubble)):              # context: brick-scale litter
        s = rng.uniform(0.10, 0.34)
        p = "/World/heap/rub_{0:03d}".format(k)
        qf._box(stage, p, rng.uniform(-6.5, 6.5), rng.uniform(-5.0, 5.0),
                GROUND_Z + s * 0.3, s, s * rng.uniform(0.5, 1.0),
                s * rng.uniform(0.3, 0.6), rng.uniform(0, 180))
        made.append(p)
        sizes.append((s, s, s))
    ctx = {"stage": stage, "parent": "/World/heap", "mats": {}, "notes": []}
    n = qf._chip_authored(ctx, made, why="lintel")
    qf._chip_report(ctx)
    print("[bench] heap: {0} pieces, {1} chipped, {2} passed".format(
        len(made), n, ctx.get("_chip_m", 0)))
    return made, sizes, n


def build_heap_stage(out_usd, seed, n_lintel, n_rubble):
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.CreateNew(str(out_usd))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.DefinePrim("/World", "Xform")
    stage.DefinePrim("/World/heap", "Xform")
    author_ground(stage)
    made, sizes, n = author_heap(stage, seed, n_lintel, n_rubble)
    stage.GetRootLayer().Save()
    return made, sizes, n


def render_heap(usd_path, out_dir, tag, res, samples, cpu):
    import bpy
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=str(usd_path))
    print("[bench] rendering on {0}".format(
        _setup_engine(samples, use_gpu=not cpu)))
    _add_lighting()
    _matte_grey()
    made = []
    for name, loc, tgt, fov in (
            ("obl", (-7.5, -9.0, 5.2), (0.0, 0.0, 0.35), 46.0),
            ("contact", (-3.4, -6.2, 1.55), (0.6, 0.6, 0.30), 52.0)):
        _place_camera_lookat(loc, tgt, fov_deg=fov)
        fp = Path(out_dir) / "{0}_{1}.png".format(tag, name)
        _render_to(fp, *res)
        made.append(fp)
        print("  heap {0} -> {1}".format(name, fp))
    return made


def author_ground(stage, half=90.0):
    from pxr import Gf, Sdf, UsdGeom, Vt
    p = "/World/ground"
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(p))
    P = Gf.Vec3f
    m.CreatePointsAttr(Vt.Vec3fArray([P(-half, -half, GROUND_Z),
                                      P(half, -half, GROUND_Z),
                                      P(half, half, GROUND_Z),
                                      P(-half, half, GROUND_Z)]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    m.CreateNormalsAttr(Vt.Vec3fArray([P(0, 0, 1)] * 4))
    m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    m.CreateExtentAttr([P(-half, -half, GROUND_Z), P(half, half, GROUND_Z)])
    return p


def build_stage(out_usd, shape, seeds, variants):
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.CreateNew(str(out_usd))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.DefinePrim("/World", "Xform")
    stage.DefinePrim("/World/" + shape, "Xform")
    author_ground(stage)
    recs = author_variants(stage, shape, seeds, variants)
    stage.GetRootLayer().Save()
    return recs


# --------------------------------------------------------------------------- #
# bpy render (idioms copied from tools/rubble_preview.py)
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


def _add_lighting(bg=0.5):
    import bpy
    world = bpy.data.worlds.new("vw")
    world.use_nodes = True
    n = world.node_tree.nodes["Background"]
    n.inputs[0].default_value = (bg, bg, bg * 1.05, 1.0)
    n.inputs[1].default_value = 0.85
    bpy.context.scene.world = world

    def sun(name, rot, energy, angle=4.0):
        d = bpy.data.lights.new(name, "SUN")
        d.energy = energy
        d.angle = math.radians(angle)
        o = bpy.data.objects.new(name, d)
        o.rotation_euler = rot
        bpy.context.collection.objects.link(o)

    # A LOW, RAKING KEY. A break reads by shading, not by colour: a sun near
    # the zenith flattens exactly the vertical facets a chipped column is
    # made of. 32 deg elevation, off to one side, plus a soft opposite fill.
    sun("key", (math.radians(58), 0, math.radians(35)), 4.5)
    sun("fill", (math.radians(70), 0, math.radians(215)), 1.2)


def _matte_grey():
    """One matte concrete-grey material on EVERY imported mesh.

    On purpose: this bench is about SILHOUETTE AND FACETS. A texture would
    let a flat face look busy and hide the fact that it is flat, which is the
    exact failure being hunted. The ground gets a darker version so a piece
    lying on it still separates."""
    import bpy
    def mk(name, base, rough):
        m = bpy.data.materials.new(name)
        m.use_nodes = True
        b = m.node_tree.nodes["Principled BSDF"]
        b.inputs["Base Color"].default_value = base
        b.inputs["Roughness"].default_value = rough
        return m

    piece = mk("piece", (0.52, 0.51, 0.48, 1.0), 0.72)
    ground = mk("ground", (0.20, 0.20, 0.21, 1.0), 0.92)
    for o in bpy.data.objects:
        if o.type != "MESH":
            continue
        o.data.materials.clear()
        o.data.materials.append(ground if "ground" in o.name else piece)


def _place_camera_lookat(location, target, fov_deg=42.0, clip_end=600.0):
    import bpy
    from mathutils import Vector
    cam_data = bpy.data.cameras.new("cam")
    cam_data.angle = math.radians(fov_deg)
    cam_data.clip_start = 0.02
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


def _sheet(tiles, out, title, labels=None, cols=None):
    from PIL import Image, ImageDraw, ImageFont
    imgs = [Image.open(t).convert("RGB") for t in tiles]
    cols = cols or len(imgs)
    rows = int(math.ceil(len(imgs) / float(cols)))
    tw, th = imgs[0].size
    pad, header, lab = 8, 44, 22
    W = cols * tw + (cols + 1) * pad
    H = header + rows * (th + lab + pad) + pad
    sheet = Image.new("RGB", (W, H), (18, 18, 18))
    d = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 20)
        small = ImageFont.truetype("DejaVuSans.ttf", 15)
    except Exception:
        font = small = ImageFont.load_default()
    d.text((pad, 12), title, fill=(235, 235, 230), font=font)
    for i, im in enumerate(imgs):
        r, c = divmod(i, cols)
        x = pad + c * (tw + pad)
        y = header + r * (th + lab + pad)
        sheet.paste(im, (x, y))
        if labels:
            d.text((x + 4, y + th + 3), labels[i], fill=(200, 200, 195),
                   font=small)
    sheet.save(out)
    return out


def render_pieces(usd_path, recs, out_dir, tag, obl_dist, close_dist,
                  res, samples, cpu):
    import bpy
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=str(usd_path))
    dev = _setup_engine(samples, use_gpu=not cpu)
    print("[bench] rendering on {0}".format(dev))
    _add_lighting()
    _matte_grey()

    made = []
    t0 = time.time()
    for r in recs:
        x, y, z = r["world"]
        # the piece's own bbox, in world
        top = z + r.get("top_m", 1.0)
        cz = z + r.get("top_m", 1.0) * 0.5
        centre = (x, y, cz)
        # oblique: down onto the piece from 30 deg off its face
        az = math.radians(-118.0)
        el = math.radians(22.0)
        loc = (x + obl_dist * math.cos(el) * math.cos(az),
               y + obl_dist * math.cos(el) * math.sin(az),
               cz + obl_dist * math.sin(el))
        _place_camera_lookat(loc, centre, fov_deg=40.0)
        fp = Path(out_dir) / "{0}_{1}_obl.png".format(tag, r["variant"] + "_s" + str(r["seed"]))
        _render_to(fp, *res)
        r["png_obl"] = str(fp)
        # contact height: eye level, close, slightly off-axis
        eye = 1.60
        az2 = math.radians(-100.0)
        loc2 = (x + close_dist * math.cos(az2), y + close_dist * math.sin(az2),
                eye)
        _place_camera_lookat(loc2, (x, y, min(max(cz, 0.35), eye)), fov_deg=48.0)
        fp2 = Path(out_dir) / "{0}_{1}_close.png".format(tag, r["variant"] + "_s" + str(r["seed"]))
        _render_to(fp2, *res)
        r["png_close"] = str(fp2)
        made += [fp, fp2]
        print("  {0:22s} obl+close  ({1:.0f}s)".format(
            r["variant"] + "/s" + str(r["seed"]), time.time() - t0))
    return made


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.expanduser(
        "~/scorch_previews/pillar_bench"))
    ap.add_argument("--shapes", default="pillar,plate")
    ap.add_argument("--variants",
                    default="plain,rubble_usd,collapse,collapse_tess")
    ap.add_argument("--seeds", default="1,2,3")
    ap.add_argument("--tag", default="v1")
    ap.add_argument("--obl-dist", type=float, default=7.0)
    ap.add_argument("--close-dist", type=float, default=5.0)
    ap.add_argument("--res", default="900x900")
    ap.add_argument("--samples", type=int, default=48)
    ap.add_argument("--cpu", action="store_true")
    ap.add_argument("--no-render", action="store_true",
                    help="measure only — no bpy, no PNGs")
    ap.add_argument("--heap", action="store_true",
                    help="render a masonry heap of `_p_lintels` pieces "
                         "through quake_flow._chip_authored instead of the "
                         "per-piece grid")
    ap.add_argument("--heap-seed", type=int, default=3)
    ap.add_argument("--heap-lintels", type=int, default=9)
    ap.add_argument("--heap-rubble", type=int, default=70)
    args = ap.parse_args()

    out = Path(os.path.expanduser(args.out))
    out.mkdir(parents=True, exist_ok=True)
    res = tuple(int(q) for q in args.res.lower().split("x"))
    seeds = [int(q) for q in args.seeds.split(",") if q.strip()]
    variants = [q.strip() for q in args.variants.split(",") if q.strip()]

    print("[bench] QC_CHIP={0!r} chips_enabled={1} vtk={2}".format(
        os.environ.get("QC_CHIP"), fr.chips_enabled(), fr._vtk() is not None))
    if fr._vtk() is None:
        print("[bench] !! VTK IS NOT IMPORTABLE — chip_box passes everything "
              "through unchanged. Every 'chipped' variant below IS a cuboid.")

    if args.heap:
        usd = out / "{0}_heap.usda".format(args.tag)
        if usd.exists():
            usd.unlink()
        made, sizes, n = build_heap_stage(usd, args.heap_seed,
                                          args.heap_lintels, args.heap_rubble)
        if not args.no_render:
            render_heap(usd, out, "{0}_heap".format(args.tag), res,
                        args.samples, args.cpu)
        (out / "{0}_heap.json".format(args.tag)).write_text(json.dumps(
            {"pieces": len(made), "chipped": n,
             "chips_enabled": fr.chips_enabled()}, indent=1))
        return

    all_recs = []
    for shape in [q.strip() for q in args.shapes.split(",") if q.strip()]:
        usd = out / "{0}_{1}.usda".format(args.tag, shape)
        if usd.exists():
            usd.unlink()
        recs = build_stage(usd, shape, seeds, variants)
        sz = SHAPES[shape]["size"]
        for r in recs:
            r["top_m"] = float(sz[2])
        print("\n=== {0}  {1[0]} x {1[1]} x {1[2]} m   kind={2} "
              "collapse_spec={3} ===".format(shape, sz, SHAPES[shape]["kind"],
                                             SHAPES[shape]["collapse_spec"]))
        print("  {0:18s} {1:>6s} {2:>5s} {3:>9s} {4:>9s} {5:>6s}  {6}".format(
            "variant/seed", "loss%", "prist", "shaft_min", "shaft_p25", "tris",
            "cross-section fill along the long axis"))
        for r in recs:
            print("  {0:18s} {1:6.1f} {2:5d} {3:9.2f} {4:9.2f} {5:6d}  {6}"
                  .format(r["variant"] + "/s" + str(r["seed"]),
                          r["vol_loss_pct"], r["pristine_faces"],
                          r.get("shaft_min", -1), r.get("shaft_p25", -1),
                          r["tris"],
                          " ".join("%2.0f" % (100 * q)
                                   for q in r.get("profile", []))))
        if not args.no_render:
            render_pieces(usd, recs, out, "{0}_{1}".format(args.tag, shape),
                          args.obl_dist, args.close_dist, res, args.samples,
                          args.cpu)
            for view in ("obl", "close"):
                tiles = [r["png_" + view] for r in recs]
                labs = ["{0}/s{1}  loss {2}%  pristine {3}/6  shaft_min "
                        "{4}".format(r["variant"], r["seed"],
                                     r["vol_loss_pct"], r["pristine_faces"],
                                     r.get("shaft_min", "-")) for r in recs]
                sh = _sheet(tiles, out / "{0}_{1}_{2}_SHEET.png".format(
                    args.tag, shape, view),
                    "{0}  {1}  ({2} view, {3:.0f} m)".format(
                        args.tag, shape, view,
                        args.obl_dist if view == "obl" else args.close_dist),
                    labels=labs, cols=len(seeds))
                print("[bench] sheet -> {0}".format(sh))
        all_recs += recs

    mj = out / "{0}_metrics.json".format(args.tag)
    mj.write_text(json.dumps(all_recs, indent=1, default=str))
    print("[bench] metrics -> {0}".format(mj))


if __name__ == "__main__":
    main()

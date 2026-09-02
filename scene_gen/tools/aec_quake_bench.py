#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["usd-core", "numpy", "vtk", "bpy", "pillow"]
# ///
"""aec_quake_bench.py — an OFFLINE look check for `disaster.aec_quake` on a
REAL local AEC brownstone row (no Isaac Sim, no Nucleus).

    uv run --python 3.13 --with usd-core --with numpy --with vtk --with bpy \
        --with pillow python scene_gen/tools/aec_quake_bench.py \
        --grade DG4 --seed 7 --out ~/scorch_previews/aec_quake/

References `scene_gen/assets/aec/brownstone/Assets/Create_Brownstone02/
Reference_Brownstone5Row.usd` (the same row `aec_burn`'s own fire round
benched), runs `aec_quake.quake_row` against the LOCAL asset mirror so the
frontage pile's debris references resolve (`asset_root=LOCAL_ASSET_ROOT`,
`flatten_instances=True` — Blender's `bpy.ops.wm.usd_import` does not bring
in `UsdGeom.PointInstancer`, the same reason `rubble_preview.py` sets both),
exports the damaged stage, and renders two views (oblique + a close nadir
over the frontage) through headless Blender/Cycles.

Render idioms (`_setup_engine`/`_add_lighting`/`_place_camera_lookat`/
`_render_to`) are COPIED from `tools/rubble_preview.py`, which itself copied
them from `render_usd.py` (owned by another agent, not to be touched) — same
discipline, one more remove: `rubble_preview.py` is fire/quake-rubble
tooling this session should not edit either, so its idioms are reproduced
here rather than imported.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, _SCENE_GEN)

from disaster import aec_quake as aq          # noqa: E402

LOCAL_ASSET_ROOT = os.path.join(_SCENE_GEN, "assets")
_ROW5 = os.path.join(LOCAL_ASSET_ROOT, "aec", "brownstone", "Assets",
                     "Create_Brownstone02", "Reference_Brownstone5Row.usd")


# --------------------------------------------------------------------------- #
# a PREVIEW-ONLY material override -- the row's own brick is MDL/vMaterials
# (`aec_burn.py`'s own docstring: "two of the five units are vMaterials
# ... a PROCEDURAL blend ... the raw MDL renders exactly as shipped"), which
# Blender's `bpy.ops.wm.usd_import` does not understand at all and silently
# substitutes a black, unlit default for -- the reason `aec_burn` itself
# ships a 2D PIL oracle (`preview_png`) rather than a photoreal bench render.
# This bind is `strongerThanDescendants` on every DIRECT CHILD of the row
# prim EXCEPT this module's own `QuakeLooks`/`QuakeDecals` scopes (so scar
# decals keep their own dark crack material) -- `aec_quake`'s debris scope
# is a SIBLING of the row prim (`aec_burn._debris_scope`), never touched by
# this at all. Never used by `disaster.aec_quake` itself; bench-only.
# --------------------------------------------------------------------------- #
def _hide_site_dressing(stage, row_prim):
    """Deactivate the asset's own `Options_Grass_*` foliage patches -- site
    dressing, not building fabric, and the preview brick override above
    (walked from the row's own DIRECT children with `strongerThanDescendants`
    cascading to every descendant) tints them the same brick-brown as the
    walls, reading as a mess of dead brown sticks that hides the frontage
    this bench exists to show. Bench-only; `aec_quake` never touches these."""
    from pxr import Usd

    n = 0
    for prim in Usd.PrimRange(row_prim):
        if prim.GetName().startswith("Options_Grass"):
            prim.SetActive(False)
            n += 1
    return n


def _preview_fallback_override(stage, row_prim, skip_names=("QuakeLooks", "QuakeDecals")):
    from pxr import Gf, Sdf, UsdShade

    mat_path = str(row_prim.GetPath()) + "/PreviewBrick"
    mat = UsdShade.Material.Define(stage, Sdf.Path(mat_path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(mat_path).AppendChild("Shader"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.42, 0.24, 0.18))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    n = 0
    for child in row_prim.GetChildren():
        if child.GetName() in skip_names:
            continue
        UsdShade.MaterialBindingAPI.Apply(child).Bind(mat, UsdShade.Tokens.strongerThanDescendants)
        n += 1
    return mat, n


# --------------------------------------------------------------------------- #
# build + damage the row, bare pxr
# --------------------------------------------------------------------------- #
def build_damaged_row(grade, seed, asset_path, out_usd):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

    stage = Usd.Stage.CreateNew(str(out_usd))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    # DELIBERATELY no `SetStageMetersPerUnit` call here: this asset is
    # authored at 0.01 m/unit (measured -- `UsdGeom.GetStageMetersPerUnit`
    # on the raw file), and an unauthored stage already defaults to that
    # same 0.01 (measured too). Forcing 1.0 here once made every `measure_
    # row` figure (and this script's own camera/ground framing) read 100x
    # too large -- `aec_quake`/`aec_burn` themselves read the mpu straight
    # off the STAGE, so a caller-declared mismatch against the referenced
    # asset's own authoring unit silently breaks every metre figure derived
    # from it, `aec_quake` included.
    root_path = "/World"
    stage.DefinePrim(root_path, "Xform")
    stage.SetDefaultPrim(stage.GetPrimAtPath(root_path))
    row_path = "{0}/row0".format(root_path)
    row = stage.DefinePrim(row_path, "Xform")
    row.GetReferences().AddReference(asset_path)

    t0 = time.time()
    stats = aq.quake_row(stage, row_path, grade=grade, seed=seed, verbose=True,
                        asset_root=LOCAL_ASSET_ROOT, flatten_instances=True)
    dt = time.time() - t0

    _hide_site_dressing(stage, row)
    _preview_fallback_override(stage, row)

    # a flat ground plane for scale/contact -- authored in the STAGE's own
    # RAW units (like everything else composed here) so Blender's importer
    # scales it by the SAME `mpu` as the row when it converts to its own
    # metre space on import; `bounds` (returned below, used only AFTER
    # import) is in real METRES instead, for camera placement in Blender's
    # own post-import space -- see the note on `SetStageMetersPerUnit` above
    # for why conflating the two once put the camera inside the building.
    mpu = float(UsdGeom.GetStageMetersPerUnit(stage))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng_ = bc.ComputeWorldBound(row).ComputeAlignedRange()
    lo, hi = rng_.GetMin(), rng_.GetMax()              # RAW stage units
    cx, cy = 0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])
    span_m = max((hi[0] - lo[0]) * mpu, (hi[1] - lo[1]) * mpu, 20.0)
    hx = hy = (span_m * 1.6) / mpu                     # back to raw units
    gz = float(lo[2]) - (0.03 / mpu)                   # 3 cm below, raw units
    gpath = root_path + "/ground"
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(gpath))
    m.CreatePointsAttr(Vt.Vec3fArray([
        Gf.Vec3f(cx - hx, cy - hy, gz), Gf.Vec3f(cx + hx, cy - hy, gz),
        Gf.Vec3f(cx + hx, cy + hy, gz), Gf.Vec3f(cx - hx, cy + hy, gz)]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * 4))
    m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateDoubleSidedAttr(True)
    m.CreateExtentAttr([Gf.Vec3f(cx - hx, cy - hy, gz), Gf.Vec3f(cx + hx, cy + hy, gz)])
    gmat = UsdShade.Material.Define(stage, Sdf.Path(root_path + "/ground_mat"))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(root_path + "/ground_mat/Shader"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.38, 0.37, 0.35))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.95)
    gmat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(gmat)

    # the frontage pile's own centre (METRES) -- a dedicated camera aims here
    # directly rather than trusting the row's oblique/front views to happen
    # to frame it (the pile sits on the row's STREET side, `aec_quake._row_
    # mass`'s "front" -- not necessarily the side either fixed view faces).
    pile_center_m = None
    if stats.get("pile"):
        debris_prim = stage.GetPrimAtPath(stats["pile"]["scope"])
        if debris_prim and debris_prim.IsValid():
            prng = bc.ComputeWorldBound(debris_prim).ComputeAlignedRange()
            if not prng.IsEmpty():
                plo, phi = prng.GetMin(), prng.GetMax()
                pile_center_m = (0.5 * (plo[0] + phi[0]) * mpu,
                                0.5 * (plo[1] + phi[1]) * mpu,
                                0.5 * (plo[2] + phi[2]) * mpu)

    stage.GetRootLayer().Save()
    # METRES, for the camera -- placed AFTER Blender's own unit-correct
    # `usd_import`, never mixed with the raw-unit authoring above.
    bounds = {"cx": cx * mpu, "cy": cy * mpu, "z0": float(lo[2]) * mpu,
             "z1": float(hi[2]) * mpu, "span": span_m,
             "pile_center": pile_center_m}
    return stats, dt, bounds


# --------------------------------------------------------------------------- #
# bpy render (copied from tools/rubble_preview.py -- see module docstring)
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


def _add_lighting(bg):
    import bpy
    world = bpy.data.worlds.new("vw")
    world.use_nodes = True
    bg_node = world.node_tree.nodes["Background"]
    bg_node.inputs[0].default_value = (bg, bg, bg, 1.0)
    bg_node.inputs[1].default_value = 0.9
    bpy.context.scene.world = world

    def sun(name, rot, energy):
        d = bpy.data.lights.new(name, "SUN")
        d.energy = energy
        d.angle = math.radians(3)
        o = bpy.data.objects.new(name, d)
        o.rotation_euler = rot
        bpy.context.collection.objects.link(o)

    sun("key", (math.radians(55), 0, math.radians(40)), 4.0)
    sun("fill", (math.radians(65), 0, math.radians(210)), 1.5)


def _place_camera(center, dist, az_deg, el_deg, fov=math.radians(42)):
    import bpy
    from mathutils import Vector
    cam_data = bpy.data.cameras.new("cam")
    cam_data.angle = fov
    cam_data.clip_start = 0.05
    cam_data.clip_end = dist * 20.0
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.collection.objects.link(cam)
    az, el = math.radians(az_deg), math.radians(el_deg)
    direction = Vector((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
    cam.location = Vector(center) + direction * dist
    cam.rotation_euler = (Vector(center) - cam.location).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam
    return cam


def _place_camera_lookat(location, target, fov=math.radians(50), clip_end=500.0):
    import bpy
    from mathutils import Vector
    cam_data = bpy.data.cameras.new("cam")
    cam_data.angle = fov
    cam_data.clip_start = 0.03
    cam_data.clip_end = clip_end
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.collection.objects.link(cam)
    loc = Vector(location)
    cam.location = loc
    cam.rotation_euler = (Vector(target) - loc).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam
    return cam


def _render_to(path, w, h):
    import bpy
    scene = bpy.context.scene
    scene.render.resolution_x = w
    scene.render.resolution_y = h
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)


def render_views(usd_path, out_dir, tag, center, res=(1280, 720), samples=64,
                 cpu=False):
    import bpy
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=str(usd_path))

    device = _setup_engine(samples, use_gpu=not cpu)
    print("[aec_quake_bench] rendering on {0}".format(device))
    _add_lighting(0.55)

    span = center["span"]
    dist = max(20.0, span * 1.4)
    obl_center = (center["cx"], center["cy"], 0.5 * (center["z0"] + center["z1"]))
    top_center = (center["cx"], center["cy"], center["z0"])
    views = [
        ("obl", obl_center, dist, 35.0, 28.0),
        ("obl_rear", obl_center, dist, 35.0 + 180.0, 28.0),
        ("top", top_center, dist * 1.1, 0.0, 82.0),
    ]
    tiles = []
    for name, c, d, az, el in views:
        _place_camera(c, d, az, el)
        fp = Path(out_dir) / "{0}_{1}.png".format(tag, name)
        _render_to(fp, *res)
        tiles.append(fp)

    # a dedicated shot AIMED AT the frontage pile -- not necessarily framed
    # by either fixed oblique above (the pile sits on the row's own STREET
    # side, whichever world direction that resolved to). The camera backs
    # away along the SAME direction from the row's own centre to the pile's
    # centre, so this works regardless of which side the pile landed on.
    pile_c = center.get("pile_center")
    if pile_c:
        dx, dy = pile_c[0] - center["cx"], pile_c[1] - center["cy"]
        norm = math.hypot(dx, dy) or 1.0
        dx, dy = dx / norm, dy / norm
        back = max(8.0, span * 0.35)
        eye = (pile_c[0] + dx * back, pile_c[1] + dy * back, pile_c[2] + back * 0.55)
        _place_camera_lookat(eye, pile_c, fov=math.radians(55))
        fp = Path(out_dir) / "{0}_pile.png".format(tag)
        _render_to(fp, *res)
        tiles.append(fp)
    return tiles


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--grade", default="DG4", choices=list(aq.LADDER))
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", default="~/scorch_previews/aec_quake/")
    ap.add_argument("--samples", type=int, default=48)
    ap.add_argument("--cpu", action="store_true")
    ap.add_argument("--w", type=int, default=1280)
    ap.add_argument("--h", type=int, default=720)
    args = ap.parse_args()

    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = "aec5row_{0}_s{1}".format(args.grade, args.seed)
    usd_out = out_dir / "{0}.usd".format(tag)

    stats, dt, bounds = build_damaged_row(args.grade, args.seed, _ROW5, usd_out)
    print("[aec_quake_bench] {0}: {1} unit(s), {2} de-instanced, killed {3}, "
          "{4} debris chunk(s), {5} scar(s), {6} window unit(s) voided, "
          "pile={7}, damage authored in {8:.3f}s".format(
              args.grade, stats["n_units"], stats["deinstanced"],
              stats["killed"], stats["debris_n"], stats["scars"],
              stats["window_units_voided"],
              stats["pile"]["tier"] if stats["pile"] else None, dt))

    tiles = render_views(usd_out, out_dir, tag, bounds, res=(args.w, args.h),
                        samples=args.samples, cpu=args.cpu)
    print("[aec_quake_bench] wrote:")
    for t in tiles:
        print("  ", t)


if __name__ == "__main__":
    main()

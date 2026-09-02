#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["usd-core", "numpy", "bpy", "pillow", "vtk", "shapely", "scipy"]
# ///
"""debris_match_bench.py — an OFFLINE look check for `quake_flow._DEBRIS_LOOK`
(round 7): does a fractured wall's debris core actually change colour family
when the per-facade table says it should, and stay put when it says it
should not?

    uv run --python 3.13 --with usd-core --with numpy --with vtk --with bpy \\
        --with pillow --with shapely --with scipy \\
        python scene_gen/tools/debris_match_bench.py \\
        --out ~/scorch_previews/debris_match/

WHY THIS EXISTS
---------------
Live-scene review, on a `commercial_mid` DG3 pile's `merged_c_brick` chunks:
"look good but seem to be the wrong material and it doesn't match the rest
of the building." `test_quake_debris_materials.py` proves the FRACTION each
family draws is right (host-side, no render) but cannot say whether the
fix reads as a colour change from any distance — that needs a render, same
as `tools/tear_edge_bench.py` for a ragged edge and `tools/pillar_break_bench.
py` for a chipped pillar.

WHAT IT AUTHORS. Four CLOSED-BOX wall runs (`quake_flow._box`, three panels
each, 4 x 3 x 0.4 m) side by side along +X, one per (family, condition):

    "01"/before   "01"/after   "04"/before   "04"/after

"before" calls `quake_flow._break(..., family=None)` — the pre-round-7 call
shape every caller used, which resolves through `_DEBRIS_LOOK_URM_DEFAULT`
(the ORIGINAL flat 0.70/0.45 brick shares, numerically unchanged since round
1). "after" passes the panel's real family. Each panel has no bound cladding
texture at all (`damage.bound_texture` returns None for it), so
`_chunk_material` always draws from the core/chunk palette — the render
shows PURE debris-core colour, nothing hidden behind a cladding photo.

Family "01" (`apartment`/`apartment_long`/...) is the actual bug this round
fixes: `bake_probe`'s own baked atlas for `SM_MBuilding01_Facade_A`/
`FirstFloor_A` is grey-tan ashlar stone, no brick anywhere, so "before" and
"after" should look VISIBLY DIFFERENT here — brick-red versus stone-grey.
Family "04" (`commercial`/`commercial_mid`/...) is the CONTROL: `bake_probe`'s
atlas for `SM_MBuilding04_Facade_A/B` is genuine brick, so this table keeps
it brick-dominant and "before"/"after" should look the SAME here — proof the
fix did not disturb the one family the live-review complaint was actually
about.

MATERIAL FALLBACK FOR BLENDER. `damage._pbr` (what `quake_flow.materials()`/
`_c_look_at` build) only ever authors an MDL `outputs:mdl:surface` — correct
for Kit/RTX, invisible to `bpy.ops.wm.usd_import`, which resolves the
universal render context (`quake_rubble_usd._add_preview_fallback`'s own
docstring: "imports into Blender with no usable shader at all and renders
solid black"). This script calls that exact helper, flat (no texture), on
every material this bench actually uses, keyed on the same rgb/roughness
`quake_flow.materials()`/`_C_TEX` already define — so the colour this bench
shows IS the colour `_t_core_mat`/`_chunk_material` chose, not a
reinterpretation of it.

WHAT THIS CANNOT SEE: the real `Brick_Wall_Worn`/kit-atlas TEXTURE detail
(coursing, weathering) — both "c_brick" and "stone" render as flat tinted
proxies here, same limitation `rubble_preview.py` documents for its own
mound materials. That needs a Kit/RTX render.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import sys
from pathlib import Path

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
if _SG not in sys.path:
    sys.path.insert(0, _SG)

import numpy as np                                          # noqa: E402

from disaster import quake_flow as qf                      # noqa: E402
from disaster import quake_rubble_usd as qru                # noqa: E402

from pxr import Sdf, Usd, UsdGeom, UsdShade                 # noqa: E402

SEED = 7
PANEL_W, PANEL_H, PANEL_D = 4.0, 3.0, 0.4
N_PANELS = 3
PANEL_GAP = 0.5
CLUSTER_GAP = 10.0
BREAK_N = 12

CONDITIONS = ("before", "after")
FAMILIES = ("01", "04")     # "01": the actual fix (stone). "04": the control
                            # (commercial_mid's own family — must not move).

# rgb/roughness for every material this bench can draw, straight out of
# `quake_flow.materials()`'s `flat` dict and `_C_TEX["brick"]` — see that
# module for the numbers' own derivation. Kept here rather than introspected
# off the stage so this bench works even if `_add_preview_fallback` cannot
# find a `diffuse_color_constant` (e.g. a material with no MDL shader at
# all, which should never happen but must not crash the render either).
_KNOWN = {
    "stone": ((0.30, 0.27, 0.23), 0.95),
    "mortar": ((0.24, 0.225, 0.205), 1.0),
    "dark_concrete": ((0.12, 0.115, 0.105), 1.0),
    "plaster": ((0.32, 0.30, 0.265), 1.0),
    "plaster_dusty": ((0.155, 0.145, 0.130), 1.0),
    "c_brick": ((0.48, 0.40, 0.36), 0.92),
}


def _cluster_label(family, cond):
    return "fam{0}_{1}".format(family, cond)


def build_stage(out_usd, seed=SEED):
    stage = Usd.Stage.CreateNew(str(out_usd))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    parent = "/World/cell"
    UsdGeom.Xform.Define(stage, Sdf.Path(parent))

    mats = qf.materials(stage, parent)
    cache = {}
    rng = random.Random(seed)
    nrng = np.random.default_rng(seed + 1)   # `fracture._seeds` needs a
                                              # numpy Generator, not `random.Random`

    cluster_centres = {}
    x = 0.0
    n_frags = {}
    for family in FAMILIES:
        for cond in CONDITIONS:
            label = _cluster_label(family, cond)
            cx0 = x
            frag_paths = []
            for i in range(N_PANELS):
                px = x + i * (PANEL_W + PANEL_GAP)
                path = "{0}/panel_{1}_{2}".format(parent, label, i)
                qf._box(stage, path, px, 0.0, PANEL_H / 2.0,
                       PANEL_W, PANEL_D, PANEL_H)
                el = {"p": {"prim_path": path},
                     "ref": (px, 0.0, PANEL_H / 2.0),
                     "out": (0.0, -1.0, 0.0), "role": "wall"}
                fam_arg = None if cond == "before" else family
                st, lo = qf._break(
                    stage, parent, el, label, BREAK_N, rng, nrng, mats,
                    cache, "urm", inner_p=1.0, core=True,
                    family=fam_arg)
                frag_paths += list(st) + list(lo)
            n_frags[label] = len(frag_paths)
            cluster_centres[label] = (
                (cx0 + (x + (N_PANELS - 1) * (PANEL_W + PANEL_GAP) + PANEL_W)) / 2.0)
            x += (N_PANELS - 1) * (PANEL_W + PANEL_GAP) + PANEL_W + CLUSTER_GAP

    # a plain ground plate, for shadow/scale only, spanning every cluster —
    # authored AFTER the loop so its extent is exact, never bound to a
    # QuakeLooks material so the fallback scan below never touches it
    total_w = x - CLUSTER_GAP
    qf._box(stage, parent + "/ground", total_w / 2.0, 0.0, -0.02,
           total_w + 16.0, 20.0, 0.04)

    # Blender-visible fallback on every QuakeLooks material this bench
    # actually used (flat only — see the module docstring's "WHAT THIS
    # CANNOT SEE"). `UsdShade.Material.Get` is the same "is this really a
    # material prim" check `_c_look_at`/`materials()` use themselves.
    scope = stage.GetPrimAtPath(parent + "/QuakeLooks")
    n_fallback = 0
    if scope and scope.IsValid():
        for prim in Usd.PrimRange(scope):
            if not UsdShade.Material.Get(stage, prim.GetPath()):
                continue
            name = prim.GetName()
            if name not in _KNOWN:
                continue
            rgb, rough = _KNOWN[name]
            qru._add_preview_fallback(stage, str(prim.GetPath()), rgb, rough)
            n_fallback += 1

    stage.GetRootLayer().Save()
    return cluster_centres, n_frags, n_fallback


# --------------------------------------------------------------------------- #
# bpy render (idiom copied from tools/tear_edge_bench.py / tools/
# rubble_preview.py / tools/pillar_break_bench.py)
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


def _add_lighting(bg=0.30):
    import bpy
    world = bpy.data.worlds.new("vw")
    world.use_nodes = True
    n = world.node_tree.nodes["Background"]
    n.inputs[0].default_value = (bg, bg, bg * 1.05, 1.0)
    n.inputs[1].default_value = 0.6
    bpy.context.scene.world = world

    def sun(name, rot, energy, angle=4.0):
        d = bpy.data.lights.new(name, "SUN")
        d.energy = energy
        d.angle = math.radians(angle)
        o = bpy.data.objects.new(name, d)
        o.rotation_euler = rot
        bpy.context.collection.objects.link(o)

    # HALVED from `tear_edge_bench.py`'s own values (round 7): those render
    # MATTE GREY (`_matte_grey`, a flat 0.55 albedo stand-in) and never
    # exercise this path with REAL material colour — at that energy, the
    # ~0.30-linear stone/mortar/brick albedos this bench actually cares
    # about (plus GI bounce off the ground plate) blew out to near-white and
    # hid the very hue difference this bench exists to show.
    sun("key", (math.radians(58), 0, math.radians(-35)), 2.0)
    sun("fill", (math.radians(70), 0, math.radians(150)), 0.55)


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


def _dim_ground():
    """The `ground` plate imports with NO bound material at all (it is never
    bound to a QuakeLooks material, on purpose — see `build_stage`), so
    Blender gives it its own bright default. Left alone, that plate bounces
    enough light back up (Cycles GI) to wash out the very albedo difference
    this bench exists to show. A plain, deliberately darker Principled stand-
    in fixes the bounce without touching a single panel's real material."""
    import bpy
    obj = bpy.data.objects.get("ground")
    if obj is None or obj.type != "MESH":
        return
    m = bpy.data.materials.new("ground_mat")
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (0.16, 0.155, 0.145, 1.0)
    b.inputs["Roughness"].default_value = 0.95
    obj.data.materials.clear()
    obj.data.materials.append(m)


def render_view(usd_path, out_png, location, target, fov, res, samples, cpu):
    import bpy
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=str(usd_path), import_materials=True)
    dev = _setup_engine(samples, use_gpu=not cpu)
    print("[debris-match] rendering {0} on {1}".format(
        os.path.basename(out_png), dev))
    _add_lighting()
    _dim_ground()
    _place_camera_lookat(location, target, fov_deg=fov)
    _render_to(out_png, *res)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.expanduser(
        "~/scorch_previews/debris_match/"))
    ap.add_argument("--res", default="1280x800")
    ap.add_argument("--samples", type=int, default=64)
    ap.add_argument("--cpu", action="store_true")
    args = ap.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    w, h = (int(v) for v in args.res.lower().split("x"))

    usd_path = out_dir / "debris_match_bench.usda"
    centres, n_frags, n_fallback = build_stage(usd_path)
    print("[debris-match] wrote {0}".format(usd_path))
    print("[debris-match] fragments per cluster: {0}".format(n_frags))
    print("[debris-match] preview-fallback materials authored: {0}".format(
        n_fallback))

    x_all = list(centres.values())
    x_lo, x_hi = min(x_all) - 6.0, max(x_all) + 6.0
    x_mid = (x_lo + x_hi) / 2.0

    def _framing(span, fov_deg, elev_frac=0.22, margin=1.18):
        """Camera (dist back, elevation) so `span` metres of the Y=0 plane,
        centred under the look-at target, fits the frame at `fov_deg` with
        `margin` headroom — computed, not eyeballed (the first cut of this
        bench guessed a fixed multiple of `span` for both distance AND
        elevation independently and only fit 2 of 4 clusters)."""
        back = (span / 2.0) / math.tan(math.radians(fov_deg / 2.0)) * margin
        return back, back * elev_frac

    # 1) OVERVIEW — all four clusters in one oblique shot: fam01 before/after
    #    on the left (must look DIFFERENT), fam04 before/after on the right
    #    (must look the SAME).
    span = x_hi - x_lo
    fov = 60.0
    back, elev = _framing(span, fov)
    render_view(usd_path, out_dir / "overview_all_four_obl.png",
               (x_mid, -back, elev), (x_mid, 0.0, 1.2),
               fov=fov, res=(w, h), samples=args.samples, cpu=args.cpu)

    # 2) CLOSE-UP — family 01 before vs after (the actual fix: brick -> stone)
    fam01_x = (centres[_cluster_label("01", "before")]
              + centres[_cluster_label("01", "after")]) / 2.0
    fam01_span = abs(centres[_cluster_label("01", "after")]
                     - centres[_cluster_label("01", "before")]) + 10.0
    fov2 = 50.0
    back2, elev2 = _framing(fam01_span, fov2)
    render_view(usd_path, out_dir / "family01_before_after_close.png",
               (fam01_x, -back2, elev2), (fam01_x, 0.0, 1.1), fov=fov2,
               res=(w, h), samples=args.samples, cpu=args.cpu)

    # 3) CLOSE-UP — family 04 (commercial_mid) before vs after (the control:
    #    must be visually indistinguishable — brick stays brick)
    fam04_x = (centres[_cluster_label("04", "before")]
              + centres[_cluster_label("04", "after")]) / 2.0
    fam04_span = abs(centres[_cluster_label("04", "after")]
                     - centres[_cluster_label("04", "before")]) + 10.0
    back3, elev3 = _framing(fam04_span, fov2)
    render_view(usd_path, out_dir / "family04_commercial_mid_control_close.png",
               (fam04_x, -back3, elev3), (fam04_x, 0.0, 1.1), fov=fov2,
               res=(w, h), samples=args.samples, cpu=args.cpu)

    print("[debris-match] done -> {0}".format(out_dir))


if __name__ == "__main__":
    main()

#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["usd-core", "numpy", "bpy", "pillow"]
# ///
"""fissure_cutout_preview.py — BEFORE/AFTER bench renders for the fissure
OPENING rework (live scene review, 2026-09-02):

    "The earthquake fissure looks completely wrong. Look at how we do pool
    in suburban. We cut out a part of the ground and have a lower 'water'
    there. Do similar for the fissure but a cracked irregular opening and a
    floor below. It doesn't have to be too wide." (user)

    uv run --python 3.13 --with usd-core --with numpy --with bpy --with pillow \
        python scene_gen/tools/fissure_cutout_preview.py \
        --out ~/scorch_previews/fissure_cutout/

ROUND 2 (coordinator review of the first cut): "a raft-class pave plate
fills the frame and the crack's recess is only a sliver ... cannot verify
'a cracked irregular opening with a floor below'". Three things changed
here, ALL BENCH-ONLY (no production default touched):

  1. `pave=False` on the direct `_c_fissure_trace` call for every "after"
     shot — the cracked-asphalt band is `_c_fissure_pave`'s own populaton,
     unrelated to this rework, and its "raft" size class can legally drop a
     metres-wide chip right beside a sub-metre-wide crack. Disabling it for
     the proof shots is the bench-only equivalent of
     `EQ_FISSURE_PAVE_DENSITY=0`, done via the kwarg `_c_fissure_trace`
     already exposes rather than an env var, so this file's own import of
     `quake_flow` is the only thing that needs touching.
  2. FIXED geometry instead of `_c_fissures`' corner-driven random walk: a
     straight-ish ~4 m segment centred on the origin, heading known in
     advance, so the camera work below is arithmetic instead of guesswork
     off an as-authored bbox.
  3. The floor mesh gets an EXPLICIT near-black `UsdPreviewSurface`
     (`_dark_floor_material`), bypassing the `_C_TEX["pit_floor"]` fallback
     chain entirely for this bench: `soil`/`silt`/`pit_floor`'s real tints
     are all close to neutral grey by design (real subsoil, not paint), so
     the OmniPBR-fallback path that is faithful to production materials
     reads as one undifferentiated grey in flat Cycles preview lighting —
     correct for that path's own purpose, useless for "is there a floor
     here" at a glance.

Renders, all under `--out`:
  fissure_before.png / fissure_after.png / fissure_before_after.png — the
    same filenames the first cut used, now BOTH pave-free and on the fixed
    segment, kept for a stable before/after diff across rounds.
  after_along_crack.png — low-oblique, looking ALONG the crack's own run,
    so both walls AND the floor between them are in view at once.
  after_top.png — ~70 deg (near-nadir) elevation, so the opening's own
    jagged outline against the cut ground reads as a shape, not a line.

Also PRINTS the measured opening width at 10 stations and the floor depth
below grade (`_measure_opening`), read back off the authored wall mesh's own
points — not re-derived from the rng, so it is what actually got authored.

The bpy idioms (`_setup_engine`/`_add_lighting`/`_place_camera`/
`_place_camera_lookat`/`_render_to`/`_contact_sheet`/`_pile_bbox`) are
`tools/rubble_preview.py`'s own, imported rather than copied — the same
precedent `fissure_raft_preview.py` (the previous round's bench) set.

THE GROUND PLANE MATTERS HERE, unlike the previous round's bench: the whole
point of this rework is that the ground itself gains a hole
(`quake_flow._c_cut_ground_openings`, the swimming-pool mechanism
`suburb_scene.apply_ground` already uses, replicated for
`scene_generator.apply_ground_planes`'s city ground), so both stages author
a real `_make_plane_mesh`-shaped rectangle under `<parent>/ground/
asphalt_base` (see `_author_ground_plane` below — copied from
`scene_generator._make_plane_mesh`'s exact construction rather than
importing that module, which is heavy and Kit-oriented) and set
`ctx["ground_root"]` to it. The BEFORE stage gets the identical plane so the
two renders differ only in what the fissure code did to the world, not in
what backdrop it did it on: pre-round-7 code never touches `ground_root` at
all (it did not exist yet), so that plane stays a flat, uncut rectangle —
which is itself part of the "before" picture: a mound sitting on top of
unbroken pavement.

PREVIEW-ONLY MATERIAL FIX, same one `rubble_preview.py`/
`fissure_raft_preview.py` needed: every ground material here
(`quake_flow._c_look`/`_C_TEX`) is `damage._pbr` — MDL-only, invisible to
`bpy.ops.wm.usd_import` — so this script adds a flat `UsdPreviewSurface`
fallback (`quake_rubble_usd._add_preview_fallback`) on every material it
actually used before rendering, and binds a plain `UsdPreviewSurface`
directly to the ground plane rather than routing it through `damage._pbr`
at all. Kit/Isaac is unaffected; this only changes what the offline Blender
preview can show.
"""
from __future__ import annotations

import math
import os
import random
import sys
from pathlib import Path

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, _SCENE_GEN)

from disaster import quake_flow as qf                   # noqa: E402
from disaster import quake_rubble_usd as qru             # noqa: E402
from tools import rubble_preview as rp                   # noqa: E402

SEED = 7
PARENT = "/World/Bldg"
GROUND_HALF_M = 40.0
# THE FIXED SEGMENT every "after" shot below draws: ~4 m (the coordinator's
# "3-5 m crack segment"), running along +X from x=-2 so the origin sits at
# its midpoint and every camera below can be aimed with plain trig instead
# of reading back an as-authored bbox first.
SEG_X0, SEG_Y0 = -2.0, 0.0
SEG_HEADING_DEG = 0.0
SEG_LENGTH_M = 4.0
# The wide end of the REAL production range (`qf.C_FISSURE_W`), not a bench
# invention: this is what a fissure looks like on the days its own
# per-station width draw lands near the top of that range, and starting
# there (rather than at the range's mean) gives the per-station noise more
# room to swing down to the narrow end too — see the printed measurements.
SEG_WIDTH_M = qf.C_FISSURE_W[1]


# --------------------------------------------------------------------------- #
# THE OLD CONSTRUCTION, reproduced verbatim (not checked out from git) so the
# "before" render uses byte-for-byte the pre-round-7 geometry: ONE continuous
# earthen ridge straddling the whole crack width, swept via
# `scour_relief.geometry({"kind": "ridge", ...})` — `_c_fissure_trace` as it
# stood immediately before this round. No pave call in it (round 5's
# cracked-asphalt band is unchanged by this rework either way), which is
# also why the "after" shots below pass `pave=False`: the comparison is
# fair only with the SAME population on both sides of it.
# --------------------------------------------------------------------------- #
def _old_c_fissure_trace(ctx, x0, y0, heading0_deg, length_m, width_m, mat,
                         tag="fissure", z0=0.0, step_m=1.2):
    rng = ctx["rng"]
    heading = math.radians(heading0_deg)
    n = max(2, int(length_m / step_m))
    stations, widths = [(x0, y0)], [width_m]
    x, y, w = x0, y0, width_m
    for _i in range(n):
        heading += math.radians(rng.uniform(-16, 16))
        nx, ny = x + math.cos(heading) * step_m, y + math.sin(heading) * step_m
        if not qf._c_ok(ctx, nx, ny):
            break
        x, y = nx, ny
        w *= rng.uniform(0.72, 0.98)
        stations.append((x, y))
        widths.append(w)
    if len(stations) < 2:
        return None
    n_st = len(stations)
    hn = qf._c_noise(rng, freqs=(0.6, 1.7), amps=(0.6, 0.3))
    h0 = width_m * rng.uniform(0.45, 0.85)
    ridge_st = []
    for k, (px, py) in enumerate(stations):
        t = k / float(n_st - 1)
        taper = math.sin(math.pi * t) ** 0.6
        hh = max(0.01, h0 * taper * (0.72 + 0.30 * hn(k * step_m)))
        ww = max(0.05, widths[k] * 0.5)
        ridge_st.append((px, py, hh, hh, ww, ww, 0.0))
    spec = {"kind": "ridge", "cls": "soil", "z": float(z0), "base": float(z0),
           "x": x0, "y": y0, "stations": ridge_st}
    from disaster import scour_relief as sr
    pts3, faces = sr.geometry(spec)
    return qf._c_geom_mesh(ctx, tag + "_mound", pts3, faces, mat)


# --------------------------------------------------------------------------- #
def _author_ground_plane(stage, path, half_m, uv_scale_m=4.0):
    """One quad `UsdGeom.Mesh` spanning `(-half_m, -half_m)-(half_m, half_m)`
    at z=0, byte-for-byte the shape `scene_generator._make_plane_mesh`
    authors (4 points, 1 face, a "st" vertex UV primvar) — copied rather
    than imported (that module pulls in the whole Kit-oriented generator for
    one rectangle). A plain `UsdPreviewSurface`, not `damage._pbr`: bpy's USD
    importer reads this natively, no fallback material needed for the
    ground plane itself (only for the crack's own MDL looks, below)."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    x0 = y0 = -half_m
    x1 = y1 = half_m
    pts = Vt.Vec3fArray([Gf.Vec3f(x0, y0, 0.0), Gf.Vec3f(x1, y0, 0.0),
                        Gf.Vec3f(x1, y1, 0.0), Gf.Vec3f(x0, y1, 0.0)])
    u_max = (x1 - x0) / uv_scale_m
    v_max = (y1 - y0) / uv_scale_m
    uvs = Vt.Vec2fArray([Gf.Vec2f(0.0, 0.0), Gf.Vec2f(u_max, 0.0),
                        Gf.Vec2f(u_max, v_max), Gf.Vec2f(0.0, v_max)])
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * 4))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.30, 0.30, 0.29)]))
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    pv.Set(uvs)

    mat_path = path + "_mat"
    mat = UsdShade.Material.Define(stage, Sdf.Path(mat_path))
    shader = UsdShade.Shader.Define(stage, Sdf.Path(mat_path + "/Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    # LIGHTER than the previous round's 0.16: the floor mesh below is now
    # the darkest thing in frame on purpose (`_dark_floor_material`), and
    # the ground needs enough separation from it that the recess reads as a
    # hole in the ground rather than the ground and the hole being the same
    # tone.
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.30, 0.30, 0.29))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.92)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return mesh


def _fallback_all_c_tex(stage, parent):
    """`_add_preview_fallback` for every `_C_TEX` look (plain and mixture
    variants 0-2) that actually got authored under `parent/QuakeLooks` —
    same idiom `fissure_raft_preview._fallback_all_c_tex` established. Gives
    the WALL/LIP meshes (still `soil`/`silt`) a real, if muted, material;
    the floor's own fallback is overridden right after by
    `_dark_floor_material`, below."""
    from pxr import Sdf, UsdShade
    for key, (_rel, rgb, rough, _scale, _desat) in qf._C_TEX.items():
        suffixes = [key] + ["{0}_{1}".format(key, i) for i in range(3)]
        for suf in suffixes:
            look_path = "{0}/QuakeLooks/c_{1}".format(parent, suf)
            mat = UsdShade.Material.Get(stage, Sdf.Path(look_path))
            if mat and mat.GetPrim().IsValid():
                qru._add_preview_fallback(stage, look_path, rgb, rough)


def _dark_floor_material(stage, parent, authored):
    """BENCH-ONLY: bind an explicit near-black `UsdPreviewSurface` directly
    onto every `"_floor_"` mesh, bypassing `_C_TEX["pit_floor"]`'s own
    fallback entirely.

    `_C_TEX["pit_floor"]`'s real tint, (0.16, 0.15, 0.14), is a deliberately
    muted dark subsoil — correct for a textured PBR render, and read at
    round-7's own pixel-sampled render as within ~15 grey levels of
    everything else in a flat-lit Cycles preview (soil/silt/asphalt are ALL
    close to neutral grey by design). That is invisible in a proof shot
    whose only job is "is there a floor here, yes or no" — so the bench
    binds its own much darker stand-in instead of trying to relight the
    production material into legibility. Kit/Isaac never sees this file at
    all, so production is untouched."""
    from pxr import Gf, Sdf, UsdShade

    mat_path = "{0}/BenchLooks/floor_dark".format(parent)
    mat = UsdShade.Material.Define(stage, Sdf.Path(mat_path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(mat_path + "/Shader"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.02, 0.02, 0.02))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    n = 0
    for p in authored:
        if "_floor_" not in p:
            continue
        prim = stage.GetPrimAtPath(p)
        if prim.IsValid():
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
            n += 1
    return n


def _measure_opening(stage, wall_path, z0, n_report=10):
    """Read the AUTHORED wall mesh back (not the rng) and print the opening
    TOP WIDTH (`|lt - rt|` per station) and FLOOR DEPTH below grade
    (`z0 - mean(rb.z, lb.z)`) at `n_report` evenly spaced stations, plus the
    min/max over every station — what the coordinator asked to see
    verified: an irregular width spread, not a constant-width slot, and a
    real depth below `z0`."""
    from pxr import UsdGeom

    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(wall_path))
    pts = mesh.GetPointsAttr().Get()
    n_st = len(pts) // 4
    rows = []
    for k in range(n_st):
        rt, rb, lb, lt = (pts[4 * k], pts[4 * k + 1],
                         pts[4 * k + 2], pts[4 * k + 3])
        width = math.hypot(lt[0] - rt[0], lt[1] - rt[1])
        depth = z0 - 0.5 * (rb[2] + lb[2])
        rows.append((k, width, depth))

    if n_st > 1:
        idxs = sorted({round(i * (n_st - 1) / (n_report - 1))
                      for i in range(n_report)})
    else:
        idxs = [0]
    print("[fissure_cutout_preview] opening measurements "
          "({0} of {1} stations):".format(len(idxs), n_st))
    print("  {0:>4s} {1:>10s} {2:>10s}".format("stn", "width_m", "depth_m"))
    for k in idxs:
        _, w, d = rows[k]
        print("  {0:4d} {1:10.3f} {2:10.3f}".format(k, w, d))
    widths = [w for _k, w, _d in rows]
    depths = [d for _k, _w, d in rows]
    print("[fissure_cutout_preview] width  min/max = {0:.3f} / {1:.3f} m "
          "(target band: roughly 0.4-1.5 m)".format(min(widths), max(widths)))
    print("[fissure_cutout_preview] depth  min/max = {0:.3f} / {1:.3f} m "
          "below grade (FISSURE_DEPTH_M = {2:.2f})".format(
              min(depths), max(depths), qf.FISSURE_DEPTH_M))
    return rows


def _print_extended_measurements():
    """A second, longer (12 m) trace, authored but never rendered, purely
    so the printed station table has enough stations to be a real
    "10 stations" sample. `SEG_LENGTH_M` (4 m, inside the coordinator's
    "3-5 m crack segment" ask) only yields 3-5 stations at the production
    default `step_m=1.2` — correct for the tight proof-shot framing above,
    too few to report against the "10 stations" ask honestly. 12 m sits
    inside `qf.C_FISSURE_M`'s own real length range (3.5-10.5 m at default
    `FISSURE_SCALE`, i.e. this is a plausible fissure, not a bench outlier)
    and yields 10-11 stations at that same default step."""
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.DefinePrim(PARENT, "Xform")
    mats = qf.materials(stage, PARENT)
    ctx = qf._c_ctx(stage, PARENT, mats, random.Random(SEED), tag="measure")
    mat = qf._c_ground_look(ctx, 0.0, 0.0, None,
                            lambda: "soil" if ctx["rng"].random() < 0.7
                            else "silt")
    qf._c_fissure_trace(ctx, 0.0, 0.0, 0.0, 12.0, SEG_WIDTH_M, mat,
                        z0=0.0, pave=False)
    walls = [p for p in ctx["authored"] if "_wall_" in p]
    if not walls:
        print("[fissure_cutout_preview] WARNING: extended trace authored "
              "no wall mesh, nothing to measure")
        return
    print("[fissure_cutout_preview] extended (12 m) trace, for a full "
          "10-station sample (not one of the rendered shots):")
    _measure_opening(stage, walls[0], z0=0.0)


def _build_stage(before, pave=False):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.DefinePrim(PARENT, "Xform")
    ground_root = PARENT + "/ground"
    UsdGeom.Scope.Define(stage, ground_root)
    _author_ground_plane(stage, ground_root + "/asphalt_base", GROUND_HALF_M)

    mats = qf.materials(stage, PARENT)
    ctx = qf._c_ctx(stage, PARENT, mats, random.Random(SEED),
                    tag="before" if before else "after")
    ctx["ground_root"] = ground_root
    ctx["ground_ssf"] = 1.0

    width_m = SEG_WIDTH_M
    mat = qf._c_ground_look(ctx, SEG_X0, SEG_Y0, None,
                            lambda: "soil" if ctx["rng"].random() < 0.7
                            else "silt")
    if before:
        _old_c_fissure_trace(ctx, SEG_X0, SEG_Y0, SEG_HEADING_DEG,
                             SEG_LENGTH_M, width_m, mat, z0=0.0)
    else:
        qf._c_fissure_trace(ctx, SEG_X0, SEG_Y0, SEG_HEADING_DEG,
                           SEG_LENGTH_M, width_m, mat, z0=0.0, pave=pave)

    _fallback_all_c_tex(stage, PARENT)
    n_dark = _dark_floor_material(stage, PARENT, ctx["authored"])
    if not before:
        print("[fissure_cutout_preview] darkened {0} floor mesh(es)".format(n_dark))

    # FRAME THE OPENING ITSELF, not every authored prim: `_c_fissure_pave`'s
    # size ladder can legally drop a metres-wide "raft"-class chip right
    # beside a sub-metre-wide crack (unrelated to this rework, unchanged by
    # it), and an authored-bbox-of-everything framing lets that chip
    # dominate the shot the opening is supposed to be in. `rp._pile_bbox`
    # (this file's own bbox helper, reused rather than re-derived) on the
    # wall+floor alone (the mound alone for "before", which has neither)
    # gives the crack's real centre/diagonal after all the rng jitter.
    frame_paths = [p for p in ctx["authored"]
                  if "_wall_" in p or "_floor_" in p]
    if not frame_paths:
        frame_paths = [p for p in ctx["authored"] if "_mound_" in p]
    center, diag = rp._pile_bbox(stage, frame_paths)

    wall_paths = [p for p in ctx["authored"] if "_wall_" in p]
    return stage, center, diag, wall_paths


def _render(stage, out_dir, name, place_camera_fn):
    usd_path = Path(out_dir) / "{0}.usda".format(name)
    stage.GetRootLayer().Export(str(usd_path))
    print("[fissure_cutout_preview] wrote {0}".format(usd_path))

    import bpy
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=str(usd_path))
    # CPU: the GPU on this host is frequently near-full from another
    # concurrent session (Isaac Sim or a sibling render), and this is a
    # small, low-sample offline preview where reliability matters more than
    # speed — a CUDA OOM here silently aborts the whole run.
    rp._setup_engine(64, use_gpu=False)
    rp._add_lighting(0.5)
    place_camera_fn()
    fp = Path(out_dir) / "{0}.png".format(name)
    rp._render_to(fp, 1280, 720)
    print("[fissure_cutout_preview] rendered {0}".format(fp))
    return fp


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    # EXPLICIT eye/target pairs everywhere below, not `_place_camera`'s
    # orbit-around-a-centre form: the first cut of this bench used that
    # form with the orbit CENTRE placed at a NEGATIVE z (inside the pit, so
    # the read looked right on paper — "aim below grade to look into the
    # hole") and a low elevation, and `center.z + dist*sin(el)` came out
    # negative too — the eye itself ended up buried in the ground plane's
    # backface, which renders as a black frame with nothing recognisable in
    # it. Choosing the eye's own height directly sidesteps that class of
    # bug entirely.

    # --- before/after, same fixed segment, PAVE DISABLED on both --------- #
    stage_b, _center_b, _diag_b, _walls_b = _build_stage(before=True)
    fp_before = _render(
        stage_b, out_dir, "fissure_before",
        lambda: rp._place_camera_lookat((-3.5, -2.0, 1.6), (0.0, 0.0, 0.1),
                                        fov=math.radians(45)))

    stage_a, _center_a, _diag_a, walls_a = _build_stage(before=False, pave=False)
    fp_after = _render(
        stage_a, out_dir, "fissure_after",
        lambda: rp._place_camera_lookat((-3.5, -2.0, 1.6), (0.0, 0.0, -0.6),
                                        fov=math.radians(45)))

    sheet = out_dir / "fissure_before_after.png"
    rp._contact_sheet([fp_before, fp_after], sheet,
                      "fissure: before (mound) | after (opening, no pave)")
    print("[fissure_cutout_preview] wrote {0}".format(sheet))

    # --- measurements, off the SAME "after" stage's wall mesh ------------- #
    if walls_a:
        _measure_opening(stage_a, walls_a[0], z0=0.0)
    else:
        print("[fissure_cutout_preview] WARNING: no wall mesh authored, "
              "nothing to measure")
    # ... and again on a SEPARATE, longer (12 m — inside production's own
    # `C_FISSURE_M` length range) trace: the tight 3-5 m segment the proof
    # shots use only has 3-5 stations at the production default
    # `step_m=1.2`, too few to be a real "10 stations" sample. Not rendered.
    _print_extended_measurements()

    # --- proof shot 1: LOW-OBLIQUE, ALONG the crack's own run ------------- #
    # Eye just short of the segment's -X end, at a crouched 1 m above
    # grade; target well past the midpoint AND well below grade, so the
    # look direction actually points down the trench's length and into it
    # at once — both walls (near one lit, far one shadowed) and the floor
    # between them end up in frame together.
    fp_along = _render(
        stage_a, out_dir, "after_along_crack",
        lambda: rp._place_camera_lookat(
            (SEG_X0 - 1.2, SEG_Y0, 1.0),
            (SEG_X0 + SEG_LENGTH_M * 0.75, SEG_Y0, -1.0),
            fov=math.radians(55)))

    # --- proof shot 2: ~70 deg (near top-down), the JAGGED OUTLINE -------- #
    # Eye nearly overhead (a small x offset only, for enough perspective
    # that this reads as a 3-D shape rather than a flat plan), looking down
    # at a point partway into the opening.
    fp_top = _render(
        stage_a, out_dir, "after_top",
        lambda: rp._place_camera_lookat((1.5, 0.0, 2.7), (0.0, 0.0, -0.8),
                                        fov=math.radians(48)))

    print("[fissure_cutout_preview] wrote {0}".format(fp_along))
    print("[fissure_cutout_preview] wrote {0}".format(fp_top))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["usd-core", "numpy", "bpy", "pillow"]
# ///
"""fissure_raft_preview.py — BEFORE/AFTER bench renders for the "fissure"
rework (live scene review, 2026-08-31/09-01):

  (a) "the fissure looks weird — use the same soil/mud material we have and
      use in suburban tornado for its path and moulds of dirt. Use the same
      code in fact to create this longer 'mould of dirt' aka fissure"
  (b) "the cracked asphalt along the fissure looks weird — make it
      irregular cracked shapes, not just rectangles"
  (c) "the fissure itself should be thicker"
  (d) on `quake_tilt/raft_t3_1`: "I like this plate ... however it's too
      straight and rectangular. We need irregular. We can do a smaller
      version of this as the fissure's cracked asphalt."

    uv run --python 3.13 --with usd-core --with numpy --with bpy --with pillow \
        python scene_gen/tools/fissure_raft_preview.py \
        --out ~/scorch_previews/fissure_rework/

Renders ONE fissure segment and ONE tilt raft, each BEFORE (the exact
pre-rework construction, reproduced verbatim below — not read from git
history, no checkout involved) and AFTER (today's `disaster.quake_flow`),
same seed, same camera. The bpy idioms (`_setup_engine`/`_add_lighting`/
`_place_camera`/`_render_to`/`_contact_sheet`) are `tools/rubble_preview.py`'s
own, imported rather than copied.

PREVIEW-ONLY MATERIAL FIX, same one `rubble_preview.py` needed: every ground
material here (`quake_flow._c_look`/`_C_TEX`) is `damage._pbr` — MDL-only,
invisible to `bpy.ops.wm.usd_import` — so this script adds a flat
`UsdPreviewSurface` fallback (`quake_rubble_usd._add_preview_fallback`) on
every material it actually used before rendering. Kit/Isaac is unaffected;
this only changes what the OFFLINE Blender preview can show.
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

M = {"cx": 0.0, "cy": 0.0, "yaw": 0.0, "W": 24.0, "D": 16.0,
     "z0": 0.0, "top": 12.0, "levels": [0.0], "module": 4.0}
SEED = 7
PARENT = "/World/Bldg"


# --------------------------------------------------------------------------- #
# THE OLD CONSTRUCTION, reproduced verbatim (not checked out from git) so the
# "before" render uses byte-for-byte the pre-rework geometry: a chain of
# thin, flat, near-identical `_box` rectangles for the crack, and an
# unchipped 8-corner box for the raft. Every helper called here
# (`_box`/`_to_world`/`_c_ground_look`/`_c_ok`/`_uid`/`_c_look`) is
# infrastructure this rework did not touch.
# --------------------------------------------------------------------------- #
def _old_c_fissures(ctx, m, corners, n_each, length, width, tag="fissure"):
    rng = ctx["rng"]
    made = []
    W, D = m["W"], m["D"]
    pts = {"SW": (-W / 2.0, -D / 2.0), "SE": (W / 2.0, -D / 2.0),
           "NE": (W / 2.0, D / 2.0), "NW": (-W / 2.0, D / 2.0)}
    for cname in (corners or list(pts)):
        lx, ly = pts[cname]
        for _q in range(rng.randrange(n_each[0], n_each[1] + 1)):
            L = rng.uniform(*length)
            heading = (math.atan2(ly, lx) + rng.uniform(-0.7, 0.7)
                      + math.radians(m["yaw"]))
            wx, wy = qf._to_world(m, lx * 1.02, ly * 1.02)
            w = rng.uniform(*width)
            mat = qf._c_ground_look(
                ctx, wx, wy, None,
                lambda: "soil" if rng.random() < 0.7 else "silt")
            step = 1.2
            for _i in range(max(2, int(L / step))):
                heading += math.radians(rng.uniform(-16, 16))
                nx2 = wx + math.cos(heading) * step
                ny2 = wy + math.sin(heading) * step
                mx, my = (wx + nx2) / 2.0, (wy + ny2) / 2.0
                if not qf._c_ok(ctx, mx, my):
                    break
                path = "{0}/{1}_{2}_{3}".format(
                    ctx["parent"], tag, ctx["tag"], qf._uid(ctx))
                qf._box(ctx["stage"], path, mx, my, m["z0"] + 0.02,
                        step * 1.12, w * rng.uniform(0.7, 1.3), 0.06,
                        math.degrees(heading), mat)
                made.append(path)
                wx, wy = nx2, ny2
                w *= rng.uniform(0.72, 0.98)
    ctx["authored"] += made
    return made


def _old_raft(ctx, m, tag="raft", dress=True):
    """`_raft`, minus the `_chip_authored` line this rework added — same
    dressing clods (`_c_raft_dress`, unchanged by this rework) so the ONLY
    difference the render shows is the plate's own edges, not a missing
    population of earth clods."""
    path = "{0}/{1}_{2}_{3}".format(
        ctx["parent"], tag, ctx["tag"], qf._uid(ctx))
    qf._box(ctx["stage"], path, m["cx"], m["cy"],
            m["z0"] - 0.06 - qf.RAFT_T / 2.0, m["W"] + 1.2, m["D"] + 1.2,
            qf.RAFT_T, m["yaw"], qf._c_look(ctx, "raft"))
    ctx["authored"].append(path)
    ctx["static_extra"].append(path)
    if dress:
        ctx["c_carry"] = list(ctx.get("c_carry", [])) + qf._c_raft_dress(ctx, m)
    return path


# --------------------------------------------------------------------------- #
def _fallback_all_c_tex(stage, parent):
    """`_add_preview_fallback` for every `_C_TEX` look (plain and mixture
    variants 0-2) that actually got authored under `parent/QuakeLooks` —
    covers whichever of soil/silt/pave/asph/raft this run's rng happened to
    draw, without hand-listing them."""
    from pxr import Sdf, UsdShade
    for key, (_rel, rgb, rough, _scale, _desat) in qf._C_TEX.items():
        suffixes = [key] + ["{0}_{1}".format(key, i) for i in range(3)]
        for suf in suffixes:
            path = "{0}/QuakeLooks/c_{1}".format(parent, suf)
            mat = UsdShade.Material.Get(stage, Sdf.Path(path))
            if mat and mat.GetPrim().IsValid():
                qru._add_preview_fallback(stage, path, rgb, rough)


def _build_stage(kind, before):
    """One in-memory stage with a ground plane + either the fissure or the
    raft, before or after. Returns `(stage, cam)`, where `cam` is either
    `("orbit", center, dist, az_deg, el_deg)` (`rp._place_camera`'s own
    args) or `("lookat", location, target)` (`rp._place_camera_lookat`'s) —
    the fissure trace is framed by its own as-authored bbox (`rp._pile_bbox`,
    reused), the raft by a fixed point on its lifted edge, which a bbox-of-
    the-whole-25x17m-plate framing cannot resolve."""
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.DefinePrim(PARENT, "Xform")
    mats = qf.materials(stage, PARENT)
    ctx = qf._c_ctx(stage, PARENT, mats, random.Random(SEED),
                    tag="before" if before else "after")

    if kind == "fissure":
        if before:
            # the LITERAL old default width — `0.06/0.22 * FISSURE_SCALE`,
            # with no `FISSURE_WIDTH_SCALE` factor at all — so "thicker" is
            # actually visible in the comparison rather than hidden by
            # rendering "before" with today's width knob.
            old_width = (0.06 * qf.FISSURE_SCALE, 0.22 * qf.FISSURE_SCALE)
            _old_c_fissures(ctx, M, ("SW",), (1, 1), qf.C_FISSURE_M, old_width)
        else:
            qf._c_fissures(ctx, M, corners=("SW",), n_each=(1, 1),
                          length=qf.C_FISSURE_M, width=qf.C_FISSURE_W)
        rp._add_ground_and_stub(stage, PARENT, M["W"], M["D"], M["top"])
        _fallback_all_c_tex(stage, PARENT)
        # FRAME THE TRACE ITSELF, not a hand-picked point — `ctx["authored"]`
        # is every box/mound/pave-plate this call made (the "after" branch's
        # cracked-asphalt band included, so it is actually IN the shot), and
        # `rp._pile_bbox` (this file's own bbox helper, reused rather than
        # re-derived) gives its real world centre/diagonal after all the
        # rng jitter, so "before" and "after" frame correctly regardless of
        # which corner/heading this seed happened to draw.
        center, diag = rp._pile_bbox(stage, ctx["authored"])
        dist = max(1.8 * diag, 6.0)
        cam = ("orbit", center, dist, 35.0, 28.0)
    else:
        if before:
            _old_raft(ctx, M)
        else:
            qf._raft(ctx, M)
        rp._add_ground_and_stub(stage, PARENT, M["W"], M["D"], M["top"])
        _fallback_all_c_tex(stage, PARENT)
        # LIFT THE RAFT CLEAR of the ground plane for the render — a raft at
        # grade is invisible from above by design (`_raft`'s own docstring);
        # the in-scene tilt that actually exposes it lives in
        # `_c_ground_response`, not reproduced here. A plain vertical lift
        # (no rotation) is enough to separate it from the ground plane
        # without also foreshortening the very corner this render exists to
        # show — earlier cuts of this bench tilted it and framed a fixed
        # point on the "lifted edge", which put the camera skimming the
        # plate's own 25 x 17 m top face (a flat ceiling filling the frame)
        # for any tilt small enough to keep the far corner from swinging out
        # of frame.
        authored = ctx["authored"]
        raft_path = authored[0] if authored else None
        if raft_path:
            qf._transform_prims(stage, [raft_path], qf._translate(0.0, 0.0, 0.5))
        # FRAME ONE CORNER, exactly the way the fissure trace is framed
        # above: `rp._pile_bbox` on the raft alone gives its real as-authored
        # extent (the chip pass can grow it slightly, `max_grow`), and a
        # close orbit on one of ITS OWN corners — not the footprint centre —
        # is what a photo of "is this edge straight or broken" is actually
        # taken from.
        if raft_path:
            _center, diag = rp._pile_bbox(stage, [raft_path])
            lo = stage.GetPrimAtPath(raft_path)
            from pxr import UsdGeom as _UsdGeom
            bc = _UsdGeom.BBoxCache(0, [_UsdGeom.Tokens.default_])
            rng = bc.ComputeWorldBound(lo).ComputeAlignedRange()
            hi_pt = tuple(rng.GetMax())
            corner = (hi_pt[0], hi_pt[1], rng.GetMin()[2])
        else:
            corner, diag = (M["W"] / 2.0, M["D"] / 2.0, 0.0), 6.0
        cam = ("orbit", corner, max(0.22 * diag, 5.0), 40.0, 30.0)
    return stage, cam


def _render_one(kind, before, out_dir):
    stage, cam = _build_stage(kind, before)
    tag = "{0}_{1}".format(kind, "before" if before else "after")
    usd_path = Path(out_dir) / "{0}.usda".format(tag)
    stage.GetRootLayer().Export(str(usd_path))
    print("[fissure_raft_preview] wrote {0}".format(usd_path))

    import bpy
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=str(usd_path))
    rp._setup_engine(64, use_gpu=True)
    rp._add_lighting(0.55)
    if cam[0] == "orbit":
        _, center, dist, az, el = cam
        rp._place_camera(center, dist, az, el)
    else:
        _, cam_loc, target = cam
        rp._place_camera_lookat(cam_loc, target, fov=math.radians(48))
    fp = Path(out_dir) / "{0}.png".format(tag)
    rp._render_to(fp, 1280, 720)
    print("[fissure_raft_preview] rendered {0}".format(fp))
    return fp


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    tiles = {}
    for kind in ("fissure", "raft"):
        for before in (True, False):
            tiles[(kind, before)] = _render_one(kind, before, out_dir)

    for kind in ("fissure", "raft"):
        sheet = out_dir / "{0}_before_after.png".format(kind)
        rp._contact_sheet([tiles[(kind, True)], tiles[(kind, False)]],
                          sheet, "{0}: before | after".format(kind))
        print("[fissure_raft_preview] wrote {0}".format(sheet))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

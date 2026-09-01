#!/usr/bin/env -S uv run --python 3.13 --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["usd-core", "numpy", "bpy", "pillow"]
# ///
"""stranded_bands_preview.py — an OFFLINE before/after look check for the
"sky-grid" fix (`quake_sliced._repair_stranded_shell_sliced` /
`_sweep_airborne_shell_sliced`).

    uv run --python 3.13 --with usd-core --with numpy --with bpy --with pillow \
        python scene_gen/tools/stranded_bands_preview.py \
        --out ~/scorch_previews/stranded_bands/

WHY A HAND-BUILT FACADE, NOT `plan_damage` ON `fake_sliced_building`. A
first version drove the real `out_of_plane` recipe on the test fixture's own
synthetic building and measured what `_repair_stranded_shell_sliced` /
`_sweep_airborne_shell_sliced` actually did to it. It never found a real
orphan to repair: `fake_sliced_building` (and the real slicer) always backs
an outer wall/parapet piece with a full-footprint "core" box one storey
down, and a `core` cell is NEVER part of a region-removal recipe's own grid
(`_Grid.runs`/`_Grid.corners` only index side-bearing cells — see
`quake_sliced._Grid.__init__`), so the core survives untouched under every
toothed gap and `_deck_support_z` — correctly — finds it. That is real and
worth knowing (a demo this literal would have UNDER-sold the bug), but it
means a stand-in built from solid boxes cannot reproduce what a genuinely
thin, single-sided clipped shell does — the exact "walls are not decks"
observation the safety net's own docstring makes, and the reason
`roof_and_parapet._ensure_roof` exists at all (a ring()'d piece routinely
comes back with NO up-facing triangle whatsoever).

So this script instead builds ONE THIN FACADE PLANE directly — 6 bay columns
x 6 storeys of narrow wall panels, no core, no interior anything, matching
what an aerial photo actually shows (an outer skin) — and TOOTHES it BY HAND
into the exact shape `_apply_region`'s own per-cell, per-storey independent
`keep_pier` draw produces on a wide multi-storey region (see the section
note above `_repair_stranded_shell_sliced` in `quake_sliced.py`): four
columns lose a contiguous storey band, and in each one exactly ONE storey
survives the toothing draw, floating over the gap — two at a shallow height
(one storey of real fall) and two at a tall one (three storeys), so both the
DROP and DELETE branches get exercised in one frame, arranged as the
"regular grid/columns" the bug report's own photos show. Two untouched bookend
columns give the eye an intact reference.

BEFORE: that state, nothing else — the toothing exactly as `_apply_region`
would leave it, no carrier, no sweep, no physics (the confirmed defect).
AFTER: the SAME state, plus `_repair_stranded_shell_sliced` and
`_sweep_airborne_shell_sliced`. Both get saved as on-disk USDs and rendered
from two oblique corners named the way the bug report's own screenshots
were (`se_obl.png` / `nw_obl.png`).
"""
from __future__ import annotations

import argparse
import math
import os
import random
import sys
import time
from pathlib import Path

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, _SCENE_GEN)

from disaster import quake_flow as qf                    # noqa: E402
from disaster import quake_sliced as qs                   # noqa: E402

# The facade: 6 bay columns x 6 storeys, side "S" of a single "main" mass.
# Columns 0 and 5 are untouched bookends. Within 1-4, a contiguous band of
# storeys [1..4] is toothed: each of those four columns keeps exactly ONE
# storey alive in that band (the toothing draw's survivor) and loses the
# rest — cols 1/3 keep the LOW survivor (storey 2: one storey of real fall
# once storey 1 is gone under it -> DROP), cols 2/4 keep the HIGH one
# (storey 4: three storeys of fall -> DELETE). Storeys 0 and 5 are always
# alive (ground floor support, and a parapet/roofline cap).
N_COLS, N_STOREYS = 6, 6
BAY_PITCH_M, STOREY_PITCH_M = 4.0, 3.0
PANEL_W, PANEL_T, PANEL_H = 3.2, 0.4, 2.4     # a THIN facade panel — no core
SURVIVOR_STOREY = {1: 2, 2: 4, 3: 2, 4: 4}     # column -> the one storey kept
                                               # alive inside the toothed band
TOOTHED_COLS = tuple(sorted(SURVIVOR_STOREY))
TOOTHED_BAND = range(1, 5)                     # storeys 1..4 are the removed band

PARENT = "/World/cell"
SCOPE = PARENT + "/pieces"


def _panel_path(col, storey):
    return "{0}/wall_S_{1}_{2:02d}".format(SCOPE, storey, col)


def _is_dead(col, storey):
    if col not in TOOTHED_COLS:
        return False                     # bookend column: fully alive
    if storey not in TOOTHED_BAND:
        return False                     # storey 0 (ground) / 5 (cap): alive
    return storey != SURVIVOR_STOREY[col]


def _author_facade(stage):
    """One thin box mesh per (column, storey) cell — the stand-in for a
    real clipped `wall`/`parapet` piece: real wall thickness (`PANEL_T`), NO
    full-footprint backing anywhere, so the ONLY thing that can ever answer
    a support query for one of these panels is another panel (or the
    ground) — exactly the "walls are not decks" shape the safety net's own
    docstring describes."""
    from pxr import Gf, Sdf, UsdGeom, Vt

    for col in range(N_COLS):
        cx = (col - (N_COLS - 1) / 2.0) * BAY_PITCH_M
        for storey in range(N_STOREYS):
            z0 = storey * STOREY_PITCH_M
            path = _panel_path(col, storey)
            hx, hy = PANEL_W / 2.0, PANEL_T / 2.0
            me = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
            pts = [Gf.Vec3f(cx - hx, -hy, z0), Gf.Vec3f(cx + hx, -hy, z0),
                  Gf.Vec3f(cx + hx, hy, z0), Gf.Vec3f(cx - hx, hy, z0),
                  Gf.Vec3f(cx - hx, -hy, z0 + PANEL_H), Gf.Vec3f(cx + hx, -hy, z0 + PANEL_H),
                  Gf.Vec3f(cx + hx, hy, z0 + PANEL_H), Gf.Vec3f(cx - hx, hy, z0 + PANEL_H)]
            faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
                    (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
            me.CreatePointsAttr(Vt.Vec3fArray(pts))
            me.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
            me.CreateFaceVertexIndicesAttr(
                Vt.IntArray([i for f in faces for i in f]))
            tan = (0.78, 0.66, 0.46)
            UsdGeom.Gprim(me).CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*tan)]))


def _info_for_facade():
    """A minimal `info` dict — just enough shape for `_shell_column_index` /
    `_deck_support_candidates` (mass levels, and one element per panel with
    `_side`/`_storey`/`_bay`/`prim_path`)."""
    top = N_STOREYS * STOREY_PITCH_M
    levels = [k * STOREY_PITCH_M for k in range(N_STOREYS)]
    elements = []
    for col in range(N_COLS):
        for storey in range(N_STOREYS):
            path = _panel_path(col, storey)
            dead = _is_dead(col, storey)
            elements.append({
                "mass": "main", "role": "wall",
                "p": {"_role": "wall", "_side": "S", "_storey": storey,
                     "_bay": col, "prim_path": path},
                "dead": dead})
    return {"masses": {"main": {"top": float(top), "levels": levels,
                               "z0": 0.0}},
           "elements": elements}


def build_state(out_usd, repair=False):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateNew(str(out_usd))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, PARENT)
    _author_facade(stage)

    info = _info_for_facade()
    removed = []
    for e in info["elements"]:
        if e["dead"]:
            path = e["p"]["prim_path"]
            qf._deactivate(stage, path)
            removed.append(path)
    plan = {"displaced": {}, "removed": removed}

    n_dropped = n_deleted = n_swept = 0
    if repair:
        ctx = {"stage": stage, "parent": PARENT, "info": info,
              "rng": random.Random(0), "static_extra": [], "notes": []}
        n_dropped, n_deleted = qs._repair_stranded_shell_sliced(stage, ctx, plan)
        n_swept = qs._sweep_airborne_shell_sliced(stage, ctx, plan, verbose=False)

    stage.GetRootLayer().Save()
    print("[stranded_bands_preview] {0}: {1} panel(s) toothed away, "
         "repair={2} (dropped={3} deleted={4} swept={5})".format(
             out_usd.name, len(removed), repair, n_dropped, n_deleted,
             n_swept))
    return info


# ---------------------------------------------------------------------------
# RENDER — copied idiom from `rubble_preview.py` (owned by another agent this
# round; copied rather than imported, same reasoning that file's own
# docstring gives for not importing `render_usd.py`).
# ---------------------------------------------------------------------------
def _setup_engine(samples=96):
    import bpy
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = samples
    scene.cycles.use_denoising = True
    scene.cycles.device = "CPU"


def _add_lighting(bg=0.6):
    import bpy
    world = bpy.data.worlds.new("vw")
    world.use_nodes = True
    bg_node = world.node_tree.nodes["Background"]
    bg_node.inputs[0].default_value = (bg, bg, bg, 1.0)
    bg_node.inputs[1].default_value = 1.0
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


def _place_camera(center, dist, az_deg, el_deg, fov=math.radians(45)):
    import bpy
    from mathutils import Vector
    cam_data = bpy.data.cameras.new("cam")
    cam_data.angle = fov
    cam_data.clip_start = 0.05
    cam_data.clip_end = dist * 20.0
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.collection.objects.link(cam)
    az, el = math.radians(az_deg), math.radians(el_deg)
    direction = Vector((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az),
                       math.sin(el)))
    cam.location = Vector(center) + direction * dist
    cam.rotation_euler = (Vector(center) - cam.location).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam
    return cam


def _render_to(path, w=1280, h=800):
    import bpy
    scene = bpy.context.scene
    scene.render.resolution_x = w
    scene.render.resolution_y = h
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)


def render(usd_path, out_prefix, center, dist):
    import bpy
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=str(usd_path))
    _setup_engine()
    _add_lighting()
    for tag, az, el in (("se_obl", -50.0, 18.0), ("nw_obl", 130.0, 18.0)):
        _place_camera(center, dist, az, el)
        out = "{0}_{1}.png".format(out_prefix, tag)
        _render_to(out)
        print("  wrote", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.expanduser(
        "~/scorch_previews/stranded_bands/"))
    args = ap.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    before_usd = out_dir / "before.usd"
    after_usd = out_dir / "after.usd"
    if before_usd.exists():
        os.remove(before_usd)
    if after_usd.exists():
        os.remove(after_usd)

    build_state(before_usd, repair=False)
    build_state(after_usd, repair=True)

    center = (0.0, 0.0, N_STOREYS * STOREY_PITCH_M / 2.0)
    dist = 40.0
    t0 = time.time()
    render(before_usd, str(out_dir / "before"), center, dist)
    render(after_usd, str(out_dir / "after"), center, dist)
    print("[stranded_bands_preview] rendered in {0:.1f}s -> {1}".format(
        time.time() - t0, out_dir))


if __name__ == "__main__":
    main()

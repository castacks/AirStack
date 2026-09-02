#!/usr/bin/env python
"""qc_break_preview.py — an OFFLINE look check for the QUAKE break-aways.

    uv run --python 3.13 --with usd-core --with numpy --with bpy --with pillow \
        python scene_gen/tools/qc_break_preview.py A.usd B.usd \
        --out ~/scorch_previews/ragged_breaks/ --side S

Renders one or more self-contained USDs (written by
`tools/qc_edge_probe.py --export`, which is `disaster.bake.export_object` with
no settle) through headless Blender/Cycles: a wide oblique on the elevation
that failed and a close look at the break itself, so the two states of the
same building can be put side by side.

The Blender idioms (`_setup_engine` / `_add_lighting` / `_place_camera` /
`_render_to`) are `tools/rubble_preview.py`'s, COPIED rather than imported —
that file belongs to another workstream and is not to be edited from here.
Materials are not expected to resolve (the kit's textures are Nucleus assets);
this is a check on the SHAPE of the break, which is what the review is about.
"""
import argparse
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

SIDE_DIR = {"S": (0.0, -1.0), "N": (0.0, 1.0), "E": (1.0, 0.0),
            "W": (-1.0, 0.0)}


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
                device = "GPU/{0}".format(backend)
                break
    if device == "CPU":
        scene.cycles.device = "CPU"
    return device


def _add_lighting(bg=0.75):
    import bpy
    world = bpy.data.worlds.new("vw")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (bg, bg, bg, 1.0)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.9
    bpy.context.scene.world = world
    for name, rot, energy in (("key", (math.radians(52), 0, math.radians(35)), 4.0),
                              ("fill", (math.radians(68), 0, math.radians(215)), 1.4)):
        d = bpy.data.lights.new(name, "SUN")
        d.energy = energy
        d.angle = math.radians(3)
        o = bpy.data.objects.new(name, d)
        o.rotation_euler = rot
        bpy.context.collection.objects.link(o)


def _place(location, target, fov_deg=40.0, clip_end=2000.0):
    import bpy
    from mathutils import Vector
    cd = bpy.data.cameras.new("cam")
    cd.angle = math.radians(fov_deg)
    cd.clip_start = 0.05
    cd.clip_end = clip_end
    cam = bpy.data.objects.new("cam", cd)
    bpy.context.collection.objects.link(cam)
    cam.location = Vector(location)
    cam.rotation_euler = (Vector(target) - Vector(location)).to_track_quat(
        "-Z", "Y").to_euler()
    bpy.context.scene.camera = cam
    return cam


def _render_to(path, w, h):
    import bpy
    sc = bpy.context.scene
    sc.render.resolution_x, sc.render.resolution_y = w, h
    sc.render.resolution_percentage = 100
    sc.render.image_settings.file_format = "PNG"
    sc.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)


def _bounds():
    import bpy
    lo = [1e30] * 3
    hi = [-1e30] * 3
    for ob in bpy.context.scene.objects:
        if ob.type != "MESH":
            continue
        for c in ob.bound_box:
            w = ob.matrix_world @ __import__("mathutils").Vector(c)
            for i in range(3):
                lo[i] = min(lo[i], w[i])
                hi[i] = max(hi[i], w[i])
    return lo, hi


def render(usd, out_dir, tag, side, samples, res, cpu, at=0.20):
    import bpy
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=str(usd))
    print("[qc-preview] {0}: {1} object(s)".format(
        tag, len(bpy.context.scene.objects)))
    print("[qc-preview] device", _setup_engine(samples, use_gpu=not cpu))
    _add_lighting()
    lo, hi = _bounds()
    ctr = [(lo[i] + hi[i]) / 2.0 for i in range(3)]
    span = max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
    dx, dy = SIDE_DIR.get(side, SIDE_DIR["S"])
    made = []
    # 1. the whole elevation, obliquely, from the side that failed
    d = span * 1.35
    _place((ctr[0] + dx * d + dy * 0.35 * d, ctr[1] + dy * d - dx * 0.35 * d,
            hi[2] * 0.85 + 6.0), (ctr[0], ctr[1], ctr[2]), fov_deg=40.0)
    p = os.path.join(out_dir, "{0}_elev.png".format(tag))
    _render_to(p, *res)
    made.append(p)
    # 2. THE END OF THE BREAK, three-quarter on. The middle of the hole is
    # a dark interior whatever the lighting; what the review is about is the
    # OUTLINE — the staircase step at the end of the lost run and the wall
    # beside it — so aim a fifth of the way along the failed elevation, at the
    # failure line, and stand well off it.
    ax = (lo[0] + at * (hi[0] - lo[0])) if abs(dx) < 0.5 else ctr[0]
    ay = (lo[1] + at * (hi[1] - lo[1])) if abs(dy) < 0.5 else ctr[1]
    aim = (ax + dx * 0.5 * (hi[1] - lo[1] if abs(dy) > 0.5 else 0.0),
           ay + dy * 0.5 * (hi[1] - lo[1] if abs(dy) > 0.5 else 0.0),
           lo[2] + 0.72 * (hi[2] - lo[2]))
    d2 = max(14.0, span * 0.55)
    _place((aim[0] + dx * d2 - dy * 0.45 * d2,
            aim[1] + dy * d2 + dx * 0.45 * d2,
            aim[2] + 0.30 * d2), aim, fov_deg=45.0)
    p = os.path.join(out_dir, "{0}_break.png".format(tag))
    _render_to(p, *res)
    made.append(p)
    for q in made:
        print("[qc-preview] wrote", q)
    return made


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("usd", nargs="+")
    ap.add_argument("--out", default=os.path.expanduser(
        "~/scorch_previews/ragged_breaks"))
    ap.add_argument("--side", default="S")
    ap.add_argument("--samples", type=int, default=48)
    ap.add_argument("--res", default="1280x720")
    ap.add_argument("--cpu", action="store_true")
    ap.add_argument("--at", type=float, default=0.20,
                    help="where along the failed elevation the close view "
                         "aims, 0..1 from the low end")
    a = ap.parse_args()
    w, _, h = a.res.partition("x")
    os.makedirs(a.out, exist_ok=True)
    for u in a.usd:
        tag = os.path.splitext(os.path.basename(u))[0]
        render(u, a.out, tag, a.side, a.samples, (int(w), int(h)), a.cpu,
               at=a.at)


if __name__ == "__main__":
    main()

#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["bpy"]
# ///
"""site_scene_png.py — render a built site-plan stage, coloured by what things are.

    uv run --script scene_gen/tools/site_scene_png.py scene.usda -o shot.png \
        --azimuth 90 --elev 33

`render_usd.py` renders any USD from an orbit of angles and is the right tool
for checking that an ASSET survived conversion. This one exists for a different
question — does the traced site read correctly as a place — and answers it two
ways that one cannot:

* **It shoots from a bearing you choose**, so the render can be lined up with
  the oblique photograph the plan was authored against. Comparing a scene to a
  reference is not possible from an arbitrary orbit position.
* **It colours by what each prim IS**, taken from the prim NAME that
  `detail/site_features.py` and the ground pass already write —
  `building_07`, `rubble_22`, `road_41`. A stand-in scene is all untextured
  boxes; rendered in one grey, a container, a rubble field and a drill tower
  are indistinguishable and the picture cannot be checked against anything.

Runs in its own Python 3.13 env via `uv run --script`, like `render_usd.py`,
so it never touches the repo's own environment.
"""

import argparse
import math
import sys
from pathlib import Path

import bpy

# Kept in step with `detail/site_features.COLOUR` and the ground pass BY HAND:
# this script runs in a different interpreter and cannot import either. It is a
# preview, so drift shows up as an odd colour rather than as a wrong scene.
KIND_RGB = {
    "building": (0.80, 0.78, 0.74), "rubble": (0.62, 0.28, 0.24),
    "wreck": (0.72, 0.42, 0.18), "pit": (0.80, 0.70, 0.48),
    "vehicle": (0.20, 0.55, 0.75), "container": (0.85, 0.55, 0.15),
    "mast": (0.70, 0.30, 0.70), "debris": (0.45, 0.42, 0.40),
    "road": (0.16, 0.16, 0.17), "junction": (0.16, 0.16, 0.17), "grass": (0.36, 0.48, 0.26),
    "tree": (0.20, 0.34, 0.15), "sign": (0.70, 0.70, 0.70),
    # Zone ground, named `zone_<role>_<i>` by the ground pass. Longest key
    # wins in `kind_of`, so `zone_grass` is not shadowed by `grass`.
    "zone_parking": (0.19, 0.19, 0.20), "zone_pad": (0.68, 0.63, 0.54),
    "zone_staging": (0.60, 0.52, 0.41), "zone_rubble": (0.46, 0.42, 0.39),
    "zone_collapsed": (0.44, 0.39, 0.35), "zone_wooded": (0.17, 0.30, 0.13),
    "zone_grass": (0.33, 0.46, 0.22), "zone_water": (0.16, 0.28, 0.35),
    "xwalk": (0.85, 0.85, 0.85), "stopbar": (0.85, 0.85, 0.85),
    "dash": (0.75, 0.66, 0.22), "drive": (0.35, 0.33, 0.31),
}
FALLBACK = (0.55, 0.55, 0.55)


def kind_of(obj):
    """The kind, from this object's name or the nearest named ancestor.

    A feature prism is an Xform `building_07` with the mesh `box` beneath it,
    so the mesh's own name says nothing — walking up is what recovers it.
    """
    node = obj
    while node is not None:
        stem = node.name.split(".")[0]
        # LONGEST PREFIX WINS. `zone_grass_3` starts with both `zone_grass` and
        # nothing else, but `grass_3` starts with `grass` — matching in dict
        # order would colour half the zones as plain grass depending on
        # insertion order, which is a bug that looks like a design choice.
        hit = [k for k in KIND_RGB if stem.startswith(k)]
        if hit:
            return max(hit, key=len)
        node = node.parent
    return None


def paint(objs):
    mats, tally = {}, {}
    for o in objs:
        if o.type != "MESH":
            continue
        k = kind_of(o)
        tally[k or "?"] = tally.get(k or "?", 0) + 1
        rgb = KIND_RGB.get(k, FALLBACK)
        if k not in mats:
            m = bpy.data.materials.new(f"m_{k or 'other'}")
            m.use_nodes = True
            bsdf = m.node_tree.nodes["Principled BSDF"]
            bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
            bsdf.inputs["Roughness"].default_value = 0.85
            mats[k] = m
        o.data.materials.clear()
        o.data.materials.append(mats[k])
    return tally


def _height(o):
    import mathutils
    zs = [(o.matrix_world @ mathutils.Vector(c))[2] for c in o.bound_box]
    return max(zs) - min(zs)


def bounds(objs):
    lo = [1e18] * 3
    hi = [-1e18] * 3
    for o in objs:
        if o.type != "MESH":
            continue
        for c in o.bound_box:
            w = o.matrix_world @ __import__("mathutils").Vector(c)
            for i in range(3):
                lo[i] = min(lo[i], w[i])
                hi[i] = max(hi[i], w[i])
    return lo, hi


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("usd")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--azimuth", type=float, default=0.0,
                    help="where the CAMERA stands, deg CCW from east: 0 is due "
                         "east looking west (how the reference oblique was "
                         "shot), 90 is due north looking south")
    ap.add_argument("--elev", type=float, default=33.0)
    ap.add_argument("--res", type=int, default=1600)
    ap.add_argument("--samples", type=int, default=64)
    ap.add_argument("--sun-az", type=float, default=-70.0,
                    help="deg; the site's own sun is SSE, which is what puts "
                         "the shadows where the aerial has them")
    ap.add_argument("--sun-elev", type=float, default=42.0)
    ap.add_argument("--keep-materials", action="store_true",
                    help="render the USD's OWN materials instead of the "
                         "by-kind colours — the only way to see whether a "
                         "texture actually resolved and how it tiles")
    args = ap.parse_args()

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=str(Path(args.usd).resolve()))
    objs = [o for o in bpy.context.scene.objects]
    if args.keep_materials:
        n_mat = sum(1 for o in objs if o.type == "MESH" and o.data.materials)
        print(f"[site_scene_png] keeping the USD's own materials "
              f"({n_mat}/{sum(1 for o in objs if o.type == 'MESH')} meshes carry one)")
    else:
        tally = paint(objs)
        print("[site_scene_png] painted: "
              + "  ".join(f"{k}={v}" for k, v in sorted(tally.items(), key=lambda kv: -kv[1])))

    # DROP ABSURD IMPORTS. The AEC vegetation USDs declare their own
    # `metersPerUnit`, and Blender's importer applies it ON TOP of the scale the
    # generator already derived — so a correct 12 m oak arrives 100x too big and
    # a 20 m one reports as 1.7 km. In USD the placement is right (checked with
    # `UsdGeom.BBoxCache`); it is the import that is wrong. Left in, one of them
    # sets the framing for the whole picture.
    huge = [o for o in objs if o.type == "MESH" and _height(o) > 120.0]
    for o in huge:
        bpy.data.objects.remove(o, do_unlink=True)
    if huge:
        print(f"[site_scene_png] dropped {len(huge)} object(s) over 120 m — "
              f"a metersPerUnit double-conversion on import, not a scene fault")
    objs = [o for o in bpy.context.scene.objects]

    lo, hi = bounds(objs)
    ctr = [(lo[i] + hi[i]) / 2.0 for i in range(3)]
    span = max(hi[0] - lo[0], hi[1] - lo[1])
    print(f"[site_scene_png] extent {hi[0]-lo[0]:.0f} x {hi[1]-lo[1]:.0f} m, "
          f"tallest {hi[2]:.1f} m")

    import mathutils
    a, e = math.radians(args.azimuth), math.radians(args.elev)
    dist = span * 1.35
    cam_pos = mathutils.Vector((ctr[0] + dist * math.cos(a) * math.cos(e),
                                ctr[1] + dist * math.sin(a) * math.cos(e),
                                ctr[2] + dist * math.sin(e)))
    cam_data = bpy.data.cameras.new("cam")
    cam_data.lens = 50
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.scene.collection.objects.link(cam)
    cam.location = cam_pos
    direction = mathutils.Vector(ctr) - cam_pos
    cam.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam

    sa, se = math.radians(args.sun_az), math.radians(args.sun_elev)
    sun_data = bpy.data.lights.new("sun", type="SUN")
    sun_data.energy = 4.0
    sun_data.angle = math.radians(1.5)
    sun = bpy.data.objects.new("sun", sun_data)
    bpy.context.scene.collection.objects.link(sun)
    sun.rotation_euler = mathutils.Vector(
        (-math.cos(sa) * math.cos(se), -math.sin(sa) * math.cos(se),
         -math.sin(se))).to_track_quat("-Z", "Y").to_euler()

    world = bpy.data.worlds.new("w")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.55, 0.68, 0.85, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.5
    bpy.context.scene.world = world

    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    try:
        prefs = bpy.context.preferences.addons["cycles"].preferences
        prefs.compute_device_type = "OPTIX"
        prefs.get_devices()
        for d in prefs.devices:
            d.use = True
        sc.cycles.device = "GPU"
    except Exception as exc:
        print(f"[site_scene_png] GPU unavailable ({exc}); CPU")
    sc.cycles.samples = args.samples
    sc.render.resolution_x = args.res
    sc.render.resolution_y = int(args.res * 0.62)
    sc.render.filepath = str(Path(args.out).resolve())
    sc.render.image_settings.file_format = "PNG"
    bpy.ops.render.render(write_still=True)
    print(f"[site_scene_png] -> {args.out}")


if __name__ == "__main__":
    main()

"""Render oblique previews of a mesh file (Blender script, factory scene).

    blender -b --factory-startup --python preview_mesh.py -- in.{usd,ply} out_prefix [elev_deg] [grey]

Four views (from N, E, S, W) at `elev_deg` (default 35), Workbench, textures if
the file has them, else vertex colours; `grey` shows bare geometry. Writes <out_prefix>_{N,E,S,W}.png.
"""
import sys, math
import bpy
from mathutils import Vector

a = sys.argv[sys.argv.index("--") + 1:]
src, prefix = a[0], a[1]
elev = math.radians(float(a[2]) if len(a) > 2 else 35)
bpy.ops.wm.read_factory_settings(use_empty=True)
if src.endswith(".ply"): bpy.ops.wm.ply_import(filepath=src)
else: bpy.ops.wm.usd_import(filepath=src)
obs = [o for o in bpy.context.scene.objects if o.type == "MESH"]
pts = [o.matrix_world @ Vector(c) for o in obs for c in o.bound_box]
lo = Vector([min(p[i] for p in pts) for i in range(3)]); hi = Vector([max(p[i] for p in pts) for i in range(3)])
ctr, rad = (lo + hi) / 2, (hi - lo).length / 2
sc = bpy.context.scene
cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam")); sc.collection.objects.link(cam); sc.camera = cam
cam.data.lens = 35; cam.data.clip_end = 10000
sc.render.engine = "BLENDER_WORKBENCH"
sh = sc.display.shading; sh.light = "STUDIO"
has_tex = any(n.type == "TEX_IMAGE" for o in obs for s in o.material_slots if s.material and s.material.node_tree
              for n in s.material.node_tree.nodes)
sh.color_type = "SINGLE" if "grey" in a[3:] else "TEXTURE" if has_tex else "VERTEX"
sc.world = bpy.data.worlds.new("w"); sc.world.color = (0.25, 0.3, 0.4)
sc.render.resolution_x, sc.render.resolution_y = 1600, 1000
for name, az in (("N", 90), ("E", 0), ("S", 270), ("W", 180)):
    d = Vector((math.cos(math.radians(az)) * math.cos(elev), math.sin(math.radians(az)) * math.cos(elev), math.sin(elev)))
    cam.location = ctr + d * rad * 2.2
    cam.rotation_euler = (-d).to_track_quat("-Z", "Y").to_euler()
    sc.render.filepath = f"{prefix}_{name}.png"; bpy.ops.render.render(write_still=True)

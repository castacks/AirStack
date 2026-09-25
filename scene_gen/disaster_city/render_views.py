"""Render the Google tiles from given pinhole cameras (Blender script, used by recon_georef.py).

    blender -b data/blender_data/disaster_city.blend --python render_views.py -- views.json out_dir

views.json: [{name, R_wc (3x3, world <- camera, OpenCV axes: x right, y down, z forward),
C (camera centre, world), fx, fy, cx, cy, w, h}]. Flat Workbench shading with
the tiles' own textures, so a render looks like a (blurry, older) photo of the site.
"""
import json, sys
import bpy
from mathutils import Matrix, Vector

a = sys.argv[sys.argv.index("--") + 1:]
views, out = json.load(open(a[0])), a[1]
sc = bpy.context.scene
for o in bpy.data.objects: o.hide_render = o.name != "Google 3D Tiles.001"
cam = bpy.data.objects["Camera"]; sc.camera = cam
cam.data.type = "PERSP"; cam.data.clip_start = 0.1; cam.data.clip_end = 5000; cam.data.sensor_fit = "HORIZONTAL"
sc.render.engine = "BLENDER_WORKBENCH"
sc.display.shading.light = "FLAT"; sc.display.shading.color_type = "TEXTURE"
sc.render.resolution_percentage = 100
flip = Matrix(((1, 0, 0), (0, -1, 0), (0, 0, -1)))       # OpenCV camera axes -> Blender camera axes
for v in views:
    W, H = v["w"], v["h"]
    sc.render.resolution_x, sc.render.resolution_y = W, H
    cam.data.sensor_width = 36.0; cam.data.lens = v["fx"] * 36.0 / W
    sc.render.pixel_aspect_x, sc.render.pixel_aspect_y = 1.0, v["fx"] / v["fy"]
    cam.data.shift_x = (W / 2 - v["cx"]) / W; cam.data.shift_y = (v["cy"] - H / 2) / W
    Rw = Matrix(v["R_wc"]) @ flip
    cam.matrix_world = Matrix.Translation(Vector(v["C"])) @ Rw.to_4x4()
    sc.render.filepath = f"{out}/{v['name']}.png"; bpy.ops.render.render(write_still=True)
print(f"render_views: {len(views)} views -> {out}")

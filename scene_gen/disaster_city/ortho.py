"""Top-down orthographic render of the Google tiles -> data/recon/ortho_site.{png,json} (Blender script).

    blender -b data/blender_data/disaster_city.blend --python ortho.py -- 75 -317.5 560x595 4480 data/recon/ortho_site.png

Args: centre x, centre y, extent (m, or WxH m), pixels across, out.png. Flat Workbench shading,
textures only, so the colours are the tiles' own. The .json next to it maps a
pixel (u, v) to world (x0 + u*m, y1 - v*m); every script that reads the ortho
uses it. The defaults above give the 12.5 cm/px site ortho the build uses.
"""
import json, sys
import bpy

a = sys.argv[sys.argv.index("--") + 1:]
cx, cy = map(float, a[:2]); ex, ey = (float(v) for v in (a[2] + "x" + a[2]).split("x")[:2]); px, out = int(a[3]), a[4]
sc = bpy.context.scene
for o in bpy.data.objects: o.hide_render = o.name != "Google 3D Tiles.001"
cam = bpy.data.objects["Camera"]; cam.location = (cx, cy, 500); cam.rotation_euler = (0, 0, 0)
cam.data.type = "ORTHO"; cam.data.ortho_scale = max(ex, ey); cam.data.clip_end = 2000
sc.camera = cam; sc.render.engine = "BLENDER_WORKBENCH"
sc.display.shading.light = "FLAT"; sc.display.shading.color_type = "TEXTURE"
sc.render.resolution_x, sc.render.resolution_y = px, round(px * ey / ex); sc.render.resolution_percentage = 100
sc.render.filepath = out; bpy.ops.render.render(write_still=True)
json.dump({"x0": cx - ex / 2, "y1": cy + ey / 2, "m_per_px": ex / px,
           "note": "top-down Workbench render of Google 3D Tiles.001; pixel (u,v) -> world (x0+u*m, y1-v*m)"},
          open(out.rsplit(".", 1)[0] + ".json", "w"))

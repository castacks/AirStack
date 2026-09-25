"""Cut a square of the Google tile mesh out of the blend (Blender script).

    blender -b data/blender_data/disaster_city.blend --python tiles_crop.py -- cx cy half out.npz

Writes world-space verts (float32), triangle faces and per-face-corner UVs are
NOT kept -- this is geometry for alignment, not for display.
"""
import sys
import bpy, numpy as np

cx, cy, half = map(float, sys.argv[sys.argv.index("--") + 1:][:3])
out = sys.argv[-1]
o = bpy.data.objects["Google 3D Tiles.001"]
me = o.data
v = np.empty(len(me.vertices) * 3, np.float32); me.vertices.foreach_get("co", v); v = v.reshape(-1, 3)
M = np.array(o.matrix_world, np.float32)
v = v @ M[:3, :3].T + M[:3, 3]
me.calc_loop_triangles()
f = np.empty(len(me.loop_triangles) * 3, np.int64); me.loop_triangles.foreach_get("vertices", f); f = f.reshape(-1, 3)
keep_v = (abs(v[:, 0] - cx) < half) & (abs(v[:, 1] - cy) < half)
keep_f = keep_v[f].all(1)
idx = np.flatnonzero(keep_v); remap = -np.ones(len(v), np.int64); remap[idx] = np.arange(len(idx))
np.savez(out, verts=v[idx], faces=remap[f[keep_f]])
print(f"tiles_crop: {len(idx)} verts, {keep_f.sum()} faces -> {out}")

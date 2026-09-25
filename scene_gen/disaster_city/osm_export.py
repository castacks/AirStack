"""Export the blend's OSM layers (Blosm import) as world-space triangles (Blender script).

    blender -b data/blender_data/disaster_city.blend --python osm_export.py -- data/recon/osm.npz

Road curves carry their Blosm width profile, so the evaluated mesh is the road
surface. Saves one (n,3,2) XY triangle array per layer: road_<class>, path_footway,
water, forest, buildings.
"""
import sys
import bpy, numpy as np

out = sys.argv[-1]
dg = bpy.context.evaluated_depsgraph_get()
res = {}
for o in bpy.data.objects:
    if not o.name.startswith("map_2.osm_") or o.type not in ("MESH", "CURVE"): continue
    ev = o.evaluated_get(dg); me = ev.to_mesh()
    me.calc_loop_triangles()
    v = np.array([(o.matrix_world @ x.co)[:2] for x in me.vertices])
    t = np.array([tri.vertices[:] for tri in me.loop_triangles], int)
    key = o.name.replace("map_2.osm_", "").replace("roads_", "road_").replace("paths_", "path_")
    if len(t): res[key] = v[t].astype(np.float32)
    ev.to_mesh_clear()
    print(f"osm_export: {key}: {len(t)} triangles")
np.savez(out, **res)

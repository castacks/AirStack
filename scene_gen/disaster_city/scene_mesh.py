"""Every visible triangle of the people-free scene around the search area -> data/recon/survivors/scene_mesh.npz.

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh scene_mesh.py     # Kit USD: the Nucleus mirrors are kit crates

What survivor_candidates.py casts rays against: disaster_city_empty.usda's render meshes,
instances expanded, invisible colliders and guides left out. Each triangle carries the scene
piece it belongs to (`site`, `B01`, `lod2/B04`, `vehicles`, `trees`, ...), so a candidate knows
what it stands on.
"""
import numpy as np
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom
from _paths import R

# the operator's default search area (x -30..90, y -460..-360) and 15 m round it: the whole site
# is >2e9 vertices, the area a few million triangles
CROP = (-45.0, 105.0, -475.0, -345.0)
stage = Usd.Stage.Open(str(R / "disaster_city_empty.usda"))
cache = UsdGeom.XformCache(Usd.TimeCode.Default())
V, F, G, groups = [], [], [], []
n_v = 0
for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)):
    if not prim.IsA(UsdGeom.Mesh):
        continue
    img = UsdGeom.Imageable(prim)
    if img.ComputeVisibility() == UsdGeom.Tokens.invisible or img.ComputePurpose() in (UsdGeom.Tokens.guide, UsdGeom.Tokens.proxy):
        continue
    m = UsdGeom.Mesh(prim)
    pts, cnt, idx = m.GetPointsAttr().Get(), m.GetFaceVertexCountsAttr().Get(), m.GetFaceVertexIndicesAttr().Get()
    if not pts or not cnt or not idx:
        continue
    pts, cnt, idx = np.asarray(pts, np.float64), np.asarray(cnt), np.asarray(idx)
    W = np.asarray(cache.GetLocalToWorldTransform(prim), np.float64)          # row-vector convention
    world = pts @ W[:3, :3] + W[3, :3]
    inside = (world[:, 0] > CROP[0]) & (world[:, 0] < CROP[1]) & (world[:, 1] > CROP[2]) & (world[:, 1] < CROP[3])
    if not inside.any():
        continue
    starts = np.concatenate([[0], np.cumsum(cnt)[:-1]])
    tris = [np.stack([idx[s], idx[s + k], idx[s + k + 1]], -1)                # fan-triangulate each polygon
            for k in range(1, cnt.max() - 1) for s in [starts[cnt > k + 1]]]
    if not tris:
        continue
    tris = np.concatenate(tris)
    tris = tris[inside[tris].any(1)]
    used, tris = np.unique(tris, return_inverse=True)
    tris = tris.reshape(-1, 3)
    parts = str(prim.GetPath()).split("/")
    group = "/".join(parts[2:4]) if parts[2] == "lod2" else parts[2]
    if group not in groups:
        groups.append(group)
    V.append(world[used].astype(np.float32)); F.append((tris + n_v).astype(np.int64)); G.append(np.full(len(tris), groups.index(group), np.int16))
    n_v += len(used)
out = R / "survivors" / "scene_mesh.npz"
out.parent.mkdir(exist_ok=True)
np.savez_compressed(out, vertices=np.concatenate(V), triangles=np.concatenate(F), group=np.concatenate(G), groups=np.array(groups))
print(f"{sum(len(f) for f in F):,} triangles in {len(groups)} pieces -> {out}")
app.close()

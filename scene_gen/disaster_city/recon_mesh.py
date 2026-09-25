"""Aligned dense cloud -> watertight 2.5D heightfield mesh of one feature -> USD.

    ~/.venvs/recon/bin/python recon_mesh.py data/recon/rubble_west R01 data/recon/tiles_R01.npz [--radius 24] [--res 0.2]

Poisson on the cloud gave a patchy shell (the far sides of slabs are never seen,
and a half-open surface is useless for collision). A pile is effectively 2.5D
from a drone's point of view, so: per `res` cell, height = 90th percentile of
the points in it, colour = their mean; cells with no points take the tile
mesh's height and the Google ortho's colour. Cells inside any other building's
LOD1 footprint (data/recon/buildings_lod1.yaml, + --margin) are dropped, and anything
over --max-h above ground is capped -- those get modelled separately. The outer 3 m is tapered onto data/recon/dtm.npz. Writes <dir>/<ID>.ply and <dir>/<ID>.usd (world coords,
Z-up metres, semantic class from the label's kind).
"""
import argparse, json
from pathlib import Path
import cv2, numpy as np, open3d as o3d, yaml
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Vt

KIND_CLASS = {"rubble": "rubble", "building": "building", "vehicle": "vehicle", "structure": "building"}
ap = argparse.ArgumentParser(); ap.add_argument("dir"); ap.add_argument("id"); ap.add_argument("tiles")
ap.add_argument("--radius", type=float, default=24); ap.add_argument("--res", type=float, default=0.2)
ap.add_argument("--margin", type=float, default=0.3, help="m kept clear round other buildings")
ap.add_argument("--max-h", type=float, default=5.5, help="cells higher than this above ground belong to something else")
a = ap.parse_args(); d = Path(a.dir)
from _paths import R as RECON, LABELS   # R is taken (grid res / rotation)
lab = next(l for l in yaml.safe_load(open(LABELS))["labels"] if l["id"] == a.id)
cx, cy = lab["at"]; R, r = a.res, a.radius

pcd = o3d.io.read_point_cloud(str(d / "dense/fused.ply"))
pcd.transform(np.array(json.load(open(d / "to_world.json"))["recon_to_world"]))
P, C = np.asarray(pcd.points), np.asarray(pcd.colors)

n = int(2 * r / R)
xs = cx - r + (np.arange(n) + 0.5) * R; ys = cy + r - (np.arange(n) + 0.5) * R
X, Y = np.meshgrid(xs, ys)
iu = ((P[:, 0] - (cx - r)) / R).astype(int); iv = (((cy + r) - P[:, 1]) / R).astype(int)
k = (iu >= 0) & (iv >= 0) & (iu < n) & (iv < n); cell = iv[k] * n + iu[k]
order = np.lexsort((P[k, 2], cell)); cell, z, col = cell[order], P[k, 2][order], C[k][order]
starts = np.r_[0, np.flatnonzero(np.diff(cell)) + 1]; counts = np.diff(np.r_[starts, len(cell)])
Z = np.full(n * n, np.nan); RGB = np.zeros((n * n, 3))
keep = counts >= 3                                           # a lone point is noise
Z[cell[starts[keep]]] = z[starts[keep] + (0.9 * (counts[keep] - 1)).astype(int)]
RGB[cell[starts[keep]]] = np.add.reduceat(col, starts)[keep] / counts[keep, None]
Z, RGB = Z.reshape(n, n), RGB.reshape(n, n, 3)
Z = np.where(np.isfinite(Z), cv2.medianBlur(np.nan_to_num(Z, nan=-1e3).astype(np.float32), 3), np.nan)   # de-spike
Z[Z < -1e2] = np.nan
have = np.isfinite(Z)

# holes: tile mesh height + Google ortho colour
t = np.load(a.tiles); scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(o3d.core.Tensor(t["verts"].astype(np.float32)), o3d.core.Tensor(t["faces"].astype(np.uint32)))
rays = np.stack([X, Y, np.full_like(X, 500), 0 * X, 0 * X, -np.ones_like(X)], -1).astype(np.float32)
TZ = 500 - scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
geo = json.load(open(RECON / "ortho_site.json")); ortho = cv2.imread(str(RECON / "ortho_site.png"))
ou = np.clip(((X - geo["x0"]) / geo["m_per_px"]).astype(int), 0, ortho.shape[1] - 1)
ov = np.clip(((geo["y1"] - Y) / geo["m_per_px"]).astype(int), 0, ortho.shape[0] - 1)
ground = np.nanpercentile(TZ, 10)
tall = have & (Z > ground + a.max_h)                          # e.g. a building's frame standing in the pile
have &= ~tall; TZ = np.minimum(TZ, ground + a.max_h)
O = ortho[ov, ou, ::-1] / 255.0
# the drone footage is brighter and bluer than Google's -- match per-channel mean/std to the ortho
mu_r, sd_r, mu_o, sd_o = RGB[have].mean(0), RGB[have].std(0), O[have].mean(0), O[have].std(0)
RGB = np.clip((RGB - mu_r) / np.maximum(sd_r, 1e-3) * sd_o + mu_o, 0, 1)
# holes near the reconstruction are filled FROM it (inpaint) -- the tiles there can be stale
# (B01's old wing roof); only holes > 4 m from any reconstructed cell fall back to the tiles
near = cv2.dilate(have.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * int(4 / R) + 1,) * 2)).astype(bool)
lo_, hi_ = np.nanmin(Z[have]), np.nanmax(Z[have])
z8 = np.round(np.nan_to_num((Z - lo_) / (hi_ - lo_)) * 65535).astype(np.uint16)
fill = (~have).astype(np.uint8)
Zi = cv2.inpaint((z8 >> 8).astype(np.uint8), fill, 5, cv2.INPAINT_TELEA).astype(float) / 255 * (hi_ - lo_) + lo_
Ci = cv2.inpaint((np.nan_to_num(RGB) * 255).astype(np.uint8), fill, 5, cv2.INPAINT_TELEA) / 255.0
Z = np.where(have, Z, np.where(near, Zi, TZ)); RGB = np.where(have[..., None], RGB, np.where(near[..., None], Ci, O))
# despike: a cell more than 0.8 m off its 5x5 median is noise at a drone's scale
med = cv2.medianBlur(Z.astype(np.float32), 5)
Z = np.where(np.abs(Z - med) > 0.8, med, Z)
print(f"{have[np.hypot(X - cx, Y - cy) < r].mean():.0%} of the disc from the reconstruction, rest from the tiles")
# taper the outer 3 m onto the bare earth (data/recon/dtm.npz from ground.py) so the pile meets the ground
dt = np.load(RECON / "dtm.npz")
G = dt["dtm"][np.clip(((float(dt["y1"]) - Y) / float(dt["res"])).astype(int), 0, dt["dtm"].shape[0] - 1),
              np.clip(((X - float(dt["x0"])) / float(dt["res"])).astype(int), 0, dt["dtm"].shape[1] - 1)]
wt = np.clip((r - np.hypot(X - cx, Y - cy)) / 3.0, 0, 1)
Z = G + (Z - G) * wt

# which cells become faces: inside the disc, outside other buildings' footprints
inside = np.hypot(X - cx, Y - cy) < r
# hero models: cut exactly their built footprint (every gprim, + margin) -- not a frame window,
# which would also cut the rubble and the leaning slab around B01 where its wing collapsed
from pxr import Usd as _Usd, UsdGeom as _UG
heroes = {}
for usd in RECON.glob("*/[A-Z]*[0-9].usd"):
    hs = _Usd.Stage.Open(str(usd)); hid = hs.GetDefaultPrim().GetName().split("_")[0]; heroes[hid] = True
    xc = _UG.XformCache()
    for q in _Usd.PrimRange(hs.GetDefaultPrim()):
        if not q.IsA(_UG.Cube): continue
        M = xc.GetLocalToWorldTransform(q)
        c = np.array([M.Transform((x, y, z)) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        if c[:, 2].max() < np.nanmin(Z) - 1: continue
        hull = cv2.convexHull(np.round(np.c_[(c[:, 0] - (cx - r)) / R, ((cy + r) - c[:, 1]) / R] * 4).astype(np.int32))
        mk = np.zeros((n, n), np.uint8); cv2.fillConvexPoly(mk, hull, 1, shift=2)
        inside &= ~cv2.dilate(mk, np.ones((2 * int(a.margin / R) + 1,) * 2, np.uint8)).astype(bool)
for b in yaml.safe_load(open(RECON / "buildings_lod1.yaml"))["buildings"]:
    if b["id"] in heroes: continue
    th = np.radians(b["yaw_deg"]); dx, dy = X - b["at"][0], Y - b["at"][1]
    lu, lv = dx * np.cos(th) + dy * np.sin(th), -dx * np.sin(th) + dy * np.cos(th)
    inside &= ~((abs(lu) < b["size_m"][0] / 2 + a.margin) & (abs(lv) < b["size_m"][1] / 2 + a.margin))

V = np.stack([X, Y, Z], -1).reshape(-1, 3); idx = np.arange(n * n).reshape(n, n)
q = inside[:-1, :-1] & inside[1:, :-1] & inside[:-1, 1:] & inside[1:, 1:]
a0, a1, a2, a3 = idx[:-1, :-1][q], idx[:-1, 1:][q], idx[1:, :-1][q], idx[1:, 1:][q]
F = np.concatenate([np.stack([a0, a2, a1], 1), np.stack([a1, a2, a3], 1)])       # CCW seen from +Z
used = np.unique(F); remap = -np.ones(n * n, int); remap[used] = np.arange(len(used))
V, F, RGB = V[used], remap[F], RGB.reshape(-1, 3)[used]
mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(F))
mesh.vertex_colors = o3d.utility.Vector3dVector(RGB); mesh.compute_vertex_normals()
o3d.io.write_triangle_mesh(str(d / f"{a.id}.ply"), mesh)
print(f"mesh: {len(V)} verts, {len(F)} tris, z {V[:, 2].min():.1f}..{V[:, 2].max():.1f}")

stage = Usd.Stage.CreateNew(str(d / f"{a.id}.usd"))
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
root = UsdGeom.Xform.Define(stage, f"/{a.id}_{lab['name']}"); stage.SetDefaultPrim(root.GetPrim())
m = UsdGeom.Mesh.Define(stage, root.GetPath().AppendChild("mesh"))
m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(V.astype(np.float32)))
m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.astype(np.int32).ravel()))
m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
m.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex).Set(Vt.Vec3fArray.FromNumpy((RGB ** 2.2).astype(np.float32)))   # displayColor is linear
m.CreateExtentAttr([tuple(V.min(0)), tuple(V.max(0))])
UsdPhysics.CollisionAPI.Apply(m.GetPrim())
UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("none")   # static triangle mesh
p = root.GetPrim(); p.AddAppliedSchema("SemanticsLabelsAPI:class")
p.CreateAttribute("semantics:labels:class", Sdf.ValueTypeNames.TokenArray).Set([KIND_CLASS[lab["kind"]]])
for key in ("id", "name", "kind"): p.SetCustomDataByKey(f"label:{key}", lab[key])
stage.Save()
print(f"wrote {d / (a.id + '.ply')} and {d / (a.id + '.usd')}")

"""A rubble pile rebuilt from rubble ASSETS on the pile's measured shape (replaces the photogrammetry surface).

    ~/.venvs/recon/bin/python rubble_pile.py R01 [--radius 24] [--seed 3]

The pile's heightfield -- the drone reconstruction (data/recon/rubble_west/<ID>.ply, recon_mesh.py) when
there is one, else the tile surface -- is right about its SHAPE and wrong about its SURFACE, so:
  1. base mound: that heightfield smoothed (sigma 1.5 m) on a 0.5 m grid, 1.6 m under it,
     faded onto bare earth at the rim -- the pile's collider, labelled rubble, dark fines;
  2. pieces: library rubble (assets/lib, `class: rubble` -- the Nucleus DebrisConcrete set and the
     standalone slabs / chunks / rebar) stacked on the mound, biggest first and nearest the crest:
     each at a free spot, sunk 35% of its height into the mound, rolled to the local slope plus up
     to 25 deg, random yaw, scaled 0.8-1.25. Placement stops at ~1.4x cover of the pile area.
Hero models' footprints (+0.5 m) are kept clear. Writes data/recon/rubble_west/R01_assets.usd.
"""
import argparse, json
from pathlib import Path
import cv2, numpy as np, yaml, open3d as o3d
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, Vt
from _paths import R, LABELS

ap = argparse.ArgumentParser(); ap.add_argument("id"); ap.add_argument("--radius", type=float, default=24.0)
ap.add_argument("--seed", type=int, default=3); ap.add_argument("--cover", type=float, default=1.4)
ap.add_argument("--big-pieces", type=int, default=25, help="how many of the biggest DebrisConcrete pieces go on first")
ap.add_argument("--max-pieces", type=int, default=2500)
a = ap.parse_args(); rng = np.random.default_rng(a.seed)
lab = next(l for l in yaml.safe_load(open(LABELS))["labels"] if l["id"] == a.id); cx, cy = lab["at"]; rad = a.radius
LIB = json.load(open(R / "assets/lib/library.json"))
PIECES = {k: v for k, v in LIB.items() if v["class"] == "rubble"}

# ---- measured shape -> smoothed mound -------------------------------------------------------------
recon = R / "rubble_west" / f"{a.id}.ply"                              # a drone reconstruction, if there is one
if recon.exists(): V = np.asarray(o3d.io.read_triangle_mesh(str(recon)).vertices)
else:                                                                   # else the tile surface (site_rasters dsm)
    t_ = np.load(R / "site_rasters.npz"); dsm = t_["dsm"]; ii, jj = np.mgrid[0:dsm.shape[0], 0:dsm.shape[1]]
    V = np.c_[float(t_["x0"]) + (jj.ravel() + 0.5) * float(t_["res"]), float(t_["y1"]) - (ii.ravel() + 0.5) * float(t_["res"]), dsm.ravel()]
    V = V[np.hypot(V[:, 0] - lab["at"][0], V[:, 1] - lab["at"][1]) < a.radius + 2]
RES = 0.5; n = int(2 * rad / RES) + 1
xs = cx - rad + np.arange(n) * RES; ys = cy + rad - np.arange(n) * RES; X, Y = np.meshgrid(xs, ys)
rs = np.load(R / "site_rasters.npz"); X0, Y1, DR = float(rs["x0"]), float(rs["y1"]), float(rs["res"])
G = rs["dtm"][np.clip(((Y1 - Y) / DR).astype(int), 0, rs["dtm"].shape[0] - 1), np.clip(((X - X0) / DR).astype(int), 0, rs["dtm"].shape[1] - 1)]
if recon.exists():
    H = np.full((n, n), np.nan); u = ((V[:, 0] - (cx - rad)) / RES).round().astype(int); v = (((cy + rad) - V[:, 1]) / RES).round().astype(int)
    k = (u >= 0) & (v >= 0) & (u < n) & (v < n); np.fmax.at(H, (v[k], u[k]), V[k, 2])
    H = np.where(np.isfinite(H), H, G)
else:
    # tiles: sample the surface directly; the site DTM (a 51 m opening) swallows half of a 40 m pile,
    # so the ground is a plane through the lowest 40% of the surface on a ring just outside the pile
    t_ = np.load(R / "site_rasters.npz"); dsm = t_["dsm"]
    H = dsm[np.clip(((Y1 - Y) / DR).astype(int), 0, dsm.shape[0] - 1), np.clip(((X - X0) / DR).astype(int), 0, dsm.shape[1] - 1)].astype(float)
    ring = (np.hypot(X - cx, Y - cy) > rad) & (np.hypot(X - cx, Y - cy) < rad + 6)
    zr = H[ring]; lo_ = zr <= np.percentile(zr, 40)
    A_ = np.c_[X[ring][lo_] - cx, Y[ring][lo_] - cy, np.ones(lo_.sum())]; cf = np.linalg.lstsq(A_, zr[lo_], rcond=None)[0]
    G = np.minimum(G, cf[0] * (X - cx) + cf[1] * (Y - cy) + cf[2])
rel = cv2.GaussianBlur((H - G).astype(np.float32), (0, 0), 1.5 / RES)
rim = np.clip((rad - np.hypot(X - cx, Y - cy)) / 4.0, 0, 1)
mound_h = np.maximum(rel - 0.35, 0) * rim; Zm = G + mound_h

# hero footprints stay clear (B01 and its collapsed wing)
clear = np.zeros((n, n), bool); hero_polys = []
for usd in R.glob("*/[A-Z]*[0-9].usd"):
    if usd.stem == a.id: continue
    hs = Usd.Stage.Open(str(usd)); xc = UsdGeom.XformCache()
    for q in Usd.PrimRange(hs.GetDefaultPrim()):
        if not q.IsA(UsdGeom.Cube): continue
        M = xc.GetLocalToWorldTransform(q); c = np.array([M.Transform((x, y, z)) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])[:, :2]
        if np.hypot(*(c.mean(0) - [cx, cy])) > rad + 10: continue
        m = np.zeros((n, n), np.uint8)
        cv2.fillConvexPoly(m, cv2.convexHull(np.round(np.c_[(c[:, 0] - (cx - rad)) / RES, ((cy + rad) - c[:, 1]) / RES]).astype(np.int32)), 1)
        clear |= cv2.dilate(m, np.ones((3, 3), np.uint8)).astype(bool)
mound_h[clear] = 0
target = G + np.maximum(rel, 0) * rim; target[clear] = G[clear]          # the finished pile's top (measured)
Zm = G + np.maximum(target - G - 1.6, 0)                                # the mound: 1.6 m under it; pieces build the rest
inside = (np.hypot(X - cx, Y - cy) < rad) & ~clear
gz, gy_ = np.gradient(Zm, RES)                                        # d/dy (rows run -y), d/dx

# ---- USD ------------------------------------------------------------------------------------------
out = R / ("rubble_west" if recon.exists() else "rubble_east") / f"{a.id}_assets.usd"; out.parent.mkdir(exist_ok=True)
st = Usd.Stage.CreateNew(str(out)); UsdGeom.SetStageUpAxis(st, "Z"); UsdGeom.SetStageMetersPerUnit(st, 1.0)
root = UsdGeom.Xform.Define(st, f"/{a.id}_{lab['name']}"); st.SetDefaultPrim(root.GetPrim())
def label(p, cls):
    p.AddAppliedSchema("SemanticsLabelsAPI:class"); p.CreateAttribute("semantics:labels:class", Sdf.ValueTypeNames.TokenArray).Set([cls])
label(root.GetPrim(), "rubble")

idx = np.arange(n * n).reshape(n, n); q = inside[:-1, :-1] | inside[1:, 1:]
a0, a1, a2, a3 = idx[:-1, :-1][q], idx[:-1, 1:][q], idx[1:, :-1][q], idx[1:, 1:][q]
F = np.concatenate([np.stack([a0, a2, a1], 1), np.stack([a1, a2, a3], 1)])
used = np.unique(F); rm = -np.ones(n * n, int); rm[used] = np.arange(len(used))
P = np.stack([X, Y, Zm], -1).reshape(-1, 3)[used]
mesh = UsdGeom.Mesh.Define(st, root.GetPath().AppendChild("mound"))
mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P.astype(np.float32)))
mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(rm[F].astype(np.int32).ravel())); mesh.CreateSubdivisionSchemeAttr("none")
mesh.CreateDisplayColorAttr([(0.10, 0.09, 0.08)])                      # dark fines between the pieces, linear
UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()); UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")

# pieces are STACKED until the pile reaches its measured height: a running top surface S starts
# 1 m under the target (the mound) and every piece raises S over its footprint; the next piece
# goes where the deficit (target - S) is largest. DebrisConcrete column fragments, authored
# standing, are laid down; corrugated sheets / rebar mats are a small share.
S = Zm.copy()                                                          # running top surface, starts at the mound
cov = np.zeros((n, n), bool)                                           # cells some piece covers
# instancing: RTX cost follows UNIQUE meshes, so the textured DebrisConcrete fragments are used freely;
# the biggest (> 400k tris) are crest pieces only
heavy = sorted([k_ for k_ in PIECES if "DebrisConcrete" in PIECES[k_]["provenance"] and PIECES[k_]["size_m"][0] > 5.0],
               key=lambda k_: -PIECES[k_]["size_m"][0] * PIECES[k_]["size_m"][1])
light_w = {k_: (0.05 if ("sheet" in k_ or "rebar" in k_) else 0.25 if "sa_chunk" in k_ else 0.4 if "rubble_sa" in k_ else 1.0)
           for k_ in PIECES if k_ not in heavy}
light, lw = list(light_w), np.array(list(light_w.values())); lw /= lw.sum()
UsdGeom.Scope.Define(st, root.GetPath().AppendChild("pieces")); UsdGeom.Scope.Define(st, root.GetPath().AppendChild("colliders"))
placed = 0; tri_sum = 0; used_names = set()
while placed < a.max_pieces:
    D = np.where(inside, target - S, 0)
    if (D[inside] < 0.4).mean() > 0.9 and cov[inside].mean() > 0.9: break
    big = placed < a.big_pieces
    name = heavy[rng.integers(len(heavy))] if big else light[rng.choice(len(light), p=lw)]
    L, W, Hh = PIECES[name]["size_m"]; s = float(rng.uniform(0.8, 1.2))
    lay = Hh > max(L, W)                                               # a column fragment: lay it down
    fL, fW, th = ((Hh, W, L) if lay else (L, W, Hh)); fL, fW, th = fL * s, fW * s, th * s
    w = (np.maximum(D, 0) ** 2 + 2.0 * ~cov).ravel() + 1e-6; w[~inside.ravel()] = 0
    ij = rng.choice(n * n, p=w / w.sum()); i_, j_ = divmod(ij, n); x, y = X[i_, j_], Y[i_, j_]
    yaw = float(rng.uniform(0, 360))
    box = cv2.boxPoints(((float((x - (cx - rad)) / RES), float(((cy + rad) - y) / RES)), (fL / RES, fW / RES), -yaw))
    m = np.zeros((n, n), np.uint8); cv2.fillConvexPoly(m, np.round(box).astype(np.int32), 1); m = m.astype(bool) & inside
    if not m.any(): continue
    base = float(np.percentile(S[m], 60)) - 0.25 * th
    base = min(base, float(np.percentile(target[m], 70)) + 0.3 - 0.8 * th)  # a thick fragment sinks in: its top stays near the measured pile
    gy2, gx2 = np.gradient(S, RES); sx, sy = float(gx2[i_, j_]), float(-gy2[i_, j_])   # rows run -y
    slope_deg = min(np.degrees(np.arctan(np.hypot(sx, sy))), 35); slope_dir = np.degrees(np.arctan2(sy, sx))
    if big and D[i_, j_] < 0.6 * th: continue                         # a big piece only where there is room for it
    tilt_ = float(-(slope_deg + rng.uniform(-18, 18)))
    def pose(xf):
        xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), base))
        xf.AddRotateZOp(opSuffix="slope").Set(float(slope_dir)); xf.AddRotateYOp(opSuffix="tilt").Set(tilt_)
        xf.AddRotateZOp(opSuffix="yaw").Set(yaw); xf.AddScaleOp().Set(Gf.Vec3f(s))
        if lay:                                                        # applied first: stand -> lie along +X, base at z = 0
            xf.AddTranslateOp(opSuffix="lay").Set(Gf.Vec3d(-Hh / 2, 0, L / 2)); xf.AddRotateYOp(opSuffix="lay").Set(90.0)
    p = st.DefinePrim(root.GetPath().AppendChild("pieces").AppendChild(f"p{placed:04d}"), "Xform"); pose(UsdGeom.Xformable(p))
    p.GetReferences().AddReference("../" + PIECES[name]["usd"][2:]); p.SetInstanceable(True); label(p, "rubble")
    # collider: an invisible box on the piece's own bounds (80%: fragments are not full boxes), same pose.
    # The mound alone sits 1.6 m under the pile top -- without these a drone flies through the pieces.
    cb = UsdGeom.Xform.Define(st, root.GetPath().AppendChild("colliders").AppendChild(f"c{placed:04d}")); pose(cb)
    cube = UsdGeom.Cube.Define(st, cb.GetPath().AppendChild("box")); cube.CreatePurposeAttr("guide")
    UsdGeom.Xformable(cube).AddTranslateOp().Set(Gf.Vec3d(0, 0, Hh / 2)); UsdGeom.Xformable(cube).AddScaleOp().Set(Gf.Vec3f(0.4 * L, 0.4 * W, 0.4 * Hh))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim()); label(cube.GetPrim(), "rubble")
    S[m] = np.maximum(S[m], base + 0.8 * th); cov |= m; tri_sum += PIECES[name]["tris"]; placed += 1; used_names.add(name)
D = np.where(inside, target - S, 0)
st.Save()
under, over = D[inside & (D > 0)], -D[inside & (D < 0)]
print(f"  still under target: {np.mean(D[inside] > 0.5):.0%} of the disc by > 0.5 m; over by > 0.5 m: {np.mean(D[inside] < -0.5):.0%}")
print(f"{a.id}: mound {len(F)} tris, {placed} pieces stacked ({len(used_names)} unique meshes, {sum(PIECES[k_]["tris"] for k_ in used_names) / 1e6:.1f}M tris; {tri_sum / 1e6:.0f}M summed over instances); "
      f"pile height target {np.max(target - G):.1f} m, reached within {np.percentile(np.abs(D[inside]), 90):.2f} m (p90) -> {out}")

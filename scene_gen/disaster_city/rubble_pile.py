"""A rubble pile rebuilt from rubble ASSETS on the pile's measured shape (replaces the photogrammetry surface).

    ~/.venvs/recon/bin/python rubble_pile.py R01 [--radius 24] [--density 0.3] [--seed 3]

The pile's heightfield -- the drone reconstruction (data/recon/rubble_west/<ID>.ply, recon_mesh.py) when
there is one, else the tile surface -- is right about its SHAPE and wrong about its SURFACE. What goes on
it follows the drone footage (clips A01/B01): a dense jumble of light concrete -- precast beams and
planks, slabs, broken columns -- with wooden pallets through it, a few pipes, a little rebar, a rusty tank.
  1. footprint: where the surface stands > 0.6 m proud, inside --radius (R02 comes out square), pulled in by --shrink, minus any --keep-out polygon;
  2. base mound: the heightfield smoothed (sigma 1.5 m) on a 0.5 m grid, 1.6 m under it, faded over
     4 m at the rim -- the pile's collider, labelled rubble, concrete-dust coloured;
  3. heaps: DebrisConcrete heaps and photoscan patches (assets/lib, `class: rubble`), stacked where the
     pile is furthest under its measured height, until it is at height and the mound is hidden;
  4. surface: beams / pallets / columns / slabs / rebar / pipes (SURFACE_MIX) at --density per m2,
     thinning to none at the rim, rolled to the local slope +-20 deg; one tank near the crest.
Every piece is an instanceable reference with a box collider (80% of its bounds) in the same pose.
Hero models' footprints (+0.5 m) are kept clear. Writes data/recon/rubble_{west,east}/<ID>_assets.usd.
"""
import argparse, json
from pathlib import Path
import cv2, numpy as np, yaml, open3d as o3d
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, Vt
from _paths import R, LABELS

ap = argparse.ArgumentParser(); ap.add_argument("id"); ap.add_argument("--radius", type=float, default=24.0)
ap.add_argument("--seed", type=int, default=3); ap.add_argument("--cover", type=float, default=1.4)
ap.add_argument("--density", type=float, default=0.3, help="surface pieces per m2 of pile")
ap.add_argument("--max-pieces", type=int, default=2500)
ap.add_argument("--keep-out", help="x1,y1,x2,y2,... world polygon the pile may not cover (the video shows open ground there)")
ap.add_argument("--shrink", type=float, default=1.5, help="m to pull the footprint in from the measured edge (the videos show a tighter pile)")
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
# footprint: where the measured surface stands > 0.6 m proud (R02 is square, not round) -- the blob at
# the label, holes filled; the rim fades over 4 m inside its edge
blob = ((rel > 0.6) & (np.hypot(X - cx, Y - cy) < rad)).astype(np.uint8)       # (the DSM also stands proud under trees)
blob = cv2.morphologyEx(blob, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)); blob = cv2.morphologyEx(blob, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
nl, cc = cv2.connectedComponents(blob); blob = cc == (cc[n // 2, n // 2] or np.bincount(cc.ravel())[1:].argmax() + 1)
ff = np.pad(~blob, 1, constant_values=True).astype(np.uint8); cv2.floodFill(ff, None, (0, 0), 0); blob |= ff[1:-1, 1:-1].astype(bool)
if a.shrink > 0:                                                        # the measured edge runs onto the sand apron: pull it in
    k_ = 2 * int(round(a.shrink / RES)) + 1
    blob = cv2.erode(blob.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_, k_))).astype(bool)
if a.keep_out:
    ko = np.array([float(v) for v in a.keep_out.split(",")]).reshape(-1, 2); m_ = np.zeros((n, n), np.uint8)
    cv2.fillPoly(m_, [np.round(np.c_[(ko[:, 0] - (cx - rad)) / RES, ((cy + rad) - ko[:, 1]) / RES]).astype(np.int32)], 1)
    blob &= ~m_.astype(bool)
rim = np.clip(cv2.distanceTransform(blob.astype(np.uint8), cv2.DIST_L2, 5) * RES / 4.0, 0, 1)
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
target = G + np.maximum(rel, 0) * rim; target[clear] = G[clear]          # the finished pile's top (measured)
Zm = G + np.maximum(target - G - 1.6, 0)                                # the mound: 1.6 m under it; pieces build the rest
inside = blob & ~clear
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
mesh.CreateDisplayColorAttr([(0.30, 0.27, 0.23)])                      # concrete dust between the pieces (A01/B01), linear
UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()); UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")

# pieces, in two layers, after the drone footage (A01/B01: a dense jumble of light concrete, no ground
# showing): (1) HEAPS -- DebrisConcrete heaps and photoscan patches, stacked where the pile is furthest
# under its measured height until the mound is hidden; (2) SURFACE -- precast beams and planks, laid
# column fragments, slabs, wooden pallets, a few pipes and rebar, one rusty tank, strewn over the heaps.
def kind(k_):
    if k_.startswith(("rubble_beam", "rubble_plank")): return "beam"
    if k_.startswith("rubble_pallet"): return "pallet"
    if k_ == "rubble_pipe": return "pipe"
    if k_ == "rubble_tank_storage": return "tank"
    if k_.startswith("rubble_sa_rebar"): return "rebar"
    if k_ in ("rubble_slab_plain", "rubble_fab_concrete_slabs"): return "slab"
    if k_.startswith("rubble_fab_") or any(t in k_ for t in ("big_", "medium_", "small_")): return "heap"
    if "DebrisConcrete" in PIECES[k_]["provenance"]: return "column"          # standing fragments, laid down
    return None
KINDS = {}
for k_ in PIECES: KINDS.setdefault(kind(k_), []).append(k_)
SCALE = {"rubble_tank_storage": 0.4}                                   # the pile's tank is ~1.7 m tall, the asset a 4.2 m silo
SURFACE_MIX = {"beam": 0.24, "pallet": 0.35, "column": 0.15, "slab": 0.14, "rebar": 0.04, "pipe": 0.04}
S = Zm.copy()                                                          # running top surface, starts at the mound
cov = np.zeros((n, n), bool)                                           # cells some piece covers
UsdGeom.Scope.Define(st, root.GetPath().AppendChild("pieces")); UsdGeom.Scope.Define(st, root.GetPath().AppendChild("colliders"))
placed = 0; tri_sum = 0; used_names = set()

def put(name, i_, j_, sink, heap):
    """One piece at grid cell (i_, j_) on the running surface S. False if it does not fit."""
    global placed, tri_sum
    L, W, Hh = PIECES[name]["size_m"]; s = SCALE.get(name, 1.0) * float(rng.uniform(0.85, 1.15))
    lay = Hh > max(L, W) and kind(name) != "tank"                      # a column fragment: lay it down
    fL, fW, th = ((Hh, W, L) if lay else (L, W, Hh)); fL, fW, th = fL * s, fW * s, th * s
    x, y, yaw = X[i_, j_], Y[i_, j_], float(rng.uniform(0, 360))
    box = cv2.boxPoints(((float((x - (cx - rad)) / RES), float(((cy + rad) - y) / RES)), (fL / RES, fW / RES), -yaw))
    m = np.zeros((n, n), np.uint8); cv2.fillConvexPoly(m, np.round(box).astype(np.int32), 1); m = m.astype(bool) & inside
    if not m.any(): return False
    base = float(np.percentile(S[m], 50)) - sink * th
    top = float(np.percentile(target[m], 80))
    if heap: base = min(base, top + 0.3 - 0.8 * th)                    # a heap's top stays near the measured pile
    elif base + th > top + 0.9: return False                           # nothing stands far above it either
    gy2, gx2 = np.gradient(S, RES); sx, sy = float(gx2[i_, j_]), float(-gy2[i_, j_])   # rows run -y
    slope_deg = min(np.degrees(np.arctan(np.hypot(sx, sy))), 35); slope_dir = np.degrees(np.arctan2(sy, sx))
    tilt_ = float(-(slope_deg * 0.5 if kind(name) == "tank" else slope_deg + rng.uniform(-20, 20)))
    def pose(xf):
        xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), base))
        xf.AddRotateZOp(opSuffix="slope").Set(float(slope_dir)); xf.AddRotateYOp(opSuffix="tilt").Set(tilt_)
        xf.AddRotateZOp(opSuffix="yaw").Set(yaw); xf.AddScaleOp().Set(Gf.Vec3f(s))
        if lay:                                                        # applied first: stand -> lie along +X, base at z = 0
            xf.AddTranslateOp(opSuffix="lay").Set(Gf.Vec3d(-Hh / 2, 0, L / 2)); xf.AddRotateYOp(opSuffix="lay").Set(90.0)
    p = st.DefinePrim(root.GetPath().AppendChild("pieces").AppendChild(f"p{placed:04d}"), "Xform"); pose(UsdGeom.Xformable(p))
    p.GetReferences().AddReference("../" + PIECES[name]["usd"][2:]); p.SetInstanceable(True); label(p, "rubble")
    # collider: an invisible box on the piece's own bounds (80%: fragments and heaps are not full boxes),
    # same pose -- the mound alone sits 1.6 m under the pile top, and a drone would fly through the pieces
    cb = UsdGeom.Xform.Define(st, root.GetPath().AppendChild("colliders").AppendChild(f"c{placed:04d}")); pose(cb)
    cube = UsdGeom.Cube.Define(st, cb.GetPath().AppendChild("box")); cube.CreatePurposeAttr("guide")
    UsdGeom.Xformable(cube).AddTranslateOp().Set(Gf.Vec3d(0, 0, Hh / 2)); UsdGeom.Xformable(cube).AddScaleOp().Set(Gf.Vec3f(0.4 * L, 0.4 * W, 0.4 * Hh))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim()); label(cube.GetPrim(), "rubble")
    S[m] = np.maximum(S[m], base + 0.8 * th); cov[m] = True; tri_sum += PIECES[name]["tris"]; placed += 1; used_names.add(name)
    return True

# (1) heaps, where the deficit is largest, until the pile is at height and the mound covered
heaps = KINDS["heap"]
for _ in range(a.max_pieces):
    D = np.where(inside, target - S, 0)
    if (D[inside] < 0.4).mean() > 0.9 and cov[inside].mean() > 0.97: break
    w = (np.maximum(D, 0) ** 2 + 2.0 * ~cov).ravel() + 1e-6; w[~inside.ravel()] = 0
    i_, j_ = divmod(int(rng.choice(n * n, p=w / w.sum())), n)
    put(heaps[rng.integers(len(heaps))], i_, j_, 0.3, True)
n_heap = placed
# (2) surface pieces over the pile at the clips' density, thinning to none at the rim (the piles taper
# onto their sand aprons); one tank near the crest
cells = np.flatnonzero((inside & (rim > 0.25)).ravel()); cw = rim.ravel()[cells] ** 2; cw /= cw.sum(); kinds_, kw = list(SURFACE_MIX), np.array(list(SURFACE_MIX.values()))
n_surf = int(a.density * inside.sum() * RES * RES)
for _ in range(4 * n_surf):
    if placed - n_heap >= n_surf: break
    kd = kinds_[rng.choice(len(kinds_), p=kw / kw.sum())]
    i_, j_ = divmod(int(cells[rng.choice(len(cells), p=cw)]), n)
    put(KINDS[kd][rng.integers(len(KINDS[kd]))], i_, j_, 0.25, False)
crest = np.flatnonzero((inside & (rim > 0.9)).ravel())
for _ in range(50):
    if put("rubble_tank_storage", *divmod(int(crest[rng.integers(len(crest))]), n), 0.3, False): break
print(f"  {n_heap} heaps, {placed - n_heap} surface pieces (incl. tank)")
D = np.where(inside, target - S, 0)
st.Save()
under, over = D[inside & (D > 0)], -D[inside & (D < 0)]
print(f"  still under target: {np.mean(D[inside] > 0.5):.0%} of the disc by > 0.5 m; over by > 0.5 m: {np.mean(D[inside] < -0.5):.0%}")
print(f"{a.id}: mound {len(F)} tris, {placed} pieces stacked ({len(used_names)} unique meshes, {sum(PIECES[k_]["tris"] for k_ in used_names) / 1e6:.1f}M tris; {tri_sum / 1e6:.0f}M summed over instances); "
      f"pile height target {np.max(target - G):.1f} m, reached within {np.percentile(np.abs(D[inside]), 90):.2f} m (p90) -> {out}")

"""Replace the tile mesh's tree and vehicle blobs with library assets.

    ~/.venvs/recon/bin/python place_assets.py      # then rerun ground.py + assemble_scene.py

Reads data/recon/site_rasters.npz (ground.py). Assets are local copies in
data/recon/assets/ (trees: NVIDIA AEC vegetation, cm Z-up; cars: the standalone
pack, metres Z-up, facing +X).

Trees: canopy-height peaks (DSM - DTM, 1 m smoothed, 5 m window) on cells the
ground pass classed vegetation, >= 3 m tall, plus a greedy fill of any such cell
with no tree within 4.5 m (the tile canopy is too blobby for peaks alone) -> one instance each, uniformly
scaled to the peak height, random species/yaw. Written as instanceable references
(`vegetation`, labelled per instance -- Isaac's semantic segmentation leaves
PointInstancer instances unlabelled) plus invisible colliders (trunk cylinder + crown sphere) so a
drone can fly under the canopy but not through it.
Vehicles: every vehicle-sized blob on roads, lots, pads and near vehicle labels
(see the vehicles section): car / van / bus sized ones get an asset, touching rows
are split into cars, and the rest (rail cars, trailers, containers) a box fitted
to the blob -- none keeps its tile mesh. Cars within 15 m of a
wreck label get a burned car. Replaced footprints -> data/recon/replaced.npz, which
ground.py cuts out of the tile surface.
Debris: pieces scattered over the rubble fields with no drone footage (R02, R03).
Writes data/recon/trees.usd, data/recon/vehicles.usd, data/recon/debris.usd.
"""
import json
from pathlib import Path
import cv2, numpy as np, open3d as o3d, yaml
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, Vt

from _paths import R, LABELS
A = R / "assets"
rng = np.random.default_rng(7)
import os; DEBUG = bool(os.environ.get("DEBUG"))
r = np.load(R / "site_rasters.npz"); X0, Y1, RES = float(r["x0"]), float(r["y1"]), float(r["res"])
DSM, DTM, CLS = r["dsm"], r["dtm"], r["cls"]; N = DSM.shape[0]
cell_xy = lambda i, j: (X0 + (j + 0.5) * RES, Y1 - (i + 0.5) * RES)

def label(prim, cls):
    prim.AddAppliedSchema("SemanticsLabelsAPI:class")
    prim.CreateAttribute("semantics:labels:class", Sdf.ValueTypeNames.TokenArray).Set([cls])

def new_stage(path, root):
    st = Usd.Stage.CreateNew(str(path)); UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(st, 1.0)
    x = UsdGeom.Xform.Define(st, root); st.SetDefaultPrim(x.GetPrim()); return st, x

def size_of(path):
    st = Usd.Stage.Open(str(path)); mpu = UsdGeom.GetStageMetersPerUnit(st)
    b = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    return np.array(b.GetMax() - b.GetMin()) * mpu, mpu

# ------------------------------------------------------------------ trees
TREES = ["White_Ash", "American_Beech", "Honey_Locust", "Largetooth_Aspen"]
proto = {t: size_of(A / f"trees/{t}.usd") for t in TREES}
chm = cv2.GaussianBlur((DSM - DTM).astype(np.float32), (0, 0), 1.0)
veg = cv2.dilate((CLS == 1).astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
peak = (chm == cv2.dilate(chm, np.ones((5, 5), np.uint8))) & veg & (chm >= 3.0)
ii, jj = np.nonzero(peak)
print(f"trees: {len(ii)} canopy peaks", end="")
# the tile canopy is blobby, so peaks alone leave gaps: greedy fill of any tall-enough
# vegetation cell with no tree within GAP m, in random order (a Poisson-disk-ish cover)
GAP = 4.5
occ = np.zeros((N, N), np.uint8)
for i, j in zip(ii, jj): cv2.circle(occ, (int(j), int(i)), int(GAP / RES), 1, -1)
ci, cj = np.nonzero(veg & (chm >= 3.0)); o = rng.permutation(len(ci)); add = []
for i, j in zip(ci[o], cj[o]):
    if not occ[i, j]: add.append((i, j)); cv2.circle(occ, (int(j), int(i)), int(GAP / RES), 1, -1)
if add: ii, jj = np.r_[ii, [a for a, _ in add]], np.r_[jj, [b for _, b in add]]
print(f" + {len(add)} gap fill = {len(ii)}")

st, root = new_stage(R / "trees.usd", "/trees")
k = rng.integers(0, len(TREES), len(ii)); h = chm[ii, jj]
s = np.clip(h / np.array([proto[TREES[q]][0][2] for q in k]), 0.4, 3.0)
yaw = rng.uniform(0, 360, len(ii))
pos = np.array([[*cell_xy(i, j), DTM[i, j]] for i, j in zip(ii, jj)])
# scene-graph instancing (instanceable references), NOT a PointInstancer: Isaac's
# semantic segmentation leaves PointInstancer instances unlabelled
UsdGeom.Scope.Define(st, "/trees/instances")
for n, (p_, q, sc, yw) in enumerate(zip(pos, k, s, yaw)):
    x = UsdGeom.Xform.Define(st, f"/trees/instances/tree_{n:04d}")
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, p_))); x.AddRotateZOp().Set(float(yw))
    x.AddScaleOp().Set(Gf.Vec3f(float(sc * proto[TREES[q]][1])))              # cm-authored -> metres
    x.GetPrim().GetReferences().AddReference(f"./assets/trees/{TREES[q]}.usd"); x.GetPrim().SetInstanceable(True)
    label(x.GetPrim(), "vegetation")
# colliders: invisible (guide purpose), static
cols = UsdGeom.Scope.Define(st, "/trees/colliders")
for n, ((x, y, z), hh, q, sc) in enumerate(zip(pos, h, k, s)):
    crown = proto[TREES[q]][0][:2].mean() * sc / 2 * 0.8
    trunk = UsdGeom.Cylinder.Define(st, f"/trees/colliders/t{n}")
    x, y, z, hh = float(x), float(y), float(z), float(hh)
    trunk.CreateRadiusAttr(max(0.12, 0.02 * hh)); trunk.CreateHeightAttr(hh * 0.45); trunk.CreateAxisAttr("Z")
    trunk.AddTranslateOp().Set(Gf.Vec3d(x, y, z + hh * 0.225)); trunk.CreatePurposeAttr("guide")
    ball = UsdGeom.Sphere.Define(st, f"/trees/colliders/c{n}")
    ball.CreateRadiusAttr(float(min(crown, hh * 0.3))); ball.AddTranslateOp().Set(Gf.Vec3d(x, y, z + hh * 0.68)); ball.CreatePurposeAttr("guide")
    for g in (trunk, ball): UsdPhysics.CollisionAPI.Apply(g.GetPrim()); label(g.GetPrim(), "vegetation")
st.Save()
print(f"  wrote {len(ii)} trees, heights {np.percentile(h, [10, 50, 90]).round(1)} m")

# ------------------------------------------------------------------ vehicles
# Detection: every blob on the fine (0.25 m) tile height map that stands 0.6-4.5 m on a road,
# a parking lot, a pad or within 12 m of a labelled vehicle, and is not a building, a hero or
# canopy. Each is fitted in the WORLD frame (principal axes) and becomes
#   car / van / bus          a library asset of that class, scaled to the fitted length
#   a touching row of cars   split into car-sized slots, one asset each
#   rail car / trailer / container / anything else vehicle-sized   a box of the fitted size
#                            (no library asset exists), coloured from the ortho
# Nothing vehicle-sized keeps its tile mesh. Cars within 15 m of a wreck label are burned cars.
CARS = {"car": ["lowpoly_sedan", "lowpoly_wagon", "lowpoly_coupe", "red_car", "lowpoly_pickup"],
        "wreck": ["burned_car_01", "burned_car_02"], "van": ["delivery_van"], "bus": ["citybus"]}
def car_usd(name): return next((A / "cars" / name).glob("*.usd*"))
csize = {n: size_of(car_usd(n))[0] for v in CARS.values() for n in v}
labels = yaml.safe_load(open(LABELS))["labels"]
wrecks = np.array([l["at"] for l in labels if any(w in l["name"] for w in ("wreck", "collapsed", "rubble", "derail"))])
vlabels = np.array([l["at"] for l in labels if l["kind"] == "vehicle"])
geo = json.load(open(R / "ortho_site.json")); ortho = cv2.imread(str(R / "ortho_site.png"))

t = np.load(R / "tiles_site.npz"); scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(o3d.core.Tensor(t["verts"].astype(np.float32)), o3d.core.Tensor(t["faces"].astype(np.uint32)))
F = 0.25; NF = int(N * RES / F)
gx = X0 + (np.arange(NF) + 0.5) * F; gy = Y1 - (np.arange(NF) + 0.5) * F
GX, GY = np.meshgrid(gx, gy)
Z = 500 - scene.cast_rays(o3d.core.Tensor(np.stack([GX, GY, np.full_like(GX, 500), 0 * GX, 0 * GX, -np.ones_like(GX)], -1).astype(np.float32)))["t_hit"].numpy()
up_ = lambda a: cv2.resize(a.astype(np.uint8), (NF, NF), interpolation=cv2.INTER_NEAREST).astype(bool)
Hf = np.nan_to_num(Z - cv2.resize(DTM, (NF, NF), interpolation=cv2.INTER_LINEAR))
where = up_(r["osm_road"] | r["road"] | (CLS == 3))
for vx, vy in vlabels: where |= np.hypot(GX - vx, GY - vy) < 12
free = ~up_(r["occupied"]) & ~up_(r["veg"]) & ~up_(r["water"])
where &= free
# rail cars are accepted anywhere free by shape (they stand on grass and track); cars, vans,
# buses and generic boxes only on roads / lots / near vehicle labels (car-sized props and junk
# on the training yards' grass read as cars otherwise)
# bridges: the deck stands above the smoothed bare earth along a road -- a big raised patch of
# road is a bridge, not a car (the NW entrance bridge was read as a row of cars)
nb, cb, sb, _ = cv2.connectedComponentsWithStats(((Hf > 0.6) & up_(r["osm_road"])).astype(np.uint8))
bridge = cv2.dilate(np.isin(cb, np.flatnonzero(sb[:, cv2.CC_STAT_AREA] * F * F > 60)[1:]).astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)
cand = cv2.morphologyEx(((Hf > 0.6) & (Hf < 5.0) & free & ~bridge).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

def world_fit(m):
    ys, xs = np.nonzero(m); P = np.c_[gx[xs], gy[ys]]; c = P.mean(0)
    w, v = np.linalg.eigh(np.cov((P - c).T) + 1e-9 * np.eye(2)); ax = v[:, 1]
    u = (P - c) @ ax; q = (P - c) @ np.array([-ax[1], ax[0]])
    c = c + ax * (np.percentile(u, 97) + np.percentile(u, 3)) / 2 + np.array([-ax[1], ax[0]]) * (np.percentile(q, 97) + np.percentile(q, 3)) / 2
    L, W = np.percentile(u, 97) - np.percentile(u, 3) + F, np.percentile(q, 97) - np.percentile(q, 3) + F
    return c, float(np.degrees(np.arctan2(ax[1], ax[0]))), float(L), float(W), float(np.percentile(Hf[m], 90)), len(xs) * F * F / max(L * W, 1e-6)

st, root = new_stage(R / "vehicles.usd", "/vehicles"); label(root.GetPrim(), "vehicle")
counts = {"asset": 0, "row": 0, "rail": 0, "proxy": 0, "skip": 0}; polys = []; serial = iter(range(10 ** 6))
def put_asset(kind, c, yaw, L, gz):
    if kind == "car" and len(wrecks) and np.hypot(*(wrecks - c).T).min() < 15: kind = "wreck"
    name = CARS[kind][rng.integers(len(CARS[kind]))]
    sc = float(np.clip(L / csize[name][0], 0.85, 1.15)); i = next(serial)
    p = st.DefinePrim(f"/vehicles/{kind}_{i:03d}", "Xform"); p.GetReferences().AddReference(f"./assets/cars/{name}/{car_usd(name).name}")
    xf = UsdGeom.Xformable(p); xf.AddTranslateOp().Set(Gf.Vec3d(float(c[0]), float(c[1]), float(gz)))
    xf.AddRotateZOp().Set(float(yaw + rng.choice([0, 180]))); xf.AddScaleOp().Set(Gf.Vec3f(sc))   # assets face +X
    UsdPhysics.CollisionAPI.Apply(p)
def put_proxy(c, yaw, L, W, top, gz):
    oc = ortho[int((geo["y1"] - c[1]) / geo["m_per_px"]), int((c[0] - geo["x0"]) / geo["m_per_px"])][::-1] / 255.0
    i = next(serial); xb = UsdGeom.Xform.Define(st, f"/vehicles/proxy_{i:03d}")
    xb.AddTranslateOp().Set(Gf.Vec3d(float(c[0]), float(c[1]), float(gz + top / 2))); xb.AddRotateZOp().Set(float(yaw))
    xb.AddScaleOp().Set(Gf.Vec3f(float(L / 2), float(W / 2), float(top / 2)))
    cube = UsdGeom.Cube.Define(st, xb.GetPath().AppendChild("box"))
    cube.CreateDisplayColorAttr([tuple(float(v) ** 2.2 for v in oc)]); UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

og = ortho[np.clip(((geo["y1"] - GY) / geo["m_per_px"]).astype(int), 0, ortho.shape[0] - 1),
           np.clip(((GX - geo["x0"]) / geo["m_per_px"]).astype(int), 0, ortho.shape[1] - 1)].astype(int)
EXG = 2 * og[..., 1] - og[..., 0] - og[..., 2]                      # a shrub on a lot is vehicle-sized too
n_, cc_ = cv2.connectedComponents(cand)
for k in range(1, n_):
    m = cc_ == k
    if m.sum() * F * F < 2.5: counts["skip"] += 1; continue           # a bollard, a sign, a bush stump
    c, yaw, L, W, top, fill = world_fit(m)
    gz = float(DTM[int((Y1 - c[1]) / RES), int((c[0] - X0) / RES)])
    if L > 60 or W > 12 or top < 0.9: counts["skip"] += 1; continue  # a wall, a slab, kerb clutter -- not a vehicle
    if np.median(EXG[m]) > 12: counts["skip"] += 1; continue          # green: vegetation
    ax = np.array([np.cos(np.radians(yaw)), np.sin(np.radians(yaw))])
    on_way = where[m].mean() > 0.5
    # tile blobs come out ~1 m wider than the vehicle (blurred mesh + shadow skirt): widths are generous
    if on_way and 3.4 <= L <= 5.9 and 1.4 <= W <= 3.2 and top < 2.4 and fill > 0.55: put_asset("car", c, yaw, L, gz); counts["asset"] += 1
    elif on_way and 5.3 < L <= 8.2 and 1.6 <= W <= 4.0 and top < 3.3 and fill > 0.55: put_asset("van", c, yaw, L, gz); counts["asset"] += 1
    elif on_way and 9.0 <= L <= 13.5 and 2.2 <= W <= 4.2 and 2.3 <= top < 4.5 and fill > 0.55: put_asset("bus", c, yaw, L, gz); counts["asset"] += 1
    elif L >= 12 and 2.2 <= W <= 4.5 and 2.0 <= top < 5.0 and fill > 0.5:  # rail car(s): one box per ~16 m
        nseg = max(1, round(L / 16.0))
        for j in range(nseg): put_proxy(c + ax * (j - (nseg - 1) / 2) * L / nseg, yaw, L / nseg - 0.5, min(W, 3.2), top, gz)
        counts["rail"] += nseg
    elif on_way and top < 2.4 and 1.4 <= W <= 3.2 and L > 5.9:          # cars end to end
        nslot = max(2, round(L / 4.8))
        for j in range(nslot): put_asset("car", c + ax * (j - (nslot - 1) / 2) * L / nslot, yaw, L / nslot - 0.3, gz)
        counts["row"] += 1
    elif on_way and top < 2.4 and 3.6 <= W <= 6.0 and L >= 3.6:         # cars side by side
        nslot = max(2, round(L / 2.7))
        for j in range(nslot): put_asset("car", c + ax * (j - (nslot - 1) / 2) * L / nslot, yaw + 90, min(W, 5.0), gz)
        counts["row"] += 1
    elif on_way and top >= 1.4 and L * W >= 8:                           # trailer, container, truck body: fitted box
        put_proxy(c, yaw, L, W, top, gz); counts["proxy"] += 1
    else:
        counts["skip"] += 1; continue                                     # clutter, crates, dumpsters, not a vehicle
    polys.append(cv2.boxPoints(((float(c[0]), float(c[1])), (L + 0.6, W + 0.6), float(yaw))))   # world coords; box orientation only matters loosely
st.Save()
np.savez(R / "replaced.npz", polys=np.array(polys, np.float32) if polys else np.zeros((0, 4, 2), np.float32))
print(f"vehicles: {counts['asset']} single assets, {counts['row']} rows split into cars, {counts['rail']} rail-car boxes, "
      f"{counts['proxy']} trailer/container boxes; {counts['skip']} small / non-vehicle blobs skipped")

# ------------------------------------------------------------------ debris
# visual detail on the rubble fields the drones never filmed (R02, R03): pieces
# from the standalone pack, resting on the tile mound (which stays the collider),
# random yaw and up to 25 deg tilt. Instanceable references, class `rubble`.
PIECES = sorted(p.name for p in (A / "debris").iterdir() if p.is_dir() and not p.name.startswith("lump"))
FIELDS = {"R02": 19.0, "R03": 18.0}                              # label -> disc radius, m
lab = {l["id"]: l for l in labels}
pts = []
for fid, rad in FIELDS.items():
    cx, cy = lab[fid]["at"]; n_ = int(np.pi * rad * rad / 4)
    a_, r_ = rng.uniform(0, 2 * np.pi, n_), rad * np.sqrt(rng.uniform(0, 1, n_))
    pts.append(np.c_[cx + r_ * np.cos(a_), cy + r_ * np.sin(a_)])
pts = np.concatenate(pts)
ray = np.c_[pts, np.full(len(pts), 500), np.zeros((len(pts), 2)), -np.ones(len(pts))].astype(np.float32)
hz = 500 - scene.cast_rays(o3d.core.Tensor(ray))["t_hit"].numpy()
ok = np.isfinite(hz); pts, hz = pts[ok], hz[ok]
st, root = new_stage(R / "debris.usd", "/debris")
UsdGeom.Scope.Define(st, "/debris/instances")
k = rng.integers(0, len(PIECES), len(pts))
for n, (p_, hz_, q, yw, tl, ax, sc) in enumerate(zip(pts, hz, k, rng.uniform(0, 360, len(pts)), rng.uniform(0, 25, len(pts)),
                                                      rng.uniform(0, 360, len(pts)), rng.uniform(0.7, 1.3, len(pts)))):
    x = UsdGeom.Xform.Define(st, f"/debris/instances/piece_{n:04d}")
    x.AddTranslateOp().Set(Gf.Vec3d(float(p_[0]), float(p_[1]), float(hz_) - 0.1))
    x.AddRotateZOp().Set(float(ax)); x.AddRotateXOp().Set(float(tl)); x.AddRotateZOp(opSuffix="yaw").Set(float(yw - ax))
    x.AddScaleOp().Set(Gf.Vec3f(float(sc)))
    x.GetPrim().GetReferences().AddReference(f"./assets/debris/{PIECES[q]}/{PIECES[q]}.usdc"); x.GetPrim().SetInstanceable(True)
    label(x.GetPrim(), "rubble")
st.Save()
print(f"debris: {len(pts)} pieces over {', '.join(FIELDS)}")

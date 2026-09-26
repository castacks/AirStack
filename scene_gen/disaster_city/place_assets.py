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
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Vt

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
counts = {"asset": 0, "lib": 0, "row": 0, "rail": 0, "proxy": 0, "skip": 0}; polys = []; serial = iter(range(10 ** 6))
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

# third-party library (asset_library.py): normalised -- long side +X, base at z = 0, metres
LIB = json.load(open(A / "lib/library.json"))
RAILCARS = ["tankcar", "tankcar_graffiti", "tankcar_cyl", "boxcar", "coalcar"]
locos = np.array([l["at"] for l in labels if "locomotive" in l["name"]])
derail = np.array([l["at"] for l in labels if "derail" in l["name"]])
# rail cars only where the site has rails: near a rail-related label (a big blob off the roads is
# otherwise just as likely a canopy, a house roof or a campus building)
railish = np.array([l["at"] for l in labels if any(w in l["name"] for w in ("rail", "tank_car", "derail", "locomotive", "tanker"))])
near_rail = lambda c: len(railish) and np.hypot(*(railish - c).T).min() < 30
def put_lib(name, c, yaw, L, W, top, gz, tilt=0.0, pitch=0.0, exact=False):
    """A library asset fitted to the blob's box: x by L, y by W, z by top, each within 25% of
    the uniform scale (so a blurred blob cannot squash the model); `exact` scales to L x W x top as given."""
    aL, aW, aH = LIB[name]["size_m"]; u = L / aL
    sx, sy, sz = (u, W / aW, top / aH) if exact else (u, float(np.clip(W / aW, 0.8 * u, 1.25 * u)), float(np.clip(top / aH, 0.75 * u, 1.25 * u)))
    i = next(serial); p = st.DefinePrim(f"/vehicles/{name}_{i:03d}", "Xform"); p.GetReferences().AddReference(LIB[name]["usd"])
    xf = UsdGeom.Xformable(p); xf.AddTranslateOp().Set(Gf.Vec3d(float(c[0]), float(c[1]), float(gz)))
    flip = rng.choice([0, 180]); xf.AddRotateZOp().Set(float(yaw + flip)); xf.AddRotateXOp().Set(float(tilt))
    if pitch: xf.AddRotateYOp().Set(float(-pitch if flip == 0 else pitch))     # pitch > 0: the car rises along yaw
    xf.AddScaleOp().Set(Gf.Vec3f(float(sx), float(sy), float(sz))); UsdPhysics.CollisionAPI.Apply(p)
    p.SetCustomDataByKey("fit", {"L": round(L, 2), "W": round(W, 2), "top": round(top, 2)})
RAIL_H = {"passenger_car": 4.2, "tankcar": 4.3, "tankcar_cyl": 4.3, "tankcar_graffiti": 4.3, "boxcar": 4.6, "locomotive": 4.6}   # real car heights, m
def emit_rail():
    """Rail cars come from the hand survey (specs/rail_cars.yaml): the tile mesh merges coupled and
    derailed cars, so blob fitting cannot separate them. Detected rail blobs only mark their tile
    geometry for removal (polys); each surveyed car becomes its asset scaled to the surveyed length."""
    for car in SURVEY:
        a, b = np.array(car["ends"], float); c = (a + b) / 2; L = float(np.linalg.norm(b - a))
        yaw = float(np.degrees(np.arctan2(b[1] - a[1], b[0] - a[0]))); gz = float(DTM[int((Y1 - c[1]) / RES), int((c[0] - X0) / RES)])
        za, zb = car.get("lift", [0.0, 0.0])                           # base raised at each end (a car resting on another)
        put_lib(car["asset"], c, yaw, L, 3.1, RAIL_H.get(car["asset"], 4.3), gz + (za + zb) / 2,
                tilt=float(car.get("tilt", 0.0)), pitch=float(np.degrees(np.arctan2(zb - za, L))), exact=True)
        polys.append(cv2.boxPoints(((float(c[0]), float(c[1])), (L + 1.0, 4.4), yaw)))
    counts["rail"] = len(SURVEY)
og = ortho[np.clip(((geo["y1"] - GY) / geo["m_per_px"]).astype(int), 0, ortho.shape[0] - 1),
           np.clip(((GX - geo["x0"]) / geo["m_per_px"]).astype(int), 0, ortho.shape[1] - 1)].astype(int)
EXG = 2 * og[..., 1] - og[..., 0] - og[..., 2]                      # a shrub on a lot is vehicle-sized too
SURVEY = yaml.safe_load(open(Path(__file__).resolve().parent / "specs/rail_cars.yaml"))
surveyed = np.zeros_like(cand)
for car in SURVEY:
    a_, b_ = np.array(car["ends"], float); c_ = (a_ + b_) / 2; L_ = float(np.linalg.norm(b_ - a_))
    box = cv2.boxPoints(((float(c_[0]), float(c_[1])), (L_ + 2.0, 5.0), float(np.degrees(np.arctan2(b_[1] - a_[1], b_[0] - a_[0])))))
    cv2.fillConvexPoly(surveyed, np.round(np.c_[(box[:, 0] - X0) / F, (Y1 - box[:, 1]) / F]).astype(np.int32), 1)
for f in (Path(__file__).resolve().parent / "specs").glob("*.yaml"):   # hero props standing where a blob would read as a vehicle
    sp = yaml.safe_load(open(f))
    if isinstance(sp, dict) and sp.get("mask_vehicles"):
        L_, W_ = sp["size_m"]; t_ = np.radians(sp["yaw_deg"]); o_ = np.array(sp["origin"][:2])
        c_ = o_ + np.array([[np.cos(t_), -np.sin(t_)], [np.sin(t_), np.cos(t_)]]) @ [L_ / 2, W_ / 2]
        box = cv2.boxPoints(((float(c_[0]), float(c_[1])), (L_ + 2.0, W_ + 2.0), float(sp["yaw_deg"])))
        cv2.fillConvexPoly(surveyed, np.round(np.c_[(box[:, 0] - X0) / F, (Y1 - box[:, 1]) / F]).astype(np.int32), 1)
cand &= 1 - surveyed                                                    # rail cars are the survey's (emit_rail)
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
    elif on_way and 5.3 < L <= 8.2 and 1.6 <= W <= 4.0 and top < 2.6 and fill > 0.55: put_asset("van", c, yaw, L, gz); counts["asset"] += 1
    elif on_way and 5.3 < L <= 8.2 and 1.6 <= W <= 4.0 and top < 3.5 and fill > 0.55:              # box truck / RV
        put_lib("motorhome" if L > 7.3 else "box_truck", c, yaw, L, W, top, gz); counts["lib"] += 1
    elif on_way and 9.0 <= L <= 13.5 and 2.2 <= W <= 4.2 and 2.5 <= top < 4.5 and fill > 0.55: put_asset("bus", c, yaw, L, gz); counts["asset"] += 1
    elif on_way and 9.0 <= L <= 13.5 and 4.2 < W <= 9.0 and 2.3 <= top < 4.5:                      # buses parked side by side
        perp = np.array([-ax[1], ax[0]]); nb = max(2, round(W / 3.0))
        for j in range(nb): put_asset("bus", c + perp * (j - (nb - 1) / 2) * W / nb, yaw, L, gz)
        counts["row"] += 1
    elif on_way and 6.0 <= L <= 9.5 and 2.0 <= W <= 4.0 and 3.3 <= top < 4.3 and fill > 0.5:      # dump / construction truck
        put_lib("construction_truck", c, yaw, L, W, top, gz); counts["lib"] += 1
    elif on_way and 5.0 <= L <= 16.5 and 2.0 <= W <= 4.0 and 1.8 <= top < 3.3 and fill > 0.55:     # container (a bus-sized one reads as a bus)
        put_lib("container_6" if L < 9 else "container_12" if L < 13.5 else "container_15", c, yaw, L, W, top, gz); counts["lib"] += 1
    elif on_way and 13.0 <= L <= 17.5 and 2.0 <= W <= 4.0 and 3.3 <= top < 4.8 and fill > 0.55:    # semi trailer
        put_lib("semi_trailer", c, yaw, L, W, top, gz); counts["lib"] += 1
    elif near_rail(c) and top >= 2.5 and (L >= 10 or m.sum() * F * F >= 40):
        counts["skip"] += 1; continue                                     # an unsurveyed rail blob: add it to specs/rail_cars.yaml
    elif on_way and top < 2.4 and 1.4 <= W <= 3.2 and L > 5.9:          # cars end to end
        nslot = max(2, round(L / 4.8))
        for j in range(nslot): put_asset("car", c + ax * (j - (nslot - 1) / 2) * L / nslot, yaw, L / nslot - 0.3, gz)
        counts["row"] += 1
    elif on_way and top < 2.4 and 3.6 <= W <= 6.0 and L >= 3.6:         # cars side by side
        nslot = max(2, round(L / 2.7))
        for j in range(nslot): put_asset("car", c + ax * (j - (nslot - 1) / 2) * L / nslot, yaw + 90, min(W, 5.0), gz)
        counts["row"] += 1
    elif on_way and top >= 1.4 and L >= 4.5 and W <= 4.5 and top <= 4.0:  # anything else vehicle-shaped: fitted box
        put_proxy(c, yaw, L, W, top, gz); counts["proxy"] += 1
    else:
        counts["skip"] += 1; continue                                     # clutter, crates, dumpsters, not a vehicle
    polys.append(cv2.boxPoints(((float(c[0]), float(c[1])), (L + 0.6, W + 0.6), float(yaw))))   # world coords; box orientation only matters loosely
emit_rail()
for car in yaml.safe_load(open(Path(__file__).resolve().parent / "specs/cars.yaml")):   # hand-surveyed road vehicles
    c = np.array(car["at"], float); gz = float(DTM[int((Y1 - c[1]) / RES), int((c[0] - X0) / RES)])
    if "lib" in car:
        put_lib(car["lib"], c, float(car["yaw"]), *car["size"], gz, exact=True); counts["lib"] += 1
        polys.append(cv2.boxPoints(((float(c[0]), float(c[1])), (car["size"][0] + 1.0, car["size"][1] + 1.0), float(car["yaw"])))); continue
    name = car["asset"]
    p = st.DefinePrim(f"/vehicles/survey_{car['id']}", "Xform"); p.GetReferences().AddReference(f"./assets/cars/{name}/{car_usd(name).name}")
    xf = UsdGeom.Xformable(p); xf.AddTranslateOp().Set(Gf.Vec3d(float(c[0]), float(c[1]), gz)); xf.AddRotateZOp().Set(float(car["yaw"]))
    UsdPhysics.CollisionAPI.Apply(p); counts["asset"] += 1
    polys.append(cv2.boxPoints(((float(c[0]), float(c[1])), (csize[name][0] + 1.0, 3.0), float(car["yaw"]))))
    if "tint" in car:
        cs = Usd.Stage.Open(str(car_usd(name)))                        # (kept open: its prims die with the stage)
        texs = [q.GetPath() for q in cs.Traverse() if q.IsA(UsdShade.Shader) and UsdShade.Shader(q).GetIdAttr().Get() == "UsdUVTexture"]
        for tp in texs:
            t = UsdShade.Shader(st.OverridePrim(p.GetPath().AppendPath(tp.MakeRelativePath(cs.GetDefaultPrim().GetPath()))))
            t.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(car["tint"], car["tint"], car["tint"], 1))
for pr in yaml.safe_load(open(Path(__file__).resolve().parent / "specs/props.yaml")):   # library props (water tower, ...)
    aL, aW, aH = LIB[pr["asset"]]["size_m"]; u = pr["height"] / aH; c = np.array(pr["at"], float)
    gz = float(DTM[int((Y1 - c[1]) / RES), int((c[0] - X0) / RES)])
    put_lib(pr["asset"], c, pr["yaw"], aL * u, aW * u, pr["height"], gz)
    polys.append(cv2.boxPoints(((float(c[0]), float(c[1])), (aL * u + 1.5, aW * u + 1.5), float(pr["yaw"]))))
st.Save()
np.savez(R / "replaced.npz", polys=np.array(polys, np.float32) if polys else np.zeros((0, 4, 2), np.float32))
print(f"vehicles: {counts['asset']} cars/vans/buses, {counts['lib']} trucks/RVs/trailers/containers, {counts['row']} rows split into cars, "
      f"{counts['rail']} rail cars, {counts['proxy']} fitted boxes left; {counts['skip']} small / non-vehicle blobs skipped")

# ------------------------------------------------------------------ debris
# visual detail on the rubble fields the drones never filmed (R02, R03): pieces
# from the standalone pack, resting on the tile mound (which stays the collider),
# random yaw and up to 25 deg tilt. Instanceable references, class `rubble`.
PIECES = sorted(p.name for p in (A / "debris").iterdir() if p.is_dir() and not p.name.startswith("lump"))
FIELDS = {"R03": 18.0}                                          # label -> disc radius, m (R02 is rubble_pile.py's)
lab = {l["id"]: l for l in labels}
pts = []
for fid, rad in FIELDS.items():
    cx, cy = lab[fid]["at"]; n_ = int(np.pi * rad * rad / 4)
    a_, r_ = rng.uniform(0, 2 * np.pi, n_), rad * np.sqrt(rng.uniform(0, 1, n_))
    pts.append(np.c_[cx + r_ * np.cos(a_), cy + r_ * np.sin(a_)])
pts = np.concatenate(pts)
from shapely.geometry import Point, Polygon as _Poly                      # not on a building standing in the field (B35 in R03)
_bld = [_Poly(r).buffer(1.0) for b in yaml.safe_load(open(R / "buildings_lod1.yaml"))["buildings"] for lv in b["levels"] for r in lv["rings"]]
pts = pts[[not any(g.contains(Point(p)) for g in _bld) for p in pts]]
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

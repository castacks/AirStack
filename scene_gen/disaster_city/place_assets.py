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
Vehicles: blobs of the `vehicle` class, re-measured on a 0.25 m heightmap and
fitted with a rectangle; car / van / bus sized ones get an asset, long rectangular
ones (rail cars, trailers, containers) and compact rectangular trucks / dumpsters a box proxy coloured from the ortho, and
anything else keeps its tile mesh. Cars within 15 m of a
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
CARS = {"car": ["lowpoly_sedan", "lowpoly_wagon", "lowpoly_coupe", "red_car", "lowpoly_pickup"],
        "wreck": ["burned_car_01", "burned_car_02"], "van": ["delivery_van"], "bus": ["citybus"]}
def car_usd(name): return next((A / "cars" / name).glob("*.usd*"))
csize = {n: size_of(car_usd(n))[0] for v in CARS.values() for n in v}
labels = yaml.safe_load(open(LABELS))["labels"]
wrecks = np.array([l["at"] for l in labels if any(w in l["name"] for w in ("wreck", "collapsed", "rubble", "derail"))])

t = np.load(R / "tiles_site.npz"); scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(o3d.core.Tensor(t["verts"].astype(np.float32)), o3d.core.Tensor(t["faces"].astype(np.uint32)))
n, cc, stats, _ = cv2.connectedComponentsWithStats(cv2.dilate((CLS == 3).astype(np.uint8), np.ones((3, 3), np.uint8)))
st, root = new_stage(R / "vehicles.usd", "/vehicles"); label(root.GetPrim(), "vehicle")
placed, kept, polys, proxies = 0, 0, [], 0
geo = json.load(open(R / "ortho_site.json")); ortho = cv2.imread(str(R / "ortho_site.png"))
F = 0.25
for c in range(1, n):
    x, y, w, hgt = stats[c, :4]
    cx0, cy1 = X0 + (x - 1) * RES, Y1 - (y - 1) * RES
    W_, H_ = int((w + 2) * RES / F), int((hgt + 2) * RES / F)
    g = np.meshgrid(cx0 + (np.arange(W_) + 0.5) * F, cy1 - (np.arange(H_) + 0.5) * F)
    ray = np.stack([g[0], g[1], np.full_like(g[0], 500), 0 * g[0], 0 * g[0], -np.ones_like(g[0])], -1).astype(np.float32)
    z = 500 - scene.cast_rays(o3d.core.Tensor(ray))["t_hit"].numpy()
    ground = DTM[np.clip(((Y1 - g[1]) / RES).astype(int), 0, N - 1), np.clip(((g[0] - X0) / RES).astype(int), 0, N - 1)]
    # a touching row (RVs, buses) reads as one blob at a low cut; cutting higher splits it --
    # try 0.6 m first, and re-cut any blob that fits no vehicle at 1.6 m
    cuts = [((z - ground) > 0.6, True)]
    for cut, first in cuts:
      recut = np.zeros_like(cut)
      m = cv2.morphologyEx(cut.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
      k2, cc2, st2, _ = cv2.connectedComponentsWithStats(m)
      for b in range(1, k2):
          if st2[b, cv2.CC_STAT_AREA] * F * F < 3: continue
          (u, v), (a, bb), ang = cv2.minAreaRect(cv2.findNonZero((cc2 == b).astype(np.uint8)))
          L, Wd = max(a, bb) * F, min(a, bb) * F
          yaw = ang if a >= bb else ang + 90
          top = np.percentile((z - ground)[cc2 == b], 90)
          kind = ("car" if 3.4 <= L <= 5.8 and 1.4 <= Wd <= 2.5 else "van" if 5.8 < L <= 7.8 and Wd <= 2.8
                  else "bus" if 9 <= L <= 13.5 and 2.2 <= Wd <= 3.3 and top > 2.3 else None)
          if kind is None:
              if first and L > 6: recut |= ((z - ground) > 1.6) & (cc2 == b)   # re-cut this blob higher
              elif (3.0 <= L <= 20 and 2.0 <= Wd <= 4.0 and top < 5 and st2[b, cv2.CC_STAT_AREA] / max(a * bb, 1) > 0.8
                    or L >= 6 and 2.0 <= Wd <= 4.0 and st2[b, cv2.CC_STAT_AREA] / max(a * bb, 1) > 0.65):
                  # rail car / trailer / container: a fitted box, coloured from the ortho, beats melted tile mesh
                  wx, wy = cx0 + (u + 0.5) * F, cy1 - (v + 0.5) * F
                  gz = float(np.median(ground[cc2 == b]))
                  oc = ortho[int((geo["y1"] - wy) / geo["m_per_px"]), int((wx - geo["x0"]) / geo["m_per_px"])][::-1] / 255.0
                  xb = UsdGeom.Xform.Define(st, f"/vehicles/proxy_{placed:03d}")
                  xb.AddTranslateOp().Set(Gf.Vec3d(float(wx), float(wy), float(gz + top / 2))); xb.AddRotateZOp().Set(float(-yaw))
                  xb.AddScaleOp().Set(Gf.Vec3f(float(L / 2), float(Wd / 2), float(top) / 2))
                  cube = UsdGeom.Cube.Define(st, xb.GetPath().AppendChild("box"))
                  cube.CreateDisplayColorAttr([tuple(float(c_) ** 2.2 for c_ in oc)]); UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                  xb.GetPrim().SetCustomDataByKey("fit", {"L": round(L, 2), "W": round(Wd, 2), "top": round(float(top), 2), "proxy": True})
                  polys.append(cv2.boxPoints(((wx, wy), (L + 0.6, Wd + 0.6), -yaw))); placed += 1; proxies += 1
              else:
                  kept += 1
                  if DEBUG: print(f"    kept L {L:.1f} W {Wd:.1f} fill {st2[b, cv2.CC_STAT_AREA] / max(a * bb, 1):.2f} top {top:.1f} at {cx0 + (u + 0.5) * F:.0f},{cy1 - (v + 0.5) * F:.0f}")
              continue
          wx, wy = cx0 + (u + 0.5) * F, cy1 - (v + 0.5) * F
          if kind == "car" and len(wrecks) and np.hypot(*(wrecks - [wx, wy]).T).min() < 15: kind = "wreck"
          name = CARS[kind][rng.integers(len(CARS[kind]))]
          sc = float(np.clip(L / csize[name][0], 0.85, 1.15))
          gz = float(np.median(ground[cc2 == b]))
          p = st.DefinePrim(f"/vehicles/{kind}_{placed:03d}", "Xform"); p.GetReferences().AddReference(f"./assets/cars/{name}/{car_usd(name).name}")
          xf = UsdGeom.Xformable(p); xf.AddTranslateOp().Set(Gf.Vec3d(wx, wy, gz))
          xf.AddRotateZOp().Set(float(-yaw + rng.choice([0, 180]))); xf.AddScaleOp().Set(Gf.Vec3f(sc))
          UsdPhysics.CollisionAPI.Apply(p)
          p.SetCustomDataByKey("fit", {"L": round(L, 2), "W": round(Wd, 2), "top": round(float(top), 2)})
          c_ = cv2.boxPoints(((wx, wy), (L + 0.6, Wd + 0.6), -yaw))
          polys.append(c_); placed += 1
      if recut.any(): cuts.append((recut, False))
st.Save()
np.savez(R / "replaced.npz", polys=np.array(polys, np.float32) if polys else np.zeros((0, 4, 2), np.float32))
print(f"vehicles: {placed - proxies} assets + {proxies} box proxies (rail cars, trailers), {kept} blobs kept as tile mesh")

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

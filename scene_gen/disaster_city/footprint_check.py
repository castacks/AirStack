"""Does every placed model stand where the 3D tiles say the thing is?

    ~/.venvs/recon/bin/python footprint_check.py [--only B01,PAD]

For every placed object -- hero buildings (B01, S01, B03, B06), the industrial
pad's members (grouped by name prefix: S03_, S05_, cabin0_, mast_, ...), LOD1
boxes, vehicles -- rasterise its plan footprint at 0.25 m and compare it with
the tile mesh's RAISED footprint (surface > 1.2 m above bare earth; 0.5 m for vehicles) around it:
the connected raised blobs it touches, clipped to 4 m around the model (a
building's blob otherwise runs on into the pile or trees it touches), minus
cells another placed object covers and green (canopy) cells.

  iou        overlap of the two footprints
  off_m      centroid offset
  dyaw       difference of the long-axis directions (deg, 0-90; only when both are elongated)
  area_x     model area / tile area

Writes data/recon/footprints.tsv (flagged rows: iou < 0.6, off_m > 1.5 or
dyaw > 12) and data/recon/footprints/<name>.jpg: the ortho with the tile blob
(yellow) and the model (red) for every flagged object.
"""
import argparse, json
from pathlib import Path
import cv2, numpy as np, open3d as o3d
from pxr import Usd, UsdGeom
from _paths import R

ap = argparse.ArgumentParser(); ap.add_argument("--only", default="")
a = ap.parse_args(); only = set(filter(None, a.only.split(",")))
RES = 0.25
geo = json.load(open(R / "ortho_site.json")); ortho = cv2.imread(str(R / "ortho_site.png"))
X0, Y1 = geo["x0"], geo["y1"]; N = int(ortho.shape[1] * geo["m_per_px"] / RES)
to_px = lambda xy: np.c_[(xy[:, 0] - X0) / RES, (Y1 - xy[:, 1]) / RES]

# tile raised mask at RES
t = np.load(R / "tiles_site.npz"); sc = o3d.t.geometry.RaycastingScene()
sc.add_triangles(o3d.core.Tensor(t["verts"].astype(np.float32)), o3d.core.Tensor(t["faces"].astype(np.uint32)))
g = X0 + (np.arange(N) + .5) * RES; X, Y = np.meshgrid(g, Y1 - (np.arange(N) + .5) * RES)
DSM = 500 - sc.cast_rays(o3d.core.Tensor(np.stack([X, Y, np.full_like(X, 500), 0 * X, 0 * X, -np.ones_like(X)], -1).astype(np.float32)))["t_hit"].numpy()
rs = np.load(R / "site_rasters.npz")
DTM = cv2.resize(rs["dtm"], (N, N), interpolation=cv2.INTER_LINEAR)
HGT = np.nan_to_num(DSM - DTM)
def raised_at(cut):
    r_ = HGT > cut; return r_, cv2.connectedComponents(r_.astype(np.uint8))[1]
RAISED = {"building": raised_at(1.2), "vehicle": raised_at(0.5)}   # a car is ~1.4 m: a 1.2 m cut keeps only its roof

stage = Usd.Stage.Open(str(R / "disaster_city.usda")); xc = UsdGeom.XformCache(); bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])

def fill_prim(mask, p):
    """Rasterise one gprim's plan footprint into mask."""
    M = xc.GetLocalToWorldTransform(p)
    if p.IsA(UsdGeom.Cube):
        c = np.array([M.Transform((x, y, z)) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    elif p.IsA(UsdGeom.Mesh):
        c = np.array([M.Transform(v) for v in UsdGeom.Mesh(p).GetPointsAttr().Get()])
    elif not p.IsA(UsdGeom.Gprim):                         # a referenced asset: hull of its meshes' actual points
        pts = [np.array([xc.GetLocalToWorldTransform(q).Transform(v) for v in (UsdGeom.Mesh(q).GetPointsAttr().Get() or [])[::7]])
               for q in Usd.PrimRange(p, Usd.TraverseInstanceProxies()) if q.IsA(UsdGeom.Mesh)]
        pts = [x for x in pts if len(x)]
        if not pts: return
        c = np.concatenate(pts)                            # (BBoxCache gave these assets an axis-aligned box)
    else:                                                  # cylinder / sphere: its own bound, then its xform
        b = bc.ComputeUntransformedBound(p); r = b.GetRange()   # (a world bound is axis-aligned once rotated)
        if r.IsEmpty(): return
        lo, hi = r.GetMin(), r.GetMax(); Mb = b.GetMatrix() * M
        c = np.array([Mb.Transform((x, y, z)) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    cv2.fillConvexPoly(mask, cv2.convexHull(np.round(to_px(c[:, :2]) * 4).astype(np.int32)), 1, shift=2)

def objects():
    for hid in ("B01", "S01", "B03", "B06"):
        p = stage.GetPrimAtPath(f"/World/{hid}")
        if p: yield hid, [q for part in p.GetChildren() if part.GetName() not in ("lights", "materials")
                          for q in Usd.PrimRange(part) if q.IsA(UsdGeom.Gprim)]
    pad = stage.GetPrimAtPath("/World/PAD")
    if pad:
        groups = {}
        for part in pad.GetChildren():
            if part.GetName() in ("lights", "materials"): continue
            groups.setdefault(part.GetName().split("_")[0], []).extend(q for q in Usd.PrimRange(part) if q.IsA(UsdGeom.Gprim))
        for k, v in groups.items(): yield f"PAD_{k}", v
    for p in stage.GetPrimAtPath("/World/lod1").GetChildren():
        if p.IsActive(): yield p.GetName().split("_")[0], [q for q in Usd.PrimRange(p) if q.IsA(UsdGeom.Gprim)]
    for p in stage.GetPrimAtPath("/World/vehicles").GetChildren():
        yield f"veh_{p.GetName()}", [p]

def axis(mask):
    pts = cv2.findNonZero(mask.astype(np.uint8))
    if pts is None or len(pts) < 8: return None, 1
    (_, _), (w, h), ang = cv2.minAreaRect(pts)
    return (ang if w >= h else ang + 90) % 180, max(w, h) / max(min(w, h), 1)

og = ortho[np.clip(((Y1 - Y) / geo["m_per_px"]).astype(int), 0, ortho.shape[0] - 1),
           np.clip(((X - X0) / geo["m_per_px"]).astype(int), 0, ortho.shape[1] - 1)].astype(int)
green = cv2.dilate(((2 * og[..., 1] - og[..., 0] - og[..., 2]) > 12).astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
foot = {}
for name, prims in objects():
    F = np.zeros((N, N), np.uint8)
    for q in prims: fill_prim(F, q)
    if F.any(): foot[name] = F.astype(bool)
occupied = np.zeros((N, N), np.uint16)
for F in foot.values(): occupied += F
rows = []; outdir = R / "footprints"; outdir.mkdir(exist_ok=True)
for name, F in foot.items():
    if only and not any(name.startswith(o) for o in only): continue
    others = cv2.dilate(((occupied - F) > 0).astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    raised, blobs = RAISED["vehicle" if name.startswith("veh_") else "building"]
    touch = np.unique(blobs[cv2.dilate(F.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool) & raised])
    near = cv2.dilate(F.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * int(4 / RES) + 1,) * 2)).astype(bool)
    T = np.isin(blobs, touch[touch > 0]) & near & ~others & ~green   # clipped: not a neighbour's, not canopy
    iou = (F & T).sum() / max((F | T).sum(), 1)
    cf, ct = np.argwhere(F).mean(0), (np.argwhere(T).mean(0) if T.any() else np.argwhere(F).mean(0))
    off = np.hypot(*(cf - ct)) * RES
    af, ef = axis(F); at, et = axis(T)
    dyaw = abs((af - at + 90) % 180 - 90) if af is not None and at is not None and ef > 1.4 and et > 1.4 else 0.0
    flag = iou < 0.6 or off > 1.5 or dyaw > 12
    rows.append((name, iou, off, dyaw, F.sum() / max(T.sum(), 1), flag))
    if flag:
        ys, xs = np.nonzero(F | T); pad = 40
        y0, y1, x0, x1 = max(ys.min() - pad, 0), min(ys.max() + pad, N), max(xs.min() - pad, 0), min(xs.max() + pad, N)
        s = geo["m_per_px"] / RES                           # ortho px per mask px
        img = cv2.resize(ortho[int(y0 / s):int(y1 / s), int(x0 / s):int(x1 / s)], (x1 - x0, y1 - y0)) // 2
        ov = img.copy(); ov[T[y0:y1, x0:x1]] = (0, 220, 255); ov[F[y0:y1, x0:x1]] = (0, 0, 255)
        ov[(F & T)[y0:y1, x0:x1]] = (0, 140, 255)
        cv2.imwrite(str(outdir / f"{name}.jpg"), cv2.resize(np.hstack([img * 2, cv2.addWeighted(img * 2, 0.4, ov, 0.6, 0)]), None, fx=1.5, fy=1.5))

with open(R / "footprints.tsv", "w") as f:
    f.write("name\tiou\toff_m\tdyaw\tarea_x\tflag\n")
    for r in rows: f.write(f"{r[0]}\t{r[1]:.2f}\t{r[2]:.1f}\t{r[3]:.0f}\t{r[4]:.2f}\t{'FLAG' if r[5] else ''}\n")
bad = [r for r in rows if r[5]]
print(f"{len(rows)} objects, {len(bad)} flagged (iou < 0.6, off > 1.5 m or dyaw > 12 deg):")
for r in sorted(bad, key=lambda r: r[1]): print(f"  {r[0]:24s} iou {r[1]:.2f}  off {r[2]:4.1f} m  dyaw {r[3]:3.0f}  area x{r[4]:.2f}")

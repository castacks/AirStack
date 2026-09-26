"""List the raw tile pieces still standing in the scene -- what no model has replaced yet.

    ~/.venvs/recon/bin/python leftovers.py [--min-h 1.2] [--min-area 3]

Reads site_ground.usd's classified tile meshes (building / vehicle / clutter / rubble; vegetation is
left out, trees replace it), welds each into connected pieces, and prints every piece taller than
`min-h` above bare earth with a footprint over `min-area` m2: class, centre, extent, height, triangle
count and the nearest LABELS feature -- biggest first. Writes data/recon/leftovers.jpg, an ortho crop
of each, for identifying them.
"""
import argparse, json
import cv2, numpy as np, yaml
from pxr import Usd, UsdGeom
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from _paths import R, LABELS

ap = argparse.ArgumentParser(); ap.add_argument("--min-h", type=float, default=1.2); ap.add_argument("--min-area", type=float, default=3.0)
a = ap.parse_args()
rs = np.load(R / "site_rasters.npz"); X0, Y1, RES = float(rs["x0"]), float(rs["y1"]), float(rs["res"])
labels = yaml.safe_load(open(LABELS))["labels"]; L = np.array([l["at"] for l in labels])
s = Usd.Stage.Open(str(R / "site_ground.usd")); rows = []
for cls in ("tiles_building", "tiles_vehicle", "tiles_clutter", "tiles_rubble"):
    p = s.GetPrimAtPath(f"/site/{cls}")
    if not p: continue
    m = UsdGeom.Mesh(p); V = np.array(m.GetPointsAttr().Get()); F = np.array(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
    _, inv = np.unique(np.round(V / 0.05).astype(np.int64), axis=0, return_inverse=True); Fw = inv.ravel()[F]   # weld texture seams
    e = np.r_[Fw[:, [0, 1]], Fw[:, [1, 2]]]; n = inv.max() + 1
    k, lab = connected_components(coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n)), directed=False)
    fl = lab[Fw[:, 0]]
    for c in range(k):
        f = fl == c
        if f.sum() < 20: continue
        P = V[np.unique(F[f])]; lo, hi = P.min(0), P.max(0); ctr = (lo + hi) / 2
        iy, ix = int((Y1 - ctr[1]) / RES), int((ctr[0] - X0) / RES)
        if not (0 <= iy < rs["dtm"].shape[0] and 0 <= ix < rs["dtm"].shape[1]): continue
        h = hi[2] - rs["dtm"][iy, ix]; area = (hi[0] - lo[0]) * (hi[1] - lo[1])
        if h < a.min_h or area < a.min_area: continue
        j = np.hypot(*(L - ctr[:2]).T).argmin()
        rows.append({"class": cls[6:], "at": [round(float(ctr[0]), 1), round(float(ctr[1]), 1)], "size": [round(float(hi[0] - lo[0]), 1), round(float(hi[1] - lo[1]), 1)],
                     "h": round(float(h), 1), "tris": int(f.sum()), "near": labels[j]["id"], "dist": round(float(np.hypot(*(L[j] - ctr[:2]))))})
rows.sort(key=lambda r: -r["size"][0] * r["size"][1] * r["h"])
for i, r in enumerate(rows): print(i, r)
print(len(rows), "leftover pieces")
geo = json.load(open(R / "ortho_site.json")); o = cv2.imread(str(R / "ortho_site.png")); mp = geo["m_per_px"]; tiles = []
for i, r in enumerate(rows[:40]):
    w = max(12.0, 1.4 * max(r["size"])); x0, y1 = r["at"][0] - w / 2, r["at"][1] + w / 2
    cr = o[max(0, int((geo["y1"] - y1) / mp)):max(0, int((geo["y1"] - y1 + w) / mp)), max(0, int((x0 - geo["x0"]) / mp)):max(0, int((x0 + w - geo["x0"]) / mp))]
    cr = cv2.resize(cr, (240, 240)) if cr.size else np.zeros((240, 240, 3), np.uint8)
    cv2.putText(cr, f"{i} {r['class']} {r['at'][0]:.0f},{r['at'][1]:.0f} h{r['h']}", (4, 16), 0, 0.45, (0, 255, 255), 2); tiles.append(cr)
if tiles:
    tiles += [np.zeros_like(tiles[0])] * (-len(tiles) % 8)
    cv2.imwrite(str(R / "leftovers.jpg"), np.concatenate([np.concatenate(tiles[i:i + 8], 1) for i in range(0, len(tiles), 8)], 0))

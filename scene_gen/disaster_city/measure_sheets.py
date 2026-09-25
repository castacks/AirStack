"""Metric measuring sheets of an aligned building cloud, for writing a build_hero spec.

    ~/.venvs/recon/bin/python measure_sheets.py data/recon/b01 B01 [--pad 4]

Local frame: the building's tile footprint (its LOD1 box in
data/recon/buildings_lod1.yaml): origin at the box's min corner, x along the
long side, y across, z up from the box's ground; the cloud may refine the yaw
by up to 5 deg. So a spec measured on these sheets lands on the tile footprint. Writes to <dir>/sheets/:
  plan_z<h>.png   points in a 0.6 m slab at height h (walls + openings in plan)
  elev_{xlo,xhi,ylo,yhi}.png  each half of the building seen side-on
all with a 1 m grid (bold every 5 m) and axis labels in metres, plus frame.json.
"""
import argparse, json
from pathlib import Path
import cv2, numpy as np, open3d as o3d, yaml

ap = argparse.ArgumentParser(); ap.add_argument("dir"); ap.add_argument("id"); ap.add_argument("--pad", type=float, default=4)
ap.add_argument("--yaw", type=float, help="force the frame yaw (deg), e.g. fitted to a wall line")
a = ap.parse_args(); d = Path(a.dir); out = d / "sheets"; out.mkdir(exist_ok=True)
from _paths import R as RECON, LABELS   # R is taken (grid res / rotation)
b = next(x for x in yaml.safe_load(open(RECON / "buildings_lod1.yaml"))["buildings"] if x["id"] == a.id)

pcd = o3d.io.read_point_cloud(str(d / "dense/fused.ply"))
pcd.transform(np.array(json.load(open(d / "to_world.json"))["recon_to_world"]))
P, C = np.asarray(pcd.points), (np.asarray(pcd.colors) * 255).astype(np.uint8)[:, ::-1]
c = np.array(b["at"]); L = max(b["size_m"]) / 2 + a.pad
P0 = P[np.hypot(*(P[:, :2] - c).T) < L + 5]; C = C[np.hypot(*(P[:, :2] - c).T) < L + 5]

# Frame = the building's footprint in the 3D tiles (its LOD1 box): origin at the box corner,
# x along the long side, y across. Anchoring here -- not on the cloud -- is what makes a spec
# measured on these sheets land on the tile footprint. (The cloud used to set the frame, and
# while the cloud was misaligned that frame, and B01, sat 13 m off.) The yaw is refined by
# the wall-histogram peak only if it stays within 5 deg of the tile box.
g = b["ground_z"]; th0 = np.radians(b["yaw_deg"])
band = (P0[:, 2] > g + 1) & (P0[:, 2] < g + 8) & (np.hypot(*(P0[:, :2] - c).T) < max(b["size_m"]) / 2 + 1)
B = P0[band, :2] - c
def peakiness(t):
    r = B @ np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    return sum((np.histogram(r[:, i], bins=np.arange(-30, 30, 0.2))[0].astype(float) ** 2).sum() for i in (0, 1))
ts = th0 + np.radians(np.arange(-5, 5.01, 0.25)); yaw = ts[np.argmax([peakiness(t) for t in ts])]
if b["size_m"][1] > b["size_m"][0]: yaw += np.pi / 2                              # x = long side
if a.yaw is not None: yaw = np.radians(a.yaw)
sx, sy = max(b["size_m"]), min(b["size_m"])
R = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
lo = np.array([-sx / 2, -sy / 2])
Q = np.c_[(P0[:, :2] - c) @ R.T - lo, P0[:, 2] - g]
size = np.array([sx, sy])
frame = {"origin_world": (c + np.linalg.inv(R) @ lo).tolist() + [g], "yaw_deg": float(np.degrees(yaw)),
         "size_m": size.tolist(), "note": "local x along long side, y across, z up from ground_z"}
json.dump(frame, open(out / "frame.json", "w"), indent=1)
print(f"yaw {np.degrees(yaw):.1f} deg, footprint {size[0]:.1f} x {size[1]:.1f} m, origin {np.round(frame['origin_world'], 2)}")

PX = 40                                                  # px per metre
def sheet(uv, col, x0, x1, y0, y1, name, xl, yl):
    W, H = int((x1 - x0) * PX), int((y1 - y0) * PX)
    img = np.full((H + 40, W + 60, 3), 30, np.uint8)
    u = ((uv[:, 0] - x0) * PX).astype(int) + 60; v = ((y1 - uv[:, 1]) * PX).astype(int)
    k = (u >= 60) & (u < W + 60) & (v >= 0) & (v < H); img[v[k], u[k]] = col[k]
    for m in range(int(np.ceil(x0)), int(x1) + 1):
        uu = int((m - x0) * PX) + 60; cv2.line(img, (uu, 0), (uu, H), (90, 90, 90) if m % 5 else (0, 200, 255), 1)
        if m % 5 == 0: cv2.putText(img, str(m), (uu - 8, H + 25), 0, 0.6, (0, 200, 255), 2)
    for m in range(int(np.ceil(y0)), int(y1) + 1):
        vv = int((y1 - m) * PX); cv2.line(img, (60, vv), (W + 60, vv), (90, 90, 90) if m % 5 else (0, 200, 255), 1)
        if m % 5 == 0: cv2.putText(img, str(m), (5, vv + 6), 0, 0.6, (0, 200, 255), 2)
    cv2.putText(img, f"{name}   x: {xl}   y: {yl}   (m)", (70, 22), 0, 0.7, (255, 255, 255), 2)
    cv2.imwrite(str(out / f"{name}.png"), img)

x0, x1, y0, y1 = -3.0, size[0] + 3.0, -3.0, size[1] + 3.0      # the tile footprint + 3 m
for zc in (1.2, 3.0, 5.5, 7.5):
    k = abs(Q[:, 2] - zc) < 0.3
    sheet(Q[k, :2], C[k], x0, x1, y0, y1, f"plan_z{zc}", "local x", "local y")
# elevations: every point inside the window, seen along -y (from y0 side) and along +x (from x1 side)
inwin = (Q[:, 0] > x0) & (Q[:, 0] < x1) & (Q[:, 1] > y0) & (Q[:, 1] < y1)
for name, ax, keep in (("elev_ylo", 0, Q[:, 1] < (y0 + y1) / 2), ("elev_yhi", 0, Q[:, 1] >= (y0 + y1) / 2),
                       ("elev_xlo", 1, Q[:, 0] < (x0 + x1) / 2), ("elev_xhi", 1, Q[:, 0] >= (x0 + x1) / 2)):
    k = inwin & keep
    lo_, hi_ = (x0, x1) if ax == 0 else (y0, y1)
    sheet(np.c_[Q[k, ax], Q[k, 2]], C[k], lo_, hi_, -1, 12, name, f"local {'xy'[ax]}", "z above ground")
json.dump({**frame, "window": [x0, x1, y0, y1]}, open(out / "frame.json", "w"), indent=1)
print(f"wrote {out}")

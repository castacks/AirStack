"""Put a COLMAP reconstruction into blend world coordinates.

    ~/.venvs/recon/bin/python recon_align.py data/recon/rubble_west data/recon/tiles_R01.npz
        -> writes <dir>/recon_ortho.jpg + <dir>/recon_height.jpg (gridded, pick points on them)
    ~/.venvs/recon/bin/python recon_align.py data/recon/rubble_west data/recon/tiles_R01.npz \\
        --pairs 861.7,316.7:48.5,-396.5 716.7,560:32.2,-414

1. gravity: the dominant plane in the dense cloud is the ground; its normal is up.
2. XY similarity from >= 2 hand-picked pairs `u,v:x,y` -- a pixel of
   recon_ortho.jpg / recon_height.jpg and the world XY of the same thing (read
   off LABELS_map.png or a tile heightmap). Automatic routes all failed here:
   SIFT on the orthos (different capture dates), height NCC (noisy, latched onto
   the wrong place) and FPFH+RANSAC (ambiguous at every scale). Building corners
   are the reliable picks.
3. Z from the tile mesh under the cloud; rigid ICP refines, scale stays from 2.
Writes <dir>/to_world.json and <dir>/align_check.jpg (Google ortho | cloud over it).
"""
import argparse, json, sys
from pathlib import Path
import cv2, numpy as np, open3d as o3d, pycolmap

ap = argparse.ArgumentParser(); ap.add_argument("dir"); ap.add_argument("tiles"); ap.add_argument("--pairs", nargs="*")
a = ap.parse_args(); d = Path(a.dir)
from _paths import R, LABELS
geo = json.load(open(R / "ortho_site.json"))
N = 1500                                                    # ortho size, px

pcd = o3d.io.read_point_cloud(str(d / "dense/fused.ply"))
P, C = np.asarray(pcd.points), np.asarray(pcd.colors)
rec = pycolmap.Reconstruction(str(max((d / "sparse").iterdir(), key=lambda p: pycolmap.Reconstruction(p).num_reg_images())))
cams = np.array([im.projection_center() for im in rec.images.values()])

# 1. gravity
ext = np.linalg.norm(np.percentile(P, 95, 0) - np.percentile(P, 5, 0))
(pa, pb, pc, pd), _ = pcd.segment_plane(0.004 * ext, 3, 2000)
up = np.array([pa, pb, pc]) / np.linalg.norm([pa, pb, pc])
if np.mean(cams @ up + pd) < 0: up = -up
x = np.cross([0, 1, 0], up); x /= np.linalg.norm(x)
R0 = np.stack([x, np.cross(up, x), up])                     # recon -> gravity frame
G = P @ R0.T
Gc = cams @ R0.T
lo, hi = Gc[:, :2].min(0), Gc[:, :2].max(0)
lo, hi = lo - 0.5 * (hi - lo).max(), hi + 0.5 * (hi - lo).max()
px = (hi - lo).max() / N
to_g = lambda u, v: np.array([lo[0] + u * px, hi[1] - v * px])

def grid(img, step=50):
    for q in range(0, N, step):
        cv2.line(img, (q, 0), (q, N), (255, 255, 255), 1); cv2.line(img, (0, q), (N, q), (255, 255, 255), 1)
        if q % (2 * step) == 0:
            cv2.putText(img, str(q), (q + 2, 14), 0, 0.45, (0, 255, 255), 1); cv2.putText(img, str(q), (2, q - 2), 0, 0.45, (255, 255, 0), 1)
    return img

u = ((G[:, 0] - lo[0]) / px).astype(int); v = ((hi[1] - G[:, 1]) / px).astype(int)
ok = (u >= 0) & (v >= 0) & (u < N) & (v < N)
o = np.argsort(G[ok, 2])                                    # higher z written last
rgb = np.zeros((N, N, 3), np.uint8); rgb[v[ok][o], u[ok][o]] = (C[ok][o][:, ::-1] * 255).astype(np.uint8)
hz = np.full((N, N), np.nan); hz[v[ok][o], u[ok][o]] = G[ok][o][:, 2]
m = np.isfinite(hz)
z0, z1 = np.nanpercentile(hz, [2, 99.5])
hc = cv2.applyColorMap(np.clip(np.nan_to_num((hz - z0) / (z1 - z0)) * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO); hc[~m] = 0
cv2.imwrite(str(d / "recon_ortho.jpg"), grid(rgb)); cv2.imwrite(str(d / "recon_height.jpg"), grid(hc))
json.dump({"R0": R0.tolist(), "lo": lo.tolist(), "hi": hi.tolist(), "px": px}, open(d / "ortho_frame.json", "w"))
if not a.pairs: sys.exit(f"wrote {d}/recon_ortho.jpg and recon_height.jpg -- pick --pairs on them")

# 2. XY similarity (least squares, 2D Umeyama)
src = np.array([to_g(*map(float, p.split(":")[0].split(","))) for p in a.pairs])
dst = np.array([list(map(float, p.split(":")[1].split(","))) for p in a.pairs])
ms, md = src.mean(0), dst.mean(0); S, D = src - ms, dst - md
U, sig, Vt = np.linalg.svd(D.T @ S)
Rxy = U @ np.diag([1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
s = sig.sum() / (S ** 2).sum()
res = np.linalg.norm((s * S @ Rxy.T + md) - dst, axis=1)
print(f"similarity: scale {s:.3f} m/unit, yaw {np.degrees(np.arctan2(Rxy[1, 0], Rxy[0, 0])):.1f} deg, pair residuals {res.round(2)} m")
T = np.eye(4); T[:2, :3] = s * Rxy @ R0[:2]; T[2, :3] = s * R0[2]; T[:2, 3] = md - s * Rxy @ ms

# 3. Z from the tiles, then rigid ICP near the picked points
t = np.load(a.tiles)
tiles = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(t["verts"].astype(float)), o3d.utility.Vector3iVector(t["faces"]))
scene = o3d.t.geometry.RaycastingScene(); scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(tiles))
W = P @ T[:3, :3].T + T[:3, 3]
near = np.linalg.norm(W[:, :2] - md, axis=1) < 60
Q = W[near][::20]
hit = scene.cast_rays(o3d.core.Tensor(np.hstack([Q[:, :2], np.full((len(Q), 1), 500), np.tile([0, 0, -1], (len(Q), 1))]).astype(np.float32)))["t_hit"].numpy()
f = np.isfinite(hit); T[2, 3] = np.median((500 - hit[f]) - Q[f, 2])
W = P @ T[:3, :3].T + T[:3, 3]
src_p = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(W[near])).voxel_down_sample(0.25)
tgt = tiles.sample_points_uniformly(1_000_000)
for thr in (2.0, 1.0, 0.5):
    r = o3d.pipelines.registration.registration_icp(src_p, tgt, thr, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(), o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
    src_p.transform(r.transformation); T = r.transformation @ T
    print(f"ICP {thr} m: fitness {r.fitness:.2f}, rmse {r.inlier_rmse:.2f} m")
json.dump({"recon_to_world": T.tolist(), "scale": s, "pairs": a.pairs, "icp_rmse_m": r.inlier_rmse, "icp_fitness": r.fitness},
          open(d / "to_world.json", "w"), indent=1)

# check: Google ortho | cloud drawn over it, 90 m window round the picks
site = cv2.imread(str(R / "ortho_site.png"))
W = P @ T[:3, :3].T + T[:3, 3]
uu = ((W[:, 0] - geo["x0"]) / geo["m_per_px"]).astype(int); vv = ((geo["y1"] - W[:, 1]) / geo["m_per_px"]).astype(int)
k = (uu >= 0) & (vv >= 0) & (uu < site.shape[1]) & (vv < site.shape[0])
o = np.argsort(W[k, 2]); chk = site.copy(); chk[vv[k][o], uu[k][o]] = (C[k][o][:, ::-1] * 255).astype(np.uint8)
cu, cv_ = int((md[0] - geo["x0"]) / geo["m_per_px"]), int((geo["y1"] - md[1]) / geo["m_per_px"]); h = int(45 / geo["m_per_px"])
cv2.imwrite(str(d / "align_check.jpg"), np.hstack([site[cv_ - h:cv_ + h, cu - h:cu + h], chk[cv_ - h:cv_ + h, cu - h:cu + h]]))

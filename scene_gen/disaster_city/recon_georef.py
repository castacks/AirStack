"""Georeference a reconstruction by render-and-compare against the Google tiles.

    ~/.venvs/recon/bin/python recon_georef.py data/recon/b01 [--model sparse_ext] [--views 24] [--iters 3]

Replaces the 2-corner `recon_align.py --pairs` + ICP route, which left B01 and
R01 ~10 m off: ICP against flat ground cannot fix a horizontal shift, and the
corner picks were wrong. Per iteration:
 1. pick `views` registered frames, spread over the flight (farthest-point on
    camera centres), and put each camera in the world with the current
    <dir>/to_world.json;
 2. render the tile mesh from exactly those cameras (render_views.py);
 3. LoFTR-match each undistorted frame to its render (fundamental-matrix
    RANSAC; SIFT found nothing across the photo / tile-texture gap), ray-cast the matched render pixels into the tile mesh -> 3D
    world points, PnP-RANSAC -> that camera's true world pose;
 4. robust similarity (Umeyama inside RANSAC on camera centres) recon -> world.
Stops when the update moves the cameras < 0.2 m (median). Writes
<dir>/to_world.json (the old one is kept as to_world_prev.json) and
<dir>/georef/ (renders, match images, a log).
"""
import argparse, json, shutil, subprocess
from pathlib import Path
import cv2, numpy as np, open3d as o3d, pycolmap
from _paths import CODE, DATA, R

ap = argparse.ArgumentParser(); ap.add_argument("dir"); ap.add_argument("--model")
ap.add_argument("--views", type=int, default=24); ap.add_argument("--iters", type=int, default=3)
a = ap.parse_args(); d = Path(a.dir); gdir = d / "georef"; gdir.mkdir(exist_ok=True)
model = d / a.model if a.model else max((d / "sparse").iterdir(), key=lambda p: pycolmap.Reconstruction(p).num_reg_images())
rec = pycolmap.Reconstruction(str(model))
t = np.load(R / "tiles_site.npz"); scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(o3d.core.Tensor(t["verts"].astype(np.float32)), o3d.core.Tensor(t["faces"].astype(np.uint32)))
import torch
from kornia.feature import LoFTR
loftr = LoFTR(pretrained="outdoor").eval().cuda()          # SIFT found nothing: photo vs 2-year-older tile texture

def match(img, ren, side=640):
    """LoFTR matches between two BGR images of equal size; returns pixel coords in each."""
    h, w = img.shape[:2]; k = side / max(h, w); sz = (int(w * k) // 8 * 8, int(h * k) // 8 * 8)
    g = lambda im: torch.from_numpy(cv2.cvtColor(cv2.resize(im, sz), cv2.COLOR_BGR2GRAY)).float()[None, None].cuda() / 255
    with torch.inference_mode(): r = loftr({"image0": g(img), "image1": g(ren)})
    c = r["confidence"].cpu().numpy() > 0.4
    sx, sy = w / sz[0], h / sz[1]
    return r["keypoints0"].cpu().numpy()[c] * [sx, sy], r["keypoints1"].cpu().numpy()[c] * [sx, sy]

ims = list(rec.images.values())
C_r = np.array([im.projection_center() for im in ims])
pick, dist = [0], np.linalg.norm(C_r - C_r[0], axis=1)                 # farthest-point sampling
while len(pick) < min(a.views, len(ims)):
    pick.append(int(dist.argmax())); dist = np.minimum(dist, np.linalg.norm(C_r - C_r[pick[-1]], axis=1))

def umeyama(A, B):
    ma, mb = A.mean(0), B.mean(0); U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]); Rm = U @ D @ Vt
    s = (S * np.diag(D)).sum() / ((A - ma) ** 2).sum(); T = np.eye(4); T[:3, :3] = s * Rm; T[:3, 3] = mb - s * Rm @ ma
    return T

if not (d / "to_world_prev.json").exists(): shutil.copy(d / "to_world.json", d / "to_world_prev.json")
T = np.array(json.load(open(d / "to_world.json"))["recon_to_world"])
log = open(gdir / "log.txt", "w")
for it in range(a.iters):
    s_ = np.cbrt(np.linalg.det(T[:3, :3])); Rs = T[:3, :3] / s_
    views = []
    for i in pick:
        im = ims[i]; cam = rec.cameras[im.camera_id]
        fx, fy, cx, cy = cam.params[:4]; k = 1014 / cam.width if cam.width > 1100 else 1.0
        Rcw = im.cam_from_world().rotation.matrix()
        views.append({"name": f"v{i:04d}", "img": im.name, "R_wc": (Rs @ Rcw.T).tolist(), "C": (T[:3, :3] @ C_r[i] + T[:3, 3]).tolist(),
                      "fx": fx * k, "fy": fy * k, "cx": cx * k, "cy": cy * k, "w": round(cam.width * k), "h": round(cam.height * k),
                      "dist": list(cam.params[4:8]), "K0": [fx, fy, cx, cy], "wh0": [cam.width, cam.height]})
    json.dump(views, open(gdir / "views.json", "w"))
    subprocess.run(["blender", "-b", str(DATA / "blender_data/disaster_city.blend"), "--python", str(CODE / "render_views.py"), "--",
                    str(gdir / "views.json"), str(gdir)], check=True, capture_output=True)
    pairs = []
    for v in views:
        img = cv2.imread(str(d / "images" / v["img"])); fx0, fy0, cx0, cy0 = v["K0"]
        img = cv2.undistort(img, np.array([[fx0, 0, cx0], [0, fy0, cy0], [0, 0, 1]]), np.array(v["dist"]))
        img = cv2.resize(img, (v["w"], v["h"]))
        ren = cv2.imread(str(gdir / f"{v['name']}.png"))
        p1, p2 = match(img, ren); p1, p2 = p1.astype(np.float32), p2.astype(np.float32)
        if len(p1) < 20: continue
        _, inl = cv2.findFundamentalMat(p1, p2, cv2.FM_RANSAC, 3.0, 0.999)
        if inl is None: continue
        p1, p2 = p1[inl.ravel() > 0], p2[inl.ravel() > 0]
        # render pixel -> world point on the tile mesh
        K = np.array([[v["fx"], 0, v["cx"]], [0, v["fy"], v["cy"]], [0, 0, 1]]); Rwc = np.array(v["R_wc"]); C = np.array(v["C"])
        dirs = (Rwc @ np.linalg.solve(K, np.c_[p2, np.ones(len(p2))].T)).T; dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        hit = scene.cast_rays(o3d.core.Tensor(np.c_[np.tile(C, (len(p2), 1)), dirs].astype(np.float32)))["t_hit"].numpy()
        f = np.isfinite(hit); X = C + dirs[f] * hit[f, None]; q = p1[f]
        if len(X) < 15: continue
        ok, rvec, tvec, pin = cv2.solvePnPRansac(X, q, K, None, reprojectionError=4.0, iterationsCount=2000, confidence=0.999)
        if not ok or pin is None or len(pin) < 15: continue
        Rn = cv2.Rodrigues(rvec)[0]; Cn = (-Rn.T @ tvec).ravel()
        pairs.append((C_r[int(v["name"][1:])], Cn, len(pin)))
        vis = cv2.drawMatches(img, [cv2.KeyPoint(*p, 3) for p in q[pin.ravel()][:60]], ren, [cv2.KeyPoint(*p, 3) for p in p2[f][pin.ravel()][:60]],
                              [cv2.DMatch(j, j, 0) for j in range(min(60, len(pin)))], None)
        cv2.imwrite(str(gdir / f"match_{v['name']}.jpg"), cv2.resize(vis, None, fx=0.5, fy=0.5))
    print(f"iter {it}: {len(pairs)}/{len(views)} views solved by PnP", file=log, flush=True)
    print(f"iter {it}: {len(pairs)}/{len(views)} views solved by PnP")
    if len(pairs) < 6: print("  too few views -- keeping the current transform"); break
    A = np.array([p[0] for p in pairs]); B = np.array([p[1] for p in pairs])
    best = None; rng = np.random.default_rng(0)
    for _ in range(2000):
        idx = rng.choice(len(A), 4, replace=False); Tm = umeyama(A[idx], B[idx])
        res = np.linalg.norm((A @ Tm[:3, :3].T + Tm[:3, 3]) - B, axis=1); inl = res < 2.0
        if best is None or inl.sum() > best[1].sum(): best = (Tm, inl)
    Tn = umeyama(A[best[1]], B[best[1]])
    res = np.linalg.norm((A @ Tn[:3, :3].T + Tn[:3, 3]) - B, axis=1)
    move = np.median(np.linalg.norm((C_r @ Tn[:3, :3].T + Tn[:3, 3]) - (C_r @ T[:3, :3].T + T[:3, 3]), axis=1))
    msg = (f"  {best[1].sum()}/{len(A)} inliers, residual median {np.median(res[best[1]]):.2f} m, scale {np.cbrt(np.linalg.det(Tn[:3, :3])):.3f} "
           f"(was {s_:.3f}), cameras moved {move:.2f} m")
    print(msg); print(msg, file=log, flush=True)
    T = Tn
    json.dump({"recon_to_world": T.tolist(), "method": "render-and-compare vs Google tiles (recon_georef.py)",
               "inliers": int(best[1].sum()), "residual_m": float(np.median(res[best[1]]))}, open(d / "to_world.json", "w"), indent=1)
    if move < 0.2: break

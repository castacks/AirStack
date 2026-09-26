"""Bake a hero building's photo atlas: project the georeferenced drone frames onto its faces.

    ~/.venvs/recon/bin/python photo_bake.py B01 b01 rubble_west      # -> data/recon/b01/B01_photo.png

build_hero.py (spec `photo: {mats: [...]}`) packs every face of those parts into one atlas and
writes <ID>_atlas.json (face rectangles, building frame -> world). Here every texel is put in
the world and coloured from the frames of the named recons (each with its to_world.json):
projected through the camera's OPENCV lens, kept where the texel faces the camera and the
building itself does not hide it (depth map ray-cast from the hero mesh), and blended with
weight (cos(incidence) / distance)^2, sharpened (^P) so the best view wins without a hard seam.
Texels no frame sees get the part's tileable library texture (materials/<mat>.png) or,
without one, the building's median colour. Writes <ID>_photo.png and <ID>_photo_seen.png (coverage).
"""
import json, sys
import cv2, numpy as np, open3d as o3d, pycolmap, torch
import torch.nn.functional as F
from pxr import Usd, UsdGeom
from _paths import R

P_SHARP, MAX_D, IMG_W = 6, 45.0, 1024
bid, recons = sys.argv[1], sys.argv[2:]
d = R / bid.lower(); A = json.load(open(d / f"{bid}_atlas.json")); SIDE = A["side"]; M = np.array(A["local_to_world"])
dev = "cuda"

# ---- texels: world position + normal for every atlas pixel of every face (with its padding) ----
PAD = 3
pts, nrm, pix, fid, mst = [], [], [], [], []
for fi, f in enumerate(A["faces"]):
    x0, y0, w, h = f["px"]; o, u, v, n = (np.array(f[k]) for k in ("o", "u", "v", "n"))
    xs, ys = np.arange(x0 - PAD, x0 + w + PAD), np.arange(y0 - PAD, y0 + h + PAD)
    gx, gy = np.meshgrid(xs, ys)
    a = np.clip((gx - x0 + 0.5) / w, 0, 1) * f["size"][0]; b = np.clip(1 - (gy - y0 + 0.5) / h, 0, 1) * f["size"][1]
    X = o + a[..., None] * u + b[..., None] * v + 0.005 * n          # 5 mm off the face: no self-occlusion
    pts.append(X.reshape(-1, 3)); nrm.append(np.broadcast_to(n, X.shape).reshape(-1, 3)); pix.append(np.c_[gy.ravel(), gx.ravel()])
    fid.append(np.full(X.shape[0] * X.shape[1], fi)); mst.append(np.c_[a.ravel() + f["st0"][0], b.ravel() + f["st0"][1]])
pts = np.concatenate(pts) @ M[:3, :3].T + M[:3, 3]; nrm = np.concatenate(nrm) @ M[:3, :3].T; pix = np.concatenate(pix)
fid, mst = np.concatenate(fid), np.concatenate(mst)
print(f"{len(pts) / 1e6:.1f} M texels")

# ---- occluder: the hero mesh itself, in world ----
st = Usd.Stage.Open(str(d / f"{bid}.usd")); V, T = [], []
for p in st.Traverse():
    if not p.IsA(UsdGeom.Mesh): continue
    g = UsdGeom.Mesh(p); X = np.array(g.GetPointsAttr().Get()); W = np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)).T
    X = X @ W[:3, :3].T + W[:3, 3]; k = len(V) and sum(len(v) for v in V)
    q = np.array(g.GetFaceVertexIndicesAttr().Get()).reshape(-1, 4)
    V.append(X); T.append(np.r_[q[:, [0, 1, 2]], q[:, [0, 2, 3]]] + k)
scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(o3d.core.Tensor(np.concatenate(V).astype(np.float32)), o3d.core.Tensor(np.concatenate(T).astype(np.uint32)))

# openness per face: 3x3 rays along its normal (fanned 30 deg); outside if most leave the building within 40 m
open_ = np.zeros(len(A["faces"]))
for fi, f in enumerate(A["faces"]):
    o, u, v, n = (M[:3, :3] @ np.array(f[k]) for k in ("o", "u", "v", "n")); o = o + M[:3, 3]
    if n[2] < -0.3: continue
    rays = []
    for a in (0.2, 0.5, 0.8):
        for b in (0.2, 0.5, 0.8):
            for tilt in (u, v):
                dvec = n + 0.5 * np.tan(np.radians(30)) * tilt * (a - 0.5) * 2
                rays.append([*(o + u * a * f["size"][0] + v * b * f["size"][1] + 0.02 * n), *dvec / np.linalg.norm(dvec)])
    hit = scene.cast_rays(o3d.core.Tensor(np.array(rays, np.float32)))["t_hit"].numpy()
    open_[fi] = float((hit > 40).mean() >= 0.5)
print(f"{int(open_.sum())}/{len(open_)} faces open to the outside")

# ---- the real surroundings: every recon's fused dense cloud, in world (occluders the model lacks) ----
cloud = []
for sub in recons:
    f = R / sub / "dense/fused.ply"
    if f.exists():
        pc = o3d.io.read_point_cloud(str(f)); pc.transform(np.array(json.load(open(R / sub / "to_world.json"))["recon_to_world"]))
        cloud.append(np.asarray(pc.points))
cloud = np.concatenate(cloud); cloud = cloud[np.linalg.norm(cloud - pts.mean(0), axis=1) < 60]
CLOUD = torch.tensor(cloud, device=dev, dtype=torch.float32); print(f"{len(cloud) / 1e6:.1f} M cloud points as occluders")

# ---- cameras ----
cams = []
for sub in recons:
    rd = R / sub; Tw = np.array(json.load(open(rd / "to_world.json"))["recon_to_world"])
    s = np.cbrt(np.linalg.det(Tw[:3, :3])); Rs = Tw[:3, :3] / s
    mdl = rd / "sparse_ext" if (rd / "sparse_ext").exists() else max((rd / "sparse").iterdir(), key=lambda p: pycolmap.Reconstruction(p).num_reg_images())
    rec = pycolmap.Reconstruction(str(mdl))
    for im in rec.images.values():
        c = rec.cameras[im.camera_id]; k = IMG_W / c.width if c.width > IMG_W else 1.0
        Rcw = im.cam_from_world().rotation.matrix()
        cams.append({"img": rd / "images" / im.name, "R": Rcw @ Rs.T, "C": Tw[:3, :3] @ im.projection_center() + Tw[:3, 3],
                     "K": np.array(c.params[:4]) * k, "dist": np.array(c.params[4:8]), "wh": (round(c.width * k), round(c.height * k))})
ctr = pts.mean(0)
cams = [c for c in cams if np.linalg.norm(c["C"] - ctr) < MAX_D + 20]
print(f"{len(cams)} cameras from {', '.join(recons)}")

def project(c, X):
    """world (N,3) torch -> pixel (N,2), depth (N,) through the OPENCV lens"""
    Xc = (X - torch.tensor(c["C"], device=dev, dtype=torch.float32)) @ torch.tensor(c["R"].T, device=dev, dtype=torch.float32)
    z = Xc[:, 2]; x, y = Xc[:, 0] / z, Xc[:, 1] / z
    k1, k2, p1, p2 = c["dist"]; r2 = x * x + y * y; rad = 1 + k1 * r2 + k2 * r2 * r2
    xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x); yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    fx, fy, cx, cy = c["K"]
    return torch.stack([fx * xd + cx, fy * yd + cy], 1), z, r2

# depth maps (the hero mesh from each camera, quarter resolution, pinhole on the undistorted grid is
# close enough for an occlusion test with a tolerance)
DS = 4
for c in cams:
    W, H = c["wh"]; fx, fy, cx, cy = c["K"]
    u, v = np.meshgrid((np.arange(W // DS) + 0.5) * DS, (np.arange(H // DS) + 0.5) * DS)
    pts_n = cv2.undistortPoints(np.c_[u.ravel(), v.ravel()].astype(np.float32)[:, None], np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]), c["dist"]).reshape(-1, 2)
    dirs = np.c_[pts_n, np.ones(len(pts_n))] @ c["R"]                  # camera -> world directions (unnormalised, z=1)
    hit = scene.cast_rays(o3d.core.Tensor(np.c_[np.tile(c["C"], (len(dirs), 1)), dirs].astype(np.float32)))["t_hit"].numpy()
    c["r2max"] = float((pts_n ** 2).sum(1).max()) * 1.05
    # cloud depth: splat the points (z-buffer), then a 3x3 min to close the gaps between them
    uvc, zc, r2c = project(c, CLOUD); q = (zc > 0.3) & (r2c < c["r2max"])
    iu, iv = (uvc[q, 0] / DS).long(), (uvc[q, 1] / DS).long(); inb = (iu >= 0) & (iu < W // DS) & (iv >= 0) & (iv < H // DS)
    dc = torch.full(((H // DS) * (W // DS),), 1e4, device=dev)
    dc.scatter_reduce_(0, iv[inb] * (W // DS) + iu[inb], zc[q][inb], reduce="amin")
    dc = -F.max_pool2d(-dc.view(1, 1, H // DS, W // DS), 3, 1, 1)
    c["depth"] = torch.minimum(torch.tensor(np.minimum(hit, 1e4).reshape(H // DS, W // DS), device=dev, dtype=torch.float32)[None, None], dc)

out = torch.zeros(len(pts), 3, device=dev); wsum = torch.zeros(len(pts), device=dev); qbest = torch.zeros(len(pts), device=dev)
Xt = torch.tensor(pts, device=dev, dtype=torch.float32); Nt = torch.tensor(nrm, device=dev, dtype=torch.float32)
for i, c in enumerate(cams):
    img = cv2.imread(str(c["img"]))
    img = cv2.resize(img, c["wh"], interpolation=cv2.INTER_AREA) if img.shape[1] != c["wh"][0] else img
    It = torch.tensor(img[..., ::-1].copy(), device=dev, dtype=torch.float32).permute(2, 0, 1)[None] / 255
    W, H = c["wh"]
    for s0 in range(0, len(pts), 4_000_000):
        X, N = Xt[s0:s0 + 4_000_000], Nt[s0:s0 + 4_000_000]
        uv, z, r2 = project(c, X)
        to_cam = torch.tensor(c["C"], device=dev, dtype=torch.float32) - X; dist = to_cam.norm(dim=1)
        cos = (to_cam * N).sum(1) / dist
        edge = torch.clamp(torch.minimum(torch.minimum(uv[:, 0], W - uv[:, 0]), torch.minimum(uv[:, 1], H - uv[:, 1])) / (0.08 * W), 0, 1)
        ok = (z > 0.3) & (cos > 0.15) & (dist < MAX_D) & (edge > 0) & (r2 < c["r2max"])   # r2: the lens model folds back outside the FOV
        if not ok.any(): continue
        g = torch.stack([uv[:, 0] / W * 2 - 1, uv[:, 1] / H * 2 - 1], 1)[None, :, None]
        dz = F.grid_sample(c["depth"], g, mode="nearest", align_corners=False)[0, 0, :, 0]
        ok &= z < dz + 0.25 + 0.03 * z                                # not hidden (the building, or anything in the cloud)
        q = torch.where(ok, (3 * cos / torch.clamp(dist, min=3.0)) ** 2 * edge, torch.zeros_like(dist))
        w = q ** P_SHARP; qbest[s0:s0 + 4_000_000] = torch.maximum(qbest[s0:s0 + 4_000_000], q)
        col = F.grid_sample(It, g, mode="bilinear", align_corners=False)[0, :, :, 0].T
        out[s0:s0 + 4_000_000] += col * w[:, None]; wsum[s0:s0 + 4_000_000] += w
    if i % 100 == 0: print(f"  {i}/{len(cams)}", flush=True)

# how much to trust the photo, per texel: the best view's quality (0.06 ~ square-on from 10 m); a face
# the video barely covers takes the tileable texture over its whole area rather than in patches
rgb = (out / torch.clamp(wsum, min=1e-30)[:, None]).cpu().numpy(); q = qbest.cpu().numpy()
# only faces that open onto the outside and do not face down: ceilings and room-side faces are seen
# through openings from few angles, with nothing in the cloud to occlude what really stands in front
alpha = np.clip(q / 0.06, 0, 1) * open_[fid]
cover = np.bincount(fid, alpha, len(A["faces"])) / np.maximum(np.bincount(fid, minlength=len(A["faces"])), 1)
alpha[cover[fid] < 0.35] = 0
LIB = json.load(open(R / "materials/materials.json"))
fill = np.zeros_like(rgb)
for mat in {f["mat"] for f in A["faces"]}:
    k = np.array([A["faces"][i]["mat"] == mat for i in range(len(A["faces"]))])[fid]
    tex = cv2.imread(str(R / "materials" / f"{mat}.png"))[..., ::-1].astype(np.float32) / 255; n = tex.shape[0]; tile = LIB[mat]["tile_m"]
    fill[k] = tex[(-mst[k, 1] / tile * n).astype(int) % n, (mst[k, 0] / tile * n).astype(int) % n]
img = np.zeros((SIDE, SIDE, 3), np.float32); A_ = np.zeros((SIDE, SIDE), np.float32)
img[pix[:, 0], pix[:, 1]] = fill; A_[pix[:, 0], pix[:, 1]] = alpha
A_ = cv2.GaussianBlur(A_, (0, 0), 3); ph = np.zeros_like(img); ph[pix[:, 0], pix[:, 1]] = rgb
img = img * (1 - A_[..., None]) + ph * A_[..., None]
print(f"photo on {(alpha > 0.5).mean() * 100:.0f}% of texels ({(cover > 0.35).sum()}/{len(cover)} faces)")
cv2.imwrite(str(d / f"{bid}_photo.png"), (np.clip(img, 0, 1) * 255).astype(np.uint8)[..., ::-1])
cv2.imwrite(str(d / f"{bid}_photo_seen.png"), (A_ * 255).astype(np.uint8))
print("->", d / f"{bid}_photo.png")

"""Find the people in the drone video and put them on the map -> data/recon/people/people.json.

    ~/.venvs/recon/bin/python find_people.py b01 rubble_west

Every registered frame of the named (georeferenced) recons goes through torchvision's
keypoint R-CNN. A person's ground point is the bottom-middle of the box for an upright one and
the middle of the box for one lying down (head and ankles cast to the ground a body-length apart
and level);
the ray through it (OPENCV lens undone) is cast into the tile mesh + the hero models, so a
person on a deck or inside B01 lands on that floor. Detections are clustered in world XY
(1.5 m); lying or sitting clusters seen twice are kept (a casualty may be filmed only a few times);
standing ones only if seen >= 20 s apart (someone who stayed -- people walking about scatter into
one-time clusters and are dropped). Posture per cluster = the majority of its detections;
yaw of a lying one from its head -> ankle direction on the ground. Writes people.json and
people/review.jpg (one crop per cluster, to check by eye).
"""
import json, sys
import cv2, numpy as np, open3d as o3d, pycolmap, torch, torchvision
from pxr import Usd, UsdGeom
from _paths import R

OUT = R / "people"; OUT.mkdir(exist_ok=True)
det = torchvision.models.detection.keypointrcnn_resnet50_fpn(weights="DEFAULT").eval().cuda()

# raycast target: the tiles + every hero's meshes (floors, decks)
t = np.load(R / "tiles_site.npz"); scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(o3d.core.Tensor(t["verts"].astype(np.float32)), o3d.core.Tensor(t["faces"].astype(np.uint32)))
for usd in R.glob("*/[BSP]*[0-9A-Z].usd"):
    st = Usd.Stage.Open(str(usd)); V, F_ = [], []
    for p in st.Traverse():
        if not p.IsA(UsdGeom.Mesh): continue
        g = UsdGeom.Mesh(p); X = np.array(g.GetPointsAttr().Get(), float); W = np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)).T
        cnt = np.array(g.GetFaceVertexCountsAttr().Get())
        if len(X) == 0 or not (cnt == 4).all(): continue
        q = np.array(g.GetFaceVertexIndicesAttr().Get()).reshape(-1, 4) + sum(len(v) for v in V)
        V.append(X @ W[:3, :3].T + W[:3, 3]); F_.append(np.r_[q[:, [0, 1, 2]], q[:, [0, 2, 3]]])
    if V: scene.add_triangles(o3d.core.Tensor(np.concatenate(V).astype(np.float32)), o3d.core.Tensor(np.concatenate(F_).astype(np.uint32)))

def ground(cam, px):
    """pixel(s) -> world point on the first surface along the camera ray"""
    fx, fy, cx, cy = cam["K"]
    n = cv2.undistortPoints(np.float32(px).reshape(-1, 1, 2), np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]), cam["dist"]).reshape(-1, 2)
    d = np.c_[n, np.ones(len(n))] @ cam["R"]; d /= np.linalg.norm(d, axis=1, keepdims=True)
    h = scene.cast_rays(o3d.core.Tensor(np.c_[np.tile(cam["C"], (len(d), 1)), d].astype(np.float32)))["t_hit"].numpy()
    return cam["C"] + d * h[:, None], h

hits = []
for sub in sys.argv[1:]:
    rd = R / sub; Tw = np.array(json.load(open(rd / "to_world.json"))["recon_to_world"]); s = np.cbrt(np.linalg.det(Tw[:3, :3])); Rs = Tw[:3, :3] / s
    mdl = rd / "sparse_ext" if (rd / "sparse_ext").exists() else max((rd / "sparse").iterdir(), key=lambda p: pycolmap.Reconstruction(p).num_reg_images())
    rec = pycolmap.Reconstruction(str(mdl))
    for k, im in enumerate(sorted(rec.images.values(), key=lambda im: im.name)):
        c = rec.cameras[im.camera_id]
        cam = {"K": np.array(c.params[:4]), "dist": np.array(c.params[4:8]), "R": im.cam_from_world().rotation.matrix() @ Rs.T,
               "C": Tw[:3, :3] @ im.projection_center() + Tw[:3, 3]}
        img = cv2.imread(str(rd / "images" / im.name))
        with torch.inference_mode():
            o = det([torch.from_numpy(img[..., ::-1].copy()).permute(2, 0, 1).float().cuda() / 255])[0]
        for b, sc, kp in zip(o["boxes"].cpu().numpy(), o["scores"].cpu().numpy(), o["keypoints"].cpu().numpy()):
            if sc < 0.85: continue
            x0, y0, x1, y1 = b; w, h = x1 - x0, y1 - y0
            if h < 25 and w < 25: continue                                     # too small to say anything
            head, ank = kp[:5, :2].mean(0), kp[15:17, :2].mean(0); hip = kp[11:13, :2].mean(0)
            # posture from the GROUND, not the image (a rolled camera tips everyone over): head and ankles
            # cast onto the surface land a body-length apart and level for someone lying; for someone
            # upright the head's ray runs on far past the feet
            P, dist = ground(cam, [[(x0 + x1) / 2, (y0 + y1) / 2], head, ank, [(x0 + x1) / 2, y1], hip])
            if not np.isfinite(dist).all() or dist[0] > 60: continue
            span = np.linalg.norm((P[1] - P[2])[:2]); level = abs(P[1, 2] - P[2, 2])
            lying = 0.9 < span < 2.4 and level < 0.5 and kp[[0, 15, 16], 2].min() > 0
            sitting = not lying and np.linalg.norm((P[4] - P[2])[:2]) < 0.9 and abs(hip[1] - ank[1]) < 0.3 * h
            if not lying: P[0] = P[3]                                          # upright: stands where its feet are
            hits.append({"frame": f"{sub}/{im.name}", "t": im.name, "xyz": P[0].round(2).tolist(), "pose": "lying" if lying else "sitting" if sitting else "standing",
                         "yaw": float(np.degrees(np.arctan2(*(P[2] - P[1])[1::-1]))) if lying else None,
                         "box": [float(v) for v in b], "score": float(sc), "dist": float(dist[0])})
        if k % 100 == 0: print(f"{sub}: {k}/{rec.num_reg_images()} frames, {len(hits)} people so far", flush=True)

# cluster in world XY; keep people who stayed put
from track_drone import frame_time as tsec       # -> (video, seconds), for "seen >= 20 s apart"
X = np.array([h["xyz"][:2] for h in hits]); lab = -np.ones(len(X), int); n = 0
for i in np.argsort([-h["score"] for h in hits]):
    if lab[i] >= 0: continue
    near = (np.linalg.norm(X - X[i], axis=1) < 1.5) & (lab < 0); lab[near] = n; n += 1
people = []
for c in range(n):
    m = [h for h, l in zip(hits, lab) if l == c]
    ts = sorted(tsec(h["t"]) for h in m); spread = ts[-1][1] - ts[0][1] if ts[0][0] == ts[-1][0] else 999   # both videos: stayed
    poses = [h["pose"] for h in m]; pose = max(set(poses), key=poses.count)
    if not ((pose != "standing" and len(m) >= 2) or (len(m) >= 3 and spread >= 20)): continue
    yaws = [h["yaw"] for h in m if h["yaw"] is not None and h["pose"] == "lying"]
    best = max(m, key=lambda h: h["score"] * (h["box"][3] - h["box"][1]))
    people.append({"id": f"P{len(people) + 1:02d}", "xyz": np.median([h["xyz"] for h in m], axis=0).round(2).tolist(), "pose": pose,
                   "votes": {p: poses.count(p) for p in set(poses)}, "yaw": float(np.median(yaws)) if yaws else None,
                   "n": len(m), "best": {k: best[k] for k in ("frame", "box")}})
json.dump({"people": people, "detections": len(hits)}, open(OUT / "people.json", "w"), indent=1)
json.dump(hits, open(OUT / "detections.json", "w"))
tiles = []
for p in people:
    f, b = p["best"]["frame"], p["best"]["box"]; img = cv2.imread(str(R / f.split("/")[0] / "images" / "/".join(f.split("/")[1:])))
    cx, cy, r = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2, max(b[2] - b[0], b[3] - b[1]) * 1.2 + 20
    crop = img[max(0, int(cy - r)):int(cy + r), max(0, int(cx - r)):int(cx + r)]
    crop = cv2.resize(crop, (240, 240)); cv2.putText(crop, f"{p['id']} {p['pose']} n{p['n']}", (4, 18), 0, 0.55, (0, 255, 255), 2); tiles.append(crop)
if tiles:
    rows = [np.concatenate(tiles[i:i + 8] + [np.zeros_like(tiles[0])] * (8 - len(tiles[i:i + 8])), 1) for i in range(0, len(tiles), 8)]
    cv2.imwrite(str(OUT / "review.jpg"), np.concatenate(rows, 0))
print(f"{len(hits)} detections -> {len(people)} people who stayed put:", {p: sum(q['pose'] == p for q in people) for p in ('lying', 'sitting', 'standing')})

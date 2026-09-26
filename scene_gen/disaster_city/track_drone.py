"""Localize the drone over the whole of videos A and B, and draw the paths on the site map.

    ~/.venvs/recon/bin/python track_drone.py frames     # 1 fps of both raw videos -> data/recon/tracks/images/{A,B}/
    ~/.venvs/recon/bin/python track_drone.py sfm        # sequential-matched COLMAP -> tracks/sparse/<n>/
    ~/.venvs/recon/bin/python track_drone.py georef     # each model into the world -> tracks/poses.json
    ~/.venvs/recon/bin/python track_drone.py plot       # -> tracks/trajectories.png

Frame names are the second of the raw video (A_0123.jpg = A.mp4 at 123 s), so a pose is a time.
georef: each SfM model is put into the world by the recons that are already georeferenced
(b01, rubble_west): their frames came from the same videos, so a frame there and here at the
same time share a camera centre (init similarity). Then render-and-compare every few frames
against the Google tiles (render_views.py + LoFTR + PnP, as recon_georef.py) and correct the
SfM drift by the per-frame PnP offsets, interpolated in time. The segments are the named clips
(clips.tsv); time outside every clip is drawn unlabelled.
"""
import json, subprocess, sys
from pathlib import Path
import numpy as np
from _paths import CODE, DATA, R

T_DIR = R / "tracks"; IMG = T_DIR / "images"
CLIPS = DATA / "mocaps/clips"


def frames():
    import imageio_ffmpeg
    for v, vf in (("A", "fps=1,scale=2028:1520"), ("B", "fps=1")):
        (IMG / v).mkdir(parents=True, exist_ok=True)
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-nostdin", "-v", "error", "-i", str(DATA / f"mocaps/raw/{v}.mp4"),
                        "-vf", vf, "-q:v", "2", "-start_number", "0", str(IMG / v / f"{v}_%04d.jpg")], check=True)
        print(v, len(list((IMG / v).glob("*.jpg"))), "frames")


def sfm():
    import pycolmap
    db, sparse = T_DIR / "database.db", T_DIR / "sparse"
    db.unlink(missing_ok=True); sparse.mkdir(exist_ok=True)
    ext = pycolmap.FeatureExtractionOptions(max_image_size=2048); ext.sift.max_num_features = 8192
    pycolmap.extract_features(db, IMG, camera_mode=pycolmap.CameraMode.PER_FOLDER,
                              reader_options=pycolmap.ImageReaderOptions(camera_model="OPENCV"), extraction_options=ext)
    pair = pycolmap.SequentialPairingOptions(); pair.overlap = 12; pair.quadratic_overlap = True
    pycolmap.match_sequential(db, pairing_options=pair)
    opts = pycolmap.IncrementalPipelineOptions(); opts.ba_refine_principal_point = False
    models = pycolmap.incremental_mapping(db, IMG, sparse, options=opts)
    for i, m in sorted(models.items()):
        names = sorted(im.name for im in m.images.values())
        print(f"model {i}: {m.num_reg_images()} images {names[0]} .. {names[-1]}, reproj {m.compute_mean_reprojection_error():.2f}px")


def clip_table():
    rows = [l.split("\t") for l in (CLIPS / "clips.tsv").read_text().splitlines()[1:]]
    sec = lambda s: sum(float(x) * 60 ** i for i, x in enumerate(reversed(s.split(":"))))
    return {r[0]: (r[1].split("/")[-1][0], sec(r[2]), sec(r[3])) for r in rows}


def known_centres():
    """(video, second) -> world camera centre, from the georeferenced recons (frames at integer seconds only)."""
    import pycolmap
    start = {k[:3]: v[1] for k, v in clip_table().items()}
    window = {"B08h": 12, "B10h": 2, "B10k": 41}                      # frames.py's 4 fps windows
    out = {}
    for sub, model in (("rubble_west", None), ("b01", "sparse_ext")):
        d = R / sub; T = np.array(json.load(open(d / "to_world.json"))["recon_to_world"])
        m = d / model if model else max((d / "sparse").iterdir(), key=lambda p: pycolmap.Reconstruction(p).num_reg_images())
        for im in pycolmap.Reconstruction(str(m)).images.values():
            stem = im.name.split("/")[-1][:-4]; pre, idx = stem.split("_")
            fps, off = (4, window[pre]) if pre in window else ((2, 0) if len(idx) == 4 else (1, 0))
            t = start[pre[:3]] + off + (int(idx) - 1) / fps
            if abs(t - round(t)) < 0.01: out[(pre[0], round(t))] = T[:3, :3] @ im.projection_center() + T[:3, 3]
    return out


def umeyama(A, B):
    ma, mb = A.mean(0), B.mean(0); U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]); Rm = U @ D @ Vt
    s = (S * np.diag(D)).sum() / ((A - ma) ** 2).sum(); T = np.eye(4); T[:3, :3] = s * Rm; T[:3, 3] = mb - s * Rm @ ma
    return T


def ransac_sim(A, B, thr, n=3000):
    best = None; rng = np.random.default_rng(0)
    for _ in range(n):
        Tm = umeyama(*(X[rng.choice(len(A), 3, replace=False)] for X in (A, B)))
        inl = np.linalg.norm(A @ Tm[:3, :3].T + Tm[:3, 3] - B, axis=1) < thr
        if best is None or inl.sum() > best.sum(): best = inl
    return umeyama(A[best], B[best]), best


def georef(passes=3, every=3):
    """World pose per frame: SfM model -> similarity (init from known_centres) + time-interpolated PnP offsets."""
    import cv2, pycolmap, torch, open3d as o3d
    from kornia.feature import LoFTR
    t = np.load(R / "tiles_site.npz"); scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.core.Tensor(t["verts"].astype(np.float32)), o3d.core.Tensor(t["faces"].astype(np.uint32)))
    loftr = LoFTR(pretrained="outdoor").eval().cuda()
    known = known_centres(); print(len(known), "known centres from b01 / rubble_west")
    poses, rdir = {}, T_DIR / "renders"; rdir.mkdir(exist_ok=True)
    key = lambda im: (im.name[0], int(im.name[-8:-4]))
    todo = {md.name: pycolmap.Reconstruction(str(md)) for md in (T_DIR / "sparse").iterdir()}
    anchor = dict(known)                  # grows as models are placed: later models tie on to placed frames +-1 s
    def ties(rec):
        out = []
        for i, im in enumerate(sorted(rec.images.values(), key=lambda im: im.name)):
            v, s = key(im)
            if (v, s) in anchor: out.append((i, anchor[(v, s)]))
            elif (v, s - 1) in anchor and (v, s + 1) in anchor: out.append((i, (anchor[(v, s - 1)] + anchor[(v, s + 1)]) / 2))
        return out
    while todo:
        name, pr = max(((n, ties(r)) for n, r in todo.items()), key=lambda x: len(x[1]))
        if len(pr) < 5: break
        rec = todo.pop(name); md = T_DIR / "sparse" / name; ims = sorted(rec.images.values(), key=lambda im: im.name)
        C = np.array([im.projection_center() for im in ims]); tt = np.array([key(im)[1] for im in ims])
        tag = f"model {md.name} ({ims[0].name[2:-4]}..{ims[-1].name[2:-4]}, {len(ims)} frames)"
        T, inl = ransac_sim(C[[i for i, _ in pr]], np.array([w for _, w in pr]), 4.0)
        if inl.sum() < 5: print(tag, f": only {inl.sum()} consistent ties, not placed"); continue
        print(tag, f": init from {inl.sum()}/{len(pr)} tied frames, scale {np.cbrt(np.linalg.det(T[:3, :3])):.3f}")
        off = np.zeros_like(C)
        for p in range(passes):
            s_ = np.cbrt(np.linalg.det(T[:3, :3])); Rs = T[:3, :3] / s_
            Cw = C @ T[:3, :3].T + T[:3, 3] + off
            views = []
            for i in range(0, len(ims), every):
                im = ims[i]; cam = rec.cameras[im.camera_id]; fx, fy, cx, cy = cam.params[:4]; k = 1014 / cam.width if cam.width > 1100 else 1.0
                views.append({"name": f"m{md.name}_{i:04d}", "i": i, "R_wc": (Rs @ im.cam_from_world().rotation.matrix().T).tolist(), "C": Cw[i].tolist(),
                              "fx": fx * k, "fy": fy * k, "cx": cx * k, "cy": cy * k, "w": round(cam.width * k), "h": round(cam.height * k),
                              "dist": list(cam.params[4:8]), "K0": [fx, fy, cx, cy]})
            json.dump(views, open(rdir / "views.json", "w"))
            subprocess.run(["blender", "-b", str(DATA / "blender_data/disaster_city.blend"), "--python", str(CODE / "render_views.py"), "--",
                            str(rdir / "views.json"), str(rdir)], check=True, capture_output=True)
            got = {}
            for v in views:
                img = cv2.imread(str(IMG / ims[v["i"]].name)); fx0, fy0, cx0, cy0 = v["K0"]
                img = cv2.resize(cv2.undistort(img, np.array([[fx0, 0, cx0], [0, fy0, cy0], [0, 0, 1]]), np.array(v["dist"])), (v["w"], v["h"]))
                ren = cv2.imread(str(rdir / f"{v['name']}.png"))
                if ren is None or ren.mean() < 5: continue
                h, w = img.shape[:2]; sz = (640, int(640 * h / w) // 8 * 8)
                g = lambda x: torch.from_numpy(cv2.cvtColor(cv2.resize(x, sz), cv2.COLOR_BGR2GRAY)).float()[None, None].cuda() / 255
                with torch.inference_mode(): r = loftr({"image0": g(img), "image1": g(ren)})
                c = r["confidence"].cpu().numpy() > 0.4; sc = np.array([w / sz[0], h / sz[1]])
                p1, p2 = ((r[f"keypoints{j}"].cpu().numpy()[c] * sc).astype(np.float32) for j in (0, 1))
                if len(p1) < 30: continue
                _, fi = cv2.findFundamentalMat(p1, p2, cv2.FM_RANSAC, 3.0, 0.999)
                if fi is None: continue
                p1, p2 = p1[fi.ravel() > 0], p2[fi.ravel() > 0]
                K = np.array([[v["fx"], 0, v["cx"]], [0, v["fy"], v["cy"]], [0, 0, 1]]); Rwc = np.array(v["R_wc"]); Cv = np.array(v["C"])
                dirs = (Rwc @ np.linalg.solve(K, np.c_[p2, np.ones(len(p2))].T)).T; dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
                hit = scene.cast_rays(o3d.core.Tensor(np.c_[np.tile(Cv, (len(p2), 1)), dirs].astype(np.float32)))["t_hit"].numpy()
                f = np.isfinite(hit)
                if f.sum() < 20: continue
                ok, rv, tv, pin = cv2.solvePnPRansac(Cv + dirs[f] * hit[f, None], p1[f], K, None, reprojectionError=4.0, iterationsCount=2000)
                if ok and pin is not None and len(pin) >= 20:
                    Rn = cv2.Rodrigues(rv)[0]; got[v["i"]] = (-Rn.T @ tv).ravel()
            for f in rdir.glob("*.png"): f.unlink()
            if len(got) < 4: print(f"  pass {p}: {len(got)} PnP fixes, stop"); break
            idx = np.array(sorted(got)); P = np.array([got[i] for i in idx])
            T, inl = ransac_sim(C[idx], P, 8.0)                        # global similarity, then the drift that is left
            res = P - (C[idx] @ T[:3, :3].T + T[:3, 3]); good = inl & (np.linalg.norm(res, axis=1) < 15)
            # median-of-5 in time against single bad PnPs, then linear in time between fixes
            rs = np.array([np.median(res[good][max(0, j - 2):j + 3], axis=0) for j in range(good.sum())])
            off = np.stack([np.interp(tt, tt[idx[good]], rs[:, k]) for k in range(3)], 1)
            print(f"  pass {p}: {len(got)}/{len(views)} PnP fixes, {good.sum()} kept, drift |off| median {np.median(np.linalg.norm(rs, axis=1)):.1f} m, "
                  f"max {np.linalg.norm(rs, axis=1).max():.1f} m", flush=True)
        Cw = C @ T[:3, :3].T + T[:3, 3] + off
        fixed = set(idx[good].tolist()) if len(got) >= 4 else set()
        for i, im in enumerate(ims):
            v, s = key(im)
            if (v, s) not in poses or i in fixed: poses[(v, s)] = (Cw[i].round(2).tolist(), i in fixed, int(md.name))
            anchor.setdefault((v, s), Cw[i])
    for n, r in todo.items(): print(f"model {n} ({r.num_reg_images()} frames): not tied to any placed frame, not placed")
    json.dump({f"{v}_{s:04d}": {"C": c, "pnp": f, "model": m} for (v, s), (c, f, m) in sorted(poses.items())},
              open(T_DIR / "poses.json", "w"), indent=0)
    print(len(poses), "frames localized ->", T_DIR / "poses.json")


def plot(out=None):
    """Both videos' paths on LABELS_map.png, one colour per named clip; unlabelled time in grey."""
    import cv2
    out = Path(out or T_DIR); poses = json.load(open(T_DIR / "poses.json")); clips = clip_table()
    base = cv2.imread(str(DATA / "LABELS_map.png")); x0, y1, m = -205.0, -20.0, 0.125          # label_map.py's window
    P = np.array([p["C"][:2] for p in poses.values()]); lo, hi = P.min(0) - 40, P.max(0) + 40          # crop to the flights
    c0, r0 = int((lo[0] - x0) / m), int((y1 - hi[1]) / m); c1, r1 = int((hi[0] - x0) / m), int((y1 - lo[1]) / m)
    c0, r0 = max(c0, 0), max(r0, 0); base = base[r0:r1, c0:c1]; x0, y1 = x0 + c0 * m, y1 - r0 * m
    k = 2000 / max(base.shape[:2]); base = cv2.resize(base, None, fx=k, fy=k); base = cv2.addWeighted(base, 0.6, np.full_like(base, 255), 0.4, 0)
    px = lambda c: (int((c[0] - x0) / m * k), int((y1 - c[1]) / m * k))
    pal = [(230, 25, 75), (60, 180, 75), (255, 225, 25), (0, 130, 200), (245, 130, 48), (145, 30, 180), (70, 240, 240),
           (240, 50, 230), (210, 245, 60), (0, 128, 128), (170, 110, 40), (128, 0, 0)]
    for v in "AB":
        img = base.copy(); legend = []
        pts = sorted((int(n[2:]), p) for n, p in poses.items() if n[0] == v)
        names = [c for c in clips if clips[c][0] == v]
        seg = lambda s: next((c for c in names if clips[c][1] <= s < clips[c][2]), None)
        for (s0, p0), (s1, p1) in zip(pts, pts[1:]):
            if s1 - s0 > 3: continue                                     # a gap: frames not localized
            c = seg(s0); col = pal[names.index(c) % len(pal)][::-1] if c else (110, 110, 110)
            cv2.line(img, px(p0["C"]), px(p1["C"]), col, 6 if c else 3, cv2.LINE_AA)
        for c in names:
            sp = [p for s, p in pts if clips[c][1] <= s < clips[c][2]]
            col = pal[names.index(c) % len(pal)][::-1]
            legend.append((c, col, f"{len(sp)}/{int(clips[c][2] - clips[c][1])} s localized"))
            if sp:
                cv2.circle(img, px(sp[0]["C"]), 9, col, -1); cv2.circle(img, px(sp[0]["C"]), 9, (0, 0, 0), 2)
                cv2.putText(img, c[:3], (px(sp[0]["C"])[0] + 10, px(sp[0]["C"])[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 5, cv2.LINE_AA)
                cv2.putText(img, c[:3], (px(sp[0]["C"])[0] + 10, px(sp[0]["C"])[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 2, cv2.LINE_AA)
        dur = max(s for s, _ in pts) if pts else 0
        legend.append(("unlabelled (not in a clip)", (110, 110, 110), f"{len(pts)} of {dur + 1} s localized in total"))
        cv2.rectangle(img, (10, 10), (720, 60 + 36 * len(legend)), (255, 255, 255), -1)
        cv2.putText(img, f"video {v}: drone path (1 fps, dot = clip start)", (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA)
        for j, (name, col, note) in enumerate(legend):
            y = 85 + 36 * j; cv2.line(img, (20, y - 8), (60, y - 8), col, 6)
            cv2.putText(img, f"{name}  ({note})", (70, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.imwrite(str(out / f"track_{v}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 90]); print("->", out / f"track_{v}.jpg")


if __name__ == "__main__":
    {"frames": frames, "sfm": sfm, "georef": georef, "plot": plot}[sys.argv[1]](*sys.argv[2:])

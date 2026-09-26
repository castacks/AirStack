"""The scene rendered from the drone's own cameras, beside the frames: the direct check that a
model matches the video.

    ~/.venvs/recon/bin/python video_compare.py b01 OUT_DIR [--n 8] [--near 47,-410 --r 40] [--names A/A03_016.jpg,...]

Picks `n` registered frames of a georeferenced recon (farthest-point over camera centres,
optionally only cameras within `r` m of `near`), then runs itself under Kit
(~/isaacsim/python.sh) to render data/recon/disaster_city.usda from each camera (world pose from
to_world.json, pinhole at the frame's focal length) and writes OUT_DIR/<frame>.jpg = the
undistorted frame | the render, plus 00_sheet.jpg. --tiles adds the raw Google tiles from the same camera
(tiles | video | ours); --res sets the render size.
"""
import argparse, json, os, subprocess, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _paths import R
W, H = (int(v) for v in (sys.argv[sys.argv.index("--res") + 1] if "--res" in sys.argv else "1024x768").split("x"))

if "--cams" not in sys.argv:                                   # recon venv: choose the frames
    import pycolmap
    ap = argparse.ArgumentParser(); ap.add_argument("recon"); ap.add_argument("out"); ap.add_argument("--n", type=int, default=8)
    ap.add_argument("--near"); ap.add_argument("--r", type=float, default=40); ap.add_argument("--names")
    ap.add_argument("--tiles", action="store_true", help="also render the raw Google tiles: tiles | video | ours"); ap.add_argument("--res", default="1024x768")
    a = ap.parse_args(); d = R / a.recon; out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    Tw = np.array(json.load(open(d / "to_world.json"))["recon_to_world"]); s = np.cbrt(np.linalg.det(Tw[:3, :3])); Rs = Tw[:3, :3] / s
    mdl = d / "sparse_ext" if (d / "sparse_ext").exists() else max((d / "sparse").iterdir(), key=lambda p: pycolmap.Reconstruction(p).num_reg_images())
    rec = pycolmap.Reconstruction(str(mdl)); ims = sorted(rec.images.values(), key=lambda im: im.name)
    C = np.array([Tw[:3, :3] @ im.projection_center() + Tw[:3, 3] for im in ims])
    if a.names:
        pick = [i for i, im in enumerate(ims) if im.name in a.names.split(",")]
    else:
        ok = np.ones(len(ims), bool)
        if a.near: ok = np.hypot(*(C[:, :2] - np.array([float(v) for v in a.near.split(",")])).T) < a.r
        cand = np.flatnonzero(ok); pick = [cand[len(cand) // 2]]; dist = np.linalg.norm(C[cand] - C[pick[0]], axis=1)
        while len(pick) < min(a.n, len(cand)):
            pick.append(cand[int(dist.argmax())]); dist = np.minimum(dist, np.linalg.norm(C[cand] - C[pick[-1]], axis=1))
    cams = []
    for i in pick:
        im = ims[int(i)]; c = rec.cameras[im.camera_id]
        cams.append({"img": str(d / "images" / im.name), "name": im.name.replace("/", "_")[:-4], "C": C[i].tolist(),
                     "R_wc": (Rs @ im.cam_from_world().rotation.matrix().T).tolist(), "params": list(c.params), "wh": [c.width, c.height]})
    json.dump(cams, open(out / "cams.json", "w"))
    subprocess.run([os.path.expanduser("~/isaacsim/python.sh"), __file__, "--cams", str(out / "cams.json"), str(out), *(["--tiles"] if a.tiles else []), "--res", a.res],
                   env={**os.environ, "OMNI_KIT_ACCEPT_EULA": "YES"}, check=True)
    sys.exit()

cams, out = json.load(open(sys.argv[2])), Path(sys.argv[3])       # Kit: render them
TILES = "--tiles" in sys.argv
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": W, "height": H, "renderer": "RaytracedLighting"})
import cv2, carb, omni.usd, omni.replicator.core as rep
from pxr import UsdGeom, Gf
ctx = omni.usd.get_context()
def render(usd):
    """every camera in `usd` -> list of BGR images (W wide)"""
    ctx.open_stage(str(usd))
    for _ in range(30): app.update()
    carb.settings.get_settings().set("/rtx/post/aa/autoExposureMode", 0)
    stage = ctx.get_stage()
    cam = UsdGeom.Camera.Define(stage, "/World/vc_cam"); cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1e5))
    op = UsdGeom.Xformable(cam).AddTransformOp()
    rp = rep.create.render_product("/World/vc_cam", (W, H)); rgb = rep.AnnotatorRegistry.get_annotator("rgb"); rgb.attach([rp])
    rep.orchestrator.step(rt_subframes=1, delta_time=0.0); outs = []
    for n, cm in enumerate(cams):
        fx, fy, cx, cy = cm["params"][:4]; cw, ch = cm["wh"]
        M = np.eye(4); M[:3, :3] = np.array(cm["R_wc"]) @ np.diag([1, -1, -1]); M[:3, 3] = cm["C"]   # OpenCV -> USD camera axes
        op.Set(Gf.Matrix4d(*M.T.ravel().tolist()))
        cam.CreateHorizontalApertureAttr(36.0); cam.CreateVerticalApertureAttr(36.0 * ch / cw); cam.CreateFocalLengthAttr(float(fx * 36.0 / cw))
        for _ in range(400 if n == 0 else 80): app.update()
        for _ in range(200):
            r = rgb.get_data()
            if r.size: break
            app.update()
        outs.append(cv2.resize(np.ascontiguousarray(r[..., :3])[..., ::-1], (W, round(W * ch / cw)))); print("VC", usd.stem, cm["name"], flush=True)
    rgb.detach([rp]); rp.destroy(); return outs
ours = render(R / "disaster_city.usda")
raw = render(R / "disaster_city_raw_tiles.usda") if TILES else None
label = lambda im, t, c: (cv2.rectangle(im, (0, 0), (im.shape[1], 50), (0, 0, 0), -1), cv2.putText(im, t, (14, 36), 0, 1.1, c, 2, cv2.LINE_AA), im)[-1]
sheet = []
for k, cm in enumerate(cams):
    fx, fy, cx, cy = cm["params"][:4]
    img = cv2.undistort(cv2.imread(cm["img"]), np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]), np.array(cm["params"][4:8]))
    img = cv2.resize(img, (ours[k].shape[1], ours[k].shape[0]))
    panels = ([label(raw[k], "Google 3D tiles (raw)", (80, 210, 255))] if TILES else []) + \
             [label(img, f"drone video  {cm['name']}", (255, 255, 255)), label(ours[k], "our scene, same camera", (120, 255, 120))]
    gap = np.full((img.shape[0], 10, 3), 20, np.uint8)
    row = np.concatenate(sum([[p, gap] for p in panels], [])[:-1], 1)
    cv2.imwrite(str(out / f"{cm['name']}.jpg"), row, [cv2.IMWRITE_JPEG_QUALITY, 90])
    sheet.append(cv2.resize(row, (row.shape[1] // 2, row.shape[0] // 2)))
cv2.imwrite(str(out / "00_sheet.jpg"), np.concatenate(sheet, 0), [cv2.IMWRITE_JPEG_QUALITY, 85])
app.close()

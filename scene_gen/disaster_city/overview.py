"""Top-down overview of the whole site: the render, and its semantic segmentation with the geofence drawn on.

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh overview.py OUT_DIR [--scene autumn] [--px 3000]

A camera straight down, north up, framing the geofence (the window the scene is built in: the
ortho / ground / LOD1 window, x -205..355, y -615..-20) with a margin and --south m more to the south, where the
site goes on. Inside the fence it is our scene; outside, the raw Google tiles (washed out) from the same camera,
for context. Writes
  OUT_DIR/overview_<scene>.jpg            the render
  OUT_DIR/overview_<scene>_semantic.jpg   Isaac's semantic segmentation, one fixed colour per class, legend
  OUT_DIR/overview_<scene>_geofence.jpg   the render with the geofence rectangle and its size drawn on
The world -> pixel map is exact (a pinhole looking straight down), so the rectangle is drawn from world corners.
"""
import json, math, sys
from pathlib import Path
arg = lambda k, d=None: sys.argv[sys.argv.index(k) + 1] if k in sys.argv else d
PX = int(arg("--px", "3000")); SOUTH = float(arg("--south", "90"))          # frame this far past the fence's south edge
from PIL import Image
_d = Path(__file__).resolve().parent / "data/recon"; _g = json.load(open(_d / "ortho_site.json"))
FW, FH = (n * _g["m_per_px"] for n in Image.open(_d / "ortho_site.png").size)   # the geofence, m
SPANX, SPANY = FW * 1.12, (FH + SOUTH) * 1.06 + FW * 0.06
PXH = int(round(PX * SPANY / SPANX))
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": PX, "height": PXH, "renderer": "RaytracedLighting"})
import numpy as np, carb, omni.usd, omni.replicator.core as rep
from pxr import UsdGeom, Gf
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _paths import R

out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True); scene = arg("--scene", "summer")
geo = json.load(open(R / "ortho_site.json"))
FX0, FY1 = geo["x0"], geo["y1"]; FX1, FY0 = FX0 + FW, FY1 - FH            # the geofence
cx = (FX0 + FX1) / 2; cy = FY1 + FW * 0.06 - SPANY / 2                      # the fence + 6% margins, and SOUTH m more south
FOCAL, AP = 30.0, 36.0; H = SPANX / 2 / (AP / 2 / FOCAL)                          # height that frames SPANX
Z = 60.0 + H

ctx = omni.usd.get_context()
def shoot(usd, with_seg, only=None):
  ctx.open_stage(str(usd))
  if only:                                                             # a pass with only these /World children active
      for p in ctx.get_stage().GetPrimAtPath("/World").GetChildren():
          if p.GetName() not in only: p.SetActive(False)
  for _ in range(30): app.update()
  carb.settings.get_settings().set("/rtx/post/aa/autoExposureMode", 0)
  st = ctx.get_stage()
  cam = UsdGeom.Camera.Define(st, "/World/overview_cam"); cam.CreateClippingRangeAttr(Gf.Vec2f(1.0, 1e5))
  cam.CreateFocalLengthAttr(FOCAL); cam.CreateHorizontalApertureAttr(AP); cam.CreateVerticalApertureAttr(AP * PXH / PX)
  UsdGeom.Xformable(cam).AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(cx, cy, Z), Gf.Vec3d(cx, cy, 0), Gf.Vec3d(0, 1, 0)).GetInverse())
  rp = rep.create.render_product("/World/overview_cam", (PX, PXH))
  rgb = rep.AnnotatorRegistry.get_annotator("rgb"); seg = rep.AnnotatorRegistry.get_annotator("semantic_segmentation", init_params={"colorize": False})
  rgb.attach([rp])
  if with_seg: seg.attach([rp])
  rep.orchestrator.step(rt_subframes=1, delta_time=0.0)
  for _ in range(700): app.update()
  for _ in range(200):
      d, sd = rgb.get_data(), (seg.get_data() if with_seg else {"data": [0]})
      if d.size and sd and np.asarray(sd["data"]).size: break
      app.update()
  im = np.ascontiguousarray(d[..., :3]).copy(); res = (im, sd if with_seg else None)
  rgb.detach([rp])
  if with_seg: seg.detach([rp])
  rp.destroy(); return res
d, sd = shoot(R / ("disaster_city_autumn.usda" if scene == "autumn" else "disaster_city.usda"), True)
raw, _ = shoot(R / "disaster_city_raw_tiles.usda", False)                  # outside the fence: the raw tiles, for context
# Isaac's semantic instance mapping runs out of uids on the full scene (thousands of instanced trees, rubble pieces
# and multi-mesh vehicles: "number of semantics is greater than the maximum number of uids") and the overflow comes
# back UNLABELLED -- mostly tree crowns. From straight above, a second pass of the trees alone recovers them:
# wherever the full pass says UNLABELLED and the trees-only pass says vegetation, it is vegetation.
_, sd_t = shoot(R / ("disaster_city_autumn.usda" if scene == "autumn" else "disaster_city.usda"), True, only={"trees", "Environment"})
f_ = PX / 2 / math.tan(math.atan(AP / 2 / FOCAL))                                  # pixels per unit at the image plane
pix = lambda x, y: (PX / 2 + (x - cx) * f_ / (Z - 60.0), PXH / 2 - (y - cy) * f_ / (Z - 60.0))   # north up (ground ~60 m)
(ax, ay), (bx, by) = pix(FX0, FY1), pix(FX1, FY0); inside = np.zeros((PXH, PX), bool); inside[int(ay):int(by), int(ax):int(bx)] = True
comp = np.where(inside[..., None], d, (raw * 0.8 + 255 * 0.2).astype(np.uint8))     # outside: the raw tiles, washed out
img = Image.fromarray(comp)
ids = np.asarray(sd["data"]).astype(np.int64); lab = {int(k): v.get("class", "unlabelled") for k, v in sd["info"]["idToLabels"].items()}
COL = {"BACKGROUND": (0, 0, 0), "UNLABELLED": (0, 0, 0), "ground": (150, 120, 80), "road": (70, 70, 75), "water": (40, 110, 200), "vegetation": (60, 160, 60), "rubble": (220, 140, 40),
       "building": (200, 60, 60), "vehicle": (160, 60, 200), "person": (255, 230, 0), "other": (150, 150, 150), "unlabelled": (20, 20, 20)}
ids_t = np.asarray(sd_t["data"]).astype(np.int64); lab_t = {int(k): v.get("class", "") for k, v in sd_t["info"]["idToLabels"].items()}
veg_t = np.isin(ids_t, [k for k, v in lab_t.items() if v.split(",")[0] == "vegetation"])
unl = np.isin(ids, [k for k, v in lab.items() if v in ("UNLABELLED", "BACKGROUND")])
VEG = max(lab) + 1; lab[VEG] = "vegetation"; ids = np.where(unl & veg_t, VEG, ids)
segimg = np.zeros(ids.shape + (3,), np.uint8)   # outside the fence: black (not our scene)
for k, name in lab.items():
    name = name.split(",")[0]; segimg[ids == k] = COL.get(name, (255, 0, 255))
seg_i = Image.fromarray(segimg)
try: font = ImageFont.truetype("DejaVuSans-Bold.ttf", PX // 60); small = ImageFont.truetype("DejaVuSans-Bold.ttf", PX // 90)
except OSError: font = small = ImageFont.load_default()
def fence(im, colour):
    dr = ImageDraw.Draw(im); a, b = pix(FX0, FY1), pix(FX1, FY0)
    dr.rectangle([a, b], outline=colour, width=max(4, PX // 400))
    dr.text((a[0] + 10, a[1] + 10), f"geofence  {FW:.0f} x {FH:.0f} m", fill=colour, font=font)
    dr.text((a[0] + 10, b[1] - PX // 40), f"x {FX0:.0f}..{FX1:.0f}, y {FY0:.0f}..{FY1:.0f} (scene metres)   N up", fill=colour, font=small)
    return im
img.save(out / f"overview_{scene}.jpg", quality=92)
fence(img.copy(), (255, 60, 60)).save(out / f"overview_{scene}_geofence.jpg", quality=92)
segimg[~inside] = 0; seg_i = Image.fromarray(segimg)
s2 = fence(seg_i.copy(), (255, 255, 255)); dr = ImageDraw.Draw(s2)
present = sorted({n.split(",")[0] for k, n in lab.items() if (ids[inside] == k).any()} - {"BACKGROUND", "UNLABELLED"})
for i, n in enumerate(present):
    y = PXH - (len(present) - i) * (PX // 45) - 20; dr.rectangle([20, y, 20 + PX // 60, y + PX // 60], fill=COL.get(n, (255, 0, 255)))
    dr.text((30 + PX // 60, y - 4), n, fill=(255, 255, 255), font=small)
s2.save(out / f"overview_{scene}_semantic.jpg", quality=92)
cnt = {lab.get(int(k), str(k)): int(v) for k, v in zip(*np.unique(ids[inside], return_counts=True))}
print("OVERVIEW", scene, present, "pixels:", cnt, flush=True)
app.close()

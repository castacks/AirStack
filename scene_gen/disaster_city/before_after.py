"""Before/after comparisons: the raw Google tiles vs the grafted scene, from identical cameras.

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh before_after.py shots.json OUT_DIR

shots.json: {name: [[x, y, z] target, azimuth_deg, elevation_deg, distance_m]}. Renders every
shot in data/recon/disaster_city_raw_tiles.usda (BEFORE: raw tiles, same sun + sky -- built by
assemble_scene.py) and in data/recon/disaster_city.usda (AFTER), in one Kit process, and writes
  OUT_DIR/before/<name>.png, OUT_DIR/after/<name>.png
  OUT_DIR/<name>.jpg            the pair side by side, labelled
  OUT_DIR/00_contact_sheet.jpg  every pair, small
"""
import json, math, sys
from pathlib import Path
from isaacsim import SimulationApp
W, H = 1600, 900
app = SimulationApp({"headless": True, "width": W, "height": H, "renderer": "RaytracedLighting"})
import numpy as np, carb, omni.usd, omni.replicator.core as rep
from pxr import UsdGeom, Gf
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _paths import R

shots = json.load(open(sys.argv[1])); out = Path(sys.argv[2])
for sub in ("before", "after"): (out / sub).mkdir(parents=True, exist_ok=True)
ctx = omni.usd.get_context()
settings = carb.settings.get_settings()

def render_scene(usd, tag):
    ctx.open_stage(str(usd))
    for _ in range(30): app.update()
    settings.set("/rtx/post/aa/autoExposureMode", 0)
    stage = ctx.get_stage()
    cam = UsdGeom.Camera.Define(stage, "/World/ba_cam"); cam.CreateFocalLengthAttr(18); cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 1e6))
    op = UsdGeom.Xformable(cam).AddTransformOp()
    rp = rep.create.render_product("/World/ba_cam", (W, H)); rgb = rep.AnnotatorRegistry.get_annotator("rgb"); rgb.attach([rp])
    rep.orchestrator.step(rt_subframes=1, delta_time=0.0)
    first = True
    for name, (tgt, az, el, dist) in shots.items():
        t = np.array(tgt, float); a, e = math.radians(az), math.radians(el)
        eye = t + np.array([math.cos(a) * math.cos(e), math.sin(a) * math.cos(e), math.sin(e)]) * dist
        op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*t), Gf.Vec3d(0, 0, 1)).GetInverse())
        for _ in range(600 if first else 60): app.update()
        first = False
        for _ in range(200):
            d = rgb.get_data()
            if d.size: break
            app.update()
        Image.fromarray(np.ascontiguousarray(d[..., :3])).save(out / tag / f"{name}.png"); print("RENDER", tag, name, flush=True)
    rgb.detach([rp]); rp.destroy()

render_scene(R / "disaster_city_raw_tiles.usda", "before")
render_scene(R / "disaster_city.usda", "after")

try: font = ImageFont.truetype("DejaVuSans-Bold.ttf", 34)
except OSError: font = ImageFont.load_default()
pairs = []
for name in shots:
    b, a = Image.open(out / "before" / f"{name}.png"), Image.open(out / "after" / f"{name}.png")
    pair = Image.new("RGB", (W * 2 + 12, H + 56), (20, 20, 20)); pair.paste(b, (0, 56)); pair.paste(a, (W + 12, 56))
    dr = ImageDraw.Draw(pair)
    dr.text((16, 10), f"BEFORE  raw Google 3D tiles   ({name})", fill=(255, 210, 60), font=font)
    dr.text((W + 28, 10), "AFTER  grafted models + recon", fill=(120, 230, 120), font=font)
    pair.save(out / f"{name}.jpg", quality=90); pairs.append(pair.resize((pair.width // 4, pair.height // 4)))
sheet = Image.new("RGB", (pairs[0].width, pairs[0].height * len(pairs)))
for i, p in enumerate(pairs): sheet.paste(p, (0, i * p.height))
sheet.save(out / "00_contact_sheet.jpg", quality=88)
app.close()

"""Render every normalised library asset (assets/lib/library.json) on its own, in host Isaac Sim.

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh asset_gallery.py OUT_DIR [name ...]

One 3/4 view per asset on a grey ground with the scene's sky, framed on its size, plus
OUT_DIR/00_gallery.jpg (all of them, labelled with name and size) -- to check scale,
orientation (long side = +X, base at z = 0) and that the textures came along.
"""
import json, math, sys
from pathlib import Path
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 960, "height": 640, "renderer": "RaytracedLighting"})
import numpy as np, carb, omni.usd, omni.replicator.core as rep
from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf
from PIL import Image, ImageDraw
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _paths import R

out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
lib = json.load(open(R / "assets/lib/library.json")); names = sys.argv[2:] or list(lib)
ctx = omni.usd.get_context(); ctx.new_stage(); stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, "Z"); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
carb.settings.get_settings().set("/rtx/post/aa/autoExposureMode", 0)
sun = UsdLux.DistantLight.Define(stage, "/Env/sun"); sun.CreateIntensityAttr(3000); UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(35, 0, 150))
dome = UsdLux.DomeLight.Define(stage, "/Env/sky"); dome.CreateIntensityAttr(1000)
dome.CreateTextureFileAttr(Sdf.AssetPath(str(R / "sky/sunflowers.hdr"))); dome.CreateTextureFormatAttr("latlong")
g = UsdGeom.Cube.Define(stage, "/Env/ground"); UsdGeom.Xformable(g).AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
UsdGeom.Xformable(g).AddScaleOp().Set(Gf.Vec3f(60, 60, 0.05)); g.CreateDisplayColorAttr([(0.45, 0.45, 0.43)])
cam = UsdGeom.Camera.Define(stage, "/Env/cam"); cam.CreateFocalLengthAttr(24); cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1e6))
op = UsdGeom.Xformable(cam).AddTransformOp()
rp = rep.create.render_product("/Env/cam", (960, 640)); rgb = rep.AnnotatorRegistry.get_annotator("rgb"); rgb.attach([rp])
rep.orchestrator.step(rt_subframes=1, delta_time=0.0)
def grab(n):
    for _ in range(n): app.update()
    for _ in range(200):
        d = rgb.get_data()
        if d.size: return np.ascontiguousarray(d[..., :3])
        app.update()
tiles = []
for i, name in enumerate(names):
    p = stage.DefinePrim(f"/Assets/{name}"); p.GetReferences().AddReference(str(R / lib[name]["usd"]))
    for other in stage.GetPrimAtPath("/Assets").GetChildren(): other.SetActive(other.GetName() == name)
    L, W, H = lib[name]["size_m"]; d = max(L, W, H) * 1.35 + 2
    eye = Gf.Vec3d(d * 0.75, -d * 0.85, H * 0.5 + d * 0.45)
    op.Set(Gf.Matrix4d().SetLookAt(eye, Gf.Vec3d(0, 0, H * 0.4), Gf.Vec3d(0, 0, 1)).GetInverse())
    im = Image.fromarray(grab(400 if i == 0 else 90)); im.save(out / f"{name}.png")
    t = im.resize((480, 320)); dr = ImageDraw.Draw(t)
    dr.rectangle([0, 0, 480, 22], fill=(0, 0, 0)); dr.text((6, 5), f"{name}  {L:.1f} x {W:.1f} x {H:.1f} m  {lib[name]['tris']} tris", fill=(255, 230, 0))
    tiles.append(t); print("GALLERY", name, flush=True)
cols = 3; rows = math.ceil(len(tiles) / cols); sheet = Image.new("RGB", (480 * cols, 320 * rows))
for i, t in enumerate(tiles): sheet.paste(t, ((i % cols) * 480, (i // cols) * 320))
sheet.save(out / "00_gallery.jpg", quality=90)
app.close()

"""Render the scene gallery: specs/gallery.yaml -> OUT_DIR/*.jpg + index.html + 00_contact_sheet.jpg.

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh gallery.py OUT_DIR [--only name,name]

Each shot is either an orbit ({target, az, el, dist} in world coordinates, as before_after.py)
or a camera ({eye, look}) -- in world coordinates, or in a hero's own frame when the shot names
it (`in: B01`: the spec's origin + yaw, the frame its parts are written in, so an interior shot
stays inside the building when the building moves). `focal` (mm on a 36 mm sensor) defaults to
18; interiors use ~12. Sections and captions go into index.html.
"""
import json, math, sys
from pathlib import Path
from isaacsim import SimulationApp
W, H = 1600, 900
app = SimulationApp({"headless": True, "width": W, "height": H, "renderer": "RaytracedLighting"})
import numpy as np, yaml, carb, omni.usd, omni.replicator.core as rep
from pxr import UsdGeom, Gf
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _paths import R, SPECS

out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
only = sys.argv[sys.argv.index("--only") + 1].split(",") if "--only" in sys.argv else None
G = yaml.safe_load(open(SPECS / "gallery.yaml"))

def frame_of(bid):
    f = SPECS / f"{bid}.yaml"
    s = yaml.safe_load(open(f if f.exists() else R / bid.lower() / f"{bid}_spec.yaml"))
    o, t = np.array(s["origin"], float), math.radians(s.get("yaw_deg", 0))
    Rz = np.array([[math.cos(t), -math.sin(t), 0], [math.sin(t), math.cos(t), 0], [0, 0, 1]])
    return lambda p: o + Rz @ np.array(p, float)

def camera(s):
    if "eye" in s:
        to = frame_of(s["in"]) if "in" in s else (lambda p: np.array(p, float))
        return to(s["eye"]), to(s["look"])
    t = np.array(s["target"], float); a, e = math.radians(s["az"]), math.radians(s["el"])
    return t + np.array([math.cos(a) * math.cos(e), math.sin(a) * math.cos(e), math.sin(e)]) * s["dist"], t

ctx = omni.usd.get_context(); ctx.open_stage(str(R / "disaster_city.usda"))
for _ in range(30): app.update()
carb.settings.get_settings().set("/rtx/post/aa/autoExposureMode", 0)
stage = ctx.get_stage()
cam = UsdGeom.Camera.Define(stage, "/World/gallery_cam"); cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1e6))
cam.CreateHorizontalApertureAttr(36.0); cam.CreateVerticalApertureAttr(36.0 * H / W)
op = UsdGeom.Xformable(cam).AddTransformOp()
rp = rep.create.render_product("/World/gallery_cam", (W, H)); rgb = rep.AnnotatorRegistry.get_annotator("rgb"); rgb.attach([rp])
rep.orchestrator.step(rt_subframes=1, delta_time=0.0)
try: font = ImageFont.truetype("DejaVuSans-Bold.ttf", 28)
except OSError: font = ImageFont.load_default()
done, first = [], True
for sec in G["sections"]:
    for s in sec["shots"]:
        if only and s["name"] not in only: continue
        eye, look = camera(s)
        op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*look), Gf.Vec3d(0, 0, 1)).GetInverse())
        cam.CreateFocalLengthAttr(float(s.get("focal", 18)))
        for _ in range(600 if first else 90): app.update()
        first = False
        for _ in range(200):
            d = rgb.get_data()
            if d.size: break
            app.update()
        im = Image.fromarray(np.ascontiguousarray(d[..., :3]))
        dr = ImageDraw.Draw(im); dr.rectangle((0, H - 50, W, H), fill=(0, 0, 0))
        dr.text((16, H - 42), f"{s['name']}   {s['caption']}", fill=(255, 255, 255), font=font)
        im.save(out / f"{s['name']}.jpg", quality=90); done.append((sec["title"], s)); print("GALLERY", s["name"], flush=True)

# index.html (relative image links, opens from the folder) + a contact sheet
html = ["<!doctype html><meta charset=utf-8><title>Disaster City gallery</title>",
        "<style>body{font:15px system-ui;background:#111;color:#ddd;margin:24px}h2{margin-top:32px}"
        "figure{display:inline-block;width:46%;margin:0 1% 18px 0;vertical-align:top}img{width:100%}figcaption{margin-top:4px}</style>",
        f"<h1>{G['title']}</h1><p>{G['intro']}</p>"]
done = [(sec["title"], s) for sec in G["sections"] for s in sec["shots"] if (out / f"{s['name']}.jpg").exists()]   # a partial run keeps the rest
for sec in G["sections"]:
    shots = [s for t, s in done if t == sec["title"]]
    if not shots: continue
    html.append(f"<h2>{sec['title']}</h2><p>{sec.get('text', '')}</p>")
    for s in shots:
        extra = "".join(f'<br><a href="{v}">{Path(v).name}</a>' for v in s.get("video", []))
        html.append(f'<figure><a href="{s["name"]}.jpg"><img src="{s["name"]}.jpg"></a><figcaption><b>{s["name"]}</b> {s["caption"]}{extra}</figcaption></figure>')
(out / "index.html").write_text("\n".join(html))
thumbs = [Image.open(out / f"{s['name']}.jpg").resize((400, 225)) for _, s in done]
sheet = Image.new("RGB", (1600, 225 * ((len(thumbs) + 3) // 4)))
for i, t in enumerate(thumbs): sheet.paste(t, ((i % 4) * 400, (i // 4) * 225))
sheet.save(out / "00_contact_sheet.jpg", quality=88)
rgb.detach([rp]); rp.destroy(); app.close()

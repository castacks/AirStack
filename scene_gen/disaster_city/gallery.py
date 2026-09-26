"""Render the scene gallery: specs/gallery.yaml -> OUT_DIR/*.jpg + index.html + 00_contact_sheet.jpg.

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh gallery.py OUT_DIR [--only name,name] [--res 2560x1440] [--pairs]

Each shot is either an orbit ({target, az, el, dist} in world coordinates, as before_after.py)
or a camera ({eye, look}) -- in world coordinates, or in a hero's own frame when the shot names
it (`in: B01`: the spec's origin + yaw, the frame its parts are written in, so an interior shot
stays inside the building when the building moves). `focal` (mm on a 36 mm sensor) defaults to
18; interiors use ~12. Sections and captions go into index.html.

--pairs also renders every shot in the raw-tiles scene (disaster_city_raw_tiles.usda, same sun and
sky) and writes OUT_DIR/pairs/<name>.jpg: BEFORE (raw Google tiles) | AFTER, linked from the index.
An interior's BEFORE shows what the tiles have there: the inside of a closed blob, or nothing.
"""
import math, sys
from pathlib import Path
arg = lambda k, d=None: sys.argv[sys.argv.index(k) + 1] if k in sys.argv else d
W, H = (int(v) for v in arg("--res", "1600x900").split("x"))
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": W, "height": H, "renderer": "RaytracedLighting"})
import numpy as np, yaml, carb, omni.usd, omni.replicator.core as rep
from pxr import UsdGeom, Gf
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _paths import R, SPECS

out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
only = arg("--only").split(",") if arg("--only") else None
pairs = "--pairs" in sys.argv
G = yaml.safe_load(open(SPECS / "gallery.yaml"))
shots = [(sec["title"], s) for sec in G["sections"] for s in sec["shots"] if not only or s["name"] in only]

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

ctx = omni.usd.get_context()
def render(usd, save):
    """every shot in `usd` -> save(shot, PIL image)"""
    ctx.open_stage(str(usd))
    for _ in range(30): app.update()
    carb.settings.get_settings().set("/rtx/post/aa/autoExposureMode", 0)
    stage = ctx.get_stage()
    cam = UsdGeom.Camera.Define(stage, "/World/gallery_cam"); cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1e6))
    cam.CreateHorizontalApertureAttr(36.0); cam.CreateVerticalApertureAttr(36.0 * H / W)
    op = UsdGeom.Xformable(cam).AddTransformOp()
    rp = rep.create.render_product("/World/gallery_cam", (W, H)); rgb = rep.AnnotatorRegistry.get_annotator("rgb"); rgb.attach([rp])
    rep.orchestrator.step(rt_subframes=1, delta_time=0.0)
    for n, (_, s) in enumerate(shots):
        eye, look = camera(s)
        op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*look), Gf.Vec3d(0, 0, 1)).GetInverse())
        cam.CreateFocalLengthAttr(float(s.get("focal", 18)))
        for _ in range(600 if n == 0 else 90): app.update()
        for _ in range(200):
            d = rgb.get_data()
            if d.size: break
            app.update()
        save(s, Image.fromarray(np.ascontiguousarray(d[..., :3]))); print("GALLERY", usd.stem, s["name"], flush=True)
    rgb.detach([rp]); rp.destroy()

try: font = ImageFont.truetype("DejaVuSans-Bold.ttf", max(20, H // 32))
except OSError: font = ImageFont.load_default()
bar = max(40, H // 18)
def caption(im, text, colour=(255, 255, 255)):
    dr = ImageDraw.Draw(im); dr.rectangle((0, H - bar, W, H), fill=(0, 0, 0)); dr.text((16, H - bar + bar // 5), text, fill=colour, font=font); return im
(out / "raw").mkdir(exist_ok=True)
def standalone(s, im):
    if pairs: im.save(out / "raw" / f"{s['name']}.png")                  # uncaptioned, for the pair
    caption(im.copy(), f"{s['name']}   {s['caption']}").save(out / f"{s['name']}.jpg", quality=92)
render(R / "disaster_city.usda", standalone)
if pairs:
    (out / "before").mkdir(exist_ok=True); (out / "pairs").mkdir(exist_ok=True)
    def pair(s, before):
        before.save(out / "before" / f"{s['name']}.png")
        after = Image.open(out / "raw" / f"{s['name']}.png")
        p = Image.new("RGB", (2 * W + 12, H), (20, 20, 20)); p.paste(caption(before, "BEFORE  raw Google 3D tiles", (255, 210, 60)), (0, 0))
        p.paste(caption(after, f"AFTER  {s['name']}: {s['caption']}", (120, 230, 120)), (W + 12, 0)); p.save(out / "pairs" / f"{s['name']}.jpg", quality=90)
    render(R / "disaster_city_raw_tiles.usda", pair)

# index.html (relative links, opens from the folder) + contact sheets, from every image present (a partial run keeps the rest)
done = [(sec["title"], s) for sec in G["sections"] for s in sec["shots"] if (out / f"{s['name']}.jpg").exists()]
html = ["<!doctype html><meta charset=utf-8><title>Disaster City gallery</title>",
        "<style>body{font:15px system-ui;background:#111;color:#ddd;margin:24px}a{color:#8cf}h2{margin-top:32px}"
        "figure{display:inline-block;width:46%;margin:0 1% 18px 0;vertical-align:top}img{width:100%}figcaption{margin-top:4px}</style>",
        f"<h1>{G['title']}</h1><p>{G['intro']}</p><p>{W} x {H}. Each shot links its before/after pair (raw Google tiles | this scene) where one was rendered.</p>"]
for sec in G["sections"]:
    shots_ = [s for t, s in done if t == sec["title"]]
    if not shots_: continue
    html.append(f"<h2>{sec['title']}</h2><p>{sec.get('text', '')}</p>")
    for s in shots_:
        extra = "".join(f'<br><a href="{v}">video frame: {Path(v).name}</a>' for v in s.get("video", []))
        if (out / "pairs" / f"{s['name']}.jpg").exists(): extra += f'<br><a href="pairs/{s["name"]}.jpg">before / after</a>'
        html.append(f'<figure><a href="{s["name"]}.jpg"><img src="{s["name"]}.jpg"></a><figcaption><b>{s["name"]}</b> {s["caption"]}{extra}</figcaption></figure>')
for d in sorted(out.glob("video_triptych_*")):             # video_compare.py --tiles output: raw tiles | drone video | ours
    imgs = sorted(f for f in d.glob("*.jpg") if not f.name.startswith("00_"))
    if not imgs: continue
    html.append(f"<h2>Against the drone video: {d.name[15:]}</h2><p>Each row: the raw Google tiles, the video frame, and this scene, all from that frame's "
                f"georeferenced camera. <a href='{d.name}/00_sheet.jpg'>all on one sheet</a></p>")
    html += [f'<figure style="width:95%"><a href="{d.name}/{f.name}"><img src="{d.name}/{f.name}"></a><figcaption>{f.stem}</figcaption></figure>' for f in imgs]
(out / "index.html").write_text("\n".join(html))
def sheet(files, name, w):
    th = [Image.open(f).resize((w, round(w * Image.open(f).height / Image.open(f).width))) for f in files]
    if not th: return
    cols = 1600 // w; sh = Image.new("RGB", (cols * w, th[0].height * ((len(th) + cols - 1) // cols)))
    for i, t in enumerate(th): sh.paste(t, ((i % cols) * w, (i // cols) * th[0].height))
    sh.save(out / name, quality=88)
sheet([out / f"{s['name']}.jpg" for _, s in done], "00_contact_sheet.jpg", 400)
sheet([out / "pairs" / f"{s['name']}.jpg" for _, s in done if (out / "pairs" / f"{s['name']}.jpg").exists()], "00_contact_sheet_pairs.jpg", 800)
app.close()

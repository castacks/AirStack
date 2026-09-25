"""Open the assembled scene in host Isaac Sim, render obliques, test colliders + semantics.

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh isaac_check.py data/recon/disaster_city.usda OUT_DIR

Shots: named (target, azimuth, elevation, distance) views -> OUT_DIR/<name>.png.
Physics: PhysX raycasts through B01's first-floor window (must pass) and into
its wall (must hit), plus one ray down onto the pile. Semantics: prim count per
class. Results -> OUT_DIR/check.json.
"""
import json, math, os, sys
from pathlib import Path
from isaacsim import SimulationApp

usd, out = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve(); out.mkdir(parents=True, exist_ok=True)
app = SimulationApp({"headless": True, "width": 1600, "height": 900, "renderer": "RaytracedLighting"})

import numpy as np, yaml
import carb, omni.usd, omni.replicator.core as rep
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, Gf, Sdf, PhysxSchema

ctx = omni.usd.get_context(); ctx.open_stage(str(usd))
for _ in range(20): app.update()
stage = ctx.get_stage()
carb.settings.get_settings().set("/rtx/post/aa/autoExposureMode", 0)
carb.settings.get_settings().set("/rtx/post/backgroundZeroAlpha/enabled", False)   # else the sky comes back black

# light: sun + sky dome (the scene carries none)
sun = UsdLux.DistantLight.Define(stage, "/World/check_sun"); sun.CreateIntensityAttr(3000); sun.CreateAngleAttr(0.5)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(35, 0, 150))
SKY = str(usd.parent / "sky/sunflowers.hdr")   # local copy of an Isaac-shipped outdoor HDR; the Nucleus one never streamed in headless
dome = UsdLux.DomeLight.Define(stage, "/World/check_sky"); dome.CreateIntensityAttr(1000); dome.CreateExposureAttr(0)
dome.CreateTextureFileAttr(SKY); dome.CreateTextureFormatAttr("latlong")
UsdPhysics.Scene.Define(stage, "/World/physics")

# ---- semantics census ----
classes = {}
for p in stage.Traverse():
    a = p.GetAttribute("semantics:labels:class")
    if a and a.Get(): classes[a.Get()[0]] = classes.get(a.Get()[0], 0) + 1
print("semantic classes:", classes)

# ---- physics: raycasts against B01 ----
frame = json.load(open(usd.parent / "b01/sheets/frame.json")); o = np.array(frame["origin_world"]); th = math.radians(frame["yaw_deg"])
L = lambda x, y, z: Gf.Vec3d(*(o + [x * math.cos(th) - y * math.sin(th), x * math.sin(th) + y * math.cos(th), z]))
import omni.timeline
tl = omni.timeline.get_timeline_interface(); tl.play()          # PhysX builds its scene on play
for _ in range(10): app.update()
tl.pause()
from omni.physx import get_physx_scene_query_interface
sq = get_physx_scene_query_interface()
def ray(a, b):
    d = np.array(b) - np.array(a); n = np.linalg.norm(d)
    h = sq.raycast_closest(carb.Float3(*map(float, a)), carb.Float3(*map(float, d / n)), float(n))
    return {"hit": bool(h["hit"]), "dist": round(h.get("distance", -1), 2), "prim": h.get("collision", "")}
tests = {
    # first-floor window of the ylo face: local x 1.2..2.2, z 4.15..5.15 (sill 0.8 over the 3.35 slab)
    "window_B01_first_floor (expect pass)": ray(L(1.7, 12.0, 4.6), L(1.7, 17.0, 4.6)),
    "wall_B01_first_floor (expect hit)": ray(L(5.5, 12.0, 4.6), L(5.5, 17.0, 4.6)),
    "doorway_B01_frame_side (expect pass)": ray(L(13.0, 17.6, 4.4), L(8.0, 17.6, 4.4)),
    "pile_R01_down (expect hit)": ray((39.0, -388.0, 120.0), (39.0, -388.0, 0.0)),
}
# a tree trunk and a placed vehicle, looked up from the stage
tpos = stage.GetPrimAtPath("/World/trees/colliders/t0").GetAttribute("xformOp:translate").Get()
if tpos: tests["tree_trunk (expect hit)"] = ray((tpos[0] - 3, tpos[1], tpos[2]), (tpos[0] + 3, tpos[1], tpos[2]))
veh = next((p for p in stage.GetPrimAtPath("/World/vehicles").GetChildren() if p.GetName().startswith("bus")), None)
if veh:
    vp = veh.GetAttribute("xformOp:translate").Get()
    tests[f"vehicle_{veh.GetName()} (expect hit)"] = ray((vp[0], vp[1], vp[2] + 30), (vp[0], vp[1], vp[2] - 1))
for k, v in tests.items(): print(f"  {k}: {v}")

# ---- renders ----
shots = {
    "site_overview_SW": ((95, -330, 58), 230, 42, 430),
    "b01_pile_SW": ((42, -405, 60), 215, 28, 75),
    "b01_frame_E": ((48, -408, 61), 20, 18, 38),
    "grid_blocks_S": ((70, -470, 60), 250, 35, 170),
    "b01_street_level": ((44, -410, 60), 120, 6, 30),
    "s01_drill_tower": ((2, -512, 67), 200, 20, 40),
    "b03_strip_mall": ((96, -413, 60), 300, 35, 40),
    "woods_grid_low": ((10, -450, 60), 250, 12, 45),
    "woods_under_canopy": ((-30, -470, 59), 30, 3, 12),
    "parking_vehicles": ((255, -405, 62), 200, 30, 40),
    "r02_debris_field": ((188, -390, 60), 230, 25, 35),
    "r03_collapsed_houses": ((72, -440, 60), 30, 30, 40),
    "industrial_pad": ((55, -525, 64), 200, 22, 55),
    "lattice_tower_close": ((80.3, -523.4, 70), 230, 12, 22),
    "b06_warehouse_canopy": ((160, -499, 64), 140, 20, 55),
}
# eye/target pairs in B01's local frame: inside the first floor, and on the deck looking in through the doorway
inside = {"b01_inside_first_floor": ((2.0, 17.0, 4.9), (10.5, 18.1, 4.6)),
          "b01_deck_to_doorway": ((16.0, 17.5, 4.8), (10.5, 18.1, 4.5))}
cam = UsdGeom.Camera.Define(stage, "/World/check_cam"); cam.CreateFocalLengthAttr(18); cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 5000))
xf = UsdGeom.Xformable(cam); op = xf.AddTransformOp()
rp = rep.create.render_product(str(cam.GetPath()), (1600, 900))
rgb = rep.AnnotatorRegistry.get_annotator("rgb"); rgb.attach([rp])
rep.orchestrator.step(rt_subframes=1, delta_time=0.0)           # activates the annotator once
from PIL import Image
def grab(n):
    for _ in range(n): app.update()
    for _ in range(200):                                          # empty for a few ticks after a jump
        d = rgb.get_data()
        if d.size: return np.ascontiguousarray(d[..., :3])
        app.update()
    raise RuntimeError("annotator returned no image")
first = True
for name, (tgt, az, el, dist) in shots.items():
    t = np.array(tgt, float); d = np.array([math.cos(math.radians(az)) * math.cos(math.radians(el)),
                                            math.sin(math.radians(az)) * math.cos(math.radians(el)), math.sin(math.radians(el))])
    eye = t + d * dist
    m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*t), Gf.Vec3d(0, 0, 1)).GetInverse()
    op.Set(m)
    Image.fromarray(grab(600 if first else 60)).save(out / f"{name}.png"); print("saved", name); first = False

# B06: from outside the +v bay door, looking in (local frame of its spec)
b6 = yaml.safe_load(open(usd.parent / "b06/B06_spec.yaml")); o6, th6 = np.array(b6["origin"]), math.radians(b6["yaw_deg"])
L6 = lambda x, y, z: Gf.Vec3d(*(o6 + [x * math.cos(th6) - y * math.sin(th6), x * math.sin(th6) + y * math.cos(th6), z]))
extra = {"b06_bay_door_in": (L6(14.0, 36.0, 2.5), L6(14.0, 10.0, 2.0))}
for name, (e, t) in list(inside.items()) + list(extra.items()):
    if name in inside: e, t = L(*e), L(*t)
    op.Set(Gf.Matrix4d().SetLookAt(e, t, Gf.Vec3d(0, 0, 1)).GetInverse())
    Image.fromarray(grab(60)).save(out / f"{name}.png"); print("saved", name)

# ---- cost: frame time at the site overview, and what the stage holds ----
import time
t, az, el, dist = shots["site_overview_SW"]
t = np.array(t, float); d = np.array([math.cos(math.radians(az)) * math.cos(math.radians(el)), math.sin(math.radians(az)) * math.cos(math.radians(el)), math.sin(math.radians(el))])
op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*(t + d * dist)), Gf.Vec3d(*t), Gf.Vec3d(0, 0, 1)).GetInverse())
grab(30); t0 = time.time()
for _ in range(60): app.update()
ms = (time.time() - t0) / 60 * 1000
tris = inst = prims = 0
for p in stage.Traverse():
    prims += 1
    if p.IsA(UsdGeom.Mesh) and UsdGeom.Imageable(p).ComputeVisibility() != "invisible":
        c = UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get()
        if c: tris += int(np.sum(np.asarray(c) - 2))
    if p.IsA(UsdGeom.PointInstancer): inst += len(UsdGeom.PointInstancer(p).GetProtoIndicesAttr().Get() or [])
    if p.IsInstance(): inst += 1
cost = {"ms_per_frame_1600x900": round(ms, 1), "prims": prims, "mesh_tris_incl_prototypes": tris, "instances": inst}
print("cost:", cost)

# ---- semantic segmentation of two views: what a perception stack would get ----
if os.environ.get("SEMSEG", "1") == "1":
    seg = rep.AnnotatorRegistry.get_annotator("semantic_segmentation", init_params={"colorize": True})
    seg.attach([rp])
    for name in ("site_overview_SW", "b01_pile_SW"):
        t, az, el, dist = shots[name]
        t = np.array(t, float); d = np.array([math.cos(math.radians(az)) * math.cos(math.radians(el)), math.sin(math.radians(az)) * math.cos(math.radians(el)), math.sin(math.radians(el))])
        op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*(t + d * dist)), Gf.Vec3d(*t), Gf.Vec3d(0, 0, 1)).GetInverse())
        grab(60)
        for _ in range(100):
            sd = seg.get_data()
            if sd and np.asarray(sd["data"]).size: break
            app.update()
        Image.fromarray(np.asarray(sd["data"])[..., :3].astype(np.uint8)).save(out / f"semseg_{name}.png")
        json.dump({str(k): v for k, v in sd["info"]["idToLabels"].items()}, open(out / f"semseg_{name}_legend.json", "w"), indent=1)
        print("saved semseg", name, sd["info"]["idToLabels"])

json.dump({"cost": cost, "classes": classes, "raycasts": tests}, open(out / "check.json", "w"), indent=1)
app.close()

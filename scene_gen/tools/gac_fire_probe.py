#!/usr/bin/env python
"""gac_fire_probe — `disaster.gac_fire` up to the atlas bake, bare USD, no
slice: measure a GAC asset, plan its fire and events from its own window
islands, rasterise the skin, bake it into the atlases, and write PNGs to
look at (the sooted atlases, the skin, and a window-island map).

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/gac_fire_probe.py SM_Building_02 F3"
"""
import os
import random
import sys
import time

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Sdf, Usd, UsdGeom                            # noqa: E402
from detail import gac_slice as gsl, gac_storey_slice as gss  # noqa: E402
from disaster import gac_fire as gf, soot_plume as spl, urban_fire as uf  # noqa: E402

OUT = "/isaac-sim/.nvidia-omniverse/logs/gac_fire_probe"
name = sys.argv[1] if len(sys.argv) > 1 else "SM_Building_02"
level = sys.argv[2] if len(sys.argv) > 2 else "F3"
os.makedirs(OUT, exist_ok=True)
t0 = time.time()
st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/W")
st.SetDefaultPrim(st.GetPrimAtPath("/W"))
cell = "/W/b0"
UsdGeom.Xform.Define(st, cell)
src = gf.place_source(st, cell, gf.GAC_DIR + name + ".usd", gf.GAC_SCALE)
wins, bbox = gsl.window_centres(st, src)
g, measured = gss.grid_for(st, src, bbox, wins, name=name)
rects = gf.window_rects(st, src)
print("[probe] bbox", bbox, "islands per side", {k: len(v) for k, v in rects.items()},
      "measured grid", measured, "%.1fs" % (time.time() - t0))
m = gf.mass_from_grid(g, bbox)
btype = "urm" if m["top"] - m["z0"] <= 25.0 else "rc"
info = {"style": "gac_" + name, "family": "01", "type": btype, "x": 0.0, "y": 0.0,
        "yaw": 0.0, "masses": {"main": m}, "elements": [], "H": m["top"] - m["z0"]}
rng = random.Random(7)
n_st = len(m["levels"])
origin = max(0, min(n_st - 1, int(round(0.25 * (n_st - 1)))))
fire = uf.plan_fire(info, level, rng, origin=origin, sides=("S", "E") if level not in ("F1", "F2") else ("S",))
prov = gf.openings_provider(rects, m)
ctx0 = {"info": info, "fire": fire, "rng": rng, "tag": "probe", "soot_openings": prov}
events = spl.plan_events(ctx0, uf._severity)
heavy = 1.0
for rname, kw in uf.LADDER.get(btype, {}).get(level, []):
    if rname == "smoke_stain":
        heavy = float((kw or {}).get("heavy", 1.0))
sk = spl.skin(ctx0, events, np.random.default_rng(spl.event_seed(ctx0) ^ 0x5EED),
              finish=fire.get("finish") or "char", glass=False, duration_scale=heavy)
print("[probe] mass W %.1f D %.1f H %.1f storeys %d; fire origin %d band %s sides %s; %s; skin %s @ %.1f px/m; %.1fs"
      % (m["W"], m["D"], m["top"] - m["z0"], n_st, fire["origin"], fire["storeys"], fire["sides"],
         spl.summarise(events), sk["rgba"].shape[:2], sk["ppm"], time.time() - t0))
spl.save_skin_png(sk, os.path.join(OUT, "%s_%s_skin.png" % (name, level)))
# island map on the skin's canvas
from PIL import Image, ImageDraw
im = Image.open(os.path.join(OUT, "%s_%s_skin.png" % (name, level))).convert("RGB")
d = ImageDraw.Draw(im)
off = sk["offsets"]
for side, rl in rects.items():
    for (u0, u1, z0, z1) in rl:
        for op in prov(ctx0, "main", side, 0) + []:
            pass
        # convert via the provider's own convention: draw every provider record instead
n_drawn = 0
for (side, stt), recs in {}.items():
    pass
for side in ("S", "E", "N", "W"):
    for stt in range(n_st):
        for op in prov(ctx0, "main", side, stt):
            a, b, z0, z1 = op["span"]
            x0 = int((off[side] + a) * sk["ppm"]); x1 = int((off[side] + b) * sk["ppm"])
            y0 = int((sk["H"] - (z1 - sk["z0"])) * sk["ppm"]); y1 = int((sk["H"] - (z0 - sk["z0"])) * sk["ppm"])
            d.rectangle([x0, y0, x1, y1], outline=(60, 220, 240))
            n_drawn += 1
for ev in events:
    col = {"flame": (235, 40, 30), "smoulder": (255, 150, 30), "out": (245, 245, 245), "stain": (60, 220, 240)}[ev["state"]]
    x0 = int((off[ev["side"]] + ev["u0"]) * sk["ppm"]); x1 = int((off[ev["side"]] + ev["u1"]) * sk["ppm"])
    y0 = int((sk["H"] - (ev["z_head"] - sk["z0"])) * sk["ppm"]); y1 = int((sk["H"] - (ev["z_sill"] - sk["z0"])) * sk["ppm"])
    d.rectangle([x0 - 2, y0 - 2, x1 + 2, y1 + 2], outline=col, width=2)
im.save(os.path.join(OUT, "%s_%s_skin_islands.png" % (name, level)))
print("[probe] drew %d window islands and %d events on the skin" % (n_drawn, len(events)))
mesh = gss.read_mesh(st, src, verbose=False)
print("[probe] mesh: %d tri(s), %d material(s); %.1fs" % (len(mesh["MID"]), len(mesh["mats"]), time.time() - t0))
sooted = gf.bake_atlases(st, cell, mesh, sk, m, OUT, verbose=True)
print("[probe] %d sooted key(s); %.1fs total -> %s" % (len(sooted), time.time() - t0, OUT))

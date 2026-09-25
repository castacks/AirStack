"""Write data/recon/pad/PAD_spec.yaml -- the fire-training rig on the grid's south pads (S02-S06).

    ~/.venvs/recon/bin/python gen_industrial_pad.py && ~/.venvs/recon/bin/python build_hero.py data/recon/pad/PAD_spec.yaml

Spec frame = world (origin 0,0,0, yaw 0); each object's base z is the bare
earth at its centre (data/recon/site_rasters.npz). Sizes from the tile mesh
(tools: minAreaRect of the > 1 m blob), identities from image (1)/(2).png:
  S03 rail tank car  18.2 x 3.5 m, top 4.6, yaw 45   -> barrel r 1.5 on two bogies
  S04 sphere vessel  5.5 m across, top 6.6           -> sphere r 2.7 on 4 legs
  S05 pipe manifold  14.7 x 7.1 m, top 5.1, yaw 13   -> 3 pipe runs on a rack + valve posts
  S06 tank on legs   7.9 x 4.7 m, top 9.7, yaw 47    -> horizontal tank on a 4-leg frame
  S02 lattice tower  blob at (80.3, -523.4), top 22.2 (the LABELS point is 9 m off);
                     3 m square, X-braced every 3.5 m, platforms at 7 / 14 / 21 m
  mast               (78.1, -510.3), top 16.8, 3 x 2.2 m -> pole + crow's-nest platform
  cabins x3          (17.6,-504.2) (92.1,-508.5) (58.0,-543.5), 8.1 x 9.8 x 3.6 m at ~-47 deg
                     -> walls + roof + one door each (door side not visible anywhere: guessed)
Thin members are real obstacles for a drone -- that is the point of modelling them.
"""
from pathlib import Path
import numpy as np, yaml

from _paths import R, LABELS
r = np.load(R / "site_rasters.npz"); X0, Y1, RES = float(r["x0"]), float(r["y1"]), float(r["res"])
gz = lambda x, y: float(r["dtm"][int((Y1 - y) / RES), int((x - X0) / RES)])
parts = []
def rot(c, yaw, p):                                                # local (along, across, up) -> world
    a = np.radians(yaw); return [round(float(v), 3) for v in (c[0] + p[0] * np.cos(a) - p[1] * np.sin(a), c[1] + p[0] * np.sin(a) + p[1] * np.cos(a), c[2] + p[2])]
def beam(n, c, yaw, a, b, t=0.2, mat="steel", **k): parts.append({"beam": {"name": n, "from": rot(c, yaw, a), "to": rot(c, yaw, b), "t": t, "mat": mat, **k}})

# S03 rail tank car
c = [42.5, -528.5]; c.append(gz(*c)); L = 16.0
beam("S03_barrel", c, 45, [-L / 2, 0, 3.1], [L / 2, 0, 3.1], t=3.0, mat="tank_black")     # square barrel: cheap, same silhouette from above
for s_ in (-1, 1):
    beam(f"S03_bogie_{s_}", c, 45, [s_ * 6 - 1.3, 0, 0.8], [s_ * 6 + 1.3, 0, 0.8], t=1.2, t2=2.6, mat="rust")
beam("S03_rail_a", c, 45, [-10, -0.72, 0.1], [10, -0.72, 0.1], t=0.15); beam("S03_rail_b", c, 45, [-10, 0.72, 0.1], [10, 0.72, 0.1], t=0.15)
# S04 spherical vessel on legs
c = [31.4, -535.2]; c.append(gz(*c))
parts.append({"sphere": {"name": "S04_vessel", "c": [c[0], c[1], c[2] + 3.9], "r": 2.7, "mat": "tank_white"}})
for i, a_ in enumerate(np.radians([45, 135, 225, 315])):
    parts.append({"cyl": {"name": f"S04_leg{i}", "c": [round(float(v), 3) for v in (c[0] + 2.2 * np.cos(a_), c[1] + 2.2 * np.sin(a_), c[2] + 1.6)], "r": 0.15, "h": 3.2, "mat": "steel"}})
# S05 pipe manifold: rack + three pipe runs + valve posts
c = [45.1, -551.4]; c.append(gz(*c)); yaw = 13
for x in (-6.5, -2, 2.5, 6.5):
    for y in (-2.5, 2.5): beam(f"S05_post_{x}_{y}", c, yaw, [x, y, 0], [x, y, 4.2], t=0.25)
    beam(f"S05_cross_{x}", c, yaw, [x, -2.6, 4.1], [x, 2.6, 4.1], t=0.25)
for i, y in enumerate((-1.5, 0, 1.5)):
    parts.append({"beam": {"name": f"S05_pipe{i}", "from": rot(c, yaw, [-7.3, y, 4.6]), "to": rot(c, yaw, [7.3, y, 4.6]), "t": 0.5, "mat": "rust"}})
    for x in (-4, 0, 4): beam(f"S05_valve{i}_{x}", c, yaw, [x, y, 0], [x, y, 4.3], t=0.35, mat="rust")
# S06 horizontal tank on a 4-leg frame
c = [65.1, -498.8]; c.append(gz(*c)); yaw = 47
for x in (-3, 3):
    for y in (-1.8, 1.8): beam(f"S06_leg_{x}_{y}", c, yaw, [x, y, 0], [x, y, 6.2], t=0.3)
    beam(f"S06_sad_{x}", c, yaw, [x, -1.9, 6.2], [x, 1.9, 6.2], t=0.35)
    beam(f"S06_brace_{x}", c, yaw, [x, -1.8, 0.5], [x, 1.8, 5.8], t=0.12)
beam("S06_tank", c, yaw, [-3.9, 0, 7.9], [3.9, 0, 7.9], t=3.4, mat="tank_white")
beam("S06_deck", c, yaw, [-3.2, 0, 6.1], [3.2, 0, 6.1], t=0.15, t2=4.0, mat="grating")
# S02 lattice tower, 3 m square, 21 m
c = [80.3, -523.4]; c.append(gz(*c)); yaw = 45; H, h2 = 21.0, 1.5
for i, (x, y) in enumerate([(-h2, -h2), (h2, -h2), (h2, h2), (-h2, h2)]):
    beam(f"S02_leg{i}", c, yaw, [x, y, 0], [x, y, H], t=0.2)
for k in range(6):
    z0, z1 = k * 3.5, (k + 1) * 3.5
    for i, ((xa, ya), (xb, yb)) in enumerate([((-h2, -h2), (h2, -h2)), ((h2, -h2), (h2, h2)), ((h2, h2), (-h2, h2)), ((-h2, h2), (-h2, -h2))]):
        beam(f"S02_x{k}_{i}a", c, yaw, [xa, ya, z0], [xb, yb, z1], t=0.08); beam(f"S02_x{k}_{i}b", c, yaw, [xb, yb, z0], [xa, ya, z1], t=0.08)
        beam(f"S02_ring{k}_{i}", c, yaw, [xa, ya, z1], [xb, yb, z1], t=0.1)
for z in (7.0, 14.0, 21.0):
    beam(f"S02_platform_{z:.0f}", c, yaw, [-h2 - 0.3, 0, z], [h2 + 0.3, 0, z], t=0.1, t2=3.6, mat="grating")
# mast with a crow's nest
c = [78.1, -510.3]; c.append(gz(*c))
parts.append({"cyl": {"name": "mast_pole", "c": [c[0], c[1], c[2] + 7.8], "r": 0.35, "h": 15.6, "mat": "steel"}})
parts.append({"cyl": {"name": "mast_nest", "c": [c[0], c[1], c[2] + 16.2], "r": 1.3, "h": 1.2, "mat": "steel"}})
# three identical cabins: walls with one door (on the local -y face) and a roof
for i, cc_ in enumerate([(17.6, -504.2), (92.1, -508.5), (58.0, -543.5)]):
    c = [*cc_, gz(*cc_)]; yaw = -47 + 90; L, W, Hc = 9.8, 8.1, 3.6
    corners = [(-L / 2, -W / 2), (L / 2, -W / 2), (L / 2, W / 2), (-L / 2, W / 2)]
    for k in range(4):
        a, b = corners[k], corners[(k + 1) % 4]; wl = (L if k % 2 == 0 else W)
        beam(f"cabin{i}_wall{k}", c, yaw, [*a, Hc / 2], [*b, Hc / 2], t=Hc, t2=0.2, mat="concrete") if k else None
    # door wall (k = 0): two segments either side of a 1.2 m door + a lintel
    beam(f"cabin{i}_wall0a", c, yaw, [-L / 2, -W / 2, Hc / 2], [-0.6, -W / 2, Hc / 2], t=Hc, t2=0.2, mat="concrete")
    beam(f"cabin{i}_wall0b", c, yaw, [0.6, -W / 2, Hc / 2], [L / 2, -W / 2, Hc / 2], t=Hc, t2=0.2, mat="concrete")
    beam(f"cabin{i}_lintel", c, yaw, [-0.6, -W / 2, 3.0], [0.6, -W / 2, 3.0], t=1.2, t2=0.2, mat="concrete")
    beam(f"cabin{i}_roof", c, yaw, [-L / 2, 0, Hc], [L / 2, 0, Hc], t=0.2, t2=W, mat="concrete")

spec = {"id": "PAD", "name": "industrial_training_pad", "semantic": "building", "origin": [0, 0, 0], "yaw_deg": 0, "parts": parts}
out = R / "pad/PAD_spec.yaml"
out.write_text("# generated by gen_industrial_pad.py -- edit that, not this\n" + yaml.safe_dump(spec, sort_keys=False, default_flow_style=None, width=200))
print(f"{len(parts)} parts -> {out}")

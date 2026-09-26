"""Write data/recon/pad/PAD_spec.yaml -- the fire-training rig on the grid's south pads -- for build_hero.py.

    ~/.venvs/recon/bin/python gen_industrial_pad.py && ~/.venvs/recon/bin/python build_hero.py data/recon/pad/PAD_spec.yaml

Every member is FITTED to its own blob in the 3D tiles, not typed in: the tile
surface is ray-cast at 0.2 m, the connected region standing > `thr` above bare
earth that contains the seed point is taken, and its centre, long-axis
direction (principal axis, in WORLD x/y -- an image-space rectangle fit once
set the tank car 90 deg off), length/width (2-98 percentile extents along the
axes) and top height (98th percentile) drive the primitives below. The seeds
only say which blob is which; identities come from image (2).png and the drone video:

  S03 rail tank car      short track along the blob axis; the car is a library asset (specs/rail_cars.yaml)
  S04 vertical vessel    flared skirt, shell, ring platform, dome (was modelled as a sphere on legs)
  S05 silo + skid        a silo at the blob's high point, a horizontal tank and an equipment skid along it
  S06 vessel on a skid   vertical vessel on a steel skid with a stair (was a horizontal tank on legs)
  S07 raised heater box  long box on two leg frames with four ports on top
  S02 frame tower        four columns, X-braced storeys, railed platforms at 8 / 13 / 16.5 m, an open
                         cage to 20 m, gas cylinders at its foot -- as the drone video shows it (A04)
  mast                   slender process column with three ring platforms and a ladder (A04)
  sheds x3               corrugated gable roof over three walls, the front open
Shapes: the Google oblique image (2).png and the A04 frames; positions and sizes: the blob fits.

Thin members are real obstacles for a drone -- that is why they are modelled.
"""
import cv2, numpy as np, open3d as o3d, yaml
from _paths import R

rs = np.load(R / "site_rasters.npz"); X0, Y1, DRES = float(rs["x0"]), float(rs["y1"]), float(rs["res"])
gz = lambda x, y: float(rs["dtm"][int((Y1 - y) / DRES), int((x - X0) / DRES)])
t = np.load(R / "tiles_site.npz"); scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(o3d.core.Tensor(t["verts"].astype(np.float32)), o3d.core.Tensor(t["faces"].astype(np.uint32)))
RES = 0.2

def fit(seed, thr=1.0, win=14.0, all_near=False):
    """Blob(s) of the tile surface > thr above bare earth around `seed` -> list of
    (centre_xy, yaw_deg of the long axis, length, width, top_height, ground_z)."""
    g = np.arange(-win, win, RES) + RES / 2
    X, Y = np.meshgrid(seed[0] + g, seed[1] - g)
    H = 500 - scene.cast_rays(o3d.core.Tensor(np.stack([X, Y, np.full_like(X, 500), 0 * X, 0 * X, -np.ones_like(X)], -1).astype(np.float32)))["t_hit"].numpy()
    G = rs["dtm"][((Y1 - Y) / DRES).astype(int), ((X - X0) / DRES).astype(int)]
    h = np.nan_to_num(H - G)
    n, cc = cv2.connectedComponents(cv2.morphologyEx((h > thr).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)))
    if all_near:
        ids = [k for k in range(1, n) if (cc == k).sum() * RES * RES > 2 and np.hypot(X[cc == k].mean() - seed[0], Y[cc == k].mean() - seed[1]) < 8]
    else:
        d = np.where(cc > 0, np.hypot(X - seed[0], Y - seed[1]), np.inf); ids = [cc.flat[d.argmin()]]
    out = []
    for k in ids:
        m = cc == k; P = np.c_[X[m], Y[m]]; c = P.mean(0)
        w, v = np.linalg.eigh(np.cov((P - c).T)); ax = v[:, 1]            # principal axis, world frame
        yaw = np.degrees(np.arctan2(ax[1], ax[0]))
        u = (P - c) @ ax; q = (P - c) @ np.array([-ax[1], ax[0]])
        L = np.percentile(u, 98) - np.percentile(u, 2); W = np.percentile(q, 98) - np.percentile(q, 2)
        cc2 = c + ax * (np.percentile(u, 98) + np.percentile(u, 2)) / 2 + np.array([-ax[1], ax[0]]) * (np.percentile(q, 98) + np.percentile(q, 2)) / 2
        out.append((cc2, yaw, L + RES, W + RES, float(np.percentile(h[m], 98)), gz(*cc2)))
    return out

parts, log = [], []
def rot(c, yaw, p):                                                # local (along, across, up) -> world
    a = np.radians(yaw); return [round(float(v), 3) for v in (c[0] + p[0] * np.cos(a) - p[1] * np.sin(a), c[1] + p[0] * np.sin(a) + p[1] * np.cos(a), c[2] + p[2])]
def beam(n, c, yaw, a, b, t=0.2, mat="steel", **k): parts.append({"beam": {"name": n, "from": rot(c, yaw, a), "to": rot(c, yaw, b), "t": t, "mat": mat, **k}})
def note(name, f): log.append(f"{name:8s} centre ({f[0][0]:6.1f},{f[0][1]:7.1f})  yaw {f[1]:6.1f}  {f[2]:5.1f} x {f[3]:4.1f} m  top {f[4]:4.1f} m")

# S03 rail tank car: barrel the blob's length, on two bogies and a short track
f = fit((40.0, -527.0)); note("S03", f[0]); (cx, cy), yaw, L, W, top, g0 = f[0]; c = [cx, cy, g0]
# (the car itself is a library tank car, placed from specs/rail_cars.yaml `pad_s03`; the pad keeps its track)
beam("S03_rail_a", c, yaw, [-L / 2 - 2, -0.72, 0.08], [L / 2 + 2, -0.72, 0.08], t=0.15)
beam("S03_rail_b", c, yaw, [-L / 2 - 2, 0.72, 0.08], [L / 2 + 2, 0.72, 0.08], t=0.15)
def top_xy(seed, win=8.0, thr=1.0):
    """world xy of the highest tile cell near `seed` (a silo's position inside a sprawling blob)"""
    g = np.arange(-win, win, RES) + RES / 2; X, Y = np.meshgrid(seed[0] + g, seed[1] - g)
    H = 500 - scene.cast_rays(o3d.core.Tensor(np.stack([X, Y, np.full_like(X, 500), 0 * X, 0 * X, -np.ones_like(X)], -1).astype(np.float32)))["t_hit"].numpy()
    k = np.nanargmax(np.where(np.isfinite(H), H, -1e9)); return np.array([X.flat[k], Y.flat[k]])
def cyl(n, c, yaw, p, r, h, mat="metal_white", axis="Z"):
    parts.append({"cyl": {"name": n, "c": rot(c, yaw, p), "r": r, "h": h, "axis": axis, "mat": mat}})
def box(n, c, yaw, a, b, mat="metal_grey"):                       # an axis-aligned box in the member's own frame
    (x0, y0, z0), (x1, y1, z1) = a, b
    beam(n, c, yaw, [x0, (y0 + y1) / 2, (z0 + z1) / 2], [x1, (y0 + y1) / 2, (z0 + z1) / 2], t=z1 - z0, t2=y1 - y0, mat=mat)
def ladder(n, c, yaw, x, y, z0, z1):
    for k, dy in enumerate((-0.22, 0.22)): beam(f"{n}_side{k}", c, yaw, [x, y + dy, z0], [x, y + dy, z1], t=0.05)
    for k, z in enumerate(np.arange(z0 + 0.3, z1, 0.6)): beam(f"{n}_rung{k}", c, yaw, [x, y - 0.22, z], [x, y + 0.22, z], t=0.04)
def ring_platform(n, c, yaw, z, r):
    cyl(f"{n}_deck", c, yaw, [0, 0, z], r, 0.08, "grating")
    for k in range(8):
        a0, a1 = 2 * np.pi * k / 8, 2 * np.pi * (k + 1) / 8
        beam(f"{n}_rail{k}", c, yaw, [r * np.cos(a0), r * np.sin(a0), z + 1.05], [r * np.cos(a1), r * np.sin(a1), z + 1.05], t=0.05)
        beam(f"{n}_post{k}", c, yaw, [r * np.cos(a0), r * np.sin(a0), z], [r * np.cos(a0), r * np.sin(a0), z + 1.05], t=0.05)

# Shapes from the Google oblique image (2).png and, for the north pad, the drone video (A04 at 0-4 s):
# S04 south pad: a vertical vessel on a flared skirt, ring platform near the top, dome cap
f = fit((31.5, -535.2)); note("S04", f[0]); (cx, cy), yaw, L, W, top, g0 = f[0]; c = [cx, cy, g0]
R4 = 1.5; zt = top - R4
cyl("S04_skirt", c, 0, [0, 0, 0.5], R4 + 0.7, 1.0); cyl("S04_shell", c, 0, [0, 0, (1.0 + zt) / 2], R4, zt - 1.0)
parts.append({"sphere": {"name": "S04_dome", "c": rot(c, 0, [0, 0, zt]), "r": R4, "mat": "metal_white"}})
ring_platform("S04_ring", c, 0, zt - 0.6, R4 + 0.7); ladder("S04_ladder", c, 0, R4 + 0.2, 0, 0, zt - 0.6)
# S05 south pad: a vertical silo at the blob's highest point, a horizontal tank and an equipment skid along the blob
f = fit((44.7, -551.4)); note("S05", f[0]); (cx, cy), yaw, L, W, top, g0 = f[0]; c = [cx, cy, g0]
a_ = np.radians(yaw); ax = np.array([np.cos(a_), np.sin(a_)]); su = float((top_xy((cx, cy)) - [cx, cy]) @ ax); sg = 1 if su >= 0 else -1
cyl("S05_silo", c, yaw, [su, 0, top / 2], 1.1, top); ladder("S05_silo_ladder", c, yaw, su, 1.3, 0, top)
cyl("S05_tank", c, yaw, [su - sg * 4.0, 0, 1.3], 0.75, 4.5, axis="X")
for k, dx in enumerate((-1.5, 1.5)): box(f"S05_saddle{k}", c, yaw, [su - sg * 4.0 + dx - 0.15, -0.6, 0], [su - sg * 4.0 + dx + 0.15, 0.6, 0.7], "steel")
beam("S05_pipe", c, yaw, [su - sg * 1.1, 0.4, 2.0], [su - sg * 7.0, 0.4, 2.0], t=0.25, mat="steel")
bx = su - sg * (L / 2 + 2.0)
box("S05_skid", c, yaw, [bx - 2.0, -1.4, 0], [bx + 2.0, 1.4, 2.6], "metal_white"); ladder("S05_skid_ladder", c, yaw, bx + 2.1, 0, 0, 2.6)
# S06 north pad: a vertical vessel on a steel skid, with a stair up to the skid
f = fit((65.9, -497.6)); note("S06", f[0]); (cx, cy), yaw, L, W, top, g0 = f[0]; c = [cx, cy, g0]
box("S06_skid", c, yaw, [-L / 2, -W / 2, 0], [L / 2, W / 2, 0.9], "steel_dark")
R6 = 1.3; cyl("S06_vessel", c, yaw, [L / 4, 0, (0.9 + top - R6) / 2], R6, top - R6 - 0.9)
parts.append({"sphere": {"name": "S06_dome", "c": rot(c, yaw, [L / 4, 0, top - R6]), "r": R6, "mat": "metal_white"}})
ladder("S06_ladder", c, yaw, L / 4 + R6 + 0.2, 0, 0.9, top - R6); ring_platform("S06_ring", [*rot(c, yaw, [L / 4, 0, 0])[:2], g0], 0, 5.0, R6 + 0.6)
parts.append({"stair": {"name": "S06_stair", "from": rot(c, yaw, [-L / 2 - 1.8, 0, 0]), "to": rot(c, yaw, [-L / 2, 0, 0.9]), "width": 1.0, "mat": "grating"}})
# S07 north pad: a long box heater raised on two leg frames, four ports on its top, a stub stack
f = fit((78.7, -495.2)); note("S07", f[0]); (cx, cy), yaw, L, W, top, g0 = f[0]; c = [cx, cy, g0]
zb = top - 2.8; box("S07_box", c, yaw, [-L / 2 + 0.4, -W / 2 + 0.6, zb], [L / 2 - 0.4, W / 2 - 0.6, top - 0.3], "metal_grey")
for k, x in enumerate(np.linspace(-L / 3, L / 3, 4)): cyl(f"S07_port{k}", c, yaw, [x, 0, top - 0.15], 0.45, 0.3, "steel_dark")
for k, x in enumerate((-L / 2 + 1.5, L / 2 - 1.5)):
    for j, y in enumerate((-W / 2 + 0.7, W / 2 - 0.7)): beam(f"S07_leg{k}{j}", c, yaw, [x, y, 0], [x, y, zb], t=0.3)
    beam(f"S07_legbrace{k}", c, yaw, [x, -W / 2 + 0.7, 0.3], [x, W / 2 - 0.7, zb - 0.2], t=0.12)
# mast (north pad): a slender process column with three ring platforms and a ladder (A04: beside the frame tower)
f = fit((78.1, -510.3)); note("mast", f[0]); (cx, cy), _, _, _, top, g0 = f[0]; c = [cx, cy, g0]
cyl("mast_column", c, 0, [0, 0, top / 2], 0.55, top, "metal_grey")
for k, z in enumerate((top * 0.33, top * 0.66, top - 1.2)): ring_platform(f"mast_ring{k}", c, 0, z, 1.4)
ladder("mast_ladder", c, 0, 0.9, 0, 0, top - 1.2)
# S02 north pad: the square steel frame tower (A04: four columns, X-braced lower storeys, two railed platforms, an open cage on top)
f = fit((80.3, -523.4)); note("S02", f[0]); (cx, cy), yaw, L, W, _, g0 = f[0]; c = [cx, cy, g0]
a2, b2, H2 = min(L, 6.0) / 2, min(W, 5.0) / 2, 20.0
corners = [(-a2, -b2), (a2, -b2), (a2, b2), (-a2, b2)]
for i, (x, y) in enumerate(corners): beam(f"S02_col{i}", c, yaw, [x, y, 0], [x, y, H2], t=0.3)
levels = [0.3, 8.0, 13.0, 16.5, H2]
for k, z in enumerate(levels[1:]):
    for i in range(4):
        (xa, ya), (xb, yb) = corners[i], corners[(i + 1) % 4]; beam(f"S02_ring{k}_{i}", c, yaw, [xa, ya, z], [xb, yb, z], t=0.25)
for k, (z0, z1) in enumerate(zip(levels, levels[1:])):
    if k == 2: continue                                               # the railed storey under the cage is open
    for i in range(4):
        (xa, ya), (xb, yb) = corners[i], corners[(i + 1) % 4]
        beam(f"S02_x{k}_{i}a", c, yaw, [xa, ya, z0], [xb, yb, z1], t=0.12); beam(f"S02_x{k}_{i}b", c, yaw, [xb, yb, z0], [xa, ya, z1], t=0.12)
for k, z in enumerate((8.0, 13.0, 16.5)):
    box(f"S02_deck{k}", c, yaw, [-a2, -b2, z], [a2, b2, z + 0.08], "grating")
    for i in range(4):
        (xa, ya), (xb, yb) = corners[i], corners[(i + 1) % 4]
        beam(f"S02_rail{k}_{i}", c, yaw, [xa, ya, z + 1.1], [xb, yb, z + 1.1], t=0.05); beam(f"S02_midrail{k}_{i}", c, yaw, [xa, ya, z + 0.55], [xb, yb, z + 0.55], t=0.04)
ladder("S02_ladder", c, yaw, a2 - 0.4, 0, 0, 16.5)
for k in range(6): cyl(f"S02_gas{k}", c, yaw, [-a2 + 0.6 + 0.5 * (k % 3), b2 + 0.8 + 0.5 * (k // 3), 0.75], 0.22, 1.5, "yellow")
# a small white equipment box in the middle of the north pad (image (2))
f = fit((62.0, -514.0), thr=0.5, win=4.0); note("prop", f[0]); (cx, cy), yaw, L, W, top, g0 = f[0]
box("prop_box", [cx, cy, g0], yaw, [-L / 2, -W / 2, 0], [L / 2, W / 2, max(top, 0.9)], "metal_white")
# three sheds: corrugated gable roof over three walls, the long front open
for i, seed in enumerate([(17.6, -504.2), (92.1, -508.5), (58.0, -543.5)]):
    f = fit(seed); note(f"shed{i}", f[0]); (cx, cy), yaw, L, W, top, g0 = f[0]; c = [cx, cy, g0]; eave = top - 0.9
    box(f"shed{i}_back", c, yaw, [-L / 2, W / 2 - 0.1, 0], [L / 2, W / 2, eave])
    for k, x in enumerate((-L / 2, L / 2 - 0.1)): box(f"shed{i}_end{k}", c, yaw, [x, -W / 2, 0], [x + 0.1, W / 2, eave])
    for k, x in enumerate(np.linspace(-L / 2 + 0.1, L / 2 - 0.1, 4)): beam(f"shed{i}_post{k}", c, yaw, [x, -W / 2 + 0.1, 0], [x, -W / 2 + 0.1, eave], t=0.15, mat="steel")
    for k, sgn in enumerate((-1, 1)):
        beam(f"shed{i}_roof{k}", c, yaw, [0, sgn * (W / 2 + 0.3), eave - 0.1], [0, 0, top], t=0.08, t2=L + 0.4, mat="metal_ribbed")
def plain(o):                                                     # numpy scalars -> python, rounded, for yaml
    if isinstance(o, dict): return {k: plain(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)): return [plain(v) for v in o]
    if isinstance(o, (float, np.floating)): return round(float(o), 3)
    return o
parts = plain(parts)
spec = {"id": "PAD", "name": "industrial_training_pad", "semantic": "building", "origin": [0, 0, 0], "yaw_deg": 0, "parts": parts}
out = R / "pad/PAD_spec.yaml"; out.parent.mkdir(parents=True, exist_ok=True)
out.write_text("# generated by gen_industrial_pad.py -- edit that, not this\n# fitted:\n" + "".join(f"#   {l}\n" for l in log)
               + yaml.safe_dump(spec, sort_keys=False, default_flow_style=None, width=200))
print("\n".join(log)); print(f"{len(parts)} parts -> {out}")

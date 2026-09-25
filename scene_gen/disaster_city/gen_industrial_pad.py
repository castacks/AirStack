"""Write data/recon/pad/PAD_spec.yaml -- the fire-training rig on the grid's south pads -- for build_hero.py.

    ~/.venvs/recon/bin/python gen_industrial_pad.py && ~/.venvs/recon/bin/python build_hero.py data/recon/pad/PAD_spec.yaml

Every member is FITTED to its own blob in the 3D tiles, not typed in: the tile
surface is ray-cast at 0.2 m, the connected region standing > `thr` above bare
earth that contains the seed point is taken, and its centre, long-axis
direction (principal axis, in WORLD x/y -- an image-space rectangle fit once
set the tank car 90 deg off), length/width (2-98 percentile extents along the
axes) and top height (98th percentile) drive the primitives below. The seeds
only say which blob is which; identities come from image (1)/(2).png:

  S03 rail tank car      short track along the blob axis; the car is a library asset (specs/rail_cars.yaml)
  S04 spherical vessel   sphere on 4 legs, radius from the blob
  S05 pipe manifold      pipe segments along the skeleton of its (Y-shaped) blobs
  S06 tank on legs       horizontal tank on a 4-leg frame
  S07 prop at (79,-497)  an elongated training prop -- modelled as a box of its fitted size
  S02 lattice tower      too thin for the tiles to measure: 3 m square, 21 m, X-braced
  mast                   pole + crow's nest at its blob
  cabins x3              walls + roof + one door each (the door side is not visible anywhere: guessed)

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
# S04 spherical vessel on legs
f = fit((31.5, -535.2)); note("S04", f[0]); (cx, cy), yaw, L, W, top, g0 = f[0]
r = (L + W) / 4
parts.append({"sphere": {"name": "S04_vessel", "c": [round(float(cx), 3), round(float(cy), 3), round(top - r, 3)], "r": round(r, 3), "mat": "tank_white"}})
for i, a_ in enumerate(np.radians([45, 135, 225, 315])):
    parts.append({"cyl": {"name": f"S04_leg{i}", "c": [round(float(v), 3) for v in (cx + 0.8 * r * np.cos(a_), cy + 0.8 * r * np.sin(a_), g0 + (top - r) / 2)],
                          "r": 0.15, "h": round(top - r, 3), "mat": "steel"}})
# S05 pipe manifold (Y-shaped in plan): pipe runs along the SKELETON of its blobs, each segment
# as thick as the blob is there, on posts every ~3 m
from skimage.morphology import skeletonize
def skeleton_segments(seed, thr=1.0, win=12.0, step=1.5):
    g = np.arange(-win, win, RES) + RES / 2
    X, Y = np.meshgrid(seed[0] + g, seed[1] - g)
    H = 500 - scene.cast_rays(o3d.core.Tensor(np.stack([X, Y, np.full_like(X, 500), 0 * X, 0 * X, -np.ones_like(X)], -1).astype(np.float32)))["t_hit"].numpy()
    G = rs["dtm"][((Y1 - Y) / DRES).astype(int), ((X - X0) / DRES).astype(int)]; h = np.nan_to_num(H - G)
    m = cv2.morphologyEx((h > thr).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, cc = cv2.connectedComponents(m)
    keep = [k for k in range(1, n) if (cc == k).sum() * RES * RES > 2 and np.hypot(X[cc == k].mean() - seed[0], Y[cc == k].mean() - seed[1]) < 8]
    m = np.isin(cc, keep); sk = skeletonize(m); dist = cv2.distanceTransform(m.astype(np.uint8), cv2.DIST_L2, 5) * RES
    ys, xs = np.nonzero(sk); pts = np.c_[X[ys, xs], Y[ys, xs]]; wid = 2 * dist[ys, xs]; top = h[ys, xs]
    # chain the skeleton into ~step m segments: greedy nearest-neighbour walks
    used = np.zeros(len(pts), bool); segs = []
    for start in np.argsort(-np.linalg.norm(pts - pts.mean(0), axis=1)):
        if used[start]: continue
        chain = [start]; used[start] = True
        while True:
            d = np.linalg.norm(pts - pts[chain[-1]], axis=1); d[used] = np.inf
            j = int(d.argmin())
            if d[j] > 2 * RES: break
            used[j] = True; chain.append(j)
        for a in range(0, len(chain) - 1, max(1, int(step / RES))):
            b = chain[min(a + int(step / RES), len(chain) - 1)]
            if np.linalg.norm(pts[b] - pts[chain[a]]) > 0.4:
                segs.append((pts[chain[a]], pts[b], float(np.median(wid[chain[a:b + 1] if b > chain[a] else [b]])), float(np.percentile(top[[chain[a], b]], 50))))
    return segs, float(G.mean())
segs, g0 = skeleton_segments((43.0, -551.0))
log.append(f"S05      skeleton: {len(segs)} pipe segments, total {sum(np.linalg.norm(b - a) for a, b, _, _ in segs):.1f} m")
for i, (a, b, w, top) in enumerate(segs):
    za = gz(*a); t_ = float(np.clip(w, 0.3, 1.0))
    parts.append({"beam": {"name": f"S05_pipe{i}", "from": [float(a[0]), float(a[1]), za + max(top - t_ / 2, t_ / 2)],
                           "to": [float(b[0]), float(b[1]), za + max(top - t_ / 2, t_ / 2)], "t": t_, "mat": "rust"}})
    if i % 2 == 0 and top > 1.2:
        parts.append({"beam": {"name": f"S05_post{i}", "from": [float(a[0]), float(a[1]), za], "to": [float(a[0]), float(a[1]), za + top - t_], "t": 0.25, "mat": "steel"}})
# S06 horizontal tank on a 4-leg frame
f = fit((65.9, -497.6)); note("S06", f[0]); (cx, cy), yaw, L, W, top, g0 = f[0]; c = [cx, cy, g0]
D = min(W, 3.4); zt = top - D / 2
for x in (-L / 2 + 0.6, L / 2 - 0.6):
    for y in (-W / 2 + 0.3, W / 2 - 0.3): beam(f"S06_leg_{x:.1f}_{y:.1f}", c, yaw, [x, y, 0], [x, y, zt - D / 2], t=0.3)
    beam(f"S06_sad_{x:.1f}", c, yaw, [x, -W / 2, zt - D / 2], [x, W / 2, zt - D / 2], t=0.35)
beam("S06_tank", c, yaw, [-L / 2, 0, zt], [L / 2, 0, zt], t=D, mat="tank_white")
# S07 elongated prop at (79,-497): a box of its fitted size
f = fit((78.7, -495.2)); note("S07", f[0]); (cx, cy), yaw, L, W, top, g0 = f[0]; c = [cx, cy, g0]
beam("S07_prop", c, yaw, [-L / 2, 0, top / 2], [L / 2, 0, top / 2], t=top, t2=W, mat="rust")
# S02 lattice tower, 3 m square, 21 m (centre from its blob; too thin to size)
f = fit((80.3, -523.4)); note("S02", f[0]); (cx, cy), _, _, _, _, g0 = f[0]; c = [cx, cy, g0]; yaw = 45; H, h2 = 21.0, 1.5
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
f = fit((78.1, -510.3)); note("mast", f[0]); (cx, cy), _, _, _, top, g0 = f[0]
parts.append({"cyl": {"name": "mast_pole", "c": [round(float(cx), 3), round(float(cy), 3), round(g0 + (top - 0.6) / 2, 3)], "r": 0.35, "h": round(top - 0.6, 3), "mat": "steel"}})
parts.append({"cyl": {"name": "mast_nest", "c": [round(float(cx), 3), round(float(cy), 3), round(g0 + top - 0.6, 3)], "r": 1.3, "h": 1.2, "mat": "steel"}})
# three cabins: walls with one door (local -y face) and a roof, each at its own fitted box
for i, seed in enumerate([(17.6, -504.2), (92.1, -508.5), (58.0, -543.5)]):
    f = fit(seed); note(f"cabin{i}", f[0]); (cx, cy), yaw, L, W, top, g0 = f[0]; c = [cx, cy, g0]; Hc = top
    corners = [(-L / 2, -W / 2), (L / 2, -W / 2), (L / 2, W / 2), (-L / 2, W / 2)]
    for k in (1, 2, 3):
        a, b = corners[k], corners[(k + 1) % 4]
        beam(f"cabin{i}_wall{k}", c, yaw, [*a, Hc / 2], [*b, Hc / 2], t=Hc, t2=0.2, mat="concrete")
    beam(f"cabin{i}_wall0a", c, yaw, [-L / 2, -W / 2, Hc / 2], [-0.6, -W / 2, Hc / 2], t=Hc, t2=0.2, mat="concrete")
    beam(f"cabin{i}_wall0b", c, yaw, [0.6, -W / 2, Hc / 2], [L / 2, -W / 2, Hc / 2], t=Hc, t2=0.2, mat="concrete")
    beam(f"cabin{i}_lintel", c, yaw, [-0.6, -W / 2, (Hc + 2.1) / 2], [0.6, -W / 2, (Hc + 2.1) / 2], t=Hc - 2.1, t2=0.2, mat="concrete")
    beam(f"cabin{i}_roof", c, yaw, [-L / 2, 0, Hc - 0.1], [L / 2, 0, Hc - 0.1], t=0.2, t2=W, mat="concrete")

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

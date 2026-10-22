"""Where a survivor could be in the search area -> data/recon/survivors/placement_candidates.json.

    ~/.venvs/recon/bin/python survivor_candidates.py      # after scene_mesh.py

Against the people-free scene's triangles (scene_mesh.npz), on a 1 m grid over the search area,
every surface a vertical ray meets is a possible support. One is kept if:

  flat      level (normal within ~25 deg of up) and level 0.6 m round it, so a person lying in
            any direction is supported; not on a tree or a vehicle
  room      a sitting person fits (nothing within 0.3 m of a vertical axis 0.45-1.0 m up), and
            for lying, the directions (of 8) in which a 1.7 m body 0.25 m up clears by 0.2 m
  seen      from 0.4 m above it, some ray rising 10-70 deg reaches the open sky: a person nobody
            can see from the air (buried, deep inside) is not a search task

and is classed by where it is, so variants keep the video's mix of difficulty:

  ground    open, on the terrain            elevated  open, >= 1.5 m above the terrain (decks, roofs)
  covered   a structure within 8 m above it (under a frame, inside a building)
  canopy    a tree within 8 m above it

The survivors seen in the video (specs/survivors.yaml) are classed the same way; the class mix is
written next to the candidates for sample_survivors.py. Every check is geometry; the people
themselves are checked again once posed (verify_people.py).
"""
import json
import math
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml

from _paths import R, SPECS

AREA = (-30.0, 90.0, -460.0, -360.0)      # the operator's default search area, twin metres
STEP = 1.0
NOT_SUPPORT = ("trees", "vehicles")
OUT = R / "survivors"

m = np.load(OUT / "scene_mesh.npz")
V, F, G, groups = m["vertices"], m["triangles"], m["group"], [str(g) for g in m["groups"]]
scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(o3d.core.Tensor(V), o3d.core.Tensor(F.astype(np.uint32)))
n = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
nz = np.abs(n[:, 2]) / np.maximum(np.linalg.norm(n, axis=1), 1e-12)
ras = np.load(R / "site_rasters.npz")


def terrain(x, y):
    r = np.clip((float(ras["y1"]) - np.asarray(y)) / float(ras["res"]), 0, ras["dtm"].shape[0] - 1).astype(int)
    c = np.clip((np.asarray(x) - float(ras["x0"])) / float(ras["res"]), 0, ras["dtm"].shape[1] - 1).astype(int)
    return ras["dtm"][r, c]


def all_hits(origins, dirs):
    """Every hit of every ray: list (per ray) of (t, triangle) sorted by t."""
    rays = o3d.core.Tensor(np.c_[origins, dirs].astype(np.float32))
    res = scene.list_intersections(rays)
    split, t, prim = res["ray_splits"].numpy(), res["t_hit"].numpy(), res["primitive_ids"].numpy()
    out = []
    for i in range(len(origins)):
        a, b = split[i], split[i + 1]
        o = np.argsort(t[a:b])
        out.append(list(zip(t[a:b][o], prim[a:b][o])))
    return out


def first_hit_z(xy, z0):
    """z of the first surface below (x, y, z0), nan if none."""
    o = np.c_[xy, np.full(len(xy), z0)]
    t = scene.cast_rays(o3d.core.Tensor(np.c_[o, np.tile([0, 0, -1.0], (len(o), 1))].astype(np.float32)))["t_hit"].numpy()
    return np.where(np.isfinite(t), z0 - t, np.nan)


def supports(x, y):
    """[(z, triangle, headroom, triangle above)] for every level surface the vertical line at (x, y) meets."""
    top = 140.0
    hits = all_hits(np.array([[x, y, top]]), np.array([[0, 0, -1.0]]))[0]
    out, above, above_tri = [], None, None
    for t, tri in hits:
        z = top - t
        if nz[tri] >= 0.9 and (above is None or above - z > 0.6):
            out.append((float(z), int(tri), float(above - z) if above is not None else math.inf, above_tri))
        above, above_tri = float(z), int(tri)
    return out


def dist(pts):
    return scene.compute_distance(o3d.core.Tensor(np.asarray(pts, np.float32))).numpy()


RING = np.array([[math.cos(a), math.sin(a)] for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)])
YAWS = np.arange(0, 360, 45)
SKY = np.array([[math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)]
                for e in np.radians([10, 25, 40, 55, 70]) for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)])


def assess(x, y, z, tri, headroom, over):
    """The candidate record for a support, or None if a person cannot be there."""
    if groups[G[tri]] in NOT_SUPPORT:
        return None
    ring = first_hit_z(np.array([x, y]) + 0.6 * RING, z + 0.5)
    if not np.all(np.abs(ring - z) < 0.25):
        return None                                                     # an edge, a step or a hole
    sit = headroom > 1.1 and dist([[x, y, z + h] for h in (0.45, 0.75, 1.0)]).min() >= 0.3
    lie = [int(a) for a in YAWS if headroom > 0.6 and dist(
        [[x + d * math.cos(math.radians(a)), y + d * math.sin(math.radians(a)), z + 0.25]
         for d in np.linspace(-0.85, 0.85, 9)]).min() >= 0.2]
    if not sit and not lie:
        return None
    t = scene.cast_rays(o3d.core.Tensor(np.c_[np.tile([x, y, z + 0.4], (len(SKY), 1)), SKY].astype(np.float32)))["t_hit"].numpy()
    sky = float(np.isinf(t).mean())
    if sky == 0.0:
        return None                                                     # nobody can see in
    elevated = z - float(terrain(x, y)) >= 1.5
    return {"position": [round(x, 2), round(y, 2), round(z, 2)], "surface": groups[G[tri]],
            "class": klass(headroom, over, elevated),
            "sit": bool(sit), "lying_yaws_deg": lie, "sky_view": round(sky, 3), "headroom_m": round(min(headroom, 99.0), 2)}


def klass(headroom, over, elevated):
    if headroom < 8.0:
        return "canopy" if groups[G[over]] == "trees" else "covered"
    return "elevated" if elevated else "ground"


cands = []
for x in np.arange(AREA[0] + STEP / 2, AREA[1], STEP):
    for y in np.arange(AREA[2] + STEP / 2, AREA[3], STEP):
        for z, tri, head, over in supports(x, y):
            c = assess(x, y, z, tri, head, over)
            if c:
                c["id"] = f"c{len(cands):05d}"
                cands.append(c)

# the video's survivors, classed the same way (the support nearest their recorded height)
video = []
for s in yaml.safe_load(open(SPECS / "survivors.yaml")):
    x, y, z = s["at"]
    sup = min(supports(x, y), key=lambda k: abs(k[0] - z), default=None)
    cls = klass(sup[2], sup[3], z - float(terrain(x, y)) >= 1.5) if sup else \
        ("elevated" if z - float(terrain(x, y)) >= 1.5 else "ground")
    video.append({"id": s["id"], "class": cls, "surface": groups[G[sup[1]]] if sup else None})
mix = {c: sum(v["class"] == c for v in video) for c in ("ground", "elevated", "covered", "canopy")}

OUT.mkdir(exist_ok=True)
(OUT / "placement_candidates.json").write_text(json.dumps({
    "schema_version": 1, "world_frame": "world", "area": AREA, "grid_m": STEP,
    "scene": "disaster_city_empty.usda", "video_class_mix": mix, "video": video, "candidates": cands}, indent=1))
counts = {c: sum(k["class"] == c for k in cands) for c in mix}
print(f"{len(cands)} candidates {counts}; the video's survivors: {mix}")
for v in video:
    print(" ", v)

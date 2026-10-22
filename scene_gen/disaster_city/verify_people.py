"""Check each posed survivor against the scene -> data/recon/survivors/v<k>/rejected.json.

    ~/.venvs/recon/bin/python verify_people.py 0 1 2 3 4 5    # after place_people.py wrote people[_v<k>].usd

Over each person's skinned body surface (people_v<k>.usd.points.npz, 4000 points; v0 is the
  sunk      more than 5% of the body over 0.15 m under that surface (the body is in the floor;
            a few hands or a face pressed into bumpy tile terrain is how the video's own
            survivors lie, so the bar is set on them: all 11 pass, the worst at 4.6%)
  touching  5+ points more than 0.12 m above their support yet within 3 cm of a scene surface:
            something other than the floor passes through or against the body
sample_survivors.py redraws. v0 is checked as the control and never redrawn.
"""
import json
import sys

import numpy as np
import open3d as o3d
import yaml

from _paths import R, SPECS

OUT = R / "survivors"
m = np.load(OUT / "scene_mesh.npz")
scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(o3d.core.Tensor(m["vertices"]), o3d.core.Tensor(m["triangles"].astype(np.uint32)))


def check(p):
    down = np.c_[p + [0, 0, 0.3], np.tile([0, 0, -1.0], (len(p), 1))].astype(np.float32)
    t = scene.cast_rays(o3d.core.Tensor(down))["t_hit"].numpy()
    above_support = (p[:, 2] + 0.3) - t - p[:, 2]                 # how far the support is above the point
    sunk = np.isfinite(t) & (above_support > 0.15)
    loose = ~np.isfinite(t) | (-above_support > 0.12)
    d = scene.compute_distance(o3d.core.Tensor(p.astype(np.float32))).numpy()
    return int(sunk.sum()), int((loose & (d < 0.03)).sum())


failed = 0
for k in (int(a) for a in sys.argv[1:]):
    layer = "people.usd" if k == 0 else f"people_v{k}.usd"
    spec = yaml.safe_load(open(SPECS / "survivors.yaml" if k == 0 else OUT / f"v{k}" / "survivors.yaml"))
    people = {p["id"]: p for p in spec}
    pts = np.load(R / f"{layer}.points.npz")
    rejected_path = OUT / f"v{k}" / "rejected.json"
    rejected = set(json.loads(rejected_path.read_text())) if rejected_path.exists() else set()
    bad = []
    for pid in pts.files:
        sunk, touching = check(pts[pid])
        if sunk > 0.05 * len(pts[pid]) or touching >= 5:
            bad.append(f"{pid}({f'sunk {sunk}' if sunk > 0.05 * len(pts[pid]) else f'touching {touching}'})")
            if k:
                rejected.add(people[pid]["candidate"])
    if k:
        rejected_path.write_text(json.dumps(sorted(rejected)))
        failed += len(bad)
    print(f"v{k}: {len(pts.files) - len(bad)}/{len(pts.files)} clear" + (f"; {', '.join(bad)}" if bad else ""))
sys.exit(1 if failed else 0)

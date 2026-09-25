"""Refine one reconstruction's alignment against another that is already aligned.

    ~/.venvs/recon/bin/python recon_icp.py data/recon/b01 data/recon/rubble_west --at 45.6,-409.8 [--radius 25]

For a second set of flights over ground an earlier reconstruction already
covers: start from <dir>/to_world.json (the 2-corner --pairs result of
recon_align.py), then scaled ICP (2 -> 0.25 m) of both clouds within `radius`
of `at` (world XY). This is how B01 was placed: its corner picks were ~2-4 m
off, and ICP against the R01 cloud (same flights, B01 in both) took it to
0.15 m RMSE. Overwrites <dir>/to_world.json (the input is kept as to_world_pairs.json).
"""
import argparse, json, shutil
from pathlib import Path
import numpy as np, open3d as o3d

reg = o3d.pipelines.registration
ap = argparse.ArgumentParser(); ap.add_argument("dir"); ap.add_argument("ref")
ap.add_argument("--at", required=True); ap.add_argument("--radius", type=float, default=25.0)
a = ap.parse_args(); d, ref = Path(a.dir), Path(a.ref); c = np.array(list(map(float, a.at.split(","))))

def load(p, T):
    pc = o3d.io.read_point_cloud(str(p / "dense/fused.ply")); pc.transform(T); P = np.asarray(pc.points)
    return pc.select_by_index(np.flatnonzero(np.hypot(*(P[:, :2] - c).T) < a.radius)).voxel_down_sample(0.15)

if not (d / "to_world_pairs.json").exists(): shutil.copy(d / "to_world.json", d / "to_world_pairs.json")
T0 = np.array(json.load(open(d / "to_world_pairs.json"))["recon_to_world"])
tgt = load(ref, np.array(json.load(open(ref / "to_world.json"))["recon_to_world"]))
src = load(d, T0)
M = np.eye(4)
for thr in (2.0, 1.0, 0.5, 0.25):
    r = reg.registration_icp(src, tgt, thr, M, reg.TransformationEstimationPointToPoint(True), reg.ICPConvergenceCriteria(max_iteration=100))
    M = r.transformation
    print(f"ICP {thr} m: fitness {r.fitness:.2f}, rmse {r.inlier_rmse:.3f} m, scale x{np.cbrt(np.linalg.det(M[:3, :3])):.3f}")
json.dump({"recon_to_world": (M @ T0).tolist(), "method": f"ICP (with scale) to {ref.name}", "fitness": r.fitness, "rmse": r.inlier_rmse},
          open(d / "to_world.json", "w"), indent=1)

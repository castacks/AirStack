#!/usr/bin/env python3
"""Score RayFronts' person channel AGAINST THE REAL CASUALTY POSITIONS.

WHY THIS EXISTS
`voxel clusters=N` is not evidence. A run can report clusters while every one
of them sits on a bush, and it can report zero while a casualty was perfectly
mapped but half a point under threshold. Counting detections answers neither
"is it right" nor "what threshold should I use". This does:

  * converts every recorded voxel cloud into WORLD ENU, the frame GT_people.json
    is written in,
  * for each of the 30 known casualties, reports the BEST person score any
    voxel near it ever reached, and whether the drone was ever close enough to
    see it at all,
  * sweeps the threshold and reports, at each one, how many real casualties are
    recovered (TP) versus how many above-threshold clusters land on nothing
    (FP) — i.e. precision/recall, not a count.

FRAME MATH (verified against the code, not assumed)
  the cloud is published in RDF (camera-optical: x=right, y=down, z=forward)
  raven_nav/behaviors/common.py:32  rdf_to_flu -> [z, -x, -y]
  world_xy = flu_xy + boot_enu_xy   (boot_enu from /robot_N/rayfronts/status)
  world_z  = flu_z                  (the status' z offset is measured but
                                     NOT applied — multi_ros logs
                                     "measured z offset 53.85 m, not applied")

USAGE
  uv run --no-project --with mcap --with mcap-ros2-support --with numpy \
      python3 scripts/raven_live/score_vs_gt.py <bag_dir> [--people <json>]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

# A voxel this far (metres, world XY) from a casualty counts as "on" it.
# vox_size is 0.5 m and a lying body is ~1.8 m long, so 2.0 m is one body
# length plus a voxel — tight enough that a hit on the next-door bush does not
# score, loose enough to survive the anchor's own error.
MATCH_RADIUS_M = 2.0
# How close the drone must have passed for a casualty to be considered
# OBSERVABLE at all. Beyond this a miss says nothing about the threshold —
# it says the search never went there. depth_limit is 20 m.
SEEN_RADIUS_M = 25.0
THRESHOLDS = (0.7, 0.6, 0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15, 0.1)


def rdf_to_world(xyz_rdf: np.ndarray, boot: np.ndarray) -> np.ndarray:
    """RDF -> FLU -> world ENU. Mirrors raven's own conversion exactly."""
    flu = np.stack([xyz_rdf[:, 2], -xyz_rdf[:, 0], -xyz_rdf[:, 1]], axis=1)
    out = flu.copy()
    out[:, 0] += boot[0]
    out[:, 1] += boot[1]
    return out


def pc2_xyz_sim(msg):
    """(N,3) xyz + (N,K) sim columns from a float32 PointCloud2."""
    names = [f.name for f in msg.fields]
    offs = [f.offset for f in msg.fields]
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3)), np.zeros((0, 0)), names
    step = msg.point_step
    buf = bytes(msg.data)
    cols = {}
    for nm, off in zip(names, offs):
        cols[nm] = np.frombuffer(buf, dtype="<f4", count=n, offset=off) \
            if step * n + off <= len(buf) + step else None
        # strided read: numpy can't stride a bytes buffer directly with count,
        # so build it properly below when the fast path is not valid.
    arr = np.frombuffer(buf, dtype=np.uint8, count=n * step).reshape(n, step)
    def col(off):
        return arr[:, off:off + 4].copy().view("<f4").reshape(n)
    xyz = np.stack([col(offs[names.index(c)]) for c in ("x", "y", "z")], axis=1)
    simnames = [c for c in names if c.startswith("sim_")]
    sim = np.stack([col(offs[names.index(c)]) for c in simnames], axis=1) \
        if simnames else np.zeros((n, 0))
    return xyz, sim, simnames


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_dir", type=Path)
    ap.add_argument("--people", type=Path,
                    default=Path("_test_freeze/raven_suburb_tornado_250/"
                                 "GT_people.json"))
    ap.add_argument("--robot", default="1")
    ap.add_argument("--radius", type=float, default=MATCH_RADIUS_M)
    # Multi-target scoring (2026-09-02 query sweep): the mission may run
    # `query: "person, casualty"` — the detection signal is then the MAX over
    # the named target columns, matched against the recorded label list.
    ap.add_argument("--targets", default="person",
                    help="comma-separated target labels; the score at a voxel "
                         "is max over these columns (default: person)")
    args = ap.parse_args()

    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError:
        sys.exit("need: uv run --no-project --with mcap "
                 "--with mcap-ros2-support --with numpy python3 "
                 + " ".join(sys.argv))

    ppl = json.load(open(args.people))["people"]
    gt = np.array([[p["x"], p["y"]] for p in ppl])

    r = args.robot
    T_VOX = f"/robot_{r}/rayfronts/msg_serv/voxels_sim/all"
    T_STATUS = f"/robot_{r}/rayfronts/status"
    T_ODOM = f"/robot_{r}/odometry_conversion/odometry"
    T_VTAB = f"/robot_{r}/debug/voxel_table"
    T_MODE = f"/robot_{r}/navigation_mode"

    boot = None
    labels: list[str] = []
    odom = []
    clouds = []
    modes = {}
    vtab_last = None

    bags = ([args.bag_dir] if (args.bag_dir / "metadata.yaml").exists()
            else sorted(p.parent for p in args.bag_dir.rglob("metadata.yaml")))
    for bag in bags:
        for f in sorted(bag.glob("*.mcap")):
            with f.open("rb") as fh:
                for _s, ch, _m, msg in make_reader(
                        fh, decoder_factories=[DecoderFactory()]
                        ).iter_decoded_messages():
                    if ch.topic == T_STATUS and boot is None:
                        try:
                            s = json.loads(msg.data)
                            if s.get("anchored"):
                                boot = np.array(s["boot_enu"][:2])
                                labels = s.get("queries", [])
                        except Exception:
                            pass
                    elif ch.topic == T_ODOM:
                        p = msg.pose.pose.position
                        odom.append((p.x, p.y))
                    elif ch.topic == T_MODE:
                        modes[msg.data] = modes.get(msg.data, 0) + 1
                    elif ch.topic == T_VTAB:
                        vtab_last = msg.data
                    elif ch.topic == T_VOX:
                        clouds.append(pc2_xyz_sim(msg))

    if boot is None:
        sys.exit("no anchored status in bag — cannot place the map in world "
                 "frame, so nothing here can be scored against GT.")
    print("=" * 74)
    print(f"GT SCORING  {args.bag_dir}")
    print("=" * 74)
    print(f"boot_enu (map origin in world ENU): "
          f"({boot[0]:.2f}, {boot[1]:.2f})")
    print(f"query columns: {[f'sim_{i}={l}' for i, l in enumerate(labels)]}")
    wanted = [t.strip() for t in args.targets.split(",") if t.strip()]
    target_cols = [i for i, l in enumerate(labels) if l in wanted]
    if not target_cols:
        target_cols = [0]
        print(f"  !! none of the target labels {wanted} appear in the "
              f"recorded columns — falling back to column 0 "
              f"('{labels[0] if labels else '?'}').")
    print(f"scoring = max over columns "
          f"{[f'sim_{i}={labels[i]}' for i in target_cols]}")
    print(f"voxel clouds recorded: {len(clouds)}   nav modes: {modes}")

    # Drone track in world, for the observability test.
    if odom:
        track = np.array(odom) + boot[None, :]
        print(f"drone track: x[{track[:,0].min():.1f},{track[:,0].max():.1f}] "
              f"y[{track[:,1].min():.1f},{track[:,1].max():.1f}]  "
              f"path {np.linalg.norm(np.diff(track,axis=0),axis=1).sum():.0f} m")
    else:
        track = np.zeros((0, 2))

    # Best person score anywhere near each casualty, over the whole run,
    # plus the global set of above-threshold voxels for the FP count.
    best = np.zeros(len(ppl))
    all_hi_world = {t: [] for t in THRESHOLDS}
    for xyz_rdf, sim, _sn in clouds:
        if xyz_rdf.shape[0] == 0 or sim.shape[1] == 0:
            continue
        w = rdf_to_world(xyz_rdf, boot)
        cols = [c for c in target_cols if c < sim.shape[1]] or [0]
        # SUM, not max: near-synonym positives split the softmax vote at true
        # targets (measured 2026-09-02: buried casualty solo 0.68 -> max 0.43
        # under person+casualty, sum 0.81) — mirror raven's detection rule.
        person = sim[:, cols].sum(axis=1)
        for i, g in enumerate(gt):
            d = np.hypot(w[:, 0] - g[0], w[:, 1] - g[1])
            near = d <= args.radius
            if near.any():
                best[i] = max(best[i], float(person[near].max()))
        for t in THRESHOLDS:
            m = person > t
            if m.any():
                all_hi_world[t].append(w[m][:, :2])

    seen = np.array([
        (np.hypot(track[:, 0] - g[0], track[:, 1] - g[1]).min() <= SEEN_RADIUS_M)
        if len(track) else False for g in gt])

    print(f"\ncasualties: {len(ppl)}   within {SEEN_RADIUS_M:.0f} m of the "
          f"drone's track at some point: {int(seen.sum())}")
    print(f"\n-- per-casualty best `person` score (match radius "
          f"{args.radius:.1f} m) --")
    print(f"{'idx':>3} {'world_x':>9} {'world_y':>9} {'dist_to_track':>13} "
          f"{'best_score':>10}  {'occlusion':<12} {'cov':>5}  seen")
    order = np.argsort(-best)
    for i in order:
        p = ppl[i]
        dmin = (np.hypot(track[:, 0] - gt[i][0],
                         track[:, 1] - gt[i][1]).min() if len(track) else -1)
        print(f"{i:3d} {p['x']:9.2f} {p['y']:9.2f} {dmin:13.1f} "
              f"{best[i]:10.3f}  {p['occlusion']:<12} {p['covered_frac']:5.2f}"
              f"  {'Y' if seen[i] else '-'}")

    print(f"\n-- threshold sweep (only the {int(seen.sum())} casualties the "
          f"drone actually approached count as recall) --")
    print(f"{'thr':>6} {'TP':>4} {'recall':>7} {'FP_clusters':>12} "
          f"{'precision':>10}   note")
    for t in THRESHOLDS:
        tp = int(((best >= t) & seen).sum())
        pts = (np.vstack(all_hi_world[t]) if all_hi_world[t]
               else np.zeros((0, 2)))
        # An above-threshold voxel is a FALSE positive when it is not within
        # the match radius of ANY casualty (not just the seen ones).
        if len(pts):
            dmat = np.hypot(pts[:, 0][:, None] - gt[None, :, 0],
                            pts[:, 1][:, None] - gt[None, :, 1])
            fp_pts = pts[dmat.min(axis=1) > args.radius]
        else:
            fp_pts = np.zeros((0, 2))
        # Cluster FP voxels at 1 m so one blob is one false positive.
        n_fp = 0
        if len(fp_pts):
            try:
                from scipy.cluster.hierarchy import fcluster, linkage
                n_fp = (len(np.unique(fcluster(linkage(fp_pts, "single"), 1.0,
                                               criterion="distance")))
                        if len(fp_pts) > 1 else 1)
            except Exception:
                n_fp = len(fp_pts)
        rec = tp / max(1, int(seen.sum()))
        prec = tp / max(1, tp + n_fp)
        note = ""
        if tp == 0:
            note = "no real casualty recovered"
        elif n_fp > 4 * max(tp, 1):
            note = "FP-dominated"
        print(f"{t:6.2f} {tp:4d} {rec:7.1%} {n_fp:12d} {prec:10.1%}   {note}")

    if vtab_last:
        print(f"\nraven's last voxel_table: {vtab_last}")
    print("\nNOTE: recall is over casualties the drone came within "
          f"{SEEN_RADIUS_M:.0f} m of. A casualty it never approached is a "
          "COVERAGE problem, not a threshold one — raise it with the search "
          "pattern, not voxel_score_threshold.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

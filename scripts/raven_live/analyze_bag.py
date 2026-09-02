#!/usr/bin/env python3
"""Score the RayFronts / RAVEN output in a recorded run — offline, no ROS.

WHY THIS EXISTS
---------------
`/{robot}/debug/voxel_table` tells you `clusters=0` and nothing else. It cannot
distinguish the three reasons a casualty was missed:

  1. the drone never overflew it          -> look at the odometry extent
  2. it was never in the voxel map        -> vox_count flat / target outside bbox
  3. it was in the map but scored too low -> the person-column histogram

Only (3) is a threshold problem. This script separates them from the bag so a
threshold can be re-picked WITHOUT re-flying, which is the whole reason
`msg_serv/voxels_sim/all` is in the mission's record list.

USAGE
-----
    python3 scripts/raven_live/analyze_bag.py <run_dir_or_bag_dir> [--robot 1]

`run_dir` may be the mission results dir (it will find iter_*/bags/*) or a bag
directory containing metadata.yaml.

Reads mcap directly via `mcap` + `mcap_ros2` so it runs on the HOST with no ROS
sourced (see the read-OSMO-bags-locally note). Install once:
    uv run --no-project --with mcap --with mcap-ros2-support --with numpy \
        python3 scripts/raven_live/analyze_bag.py ...
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

# Softmax thresholds to report the survivor count at. These bracket the
# mission's current voxel_score_threshold (0.5) on both sides so the report
# shows what LOWERING it would actually buy.
THRESHOLD_LADDER = (0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.15, 0.1, 0.05)


def find_bags(root: Path) -> list[Path]:
    if (root / "metadata.yaml").exists():
        return [root]
    out = sorted(p.parent for p in root.rglob("metadata.yaml"))
    if not out:
        sys.exit(f"no bag (metadata.yaml) found under {root}")
    return out


def pc2_to_array(msg) -> tuple[np.ndarray, list[str]]:
    """Decode a PointCloud2 into (N, F) float64 + field names.

    Only handles the float32 layout RayFronts publishes (x,y,z,sim_*), which is
    all we need — anything else is reported rather than guessed at.
    """
    names = [f.name for f in msg.fields]
    offsets = [f.offset for f in msg.fields]
    dtypes = {f.datatype for f in msg.fields}
    if dtypes != {7}:  # 7 == FLOAT32
        raise ValueError(f"unexpected datatypes {dtypes}; only float32 handled")
    n = msg.width * msg.height
    step = msg.point_step
    buf = bytes(msg.data)
    arr = np.zeros((n, len(names)), dtype=np.float64)
    for j, off in enumerate(offsets):
        col = np.frombuffer(buf, dtype="<f4", count=n, offset=off) \
            if step == 4 * len(names) and off == 4 * j else None
        if col is None:
            col = np.array([struct.unpack_from("<f", buf, i * step + off)[0]
                            for i in range(n)])
        arr[:, j] = col
    return arr, names


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--robot", default="1")
    ap.add_argument("--target", default="person",
                    help="query label whose column is scored")
    args = ap.parse_args()

    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError:
        sys.exit("need: uv run --no-project --with mcap --with mcap-ros2-support "
                 "--with numpy python3 " + " ".join(sys.argv))

    r = args.robot
    T_VOX = f"/robot_{r}/rayfronts/msg_serv/voxels_sim/all"
    T_RAY = f"/robot_{r}/rayfronts/msg_serv/rays_sim/all"
    T_STATUS = f"/robot_{r}/rayfronts/status"
    T_ODOM = f"/robot_{r}/odometry_conversion/odometry"
    T_VTAB = f"/robot_{r}/debug/voxel_table"
    T_MODE = f"/robot_{r}/navigation_mode"
    T_CONF = f"/robot_{r}/raven_nav/confirmed_targets"
    WANT = {T_VOX, T_RAY, T_STATUS, T_ODOM, T_VTAB, T_MODE, T_CONF}

    odom: list[tuple[float, float, float]] = []
    modes: dict[str, int] = defaultdict(int)
    status_last = None
    vox_counts: list[int] = []
    best = None            # (max_person_score, arr, names, seq)
    person_max_series: list[float] = []
    vtab_last = None
    n_conf = 0
    counts: dict[str, int] = defaultdict(int)

    for bag in find_bags(args.run_dir):
        for f in sorted(bag.glob("*.mcap")):
            with f.open("rb") as fh:
                reader = make_reader(fh, decoder_factories=[DecoderFactory()])
                for _sch, ch, _m, msg in reader.iter_decoded_messages():
                    t = ch.topic
                    if t not in WANT:
                        continue
                    counts[t] += 1
                    if t == T_ODOM:
                        p = msg.pose.pose.position
                        odom.append((p.x, p.y, p.z))
                    elif t == T_MODE:
                        modes[msg.data] += 1
                    elif t == T_STATUS:
                        status_last = msg.data
                    elif t == T_VTAB:
                        vtab_last = msg.data
                    elif t == T_CONF:
                        n_conf += 1
                    elif t == T_VOX:
                        try:
                            arr, names = pc2_to_array(msg)
                        except ValueError as e:
                            print(f"  !! {e}")
                            continue
                        vox_counts.append(arr.shape[0])
                        sim = [i for i, n in enumerate(names)
                               if n.startswith("sim_")]
                        if not sim or arr.shape[0] == 0:
                            continue
                        col = sim[-1]           # target is appended last
                        pmax = float(arr[:, col].max())
                        person_max_series.append(pmax)
                        if best is None or pmax > best[0]:
                            best = (pmax, arr, names, counts[t])

    print("=" * 72)
    print(f"RUN: {args.run_dir}")
    print("=" * 72)
    print("\n-- message counts --")
    for t in sorted(counts):
        print(f"  {counts[t]:6d}  {t}")
    missing = WANT - set(counts)
    for t in sorted(missing):
        print(f"  {'NONE':>6}  {t}   <-- never recorded")

    print("\n-- 1. did the drone move? --")
    if odom:
        a = np.array(odom)
        span = a.max(0) - a.min(0)
        path = float(np.linalg.norm(np.diff(a, axis=0), axis=1).sum())
        print(f"  samples      {len(a)}")
        print(f"  x [{a[:,0].min():8.2f}, {a[:,0].max():8.2f}]  span {span[0]:7.2f} m")
        print(f"  y [{a[:,1].min():8.2f}, {a[:,1].max():8.2f}]  span {span[1]:7.2f} m")
        print(f"  z [{a[:,2].min():8.2f}, {a[:,2].max():8.2f}]  span {span[2]:7.2f} m")
        print(f"  path length  {path:.1f} m")
        if path < 10.0:
            print("  !! STATIONARY — the drone never flew. Nothing below is")
            print("     evidence about detection; fix the local planner first.")
    else:
        print("  no odometry recorded")

    print("\n-- 2. navigation tiers --")
    print(f"  {dict(modes) or 'none recorded'}")
    print(f"  confirmed_targets msgs: {n_conf}")

    print("\n-- 3. was anything mapped? --")
    if vox_counts:
        print(f"  voxels_sim/all msgs {len(vox_counts)}, "
              f"points min {min(vox_counts)} max {max(vox_counts)}")
    else:
        print("  voxels_sim/all NEVER published — the map had no occupied")
        print("  voxels at all (everything past depth_limit becomes a ray).")
    if status_last:
        try:
            s = json.loads(status_last)
            print(f"  status: anchored={s.get('anchored')} "
                  f"frames={s.get('frames_robot')} vox={s.get('vox_count')} "
                  f"rays={s.get('ray_count')} queries={len(s.get('queries', []))}")
        except json.JSONDecodeError:
            print(f"  status(raw): {status_last[:200]}")

    print(f"\n-- 4. '{args.target}' score distribution (the threshold question) --")
    if best is None:
        print("  no decodable voxel cloud — cannot advise a threshold.")
    else:
        pmax, arr, names, seq = best
        sim = [i for i, n in enumerate(names) if n.startswith("sim_")]
        col = sim[-1]
        v = arr[:, col]
        print(f"  best frame: msg #{seq}, {arr.shape[0]} voxels, "
              f"{len(sim)} query columns")
        print(f"  peak over the WHOLE run: {max(person_max_series):.4f}")
        print(f"  in that frame: max {v.max():.4f}  mean {v.mean():.4f}  "
              f"p99 {np.percentile(v, 99):.4f}")
        print("  voxels surviving each threshold:")
        for th in THRESHOLD_LADDER:
            k = int((v > th).sum())
            flag = "  <-- mission setting" if abs(th - 0.5) < 1e-9 else ""
            print(f"    > {th:<5}  {k:6d}{flag}")
        S = arr[:, sim]
        am = S.argmax(1)
        u, c = np.unique(am, return_counts=True)
        top = sorted(zip(c, u), reverse=True)[:8]
        print("  argmax column histogram (col:count): "
              f"{[(int(x), int(y)) for y, x in top]}")
        print(f"  per-column max: {np.round(S.max(0), 3).tolist()}")
        peak = max(person_max_series)
        print("\n  ADVICE:")
        if peak < 0.05:
            print("   target never scored. Either it was never in view (check")
            print("   the odometry span vs the casualty position) or the label")
            print("   is wrong for this scene. Do NOT lower the threshold yet.")
        else:
            print(f"   set voxel_score_threshold a little under the peak "
                  f"({peak:.3f}) — try {max(0.05, peak * 0.6):.2f} — and check")
            print("   the argmax histogram above for what would come in with it.")

    if vtab_last:
        print(f"\n-- 5. raven's last voxel_table --\n  {vtab_last}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Score how many ground-truth objects a mission's drones *visited*.

Reads a recorded mission mcap (the `record.scope: gcs` bag) and, using only data
already in the bag in a single common frame ('map'):

  * GT objects  -> /gcs/annotations/bboxes   (visualization_msgs/MarkerArray)
      one '<class>_fill' CUBE marker per object: pose=center, scale=size, ns=class
  * drone paths -> /gcs/robot_markers         (visualization_msgs/MarkerArray)
      'robot_meshes' markers (one per robot, marker.id = (robot_idx-1)*10)

Both topics are published by the GCS visualizer in the same 'map' frame, so no
GPS / odom-frame conversion is needed.

A GT object counts as *visited* if any drone's trajectory came within --threshold
metres (default 3 m, horizontal) of the object. Distance is to the object's XY
footprint (the AABB rectangle from center +/- size/2; 0 if a drone passed over
it) — use --to-center for distance to the box center instead, or --3d for full
3D distance.

Only GT classes matching the mission query are scored (--queries); for
frontier_vs_raven ConstructionSite that's "Forklift, Tower crane, blue tarp".

Usage:
  python3 osmo/analyze_visited.py <iter_dir | bag.mcap> \
      [--queries "Forklift, Tower crane, blue tarp"] [--threshold 3.0] \
      [--to-center] [--3d] [--stride 1]

Deps (no ROS install needed): mcap, mcap_ros2, numpy.
"""
import argparse
import os
import sys

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

ANNOT_TOPIC = "/gcs/annotations/bboxes"
ROBOT_TOPIC = "/gcs/robot_markers"
MARKER_CUBE = 1   # visualization_msgs/Marker.CUBE
MARKER_MESH = 10  # MESH_RESOURCE


def _norm(s: str) -> str:
    return " ".join(s.strip().lower().split())


def _class_matches_query(cls: str, query_terms) -> bool:
    """A GT class matches a query term if every word of the term is in the class,
    or the term (spaces removed) is a substring of the class (spaces removed).
    This matches 'tower crane'->'orange towercrane' and 'blue tarp'->'blue tarp'
    while NOT matching 'blue tarp'->'orange tarp'."""
    c = _norm(cls)
    c_words = set(c.split())
    c_ns = c.replace(" ", "")
    for term in query_terms:
        t = _norm(term)
        if not t:
            continue
        t_words = set(t.split())
        if t_words and t_words.issubset(c_words):
            return True
        if t.replace(" ", "") in c_ns:
            return True
    return False


def _resolve_bag(path: str) -> str:
    if os.path.isfile(path) and path.endswith(".mcap"):
        return path
    # treat as an iteration directory: find bags/gcs/*.mcap
    candidates = []
    for root, _dirs, files in os.walk(path):
        for f in files:
            if f.endswith(".mcap"):
                candidates.append(os.path.join(root, f))
    if not candidates:
        sys.exit(f"No .mcap found under {path}")
    # prefer the gcs bag
    gcs = [c for c in candidates if os.sep + "gcs" + os.sep in c]
    return (gcs or candidates)[0]


def _xy_dist_to_footprint(pts_xy, cx, cy, sx, sy):
    """Horizontal distance from each point to the axis-aligned rectangle
    [cx±sx/2, cy±sy/2] (0 inside). pts_xy: (N,2)."""
    hx, hy = sx / 2.0, sy / 2.0
    dx = np.maximum(np.abs(pts_xy[:, 0] - cx) - hx, 0.0)
    dy = np.maximum(np.abs(pts_xy[:, 1] - cy) - hy, 0.0)
    return np.sqrt(dx * dx + dy * dy)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("path", help="iteration dir or .mcap bag")
    ap.add_argument("--queries", default="Forklift, Tower crane, blue tarp",
                    help="comma-separated query terms; only matching GT classes are scored")
    ap.add_argument("--threshold", type=float, default=3.0, help="visit radius (m)")
    ap.add_argument("--to-center", action="store_true",
                    help="distance to box center instead of footprint")
    ap.add_argument("--3d", dest="three_d", action="store_true",
                    help="use full 3D distance (default: horizontal XY)")
    ap.add_argument("--chunk-stride", type=int, default=10,
                    help="sample every Nth bag chunk for the trajectory (the bag "
                         "can be multi-GB; this seeks via the index instead of "
                         "decompressing the whole file). 1 = every chunk")
    args = ap.parse_args(argv)

    bag = _resolve_bag(args.path)
    query_terms = [q.strip() for q in args.queries.split(",") if q.strip()]
    print(f"bag:       {bag}")
    print(f"queries:   {query_terms}")
    print(f"threshold: {args.threshold} m  ({'3D' if args.three_d else 'horizontal XY'}, "
          f"{'to center' if args.to_center else 'to box footprint'})\n")

    # ── read bag ─────────────────────────────────────────────────────────────
    # Decode selectively (the bag has tens of thousands of MarkerArrays):
    # GT is static so one annotation message is enough; robot_markers is
    # subsampled by --stride. iter_messages scans raw (fast); we only decode
    # what we keep.
    gt = {}            # (class, id) -> dict(center, size)
    traj = {}          # robot_idx -> list of [x, y, z]
    factory = DecoderFactory()
    _dec_cache = {}

    def decode(channel, schema, data):
        d = _dec_cache.get(channel.id)
        if d is None:
            d = factory.decoder_for(channel.message_encoding, schema)
            _dec_cache[channel.id] = d
        return d(data)

    n_annot_msgs = 0
    n_robot_chunks = 0
    n_robot_msgs = 0
    with open(bag, "rb") as f:
        reader = make_reader(f)
        summary = reader.get_summary()
        if summary is None:
            sys.exit("bag has no summary/index (cannot seek efficiently)")
        robot_cid = next((c.id for c in summary.channels.values()
                          if c.topic == ROBOT_TOPIC), None)
        if robot_cid is None:
            sys.exit(f"no {ROBOT_TOPIC} channel in bag")

        # GT: decode the first few annotation messages, keep the richest one.
        # log_time_order=False avoids a full-bag merge-sort (pathological on a
        # multi-GB bag); GT is static so an early break is enough.
        for schema, channel, message in reader.iter_messages(
                topics=[ANNOT_TOPIC], log_time_order=False):
            n_annot_msgs += 1
            msg = decode(channel, schema, message.data)
            cur = {}
            for mk in msg.markers:
                if mk.type == MARKER_CUBE and mk.ns.endswith("_fill"):
                    cls = mk.ns[:-len("_fill")]
                    p, s = mk.pose.position, mk.scale
                    cur[(cls, mk.id)] = {
                        "class": cls,
                        "center": np.array([p.x, p.y, p.z], float),
                        "size": np.array([s.x, s.y, s.z], float),
                    }
            if len(cur) > len(gt):
                gt = cur
            if n_annot_msgs >= 5:   # GT is static; a handful is plenty
                break

        # Trajectory: sample every Nth chunk that contains robot_markers and read
        # only that chunk's short time window. Chunks are time-ordered, so this
        # samples the path across the whole mission while decompressing only a
        # fraction of the (possibly multi-GB) bag.
        rchunks = sorted(
            (ci for ci in summary.chunk_indexes
             if robot_cid in ci.message_index_offsets),
            key=lambda c: c.message_start_time)
        for k, ci in enumerate(rchunks):
            if k % max(1, args.chunk_stride):
                continue
            n_robot_chunks += 1
            for schema, channel, message in reader.iter_messages(
                    topics=[ROBOT_TOPIC],
                    start_time=ci.message_start_time,
                    end_time=ci.message_end_time + 1,
                    log_time_order=False):
                n_robot_msgs += 1
                msg = decode(channel, schema, message.data)
                for mk in msg.markers:
                    if mk.type == MARKER_MESH and mk.ns == "robot_meshes":
                        idx = mk.id // 10 + 1
                        p = mk.pose.position
                        traj.setdefault(idx, []).append([p.x, p.y, p.z])

    if not gt:
        sys.exit("No GT _fill markers found on /gcs/annotations/bboxes")
    if not traj:
        sys.exit("No robot_meshes markers found on /gcs/robot_markers")

    traj = {k: np.asarray(v, float) for k, v in sorted(traj.items())}
    all_pts = np.vstack(list(traj.values()))
    print(f"GT objects in bag: {len(gt)}  | robots: {sorted(traj)}  | "
          f"trajectory samples: {len(all_pts)} "
          f"(sampled {n_robot_chunks} chunks, {n_robot_msgs} robot_markers msgs)\n")

    # ── filter GT to the query classes ───────────────────────────────────────
    query_gt = [g for g in gt.values()
                if _class_matches_query(g["class"], query_terms)]
    matched_classes = sorted({g["class"] for g in query_gt})
    skipped_classes = sorted({g["class"] for g in gt.values()} - set(matched_classes))
    print(f"matched GT classes ({len(query_gt)} objects): {matched_classes}")
    print(f"ignored GT classes (not in query): {skipped_classes}\n")

    # ── score each query GT object ───────────────────────────────────────────
    def dist_for(pts, g):
        c, s = g["center"], g["size"]
        if args.three_d:
            if args.to_center:
                return np.linalg.norm(pts - c, axis=1)
            # 3D distance to the box AABB
            d = np.maximum(np.abs(pts - c) - s / 2.0, 0.0)
            return np.linalg.norm(d, axis=1)
        pts_xy = pts[:, :2]
        if args.to_center:
            return np.linalg.norm(pts_xy - c[:2], axis=1)
        return _xy_dist_to_footprint(pts_xy, c[0], c[1], s[0], s[1])

    rows = []
    visited = 0
    for g in sorted(query_gt, key=lambda x: (x["class"], tuple(x["center"]))):
        best_d, best_robot = float("inf"), None
        for ridx, pts in traj.items():
            d = dist_for(pts, g).min()
            if d < best_d:
                best_d, best_robot = d, ridx
        hit = best_d <= args.threshold
        visited += hit
        rows.append((g["class"], g["center"], best_d, best_robot, hit))

    # ── report ───────────────────────────────────────────────────────────────
    print(f"{'VISITED?':9s} {'class':18s} {'center (x,y,z)':24s} "
          f"{'min dist':>9s}  by")
    print("-" * 72)
    for cls, c, d, ridx, hit in rows:
        ctr = f"({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f})"
        print(f"{'  YES  ' if hit else '   no  '}  {cls:18s} {ctr:24s} "
              f"{d:8.2f}m  robot_{ridx}")

    # per-robot tally (a robot 'gets credit' for an object it alone is closest to)
    per_robot = {r: 0 for r in traj}
    for _cls, _c, d, ridx, hit in rows:
        if hit:
            per_robot[ridx] += 1

    print("\n" + "=" * 72)
    print(f"VISITED {visited} / {len(query_gt)} query GT objects "
          f"(<= {args.threshold} m)")
    print("closest-robot credit: " +
          ", ".join(f"robot_{r}: {n}" for r, n in per_robot.items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())

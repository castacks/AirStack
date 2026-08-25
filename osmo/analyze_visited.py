#!/usr/bin/env python3
"""Score which ground-truth objects a mission's drones *visited*, by which robot,
and at what time in the run — from a recorded `record.scope: gcs` mcap.

Uses only data already in the bag, all in the global 'map' frame (no GPS/odom
transform):

  * GT objects   -> /gcs/annotations/bboxes  (visualization_msgs/MarkerArray)
      one '<class>_fill' CUBE marker per object: pose=center, scale=size, ns=class
  * drone paths  -> /gcs/robot_markers       (visualization_msgs/MarkerArray)
      robot_<N>_traj : the trajectory_controller's recent path segment (~100
                       pts/msg); unioned across sampled messages -> dense path
      robot_<N>_axes : 3 axis arrows; the first arrow's start is the robot's
                       current position (used where _traj is absent)

A GT object counts as *visited* if a drone came within --threshold m (default 3 m,
horizontal) of it. Distance is to the object's XY footprint (AABB center ± size/2;
0 if a drone passed over it) — use --to-center or --3d to change that. The visit
TIME is the timestamp of the earliest sampled robot_markers message in which that
robot's points fall within the threshold, expressed as seconds since the first
robot_markers message (≈ recording start); it is accurate to the sampling window
(a few seconds).

Only GT classes matching the scene's frontier_vs_raven query are scored. The
scene (hence query) is auto-detected from the path (retro / constr / fire);
override with --queries.

Usage:
  python3 osmo/analyze_visited.py <iter_dir | bag.mcap> [--json OUT.json]
      [--threshold 3.0] [--queries "..."] [--chunk-stride 10] [--to-center] [--3d]

Default --json (when given an iter dir): <iter_dir>/visited_analysis.json.
Deps (no ROS install needed): mcap, mcap_ros2, numpy.
"""
import argparse
import io
import json
import os
import sys

import numpy as np
from mcap.exceptions import EndOfFile
from mcap.reader import make_reader
from mcap.records import Chunk as ChunkRec, Message as MsgRec
from mcap.stream_reader import StreamReader, breakup_chunk
from mcap_ros2.decoder import DecoderFactory

ANNOT_TOPIC = "/gcs/annotations/bboxes"
ROBOT_TOPIC = "/gcs/robot_markers"
MARKER_CUBE = 1   # visualization_msgs/Marker.CUBE

# Per-scene query from osmo/missions/frontier_vs_raven_compare.yaml.
SCENE_QUERIES = {
    "retro": "house",
    "constr": "Forklift, Tower crane, blue tarp",
    "fire": "red walled building, water tower, radio tower",
}


def _detect_scene(path):
    p = path.lower()
    for key in SCENE_QUERIES:
        if key in p:
            return key
    return None


def _norm(s):
    return " ".join(s.strip().lower().split())


def _class_matches_query(cls, query_terms):
    """GT class matches a query term if every word of the term is in the class,
    or the term (spaces removed) is a substring of the class (spaces removed).
    Matches 'tower crane'->'orange towercrane', 'blue tarp'->'blue tarp', but
    NOT 'blue tarp'->'orange tarp'."""
    c = _norm(cls)
    c_words = set(c.split())
    c_ns = c.replace(" ", "")
    for term in query_terms:
        t = _norm(term)
        if not t:
            continue
        if set(t.split()).issubset(c_words):
            return True
        if t.replace(" ", "") in c_ns:
            return True
    return False


def _resolve_bag(path):
    if os.path.isfile(path) and path.endswith(".mcap"):
        return path
    cands = [os.path.join(r, f)
             for r, _d, fs in os.walk(path) for f in fs if f.endswith(".mcap")]
    if not cands:
        sys.exit(f"No .mcap found under {path}")
    gcs = [c for c in cands if os.sep + "gcs" + os.sep in c]
    return (gcs or cands)[0]


def _dist(pts, g, three_d, to_center):
    """Min distance from points (N,3) to GT box g."""
    c, s = g["center"], g["size"]
    if three_d:
        if to_center:
            return float(np.linalg.norm(pts - c, axis=1).min())
        d = np.maximum(np.abs(pts - c) - s / 2.0, 0.0)
        return float(np.linalg.norm(d, axis=1).min())
    pxy = pts[:, :2]
    if to_center:
        return float(np.linalg.norm(pxy - c[:2], axis=1).min())
    dx = np.maximum(np.abs(pxy[:, 0] - c[0]) - s[0] / 2.0, 0.0)
    dy = np.maximum(np.abs(pxy[:, 1] - c[1]) - s[1] / 2.0, 0.0)
    return float(np.sqrt(dx * dx + dy * dy).min())


def _chunk_messages(f, ci, channel_id):
    """Yield (log_time, data) for one channel from a chunk, read by byte offset."""
    f.seek(ci.chunk_start_offset)
    sr = StreamReader(io.BytesIO(f.read(ci.chunk_length)),
                      skip_magic=True, emit_chunks=True)
    try:
        for rec in sr.records:
            if isinstance(rec, ChunkRec):
                for inner in breakup_chunk(rec):
                    if isinstance(inner, MsgRec) and inner.channel_id == channel_id:
                        yield inner.log_time, inner.data
                return
    except EndOfFile:
        return


def analyze(bag, query_terms, threshold, chunk_stride, three_d, to_center):
    factory = DecoderFactory()
    gt = {}            # (class, id) -> {class, center, size}
    n_annot_msgs = n_rm_chunks = n_rm_msgs = 0

    with open(bag, "rb") as f:
        reader = make_reader(f)
        summary = reader.get_summary()
        if summary is None:
            sys.exit("bag has no summary/index (cannot seek efficiently)")
        chans, schemas = summary.channels, summary.schemas

        def cid_for(topic):
            return next((cid for cid, c in chans.items() if c.topic == topic), None)

        def decoder_for(cid):
            ch = chans[cid]
            return factory.decoder_for(ch.message_encoding, schemas[ch.schema_id])

        def chunks_with(cid):
            return sorted((ci for ci in summary.chunk_indexes
                           if cid in ci.message_index_offsets),
                          key=lambda c: c.message_start_time)

        # GT: static & republished — read one annotation msg from each of the
        # first few annotation chunks, keep the richest.
        annot_cid = cid_for(ANNOT_TOPIC)
        if annot_cid is not None:
            adec = decoder_for(annot_cid)
            for ci in chunks_with(annot_cid)[:5]:
                for _t, data in _chunk_messages(f, ci, annot_cid):
                    n_annot_msgs += 1
                    cur = {}
                    for mk in adec(data).markers:
                        if mk.type == MARKER_CUBE and mk.ns.endswith("_fill"):
                            cls = mk.ns[:-len("_fill")]
                            p, s = mk.pose.position, mk.scale
                            cur[(cls, mk.id)] = {
                                "class": cls,
                                "center": np.array([p.x, p.y, p.z], float),
                                "size": np.array([s.x, s.y, s.z], float)}
                    if len(cur) > len(gt):
                        gt = cur
                    break
        if not gt:
            sys.exit("No GT _fill markers on /gcs/annotations/bboxes")

        # query GT (stable ids), with running per-(target, robot) state.
        query_gt = [g for g in gt.values()
                    if _class_matches_query(g["class"], query_terms)]
        if not query_gt:
            sys.exit(f"No GT classes match query {query_terms}")
        state = [{"g": g, "min_dist": {}, "visit_t": {}} for g in query_gt]

        # Trajectory: sample every Nth robot_markers chunk; per message, group
        # this robot's points (traj segment + axes start) and update each
        # target's per-robot min distance and first-within-threshold time.
        robot_cid = cid_for(ROBOT_TOPIC)
        if robot_cid is None:
            sys.exit(f"no {ROBOT_TOPIC} channel in bag")
        rdec = decoder_for(robot_cid)
        rchunks = chunks_with(robot_cid)
        t0 = None
        t_last = None
        robots = set()
        for k, ci in enumerate(rchunks):
            if k % max(1, chunk_stride):
                continue
            n_rm_chunks += 1
            for log_time, data in _chunk_messages(f, ci, robot_cid):
                n_rm_msgs += 1
                t0 = log_time if t0 is None else min(t0, log_time)
                t_last = log_time if t_last is None else max(t_last, log_time)
                per_robot = {}
                for mk in rdec(data).markers:
                    parts = mk.ns.split("_")
                    if len(parts) < 3 or parts[0] != "robot" or not parts[1].isdigit():
                        continue
                    r, kind = int(parts[1]), parts[-1]
                    if kind == "traj" and mk.points:
                        per_robot.setdefault(r, []).extend(
                            [pt.x, pt.y, pt.z] for pt in mk.points)
                    elif kind == "axes" and mk.points and mk.id % 10 == 2:
                        p = mk.points[0]
                        per_robot.setdefault(r, []).append([p.x, p.y, p.z])
                for r, pts in per_robot.items():
                    robots.add(r)
                    arr = np.asarray(pts, float)
                    for st in state:
                        d = _dist(arr, st["g"], three_d, to_center)
                        if r not in st["min_dist"] or d < st["min_dist"][r]:
                            st["min_dist"][r] = d
                        if d <= threshold and r not in st["visit_t"]:
                            st["visit_t"][r] = log_time
                break  # one robot_markers message per chunk

    if t0 is None:
        sys.exit("No robot_markers messages decoded")

    def t_s(t):
        return round((t - t0) / 1e9, 1)

    targets = []
    n_visited = 0
    for st in state:
        g = st["g"]
        visit_t = st["visit_t"]
        visited = bool(visit_t)
        n_visited += visited
        by, vt = None, None
        if visited:
            by = min(visit_t, key=visit_t.get)
            vt = t_s(visit_t[by])
        per_robot = {}
        for r in sorted(set(st["min_dist"]) | set(visit_t)):
            per_robot[f"robot_{r}"] = {
                "min_dist_m": round(st["min_dist"].get(r, float("inf")), 2),
                "visit_time_s": t_s(visit_t[r]) if r in visit_t else None,
            }
        targets.append({
            "class": g["class"],
            "center": [round(float(v), 2) for v in g["center"]],
            "size": [round(float(v), 2) for v in g["size"]],
            "visited": visited,
            "visited_by": f"robot_{by}" if by is not None else None,
            "visit_time_s": vt,
            "min_dist_m": round(min(st["min_dist"].values()), 2) if st["min_dist"] else None,
            "per_robot": per_robot,
        })
    targets.sort(key=lambda x: (x["class"], x["center"]))
    # Sorted list of visit times (s) across all visited targets — convenient for
    # building visited-over-time / progress curves.
    visit_times_s = sorted(t["visit_time_s"] for t in targets
                           if t["visited"] and t["visit_time_s"] is not None)

    return {
        "queries": query_terms,
        "threshold_m": threshold,
        "distance": ("center" if to_center else "footprint")
                    + ("_3d" if three_d else "_xy"),
        "robots": sorted(f"robot_{r}" for r in robots),
        "run_duration_s": round((t_last - t0) / 1e9, 1),
        "num_query_gt": len(query_gt),
        "num_visited": n_visited,
        "visit_times_s": visit_times_s,
        "trajectory_msgs": n_rm_msgs,
        "targets": targets,
    }


def _print_report(res, bag):
    print(f"bag:       {bag}")
    print(f"queries:   {res['queries']}")
    print(f"threshold: {res['threshold_m']} m ({res['distance']})  "
          f"robots: {res['robots']}  duration: {res['run_duration_s']}s\n")
    print(f"{'VISITED?':9s} {'class':18s} {'center (x,y)':18s} {'min dist':>8s} "
          f"{'t (s)':>7s}  by")
    print("-" * 72)
    for t in res["targets"]:
        c = t["center"]
        print(f"{'  YES  ' if t['visited'] else '   no  '}  {t['class']:18s} "
              f"({c[0]:.1f}, {c[1]:.1f}){'':4s} {t['min_dist_m']:7.2f}m "
              f"{(str(t['visit_time_s']) if t['visit_time_s'] is not None else '-'):>7s}  "
              f"{t['visited_by'] or ''}")
    print("\n" + "=" * 72)
    print(f"VISITED {res['num_visited']} / {res['num_query_gt']} query GT objects "
          f"(<= {res['threshold_m']} m)")
    if res.get("visit_times_s"):
        print("visit times (s): " + ", ".join(str(t) for t in res["visit_times_s"]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("path", help="iteration dir or .mcap bag")
    ap.add_argument("--json", default=None,
                    help="write result JSON here (default: <iter_dir>/visited_analysis.json)")
    ap.add_argument("--queries", default=None,
                    help="comma-separated query terms (default: auto by scene)")
    ap.add_argument("--threshold", type=float, default=3.0, help="visit radius (m)")
    ap.add_argument("--to-center", action="store_true",
                    help="distance to box center instead of footprint")
    ap.add_argument("--3d", dest="three_d", action="store_true",
                    help="use full 3D distance (default: horizontal XY)")
    ap.add_argument("--chunk-stride", type=int, default=10,
                    help="decode every Nth robot_markers chunk (the bag can be GB)")
    args = ap.parse_args(argv)

    bag = _resolve_bag(args.path)
    if args.queries:
        query_terms = [q.strip() for q in args.queries.split(",") if q.strip()]
    else:
        scene = _detect_scene(bag)
        if scene is None:
            sys.exit("could not auto-detect scene from path; pass --queries")
        query_terms = [q.strip() for q in SCENE_QUERIES[scene].split(",") if q.strip()]

    res = analyze(bag, query_terms, args.threshold, args.chunk_stride,
                  args.three_d, args.to_center)

    # Annotate with run identity parsed from the dir name (e.g. constr_r1__lvlm).
    run_dir = args.path if os.path.isdir(args.path) else os.path.dirname(
        os.path.dirname(os.path.dirname(bag)))
    run_name = os.path.basename(os.path.normpath(run_dir))
    res = {"run": run_name, "scene": _detect_scene(bag), "bag": bag, **res}
    if "__" in run_name:
        head, res["method"] = run_name.split("__", 1)
        bits = head.split("_")
        if len(bits) >= 2:
            res["replica"] = bits[-1]

    _print_report(res, bag)

    out = args.json
    if out is None and os.path.isdir(args.path):
        out = os.path.join(args.path, "visited_analysis.json")
    if out:
        with open(out, "w") as fh:
            json.dump(res, fh, indent=2)
        print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

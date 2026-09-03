#!/usr/bin/env python3
"""fix_bag_voxel_frames.py — translate recorded RayFronts voxel clouds into the
GCS world frame, offline, without ROS.

WHY THIS EXISTS
----------------
RayFronts publishes each robot's voxel clouds in that ROBOT'S OWN LOCAL `map`
frame (origin at its spawn/takeoff point). On the LIVE GCS,
gcs/ros_ws/src/gcs_visualizer/gcs_visualizer/foxglove_visualizer_node.py fixes
this on the fly: it measures each robot's boot ENU (`enu(fix) - odom`, locked
after MAP_ORIGIN_SAMPLES=10 GPS/odom pairs — see `_update_map_origin`) and
republishes the clouds translated, under `/rayfronts_debug/<robot>/...`.

The BAGS record the RAW `/robot_N/rayfronts/...` topics (see the `record:`
blocks in osmo/missions/raven_shared_t250_*robot.yaml), so replaying one in
Foxglove renders every voxel cloud offset by the spawn point (tens to
hundreds of metres, exactly `ORIGIN_LAT/LON`-relative ENU minus that robot's
takeoff ENU). This script produces a NEW bag with those clouds shifted into
the same frame the live GCS uses, so an old recording can be inspected
correctly without re-flying. THE ORIGINAL BAG IS NEVER MODIFIED.

FRAME MATH (mirrors the live node and scripts/raven_live/score_vs_gt.py
exactly — see their comments for the underlying derivation)
----------------------------------------------------------------------
Two conventions apply, by sub-topic:

  voxels_sim/*, rays_sim/*   RDF (camera-optical: x=right, y=down, z=forward).
                             Rotate to FLU as [z, -x, -y] (RDF_TO_FLU_QUAT in
                             foxglove_visualizer_node.py; rdf_to_world() in
                             score_vs_gt.py), THEN add the robot's boot ENU to
                             x and y only. rays_sim/* is not itself handled by
                             the live node (only recorded raw), but the
                             mission yaml describes it as "the out-of-range
                             half of the SAME frame" as voxels_sim — treated
                             identically here. UNVERIFIED IN A RENDER for
                             rays_sim specifically; flag if a render disagrees.

  voxel_rgb                  Already FLU at the source — rayfronts' own ROS
                             visualizer applies rdf2flu before publishing (see
                             RGB_VOXELS_SUFFIX comment in
                             foxglove_visualizer_node.py) — so translate x/y
                             only, no rotation (translate_point_cloud2() in
                             gcs_utils.py).

In both cases z is NEVER offset: boot_enu's z component is an MSL-datum
difference that raven/rayfronts never applies (multi_ros.py: "z is left
alone: every robot's z is AGL"; score_vs_gt.py: "the status' z offset is
measured but NOT applied"). frame_id becomes 'map' — the GCS's global-ENU
frame (foxglove_visualizer_node.py stamps a `world`->`map` identity static
transform and every republished topic 'map'; see its __init__ comment).

BOOT ENU RECOVERY
-----------------
Preferred: `/gcs/<robot>/map_origin` (geometry_msgs/PointStamped, LATCHED) —
the exact value foxglove_visualizer_node locks and publishes. Only present in
a bag if a mission's record list requests it (none currently do).

Fallback (what score_vs_gt.py uses, and what today's missions actually
record): `/robot_N/rayfronts/status` (std_msgs/String, JSON, LATCHED, 1 Hz —
see common/rayfronts/rayfronts/multi_robot_mapping_server.py:publish_status_
once). The FIRST message with `anchored: true` carries the final, already-
averaged `boot_enu` (mean over `anchor_samples` GPS/odom pairs — see
common/rayfronts/rayfronts/datasets/multi_ros.py) and it never changes after
that, so first-anchored is exactly as good as last-anchored.

A robot with neither topic in the bag cannot be placed: its clouds are copied
through UNCHANGED (still local-frame, frame_id untouched) and a warning is
printed. Use `--boot robot_1=12.0,-46.5` to supply it manually (e.g. from the
mission yaml's SPAWN_CONFIGS, which is an *approximation* of the true locked
boot ENU — prefer a bag/topic source whenever one exists).

USAGE
-----
    uv run --no-project --with mcap --with mcap-ros2-support --with numpy \\
        --with pyyaml python3 scripts/raven_live/fix_bag_voxel_frames.py \\
        <bag_dir_or_run_dir> [--out DIR] [--boot robot_1=X,Y ...] [--force]

`bag_dir_or_run_dir` may be a single bag directory (containing metadata.yaml)
or a run directory containing several (e.g. `.../iter_001/bags/`, with `gcs/`
and/or `robot_N/` subdirs) — every bag found is fixed independently, each
into its own `<name>_worldframe` sibling directory (a normal ros2-bag-shaped
directory: one merged .mcap + a metadata.yaml derived from the original).
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import sys
from pathlib import Path
from typing import Optional

import numpy as np

try:
    from mcap.reader import make_reader
    from mcap.writer import CompressionType
    from mcap.writer import Writer as McapWriter
    from mcap_ros2.decoder import DecoderFactory
    from mcap_ros2._dynamic import serialize_dynamic
except ImportError:
    sys.exit(
        "need: uv run --no-project --with mcap --with mcap-ros2-support "
        "--with numpy --with pyyaml python3 " + " ".join(sys.argv))

# The GCS's global-ENU frame. foxglove_visualizer_node.py stamps every
# translated topic 'map' and publishes a static world->map IDENTITY
# transform ("On the GCS, `map` IS global ENU ... world->map is identity
# HERE by definition" — see its __init__).
WORLD_FRAME_ID = "map"

FLOAT32 = 7  # sensor_msgs/msg/PointField.FLOAT32

MAP_ORIGIN_RE = re.compile(r"^/gcs/(robot_\w+)/map_origin$")
RAYFRONTS_STATUS_RE = re.compile(r"^/(robot_\w+)/rayfronts/status$")
# Every rayfronts PointCloud2 this script rewrites, and which sub-pattern
# needs the RDF->FLU rotation (see module docstring). voxels_sim/rays_sim
# live under the msg_serv/ infix (VOXELS_INFIX in foxglove_visualizer_node.py
# = '/rayfronts/msg_serv/'); voxel_rgb does not (RGB_VOXELS_SUFFIX =
# '/rayfronts/voxel_rgb').
RAYFRONTS_CLOUD_RE = re.compile(
    r"^/(robot_\w+)/rayfronts/(msg_serv/voxels_sim/.+|msg_serv/rays_sim/.+|voxel_rgb)$")
RDF_SUBTOPIC_RE = re.compile(r"^msg_serv/(voxels_sim|rays_sim)/")


# ── translation math (unit-tested directly) ─────────────────────────────────

def rdf_xyz_to_flu(x: np.ndarray, y: np.ndarray, z: np.ndarray):
    """[x,y,z] (RDF, camera-optical) -> [z,-x,-y] (FLU). Exactly
    RDF_TO_FLU_QUAT in foxglove_visualizer_node.py / rdf_to_world() in
    score_vs_gt.py, expressed as the equivalent axis permutation."""
    return z, -x, -y


def transform_cloud_bytes(data: bytes, fields, point_step: int, n: int,
                           bx: float, by: float, is_rdf: bool) -> bytes:
    """Return NEW packed point bytes with x/y shifted into the world frame
    (z untouched). `fields` is the decoded PointCloud2 `fields` list-of-
    namespaces (each with .name/.offset/.datatype). Raises ValueError if the
    layout isn't the plain float32 x/y/z every rayfronts cloud in this repo
    publishes (caller should treat that as "cannot fix this message").
    """
    offs = {f.name: int(f.offset) for f in fields}
    dtypes = {f.name: int(f.datatype) for f in fields}
    for k in ("x", "y", "z"):
        if k not in offs:
            raise ValueError(f"cloud has no '{k}' field")
        if dtypes[k] != FLOAT32:
            raise ValueError(f"'{k}' field is not float32 (datatype={dtypes[k]})")
    if point_step <= 0 or len(data) < n * point_step:
        raise ValueError(
            f"data too short: {len(data)} bytes for {n} points at "
            f"point_step={point_step}")

    buf = np.frombuffer(data, dtype=np.uint8, count=n * point_step).copy()
    view = buf.reshape(n, point_step)
    ox, oy, oz = offs["x"], offs["y"], offs["z"]

    def col(o):
        return view[:, o:o + 4].view("<f4").reshape(n)

    if is_rdf:
        x, y, z = col(ox).copy(), col(oy).copy(), col(oz).copy()
        flu_x, flu_y, flu_z = rdf_xyz_to_flu(x, y, z)
        col(ox)[:] = (flu_x + np.float32(bx)).astype("<f4")
        col(oy)[:] = (flu_y + np.float32(by)).astype("<f4")
        col(oz)[:] = flu_z.astype("<f4")
    else:
        col(ox)[:] += np.float32(bx)
        col(oy)[:] += np.float32(by)
        # z untouched — boot_enu z is an MSL-datum offset, never applied.

    return buf.tobytes()


# ── bag discovery (same convention as analyze_bag.py / score_vs_gt.py) ──────

def find_bags(root: Path) -> list[Path]:
    if (root / "metadata.yaml").exists():
        return [root]
    out = sorted(p.parent for p in root.rglob("metadata.yaml"))
    if not out:
        sys.exit(f"no bag (metadata.yaml) found under {root}")
    return out


# ── pass 1: recover each robot's boot ENU ────────────────────────────────────

def discover_boots(mcap_paths: list[Path]) -> tuple[dict, dict]:
    """Scan `mcap_paths` for /gcs/<robot>/map_origin and
    /robot_N/rayfronts/status. Returns (boot_from_origin, boot_from_status),
    each {robot_name: (x, y)}. Cheap: only messages on those two topic
    patterns are ever decoded."""
    boot_origin: dict[str, tuple[float, float]] = {}
    boot_status: dict[str, tuple[float, float]] = {}
    factory = DecoderFactory()
    decoders: dict[int, object] = {}

    for path in mcap_paths:
        with path.open("rb") as fh:
            reader = make_reader(fh)
            for schema, channel, message in reader.iter_messages():
                m_origin = MAP_ORIGIN_RE.match(channel.topic)
                m_status = None if m_origin else RAYFRONTS_STATUS_RE.match(channel.topic)
                if not m_origin and not m_status:
                    continue
                if m_status and m_status.group(1) in boot_status:
                    continue  # first-anchored already found for this robot
                if schema is None:
                    continue
                decoder = decoders.get(schema.id)
                if decoder is None:
                    decoder = factory.decoder_for(channel.message_encoding, schema)
                    decoders[schema.id] = decoder
                if decoder is None:
                    continue
                msg = decoder(message.data)
                if m_origin:
                    robot = m_origin.group(1)
                    boot_origin[robot] = (float(msg.point.x), float(msg.point.y))
                else:
                    robot = m_status.group(1)
                    try:
                        s = json.loads(msg.data)
                    except (json.JSONDecodeError, TypeError):
                        continue
                    if s.get("anchored") and s.get("boot_enu"):
                        be = s["boot_enu"]
                        boot_status[robot] = (float(be[0]), float(be[1]))

    return boot_origin, boot_status


# ── pass 2: write the translated bag ────────────────────────────────────────

class _Registrar:
    """Content-keyed schema/channel registration onto one output Writer, so
    merging several input .mcap parts (or re-registering the same topic
    across them) never creates duplicate schema/channel records."""

    def __init__(self, writer: McapWriter):
        self.writer = writer
        self._schema_ids: dict[tuple, int] = {}
        self._channel_ids: dict[str, int] = {}

    def schema_id(self, schema) -> Optional[int]:
        if schema is None:
            return None
        key = (schema.name, schema.encoding, schema.data)
        sid = self._schema_ids.get(key)
        if sid is None:
            sid = self.writer.register_schema(
                name=schema.name, encoding=schema.encoding, data=schema.data)
            self._schema_ids[key] = sid
        return sid

    def channel_id(self, channel, schema) -> int:
        cid = self._channel_ids.get(channel.topic)
        if cid is None:
            cid = self.writer.register_channel(
                topic=channel.topic, message_encoding=channel.message_encoding,
                schema_id=self.schema_id(schema), metadata=dict(channel.metadata))
            self._channel_ids[channel.topic] = cid
        return cid


def fix_bag(bag_dir: Path, out_dir: Path,
            manual_boots: Optional[dict] = None,
            force: bool = False) -> dict:
    """Fix one bag directory (containing metadata.yaml + one or more .mcap
    files) into `out_dir`. Never touches `bag_dir`. Returns a summary dict:
    {"boots": {robot: (x, y, source)}, "missing_boot_robots": [...],
     "fixed": {topic: count}, "passthrough": {topic: count}}.
    """
    import yaml  # local: only needed for metadata.yaml, not the core math

    bag_dir = bag_dir.resolve()
    out_dir = out_dir.resolve()
    if out_dir == bag_dir:
        raise ValueError("refusing to write the fixed bag over the original")
    if out_dir.exists():
        if not force:
            raise FileExistsError(
                f"{out_dir} already exists (pass --force to overwrite)")
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)

    mcap_paths = sorted(bag_dir.glob("*.mcap"))
    if not mcap_paths:
        raise FileNotFoundError(f"no .mcap files under {bag_dir}")

    boot_origin, boot_status = discover_boots(mcap_paths)
    boots: dict[str, tuple[float, float, str]] = {}
    for robot, (x, y) in boot_status.items():
        boots[robot] = (x, y, "status")
    for robot, (x, y) in boot_origin.items():
        boots[robot] = (x, y, "map_origin")  # preferred: overrides status
    for robot, (x, y) in (manual_boots or {}).items():
        boots[robot] = (x, y, "manual")

    out_mcap_name = f"{bag_dir.name}_0.mcap"
    out_mcap_path = out_dir / out_mcap_name
    writer = McapWriter(str(out_mcap_path), compression=CompressionType.ZSTD)
    writer.start(profile="ros2", library="fix_bag_voxel_frames.py")
    reg = _Registrar(writer)

    factory = DecoderFactory()
    decoders: dict[int, object] = {}
    encoders: dict[int, object] = {}

    fixed_counts: dict[str, int] = {}
    passthrough_counts: dict[str, int] = {}
    warned_missing: set[str] = set()

    def get_decoder(schema, encoding):
        d = decoders.get(schema.id)
        if d is None:
            d = factory.decoder_for(encoding, schema)
            decoders[schema.id] = d
        return d

    def get_encoder(schema):
        e = encoders.get(schema.id)
        if e is None:
            type_dict = serialize_dynamic(schema.name, schema.data.decode())
            e = type_dict[schema.name]
            encoders[schema.id] = e
        return e

    try:
        for path in mcap_paths:
            with path.open("rb") as fh:
                reader = make_reader(fh)
                for schema, channel, message in reader.iter_messages():
                    out_cid = reg.channel_id(channel, schema)
                    m = RAYFRONTS_CLOUD_RE.match(channel.topic) if schema else None
                    if not m:
                        writer.add_message(
                            channel_id=out_cid, log_time=message.log_time,
                            publish_time=message.publish_time,
                            sequence=message.sequence, data=message.data)
                        continue

                    robot, suffix = m.group(1), m.group(2)
                    boot = boots.get(robot)
                    if boot is None:
                        if robot not in warned_missing:
                            print(f"WARNING: no boot ENU for {robot} "
                                  f"(no /gcs/{robot}/map_origin or anchored "
                                  f"/{robot}/rayfronts/status in this bag) — "
                                  f"its rayfronts clouds are copied through "
                                  f"UNCHANGED, still in the local frame",
                                  file=sys.stderr)
                            warned_missing.add(robot)
                        writer.add_message(
                            channel_id=out_cid, log_time=message.log_time,
                            publish_time=message.publish_time,
                            sequence=message.sequence, data=message.data)
                        passthrough_counts[channel.topic] = (
                            passthrough_counts.get(channel.topic, 0) + 1)
                        continue

                    decoder = get_decoder(schema, channel.message_encoding)
                    if decoder is None:
                        writer.add_message(
                            channel_id=out_cid, log_time=message.log_time,
                            publish_time=message.publish_time,
                            sequence=message.sequence, data=message.data)
                        continue

                    msg = decoder(message.data)
                    try:
                        n = int(msg.width) * int(msg.height)
                        new_data = transform_cloud_bytes(
                            msg.data, msg.fields, int(msg.point_step), n,
                            boot[0], boot[1], is_rdf=bool(RDF_SUBTOPIC_RE.match(suffix)))
                    except ValueError as e:
                        print(f"WARNING: {channel.topic}: {e} — copied through "
                              f"unchanged", file=sys.stderr)
                        writer.add_message(
                            channel_id=out_cid, log_time=message.log_time,
                            publish_time=message.publish_time,
                            sequence=message.sequence, data=message.data)
                        passthrough_counts[channel.topic] = (
                            passthrough_counts.get(channel.topic, 0) + 1)
                        continue

                    msg.data = new_data
                    msg.header.frame_id = WORLD_FRAME_ID
                    encoder = get_encoder(schema)
                    writer.add_message(
                        channel_id=out_cid, log_time=message.log_time,
                        publish_time=message.publish_time,
                        sequence=message.sequence, data=encoder(msg))
                    fixed_counts[channel.topic] = fixed_counts.get(channel.topic, 0) + 1
    finally:
        writer.finish()

    meta_path = bag_dir / "metadata.yaml"
    if meta_path.exists():
        with meta_path.open() as f:
            orig_meta = yaml.safe_load(f)
        out_meta = rewrite_metadata(orig_meta, out_mcap_name)
        with (out_dir / "metadata.yaml").open("w") as f:
            yaml.safe_dump(out_meta, f, sort_keys=False)

    return {
        "boots": boots,
        "missing_boot_robots": sorted(warned_missing),
        "fixed": fixed_counts,
        "passthrough": passthrough_counts,
        "out_dir": str(out_dir),
    }


def rewrite_metadata(orig_meta: dict, out_mcap_name: str) -> dict:
    """Return a copy of a ros2-bag `metadata.yaml` dict pointed at ONE merged
    output file. Topic list / per-topic message counts / the top-level
    duration+starting_time+message_count are unchanged (this tool edits
    existing PointCloud2 payloads in place; it never adds or drops
    messages) — only the per-file breakdown collapses to a single entry."""
    import copy as _copy
    meta = _copy.deepcopy(orig_meta)
    info = meta["rosbag2_bagfile_information"]
    info["relative_file_paths"] = [out_mcap_name]
    info["files"] = [{
        "path": out_mcap_name,
        # Deep-copied (not shared references) so pyyaml doesn't emit YAML
        # anchors/aliases here -- valid, but needlessly unusual output.
        "starting_time": _copy.deepcopy(info["starting_time"]),
        "duration": _copy.deepcopy(info["duration"]),
        "message_count": info["message_count"],
    }]
    return meta


def _parse_manual_boot(spec: str) -> tuple[str, tuple[float, float]]:
    robot, _, xy = spec.partition("=")
    x_str, _, y_str = xy.partition(",")
    return robot.strip(), (float(x_str), float(y_str))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag_dir", type=Path,
                     help="a bag dir (metadata.yaml) or a run dir containing several")
    ap.add_argument("--out", type=Path, default=None,
                     help="output dir (only valid with a single bag_dir match); "
                          "default: '<bag_dir>_worldframe' next to each bag found")
    ap.add_argument("--suffix", default="_worldframe",
                     help="sibling-directory suffix when --out is not given")
    ap.add_argument("--boot", action="append", default=[],
                     metavar="robot_N=X,Y",
                     help="manual boot-ENU override, e.g. robot_1=12.0,-46.5 "
                          "(repeatable). Overrides discovered values for that robot.")
    ap.add_argument("--force", action="store_true",
                     help="overwrite an existing output directory")
    args = ap.parse_args()

    manual = dict(_parse_manual_boot(s) for s in args.boot)
    bags = find_bags(args.bag_dir)
    if args.out is not None and len(bags) != 1:
        sys.exit(f"--out given but {len(bags)} bags found under {args.bag_dir}; "
                  f"pass a single bag dir with --out, or omit it")

    ok = True
    for bag in bags:
        out_dir = args.out if args.out is not None else bag.parent / f"{bag.name}{args.suffix}"
        print("=" * 74)
        print(f"FIXING  {bag}")
        print(f"  ->    {out_dir}")
        try:
            summary = fix_bag(bag, out_dir, manual_boots=manual, force=args.force)
        except FileExistsError as e:
            print(f"SKIP: {e}", file=sys.stderr)
            ok = False
            continue
        for robot, (x, y, src) in sorted(summary["boots"].items()):
            print(f"  {robot}: boot_enu=({x:.2f}, {y:.2f})  source={src}")
        for robot in summary["missing_boot_robots"]:
            print(f"  {robot}: NO BOOT FOUND — copied through unchanged")
        for topic, n in sorted(summary["fixed"].items()):
            print(f"  fixed        {n:6d}  {topic}")
        for topic, n in sorted(summary["passthrough"].items()):
            print(f"  passthrough  {n:6d}  {topic}  (rayfronts cloud, NOT translated)")
        if summary["missing_boot_robots"]:
            ok = False
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

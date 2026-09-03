#!/usr/bin/env python3
"""Host-runnable test for fix_bag_voxel_frames.py — no ROS, no Isaac Sim.

Builds a SYNTHETIC two-robot mcap bag (a raw RDF voxels_sim/all cloud, an
already-FLU voxel_rgb cloud, a rayfronts/status String carrying a known
boot_enu, a /gcs/<robot>/map_origin PointStamped that must win over status,
one robot with NO boot source at all, and one innocuous passthrough topic),
runs it through fix_bag(), and checks the translated coordinates against the
transform computed by hand.

Run:
    uv run --no-project --with mcap --with mcap-ros2-support --with numpy \\
        --with pyyaml python3 scripts/raven_live/test_fix_bag_voxel_frames.py
"""
from __future__ import annotations

import importlib.util
import json
import shutil
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from mcap.reader import make_reader
from mcap.writer import CompressionType
from mcap.writer import Writer as McapWriter
from mcap_ros2.decoder import DecoderFactory
from mcap_ros2._dynamic import serialize_dynamic

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    "fix_bag_voxel_frames", HERE / "fix_bag_voxel_frames.py")
fbvf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fbvf)  # type: ignore[union-attr]

# ── message definitions (PointCloud2 text is byte-identical to what
# rosbag2's mcap plugin actually writes -- lifted verbatim from a real
# recorded bag's Schema record so the CDR parser sees exactly what it would
# on a real bag) ──────────────────────────────────────────────────────────
POINTCLOUD2_SCHEMA = """\
std_msgs/Header header

uint32 height
uint32 width

PointField[] fields

bool    is_bigendian
uint32  point_step
uint32  row_step
uint8[] data

bool is_dense

================================================================================
MSG: sensor_msgs/PointField
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name
uint32 offset
uint8  datatype
uint32 count

================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""

STRING_SCHEMA = "string data\n"

POINTSTAMPED_SCHEMA = """\
std_msgs/Header header
Point point

================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec

================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
"""

_pc2_encoder = serialize_dynamic(
    "sensor_msgs/msg/PointCloud2", POINTCLOUD2_SCHEMA)["sensor_msgs/msg/PointCloud2"]
_string_encoder = serialize_dynamic(
    "std_msgs/msg/String", STRING_SCHEMA)["std_msgs/msg/String"]
_pointstamped_encoder = serialize_dynamic(
    "geometry_msgs/msg/PointStamped", POINTSTAMPED_SCHEMA)["geometry_msgs/msg/PointStamped"]


def _pc2_bytes(xyz: np.ndarray, frame_id="local") -> bytes:
    n = xyz.shape[0]
    point_step = 12
    fields = [
        SimpleNamespace(name="x", offset=0, datatype=7, count=1),
        SimpleNamespace(name="y", offset=4, datatype=7, count=1),
        SimpleNamespace(name="z", offset=8, datatype=7, count=1),
    ]
    buf = np.zeros((n, point_step), dtype=np.uint8)
    buf[:, 0:4].view("<f4")[:, 0] = xyz[:, 0].astype("<f4")
    buf[:, 4:8].view("<f4")[:, 0] = xyz[:, 1].astype("<f4")
    buf[:, 8:12].view("<f4")[:, 0] = xyz[:, 2].astype("<f4")
    msg = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id=frame_id),
        height=1, width=n, fields=fields, is_bigendian=False,
        point_step=point_step, row_step=point_step * n,
        data=buf.tobytes(), is_dense=True,
    )
    return _pc2_encoder(msg)


def _string_bytes(s: str) -> bytes:
    return _string_encoder(SimpleNamespace(data=s))


def _pointstamped_bytes(x: float, y: float, z: float) -> bytes:
    obj = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id="map"),
        point=SimpleNamespace(x=x, y=y, z=z),
    )
    return _pointstamped_encoder(obj)


def _write_metadata(bag_dir: Path, n_msgs: int) -> None:
    (bag_dir / "metadata.yaml").write_text(f"""\
rosbag2_bagfile_information:
  version: 9
  storage_identifier: mcap
  duration:
    nanoseconds: {n_msgs}
  starting_time:
    nanoseconds_since_epoch: 1
  message_count: {n_msgs}
  topics_with_message_count: []
  compression_format: ""
  compression_mode: ""
  relative_file_paths:
    - test_0.mcap
  files:
    - path: test_0.mcap
      starting_time:
        nanoseconds_since_epoch: 1
      duration:
        nanoseconds: {n_msgs}
      message_count: {n_msgs}
  custom_data: ~
  ros_distro: jazzy
""")


def build_synthetic_bag(bag_dir: Path) -> None:
    """Two robots:
      robot_1 — boot from /gcs/robot_1/map_origin (100, 50), which must WIN
                over a deliberately-wrong /robot_1/rayfronts/status boot_enu
                (999, 999) also present in the bag.
      robot_2 — boot ONLY from /robot_2/rayfronts/status (-30, 200);
                anchored:false message first (must be ignored), then
                anchored:true.
      robot_3 — rayfronts topics present but NO boot source anywhere ->
                must be copied through unchanged.
    Plus one innocuous non-rayfronts topic to prove passthrough leaves other
    data alone.
    """
    bag_dir.mkdir(parents=True, exist_ok=True)
    writer = McapWriter(str(bag_dir / "test_0.mcap"), compression=CompressionType.ZSTD)
    writer.start(profile="ros2", library="test_fix_bag_voxel_frames.py")

    sch_pc2 = writer.register_schema(
        name="sensor_msgs/msg/PointCloud2", encoding="ros2msg",
        data=POINTCLOUD2_SCHEMA.encode())
    sch_str = writer.register_schema(
        name="std_msgs/msg/String", encoding="ros2msg", data=STRING_SCHEMA.encode())
    sch_ps = writer.register_schema(
        name="geometry_msgs/msg/PointStamped", encoding="ros2msg",
        data=POINTSTAMPED_SCHEMA.encode())

    def ch(topic, schema_id):
        return writer.register_channel(topic=topic, message_encoding="cdr", schema_id=schema_id)

    ch_r1_status = ch("/robot_1/rayfronts/status", sch_str)
    ch_r1_origin = ch("/gcs/robot_1/map_origin", sch_ps)
    ch_r1_voxall = ch("/robot_1/rayfronts/msg_serv/voxels_sim/all", sch_pc2)
    ch_r1_rgb = ch("/robot_1/rayfronts/voxel_rgb", sch_pc2)
    ch_r2_status = ch("/robot_2/rayfronts/status", sch_str)
    ch_r2_rays = ch("/robot_2/rayfronts/msg_serv/rays_sim/all", sch_pc2)
    ch_r3_rgb = ch("/robot_3/rayfronts/voxel_rgb", sch_pc2)
    ch_odom = ch("/robot_1/odometry_conversion/odometry", sch_str)  # bogus type, passthrough-only

    t = 0

    def emit(channel_id, data):
        nonlocal t
        t += 1
        writer.add_message(channel_id=channel_id, log_time=t, publish_time=t,
                            sequence=0, data=data)

    # robot_1: WRONG boot via status (must be overridden by map_origin)...
    emit(ch_r1_status, _string_bytes(json.dumps(
        {"anchored": True, "boot_enu": [999.0, 999.0, 0.0]})))
    # ...then the authoritative map_origin.
    emit(ch_r1_origin, _pointstamped_bytes(100.0, 50.0, 0.0))
    # RDF cloud: two points.
    emit(ch_r1_voxall, _pc2_bytes(np.array([[1.0, 2.0, 3.0], [4.0, -1.0, 2.0]])))
    # Already-FLU cloud: one point.
    emit(ch_r1_rgb, _pc2_bytes(np.array([[10.0, -5.0, 1.5]])))

    # robot_2: an early UNANCHORED status (must be ignored), then anchored.
    emit(ch_r2_status, _string_bytes(json.dumps({"anchored": False})))
    emit(ch_r2_status, _string_bytes(json.dumps(
        {"anchored": True, "boot_enu": [-30.0, 200.0, 0.0]})))
    emit(ch_r2_rays, _pc2_bytes(np.array([[0.5, 0.0, 10.0]])))

    # robot_3: no boot source anywhere -> must pass through unchanged.
    emit(ch_r3_rgb, _pc2_bytes(np.array([[7.0, 8.0, 9.0]]), frame_id="local"))

    # innocuous other-topic message, must be byte-identical on the far side.
    emit(ch_odom, _string_bytes("unrelated"))

    writer.finish()
    _write_metadata(bag_dir, t)


def _read_xyz(msg) -> np.ndarray:
    n = msg.width * msg.height
    step = msg.point_step
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)
    offs = {f.name: f.offset for f in msg.fields}
    out = np.zeros((n, 3), dtype=np.float32)
    for i, k in enumerate(("x", "y", "z")):
        out[:, i] = buf[:, offs[k]:offs[k] + 4].copy().view("<f4").reshape(n)
    return out


def read_all(bag_dir: Path) -> dict:
    mcap_path = sorted(bag_dir.glob("*.mcap"))[0]
    out = {}
    with mcap_path.open("rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _schema, channel, _message, msg in reader.iter_decoded_messages():
            out.setdefault(channel.topic, []).append(msg)
    return out


def check(label: str, cond: bool, failures: list):
    print(("PASS" if cond else "FAIL") + f"  {label}")
    if not cond:
        failures.append(label)


def main() -> int:
    failures: list = []
    with tempfile.TemporaryDirectory(prefix="fbvf_test_") as tmp:
        tmp = Path(tmp)
        bag_dir = tmp / "gcs"
        out_dir = tmp / "gcs_worldframe"
        build_synthetic_bag(bag_dir)

        # ── 1. unit-level: the pure translation math, no mcap involved ──────
        rdf = np.array([1.0, 2.0, 3.0])
        flu = fbvf.rdf_xyz_to_flu(*rdf)
        check("rdf_xyz_to_flu([1,2,3]) == (3,-1,-2)",
              flu == (3.0, -1.0, -2.0), failures)

        pts = np.array([[1.0, 2.0, 3.0], [4.0, -1.0, 2.0]])
        packed = _pc2_bytes(pts)
        # strip through the encoder back to a raw point buffer by decoding
        # via the same schema, to get (fields, point_step) for the call:
        from mcap_ros2._dynamic import generate_dynamic
        dec = generate_dynamic("sensor_msgs/msg/PointCloud2", POINTCLOUD2_SCHEMA)[
            "sensor_msgs/msg/PointCloud2"]
        decoded = dec(packed)
        new_bytes = fbvf.transform_cloud_bytes(
            decoded.data, decoded.fields, decoded.point_step, 2,
            bx=100.0, by=50.0, is_rdf=True)
        arr = np.frombuffer(new_bytes, dtype=np.uint8).reshape(2, 12)
        got_xyz = np.stack([arr[:, o:o + 4].view("<f4").reshape(2) for o in (0, 4, 8)], axis=1)
        want_xyz = np.array([[103.0, 49.0, -2.0], [102.0, 46.0, 1.0]])
        check("transform_cloud_bytes RDF->world matches hand computation",
              np.allclose(got_xyz, want_xyz), failures)

        # ── 2. end-to-end: build -> fix_bag -> read back ────────────────────
        summary = fbvf.fix_bag(bag_dir, out_dir)
        boots = summary["boots"]
        check("robot_1 boot resolves to map_origin's value, not status's",
              boots.get("robot_1") == (100.0, 50.0, "map_origin"), failures)
        check("robot_2 boot resolves from the (first-anchored) status message",
              boots.get("robot_2") == (-30.0, 200.0, "status"), failures)
        check("robot_3 has no discoverable boot",
              "robot_3" not in boots, failures)
        check("robot_3 reported as missing in the summary",
              "robot_3" in summary["missing_boot_robots"], failures)

        msgs = read_all(out_dir)

        vox = msgs["/robot_1/rayfronts/msg_serv/voxels_sim/all"][0]
        check("robot_1 voxels_sim/all frame_id -> 'map'",
              vox.header.frame_id == "map", failures)
        check("robot_1 voxels_sim/all RDF->FLU->world matches hand computation",
              np.allclose(_read_xyz(vox), [[103.0, 49.0, -2.0], [102.0, 46.0, 1.0]]),
              failures)

        rgb = msgs["/robot_1/rayfronts/voxel_rgb"][0]
        check("robot_1 voxel_rgb frame_id -> 'map'",
              rgb.header.frame_id == "map", failures)
        check("robot_1 voxel_rgb translate-only matches hand computation",
              np.allclose(_read_xyz(rgb), [[110.0, 45.0, 1.5]]), failures)

        rays = msgs["/robot_2/rayfronts/msg_serv/rays_sim/all"][0]
        # RDF (0.5, 0.0, 10.0) -> flu (10.0, -0.5, 0.0) -> world (10-30, -0.5+200, 0)
        check("robot_2 rays_sim/all RDF->FLU->world matches hand computation",
              np.allclose(_read_xyz(rays), [[-20.0, 199.5, 0.0]]), failures)

        r3 = msgs["/robot_3/rayfronts/voxel_rgb"][0]
        check("robot_3 (no boot) copied through UNCHANGED: frame_id stays 'local'",
              r3.header.frame_id == "local", failures)
        check("robot_3 (no boot) copied through UNCHANGED: xyz untouched",
              np.allclose(_read_xyz(r3), [[7.0, 8.0, 9.0]]), failures)

        odom = msgs["/robot_1/odometry_conversion/odometry"][0]
        check("unrelated topic passed through byte-identical",
              odom.data == "unrelated", failures)

        check("original bag directory untouched (still has its own metadata.yaml)",
              (bag_dir / "metadata.yaml").exists() and (bag_dir / "test_0.mcap").exists(),
              failures)
        check("output metadata.yaml was generated",
              (out_dir / "metadata.yaml").exists(), failures)

        # ── 3. manual --boot override wins over everything ──────────────────
        out_dir2 = tmp / "gcs_worldframe_manual"
        summary2 = fbvf.fix_bag(bag_dir, out_dir2,
                                 manual_boots={"robot_1": (1.0, 2.0)})
        check("manual boot override wins over map_origin/status",
              summary2["boots"]["robot_1"] == (1.0, 2.0, "manual"), failures)

        # ── 4. refuses to clobber an existing output without --force ────────
        raised = False
        try:
            fbvf.fix_bag(bag_dir, out_dir)
        except FileExistsError:
            raised = True
        check("fix_bag refuses to overwrite an existing output dir without force",
              raised, failures)

    print()
    if failures:
        print(f"{len(failures)} FAILURE(S): {failures}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

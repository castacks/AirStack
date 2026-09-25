# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""End-to-end test of scripts/mtl_foxglove.py on a synthetic 3-robot run.

Writes per-robot rosbag2-style MCAP files with the SAME ROS 2 message definitions the
robots record (mtl_msgs from this repo, standard messages inline), runs the exporter,
and checks the team file: topics, schemas, the TF tree, camera frames, raw copies.
Skipped when the host-side viz dependencies (scripts/requirements-mtl-viz.txt) are
not installed.
"""

import importlib.util
import json
import math
import re
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

REPO = Path(__file__).resolve().parents[6]

STD = {
    "builtin_interfaces/Time": "int32 sec\nuint32 nanosec",
    "std_msgs/Header": "builtin_interfaces/Time stamp\nstring frame_id",
    "geometry_msgs/Point": "float64 x\nfloat64 y\nfloat64 z",
    "geometry_msgs/Vector3": "float64 x\nfloat64 y\nfloat64 z",
    "geometry_msgs/Quaternion": "float64 x\nfloat64 y\nfloat64 z\nfloat64 w",
    "geometry_msgs/Pose": "geometry_msgs/Point position\ngeometry_msgs/Quaternion orientation",
    "geometry_msgs/PoseWithCovariance": "geometry_msgs/Pose pose\nfloat64[36] covariance",
    "geometry_msgs/Twist": "geometry_msgs/Vector3 linear\ngeometry_msgs/Vector3 angular",
    "geometry_msgs/TwistWithCovariance": "geometry_msgs/Twist twist\nfloat64[36] covariance",
    "nav_msgs/Odometry": ("std_msgs/Header header\nstring child_frame_id\n"
                          "geometry_msgs/PoseWithCovariance pose\ngeometry_msgs/TwistWithCovariance twist"),
    "sensor_msgs/Image": ("std_msgs/Header header\nuint32 height\nuint32 width\nstring encoding\n"
                          "uint8 is_bigendian\nuint32 step\nuint8[] data"),
    "sensor_msgs/RegionOfInterest": "uint32 x_offset\nuint32 y_offset\nuint32 height\nuint32 width\nbool do_rectify",
    "sensor_msgs/CameraInfo": ("std_msgs/Header header\nuint32 height\nuint32 width\nstring distortion_model\n"
                               "float64[] d\nfloat64[9] k\nfloat64[9] r\nfloat64[12] p\nuint32 binning_x\n"
                               "uint32 binning_y\nsensor_msgs/RegionOfInterest roi"),
    "sensor_msgs/NavSatStatus": "int8 status\nuint16 service",
    "sensor_msgs/NavSatFix": ("std_msgs/Header header\nsensor_msgs/NavSatStatus status\nfloat64 latitude\n"
                              "float64 longitude\nfloat64 altitude\nfloat64[9] position_covariance\n"
                              "uint8 position_covariance_type"),
    "std_msgs/String": "string data",
}
PRIMS = {"bool", "byte", "char", "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64",
         "float32", "float64", "string", "wstring"}


def _msg_text(name: str) -> str:
    if name in STD:
        return STD[name]
    pkg, typ = name.split("/")
    for root in (REPO / "common/ros_packages/msgs", REPO / "robot/ros_ws/src"):
        hit = next(root.glob(f"**/{pkg}/msg/{typ}.msg"), None)
        if hit:
            return hit.read_text()
    raise FileNotFoundError(name)


def _deps(pkg: str, text: str) -> list[str]:
    out = []
    for raw in text.splitlines():
        ln = raw.split("#", 1)[0].strip()
        if not ln or "=" in ln.split()[0] or re.match(r"^[A-Z0-9_]+\s*=", ln):
            continue
        parts = ln.split()
        if len(parts) < 2 or re.match(r"^\w+\s+[A-Z0-9_]+\s*=", ln):
            continue
        t = re.sub(r"\[.*\]$", "", parts[0])
        t = re.sub(r"<=\d+$", "", t)
        if t in PRIMS:
            continue
        if t == "Header":
            t = "std_msgs/Header"
        elif "/" not in t:
            t = f"{pkg}/{t}"
        out.append(t.replace("/msg/", "/"))
    return out


def msgdef(name: str) -> str:
    seen, order, stack = set(), [], [name]
    while stack:
        n = stack.pop()
        if n in seen:
            continue
        seen.add(n)
        order.append(n)
        stack.extend(_deps(n.split("/")[0], _msg_text(n)))
    body = _msg_text(name)
    for dep in order[1:]:
        body += "\n" + "=" * 80 + f"\nMSG: {dep}\n" + _msg_text(dep)
    return body


def _load_exporter():
    spec = importlib.util.spec_from_file_location("mtl_foxglove", REPO / "scripts/mtl_foxglove.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["mtl_foxglove"] = mod   # dataclasses resolve annotations through sys.modules
    spec.loader.exec_module(mod)
    return mod


def _hdr(t_ns, frame="map"):
    return {"stamp": {"sec": t_ns // 10**9, "nanosec": t_ns % 10**9}, "frame_id": frame}


def _write_robot_bag(path: Path, name: str, home, t0: int, heading: float, n_s: float = 12.0):
    from mcap_ros2.writer import Writer
    R = f"/{name}"
    with path.open("wb") as f:
        w = Writer(f)
        sch = {k: w.register_msgdef(k, msgdef(k)) for k in (
            "nav_msgs/Odometry", "geometry_msgs/Vector3", "sensor_msgs/Image", "sensor_msgs/CameraInfo",
            "sensor_msgs/NavSatFix", "mtl_msgs/FollowerStatus", "mtl_msgs/SearchPlan", "std_msgs/String")}
        dt = 50_000_000
        speed = 6.0
        # plan (latched, first)
        wps = [{"position": {"x": speed * s * math.cos(heading), "y": speed * s * math.sin(heading), "z": 30.0},
                "yaw": heading, "velocity": speed} for s in range(0, int(n_s) + 1)]
        plan = {"header": _hdr(t0), "plan_id": f"x/{name}/T", "run_id": "T", "scenario_name": "x",
                "agent_name": name, "start_mission": True,
                "map_origin_in_world": {"x": home[0], "y": home[1], "z": 0.0},
                "trajectory": {"header": _hdr(t0), "waypoints": wps},
                "boresight": [{"x": p["position"]["x"] + 17, "y": p["position"]["y"], "z": -0.0} for p in wps]}
        w.write_message(f"{R}/search/plan", sch["mtl_msgs/SearchPlan"], plan, log_time=t0, publish_time=t0)
        cam = {"header": _hdr(t0, "camera_optical_frame"), "height": 48, "width": 64, "distortion_model": "plumb_bob",
               "d": [0.0] * 5, "k": [55.4, 0, 32, 0, 55.4, 24, 0, 0, 1], "r": [1, 0, 0, 0, 1, 0, 0, 0, 1],
               "p": [55.4, 0, 32, 0, 0, 55.4, 24, 0, 0, 0, 1, 0]}
        w.write_message(f"{R}/gimbal/camera_info", sch["sensor_msgs/CameraInfo"], cam, log_time=t0, publish_time=t0)
        for k in range(int(n_s * 20)):
            t = t0 + k * dt
            s = k * dt / 1e9
            x, y = speed * s * math.cos(heading), speed * s * math.sin(heading)
            cz, sz = math.cos(heading / 2), math.sin(heading / 2)
            odom = {"header": _hdr(t), "child_frame_id": "base_link",
                    "pose": {"pose": {"position": {"x": x, "y": y, "z": 30.0},
                                      "orientation": {"x": 0.0, "y": 0.0, "z": sz, "w": cz}}},
                    "twist": {"twist": {"linear": {"x": speed * math.cos(heading), "y": speed * math.sin(heading),
                                                   "z": 0.0}}}}
            w.write_message(f"{R}/odometry_conversion/odometry", sch["nav_msgs/Odometry"], odom, log_time=t, publish_time=t)
            g = {"x": 0.0, "y": math.radians(60), "z": heading}
            w.write_message(f"{R}/gimbal/state", sch["geometry_msgs/Vector3"], g, log_time=t, publish_time=t)
            w.write_message(f"{R}/gimbal/cmd_pitch_yaw", sch["geometry_msgs/Vector3"], g, log_time=t, publish_time=t)
            state = "SEARCH" if s < n_s - 1 else "COMPLETE"
            stt = {"header": _hdr(t), "plan_id": f"x/{name}/T", "state": 2 if state == "SEARCH" else 3,
                   "state_name": state, "progress_m": speed * s, "total_m": speed * n_s,
                   "remaining_m": speed * (n_s - s), "cross_track_error_m": 0.3, "elapsed_s": s, "track_index": k,
                   "carrot": {"x": x + 14, "y": y, "z": 30.0}, "aim_point": {"x": x + 17, "y": y, "z": 0.0},
                   "gimbal_cmd": g}
            w.write_message(f"{R}/search/follower_status", sch["mtl_msgs/FollowerStatus"], stt, log_time=t, publish_time=t)
            if k % 4 == 0:
                img = {"header": _hdr(t, "camera_optical_frame"), "height": 48, "width": 64, "encoding": "rgb8",
                       "is_bigendian": 0, "step": 64 * 3, "data": bytes((k * 7 + i) % 256 for i in range(64 * 48 * 3))}
                w.write_message(f"{R}/gimbal/rgb", sch["sensor_msgs/Image"], img, log_time=t, publish_time=t)
                fix = {"header": _hdr(t, "base_link"), "status": {"status": 0, "service": 1},
                       "latitude": 38.7 + y / 111000, "longitude": -9.1 + x / 87000, "altitude": 30.0,
                       "position_covariance": [0.0] * 9, "position_covariance_type": 0}
                w.write_message(f"{R}/interface/mavros/global_position/global", sch["sensor_msgs/NavSatFix"], fix,
                                log_time=t, publish_time=t)
            if k % 20 == 0:
                w.write_message(f"{R}/search/metrics", sch["std_msgs/String"], {"data": json.dumps({"t": s})},
                                log_time=t, publish_time=t)
        w.finish()


def test_team_export_from_synthetic_bags(tmp_path):
    for mod in ("foxglove", "mcap", "mcap_ros2", "numpy", "PIL"):
        pytest.importorskip(mod)
    if not (REPO / "common/ros_packages/msgs/airstack_msgs").is_dir():
        pytest.skip("airstack_msgs definitions not in this checkout")
    scenario = json.loads((REPO / "stacks/mtl_search/config/scenario.json").read_text())
    run = tmp_path / "T"
    t0 = 1_700_000_000 * 10**9
    for i, a in enumerate(scenario["team"]["agents"]):
        home = (a["home_ned"][1], a["home_ned"][0])
        (run / a["name"] / "bag").mkdir(parents=True)
        _write_robot_bag(run / a["name"] / "bag" / "bag_0.mcap", a["name"], home, t0 + i * 10**8, math.radians(45 + 5 * i))

    fx = _load_exporter()
    out = run / "foxglove" / "T.mcap"
    res = fx.build(run, out, images=True, jpeg_quality=70, image_hz=0.0, raw=True, static_period_s=1.0)
    fx.write_layout(out.parent / "mtl_layout.json", res["robots"])
    assert res["robots"] == ["robot_1", "robot_2", "robot_3"]
    assert all(n == 60 for n in res["images"].values()), res["images"]

    from mcap.reader import make_reader
    with out.open("rb") as f:
        rd = make_reader(f)
        summ = rd.get_summary()
        topics = {c.topic: summ.schemas[c.schema_id].name for c in summ.channels.values()}
        counts = {summ.channels[k].topic: v for k, v in summ.statistics.channel_message_counts.items()}
        last = -1
        for _s, _c, m in rd.iter_messages(log_time_order=False):
            assert m.log_time >= last, "team file must be time-ordered"
            last = m.log_time
    for need, schema in {"/tf": "foxglove.FrameTransforms", "/world/area": "foxglove.SceneUpdate",
                         "/world/belief": "foxglove.Grid", "/world/residual": "foxglove.Grid",
                         "/world/targets": "foxglove.SceneUpdate",
                         "/robot_2/camera/image": "foxglove.CompressedImage",
                         "/robot_2/camera/calibration": "foxglove.CameraCalibration",
                         "/robot_2/trail": "foxglove.SceneUpdate", "/robot_2/sensor": "foxglove.SceneUpdate",
                         "/robot_2/gps": "foxglove.LocationFix", "/events": "foxglove.Log",
                         "/raw/robot_2/odometry_conversion/odometry": "nav_msgs/msg/Odometry",
                         "/raw/robot_2/search/metrics": "std_msgs/msg/String"}.items():
        assert need in topics, f"missing {need}; have {sorted(topics)}"
        assert topics[need].replace("/msg/", "/") == schema.replace("/msg/", "/"), (need, topics[need])
    assert "/raw/robot_2/gimbal/rgb" not in topics           # images are re-encoded, not copied raw
    assert counts["/robot_1/camera/image"] == 60
    assert counts["/robot_1/telemetry"] >= 100
    layout = json.loads((out.parent / "mtl_layout.json").read_text())
    assert "3D!team" in layout["configById"] and "Image!robot_3" in layout["configById"]
    assert "Plot!residual" in layout["configById"]
    s = res["summary"]  # residual belief of the synthetic sorties, P(target missed)
    assert 0.0 < s["residual_belief_mass"] < 1.0 and s["prior_belief_mass"] == pytest.approx(1.0, abs=1e-9)

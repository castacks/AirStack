"""One isolated capture. Raw depth and instance IDs come from one camera frame."""
import json
from pathlib import Path
import time

from isaacsim import SimulationApp

IN = Path("/capture-input")
OUT = Path("/capture-output")
CONFIG = json.loads((IN / "fixture.json").read_text())
app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})


def capture():
    import numpy as np
    import omni.graph.core as og
    import omni.usd
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from isaacsim.core.api import World
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.sensors.camera import Camera
    from pxr import Gf, UsdGeom
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from tf2_ros import Buffer, TransformBroadcaster, TransformListener

    def write(name, value):
        (OUT / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")

    def stamp(msg):
        return msg.sec * 1_000_000_000 + msg.nanosec

    enable_extension("isaacsim.ros2.bridge")
    app.update()
    if not omni.usd.get_context().open_stage(str(IN / "stage.usda")):
        raise RuntimeError("stage open failed")
    app.update()
    stage = omni.usd.get_context().get_stage()
    world = World(physics_dt=.1, rendering_dt=.1, stage_units_in_meters=1.0)
    camera = Camera(CONFIG["camera_prim"], resolution=(33, 25), frequency=10)
    world.reset()
    camera.initialize()
    camera.add_distance_to_image_plane_to_frame()
    camera.add_instance_id_segmentation_to_frame()
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/ReceiptClock", "evaluator_name": "execution"},
        {keys.CREATE_NODES: [("Tick", "omni.graph.action.OnPlaybackTick"),
                            ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                            ("Publish", "isaacsim.ros2.bridge.ROS2PublishClock")],
         keys.CONNECT: [("Tick.outputs:tick", "Publish.inputs:execIn"),
                        ("Time.outputs:simulationTime", "Publish.inputs:timeStamp")]})
    rclpy.init()
    node = rclpy.create_node("ore_isolated_geometry_receipt")
    clocks = []
    clock_sub = node.create_subscription(Clock, "/clock", lambda msg: clocks.append(stamp(msg.clock)), 10)
    broadcaster = TransformBroadcaster(node)
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, node)

    def spin_until(predicate, label):
        deadline = time.monotonic() + 2
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if not predicate():
            raise RuntimeError(f"timed out waiting for {label}")

    def tick():
        count = len(clocks)
        world.step(render=True)
        spin_until(lambda: len(clocks) > count, "Isaac /clock")
        if len(clocks) != count + 1:
            raise RuntimeError("multiple clocks for one simulator step")
        return clocks[-1]

    def survey():
        mesh = UsdGeom.Mesh.Get(stage, CONFIG["target_prim"])
        matrix = mesh.ComputeLocalToWorldTransform(0)
        verts = [list(matrix.Transform(Gf.Vec3d(*point))) for point in mesh.GetPointsAttr().Get()]
        renderables = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdGeom.Gprim)]
        return {"meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
                "up_axis": UsdGeom.GetStageUpAxis(stage), "renderables": renderables,
                "vertices_m": verts, "layer_identifiers": [layer.identifier for layer in stage.GetUsedLayers()]}

    def raw_frame(prefix):
        frame = camera.get_current_frame()
        depth = np.array(frame["distance_to_image_plane"])
        instance = frame["instance_id_segmentation"]
        ids = np.array(instance["data"])
        if depth.shape != (25, 33) or depth.dtype != np.float32 or ids.shape != depth.shape:
            raise RuntimeError(f"unexpected native data {depth.shape} {depth.dtype} {ids.shape}")
        np.save(OUT / f"{prefix}-depth.npy", depth, allow_pickle=False)
        np.save(OUT / f"{prefix}-ids.npy", ids, allow_pickle=False)
        labels = instance["info"]["idToLabels"]
        labels = json.loads(labels) if isinstance(labels, str) else labels
        uv = CONFIG["pixels"]
        samples = [{"uv": [u, v], "axial_m": float(depth[v,u]),
                    "first_hit": labels[str(int(ids[v,u]))]} for u,v in uv]
        record = {"rendering_frame": int(frame["rendering_frame"]),
                  "rendering_time": float(frame["rendering_time"]),
                  "labels": labels, "samples": samples, "survey": survey()}
        write(f"{prefix}-render.json", record)
        if any(row["first_hit"] != CONFIG["target_prim"] for row in samples):
            raise RuntimeError(f"{prefix}: no unambiguous target instance")
        return record

    def matrix_for_camera():
        measured = UsdGeom.Camera.Get(stage, CONFIG["camera_prim"]).ComputeLocalToWorldTransform(0)
        origin = list(measured.Transform(Gf.Vec3d(0, 0, 0)))
        optical_axes = [list(measured.TransformDir(Gf.Vec3d(*axis))) for axis in
                        ((1,0,0), (0,-1,0), (0,0,-1))]
        matrix = np.array(optical_axes).T
        rotation = Gf.Matrix3d(*(float(x) for x in matrix.T.flat)).ExtractRotation().GetQuat()
        return origin, matrix, rotation

    def observation(index, time_ns):
        prefix = f"tick-{index}"
        frame = raw_frame(prefix)
        if round(frame["rendering_time"] * 1_000_000_000) != time_ns:
            raise RuntimeError(f"render timestamp mismatch: {frame['rendering_time']}, clock={time_ns}")
        k = camera.get_intrinsics_matrix().tolist()
        # Isaac's image coordinates run edge-to-edge. Integer array indices
        # denote centers displaced by half a pixel from those coordinates.
        intrinsics = dict(width=33, height=25, fx=k[0][0], fy=k[1][1],
                          cx=k[0][2] - .5, cy=k[1][2] - .5)
        if intrinsics != CONFIG["intrinsics"]:
            raise RuntimeError(f"intrinsics mismatch: {intrinsics}")
        origin, matrix, q = matrix_for_camera()
        t = TransformStamped()
        t.header.stamp = Time(nanoseconds=time_ns).to_msg()
        t.header.frame_id, t.child_frame_id = CONFIG["stage_frame"], CONFIG["camera_frame"]
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = origin
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z = q.GetImaginary()
        t.transform.rotation.w = q.GetReal()
        broadcaster.sendTransform(t)
        query = Time(nanoseconds=time_ns)
        spin_until(lambda: tf_buffer.can_transform(t.header.frame_id, t.child_frame_id, query), "exact TF")
        got = tf_buffer.lookup_transform(t.header.frame_id, t.child_frame_id, query)
        tr, rot = got.transform.translation, got.transform.rotation
        tf_xyz = [tr.x, tr.y, tr.z]
        tf_quat = [rot.x, rot.y, rot.z, rot.w]
        tf_matrix = Gf.Matrix3d(Gf.Quatd(rot.w, Gf.Vec3d(*tf_quat[:3])))
        write(f"{prefix}-raw.json", {"tick": index, "clock_ns": time_ns, "camera_intrinsics_native": k,
                                    "camera_origin_m": origin, "camera_optical_to_stage": matrix.tolist(),
                                    "tf_stamp_ns": stamp(got.header.stamp), "tf_parent": got.header.frame_id,
                                    "tf_child": got.child_frame_id, "tf_translation": tf_xyz,
                                    "tf_quaternion_xyzw": tf_quat,
                                    "tf_optical_to_stage": np.array(tf_matrix).T.tolist()})

    try:
        world.play()
        # Let local DDS discover before taking any depth observation.
        for _ in range(3):
            app.update()
        target = stage.GetPrimAtPath(CONFIG["target_prim"]).GetAttribute("xformOp:translate")
        controls = []
        for metres in (5, 15):
            target.Set(Gf.Vec3d(0, metres - 10, 0))
            for _ in range(3):
                tick()
            frame = raw_frame(f"control-{metres}")
            value = frame["samples"][0]["axial_m"]
            if abs(value - metres) + float(np.spacing(np.float32(value))) >= CONFIG["maximum_range_error_m"]:
                raise RuntimeError(f"{metres} m control outside predeclared ceiling: {value}")
            controls.append(value)
            print(f"control {metres} m passed: {value}", flush=True)
        write("preflight.json", {"control_axial_m": controls, "renderer": "RaytracedLighting"})
        target.Set(Gf.Vec3d(0, 0, 0))
        for _ in range(3):
            tick()
        epoch = clocks[-1]
        selected = []
        for index in range(1, 31):
            now = tick()
            if now != epoch + index * CONFIG["tick_period_ns"]:
                write("clock-failure.json", {"epoch_ns": epoch, "tick": index, "observed_ns": now,
                                             "raw_received_ns": clocks})
                raise RuntimeError(f"simulation clock cadence failed at tick {index}")
            if index >= CONFIG["issue_tick"]:
                selected.append(now)
            if index in (CONFIG["issue_tick"], CONFIG["target_tick"]):
                observation(index, now)
        write("clock.json", {"epoch_ns": epoch, "samples_ns": selected, "raw_received_ns": clocks})
    finally:
        node.destroy_node()
        rclpy.shutdown()


try:
    capture()
finally:
    app.close()

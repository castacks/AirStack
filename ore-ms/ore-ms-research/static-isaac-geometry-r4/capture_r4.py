"""Bounded stationary geometry capture with R3 native writer and shutdown evidence."""
import json
import math
from pathlib import Path
import struct
import time

from capture_one_r3 import attach_writers
from readiness import acquire, DEPTH, INSTANCE, writer_packet
from recorder import CAMERA, HEIGHT, TARGET, WIDTH, append_record, checkpoint, phase, write_artifact, write_json
from supervise_r3 import complete_acquisition
from review_r4 import raw_hashes, validate_controls

INPUT = Path("/capture-input")
OUTPUT = Path("/capture-output")
SPEC = json.loads((INPUT / "fixture.json").read_text()) if INPUT.exists() else None
NS = 1_000_000_000


def capture(app):
    with phase(OUTPUT, "stage_camera_writers"):
        import omni.usd
        import omni.replicator.core as rep
        from isaacsim.core.api import World
        from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
        from isaacsim.sensors.camera import Camera
        from pxr import Gf, UsdGeom
        context = omni.usd.get_context()
        if not context.open_stage(str(INPUT / "stage.usda")):
            raise RuntimeError("cannot open isolated stage")
        app.update()
        stage = context.get_stage()
        camera_prim = UsdGeom.Camera.Get(stage, CAMERA)
        camera_xform = UsdGeom.Xformable(camera_prim)
        authored_ops = camera_xform.GetOrderedXformOps()
        authored_values = [op.Get() for op in authored_ops]
        world = World(physics_dt=.1, rendering_dt=.1, stage_units_in_meters=1.0)
        camera = Camera(CAMERA, resolution=(WIDTH, HEIGHT), frequency=10)
        world.reset()
        camera.initialize(attach_rgb_annotator=False)
        # Camera preparation may canonicalize xform ops. Restore the stage's
        # authored stationary pose before the first acquisition.
        for op, value in zip(authored_ops, authored_values, strict=True):
            op.Set(value)
        camera_xform.SetXformOpOrder(authored_ops)
        product = camera.get_render_product_path()
        rep.orchestrator.set_capture_on_play(False)
        depth_writer, id_writer = attach_writers(rep, product)
        timestamp = _isaacsim_core_nodes.acquire_interface().get_sim_time_at_time
        mesh = UsdGeom.Mesh.Get(stage, TARGET)
        translation = mesh.GetPrim().GetAttribute("xformOp:translate")
        camera_native = camera.get_intrinsics_matrix().tolist()
        if (camera_xform.TransformMightBeTimeVarying() or mesh.TransformMightBeTimeVarying()
                or mesh.GetPointsAttr().ValueMightBeTimeVarying()):
            raise RuntimeError("nonstationary fixture transform")

    def step():
        depth_writer.packet = {}
        id_writer.packet = {}
        rep.orchestrator.step(rt_subframes=1, pause_timeline=True, delta_time=.1, wait_for_render=True)
        return writer_packet(depth_writer.packet, DEPTH), writer_packet(id_writer.packet, INSTANCE)

    previous_ref = None

    def save(name, collect, clock=None):
        nonlocal previous_ref
        folder = OUTPUT / name
        folder.mkdir()
        with phase(OUTPUT, name):
            native = acquire(folder, collect, timestamp)
            reference = native.association["depth_reference"]
            if previous_ref is not None and reference[0] * previous_ref[1] <= previous_ref[0] * reference[1]:
                raise RuntimeError("stale or reordered native render reference")
            previous_ref = reference
            simulation_s = native.association["rendering_time_s"]
            at = simulation_s if clock is None else clock() / NS
            sample_time = at * stage.GetTimeCodesPerSecond()
            camera_matrix = camera_prim.ComputeLocalToWorldTransform(sample_time)
            target_matrix = mesh.ComputeLocalToWorldTransform(sample_time)
            write_artifact(folder / "depth.f32le", native.depth)
            write_artifact(folder / "instances.u32le", native.instances)
            write_json(folder / "frame.json", {
                "schema": 2, "source": "isaac", "width": WIDTH, "height": HEIGHT,
                "rendering_time_s": simulation_s, "frame_association": native.association,
                "render_product": product, "camera_prim": CAMERA, "target_prim": TARGET,
                "stage_meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
                "stage_up_axis": str(UsdGeom.GetStageUpAxis(stage)), "intrinsics_native": camera_native,
                "camera_local_to_world_usd": [list(row) for row in camera_matrix],
                "depth_annotator": DEPTH, "instance_annotator": INSTANCE,
                "depth_native_dtype": "float32", "instance_native_dtype": "uint32",
                "id_to_labels": native.labels,
            })
            write_json(folder / "survey.json", {
                "usd_time_code": sample_time, "simulation_clock_ns": None if clock is None else clock(),
                "vertices_m": [list(target_matrix.Transform(Gf.Vec3d(*p)))
                               for p in mesh.GetPointsAttr().Get(sample_time)],
                "renderable_prims": [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdGeom.Gprim)],
                "used_layers": [layer.identifier for layer in stage.GetUsedLayers()],
            })
            complete_acquisition(folder, "isaac")
            return camera_matrix, simulation_s

    for metres in (5, 15):
        translation.Set(Gf.Vec3d(0, metres - 10, 0))
        save(f"control-{metres}", step)
        with phase(OUTPUT, f"control-{metres}-analytic"):
            validate_controls(OUTPUT, SPEC, (metres,))
    translation.Set(Gf.Vec3d(0, 0, 0))

    with phase(OUTPUT, "ros_clock_setup"):
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.ros2.bridge")
        app.update()
        import omni.graph.core as og
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from rclpy.time import Time
        from rosgraph_msgs.msg import Clock
        from tf2_ros import Buffer, TransformBroadcaster, TransformListener
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/World/ReceiptClock", "evaluator_name": "execution"},
            {keys.CREATE_NODES: [("Tick", "omni.graph.action.OnPlaybackTick"),
                                ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                                ("Publish", "isaacsim.ros2.bridge.ROS2PublishClock")],
             keys.CONNECT: [("Tick.outputs:tick", "Publish.inputs:execIn"),
                            ("Time.outputs:simulationTime", "Publish.inputs:timeStamp")]})
        rclpy.init()
        node = rclpy.create_node("ore_r4_isolated_receipt")
        clock_messages = []

        def record_clock(msg):
            clock_messages.append(msg.clock.sec * NS + msg.clock.nanosec)
            append_record(OUTPUT / "clock-raw.jsonl", {"sec": msg.clock.sec, "nanosec": msg.clock.nanosec})

        subscription = node.create_subscription(Clock, "/clock", record_clock, 10)
        broadcaster = TransformBroadcaster(node)
        buffer = Buffer()
        listener = TransformListener(buffer, node)

    def wait_for(predicate, what):
        deadline = time.monotonic() + 3
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if not predicate():
            raise RuntimeError(f"timed out waiting for {what}")

    def clock_step():
        before = len(clock_messages)
        packets = step()
        wait_for(lambda: len(clock_messages) > before, "Isaac /clock")
        rclpy.spin_once(node, timeout_sec=0.0)
        if len(clock_messages) != before + 1:
            raise RuntimeError("ambiguous /clock messages for a step")
        return packets

    def tf(folder, measured, source_stamp):
        axis = [list(measured.TransformDir(Gf.Vec3d(*xyz))) for xyz in
                ((1, 0, 0), (0, -1, 0), (0, 0, -1))]
        origin = list(measured.Transform(Gf.Vec3d(0, 0, 0)))
        quat = Gf.Matrix3d(*(v for row in axis for v in row)).ExtractRotation().GetQuat()
        stamp = Time(nanoseconds=source_stamp)
        msg = TransformStamped()
        msg.header.stamp, msg.header.frame_id = stamp.to_msg(), SPEC["stage_frame"]
        msg.child_frame_id = SPEC["camera_frame"]
        tr = msg.transform.translation
        tr.x, tr.y, tr.z = origin
        rot = msg.transform.rotation
        rot.x, rot.y, rot.z = quat.GetImaginary()
        rot.w = quat.GetReal()
        broadcaster.sendTransform(msg)
        args = (msg.header.frame_id, msg.child_frame_id, stamp)
        wait_for(lambda: buffer.can_transform(*args), "exact-time camera TF")
        got = buffer.lookup_transform(*args)
        tr, rot = got.transform.translation, got.transform.rotation
        transpose = Gf.Matrix3d(Gf.Quatd(rot.w, Gf.Vec3d(rot.x, rot.y, rot.z))).GetTranspose()
        write_json(folder / "tf.json", {"parent": got.header.frame_id, "child": got.child_frame_id,
                   "stamp_ns": got.header.stamp.sec * NS + got.header.stamp.nanosec,
                   "origin_m": [tr.x, tr.y, tr.z], "optical_to_stage": [list(row) for row in transpose],
                   "quaternion_xyzw": [rot.x, rot.y, rot.z, rot.w]})

    try:
        with phase(OUTPUT, "clock_epoch"):
            clock_step()
            epoch = clock_messages[-1]
        samples = []
        with phase(OUTPUT, "timed_geometry"):
            for tick in range(1, SPEC["target_tick"] + 1):
                expected_ns = epoch + tick * SPEC["tick_period_ns"]
                name = {SPEC["issue_tick"]: "issue", SPEC["target_tick"]: "target"}.get(tick)
                if name is None:
                    clock_step()
                else:
                    def collect():
                        if len(clock_messages) > tick:
                            raise RuntimeError("native pair unavailable at the exact scheduled tick")
                        return clock_step()

                    measured, native_s = save(name, collect, lambda: clock_messages[-1])
                    # The R3 binding returns a native floating timestamp. Require
                    # exact f32 representation of the witnessed ROS stamp, plus
                    # the independently checked integer clock cadence.
                    if native_s != struct.unpack("<f", struct.pack("<f", clock_messages[-1] / NS))[0]:
                        raise RuntimeError("native writer reference does not resolve to the witnessed clock stamp")
                    tf(OUTPUT / name, measured, clock_messages[-1])
                current = clock_messages[-1]
                if current != expected_ns:
                    append_record(OUTPUT / "clock-checks.jsonl",
                                  {"tick": tick, "expected_ns": expected_ns, "actual_ns": current})
                    raise RuntimeError(f"nonexact /clock cadence at tick {tick}: {current} vs {expected_ns}")
                append_record(OUTPUT / "clock-checks.jsonl", {"tick": tick, "actual_ns": current})
                if tick >= SPEC["issue_tick"]:
                    samples.append(current)
            write_json(OUTPUT / "clock.json", {"source": "IsaacReadSimulationTime:/clock", "epoch_ns": epoch,
                       "period_ns": SPEC["tick_period_ns"], "samples_ns": samples})
    finally:
        node.destroy_node()
        rclpy.shutdown()

    checkpoint(OUTPUT, "sequence", "captured", sha256=raw_hashes(OUTPUT))


if __name__ == "__main__":
    with phase(OUTPUT, "isaac_startup"):
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    try:
        capture(app)
    finally:
        with phase(OUTPUT, "shutdown"):
            app.close()

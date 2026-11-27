"""One stationary Isaac capture with independently referenced writer packets."""

from copy import deepcopy
from pathlib import Path

from readiness import DEPTH, INSTANCE, acquire, writer_packet
from recorder import CAMERA, HEIGHT, TARGET, WIDTH, phase, write_artifact, write_json

STAGE = Path("/capture-input/stage.usda")
OUTPUT = Path("/capture-output")


def attach_writers(rep, render_product):
    class FrameWriter(rep.Writer):
        def __init__(self):
            self.data_structure = "legacy"  # Flat keys for exactly one render product.
            self.packet = {}
            params = {"colorize": False} if self.sensor == INSTANCE else {}
            self.add_annotator(rep.AnnotatorRegistry.get_annotator(self.sensor, init_params=params),
                               name=self.sensor)
            self.add_annotator(rep.AnnotatorRegistry.get_annotator("ReferenceTime"),
                               name="ReferenceTime")

        def write(self, data):
            self.packet = deepcopy(data)

    class OreDepthWriter(FrameWriter):
        sensor = DEPTH

    class OreInstanceWriter(FrameWriter):
        sensor = INSTANCE

    writers = []
    for writer_class in (OreDepthWriter, OreInstanceWriter):
        rep.WriterRegistry.register(writer_class)
        writer = rep.WriterRegistry.get(writer_class.__name__)
        writer.initialize()
        writer.attach([render_product])
        writers.append(writer)
    return writers


def capture():
    with phase(OUTPUT, "isaac_startup"):
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    try:
        with phase(OUTPUT, "stage_open"):
            import omni.usd

            context = omni.usd.get_context()
            if not context.open_stage(str(STAGE)):
                raise RuntimeError("isolated USD stage open failed")
            app.update()
            stage = context.get_stage()
            if stage is None:
                raise RuntimeError("isolated USD stage is unavailable")

        with phase(OUTPUT, "camera_initialize"):
            from isaacsim.core.api import World
            from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
            from isaacsim.sensors.camera import Camera
            from pxr import UsdGeom

            world = World(physics_dt=.1, rendering_dt=.1, stage_units_in_meters=1.0)
            camera = Camera(CAMERA, resolution=(WIDTH, HEIGHT), frequency=10)
            world.reset()
            camera.initialize(attach_rgb_annotator=False)
            render_product = camera.get_render_product_path()
            if not render_product:
                raise RuntimeError("camera did not create a render product")
            core_nodes = _isaacsim_core_nodes.acquire_interface()

        with phase(OUTPUT, "annotators_attach"):
            import omni.replicator.core as rep

            rep.orchestrator.set_capture_on_play(False)
            depth_writer, instance_writer = attach_writers(rep, render_product)

        def step_and_read():
            depth_writer.packet = {}
            instance_writer.packet = {}
            rep.orchestrator.step(rt_subframes=1, pause_timeline=True, delta_time=.1,
                                  wait_for_render=True)
            return (writer_packet(depth_writer.packet, DEPTH),
                    writer_packet(instance_writer.packet, INSTANCE))

        with phase(OUTPUT, "frame_readiness"):
            frame = acquire(OUTPUT, step_and_read, core_nodes.get_sim_time_at_time)

        with phase(OUTPUT, "write_artifacts"):
            camera_transform = UsdGeom.Camera.Get(stage, CAMERA).ComputeLocalToWorldTransform(0)
            metadata = {
                "schema": 2, "source": "isaac", "width": WIDTH, "height": HEIGHT,
                "rendering_time_s": frame.association["rendering_time_s"],
                "frame_association": frame.association, "render_product": render_product,
                "camera_prim": CAMERA, "target_prim": TARGET,
                "stage_meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
                "stage_up_axis": str(UsdGeom.GetStageUpAxis(stage)),
                "intrinsics_native": camera.get_intrinsics_matrix().tolist(),
                "camera_local_to_world_usd": [list(row) for row in camera_transform],
                "depth_annotator": DEPTH, "instance_annotator": INSTANCE,
                "depth_native_dtype": "float32", "instance_native_dtype": "uint32",
                "id_to_labels": frame.labels,
            }
            write_artifact(OUTPUT / "depth.f32le", frame.depth)
            write_artifact(OUTPUT / "instances.u32le", frame.instances)
            write_json(OUTPUT / "frame.json", metadata)
    finally:
        with phase(OUTPUT, "shutdown"):
            app.close()


if __name__ == "__main__":
    capture()

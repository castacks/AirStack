"""Diagnostic worker for one native Isaac depth/instance frame on the R1 stage."""

import json
from pathlib import Path

from recorder import CAMERA, HEIGHT, TARGET, WIDTH, phase, write_artifact, write_json

STAGE = Path("/capture-input/stage.usda")
OUTPUT = Path("/capture-output")


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
            import numpy as np
            from isaacsim.core.api import World
            from isaacsim.sensors.camera import Camera
            from pxr import UsdGeom

            world = World(physics_dt=.1, rendering_dt=.1, stage_units_in_meters=1.0)
            camera = Camera(CAMERA, resolution=(WIDTH, HEIGHT), frequency=10)
            world.reset()
            camera.initialize()
            camera.add_distance_to_image_plane_to_frame()
            camera.add_instance_id_segmentation_to_frame({"colorize": False})

        with phase(OUTPUT, "simulation_start"):
            world.play()
        for step in range(1, 4):
            with phase(OUTPUT, f"render_step_{step}"):
                world.step(render=True)

        with phase(OUTPUT, "frame_readout"):
            frame = camera.get_current_frame(clone=True)
            instance = frame["instance_id_segmentation"]
            depth = np.asarray(frame["distance_to_image_plane"])
            ids = np.asarray(instance["data"])
            if depth.shape != (HEIGHT, WIDTH) or depth.dtype != np.float32:
                raise ValueError(f"unexpected native depth {depth.shape} {depth.dtype}")
            if ids.shape == (HEIGHT, WIDTH, 1):
                ids = ids[:, :, 0]
            if ids.shape != depth.shape or ids.dtype != np.uint32:
                raise ValueError(f"unexpected native instance data {ids.shape} {ids.dtype}")
            depth_bytes = depth.astype("<f4", copy=False).tobytes()
            instance_bytes = ids.astype("<u4", copy=False).tobytes()
            labels = instance["info"]["idToLabels"]
            if isinstance(labels, str):
                labels = json.loads(labels)
            labels = {str(key): value for key, value in labels.items()}
            camera_transform = UsdGeom.Camera.Get(stage, CAMERA).ComputeLocalToWorldTransform(0)
            metadata = {
                "schema": 1, "source": "isaac", "width": WIDTH, "height": HEIGHT,
                "rendering_frame": int(frame["rendering_frame"]),
                "rendering_time_s": float(frame["rendering_time"]),
                "camera_prim": CAMERA, "target_prim": TARGET,
                "stage_meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
                "stage_up_axis": str(UsdGeom.GetStageUpAxis(stage)),
                "intrinsics_native": camera.get_intrinsics_matrix().tolist(),
                "camera_local_to_world_usd": [list(row) for row in camera_transform],
                "depth_annotator": "distance_to_image_plane",
                "instance_annotator": "instance_id_segmentation",
                "depth_native_dtype": str(depth.dtype), "instance_native_dtype": str(ids.dtype),
                "id_to_labels": labels,
            }

        with phase(OUTPUT, "write_artifacts"):
            write_artifact(OUTPUT / "depth.f32le", depth_bytes)
            write_artifact(OUTPUT / "instances.u32le", instance_bytes)
            write_json(OUTPUT / "frame.json", metadata)
    finally:
        with phase(OUTPUT, "shutdown"):
            app.close()


if __name__ == "__main__":
    capture()

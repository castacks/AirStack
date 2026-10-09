#!/usr/bin/env python
"""Office scene with two labelled visual markers for RRM evidence evaluation.

The marker positions are a scene/test fixture, not RRM coordinates.  RRM sees only
semantic IDs through its task-scoped catalog; a later drone adapter owns map waypoints.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pegasus_app import create_simulation_app

# Pin this evaluation scene to GPU 0 even when Compose launches it automatically.
# Append before SimulationApp parses Kit flags; no change to other scenes.
sys.argv.extend([
    "--/renderer/activeGpu=0", "--/renderer/multiGpu/enabled=false",
    "--/physics/cudaDevice=0",
])
simulation_app = create_simulation_app()

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS  # noqa: E402
from pegasus_app import PegasusApp, resolve_scene_from_env  # noqa: E402


class RrmOfficeVisualEval(PegasusApp):
    """Add static marker prims after normal Office preparation and before drone spawn."""

    def post_scene_prep(self, stage):
        from pxr import Gf, Sdf, UsdGeom, UsdShade
        from isaacsim.core.utils.semantics import add_update_semantics

        markers = (
            ("blue_marker", (4.0, 0.0, 1.0), (0.05, 0.2, 1.0)),
            ("orange_marker", (4.0, -1.5, 1.0), (1.0, 0.22, 0.03)),
        )
        UsdGeom.Xform.Define(stage, "/World/RRMMarkers")
        for entity_id, position, color in markers:
            marker = UsdGeom.Cube.Define(stage, f"/World/RRMMarkers/{entity_id}")
            marker.CreateSizeAttr(0.8)
            marker.AddTranslateOp().Set(Gf.Vec3d(*position))
            marker.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            marker.GetPrim().CreateAttribute("rrm:entity_id", Sdf.ValueTypeNames.String).Set(entity_id)
            marker.GetPrim().CreateAttribute("rrm:kind", Sdf.ValueTypeNames.String).Set(
                entity_id.removesuffix("_marker") + " navigation marker")
            add_update_semantics(marker.GetPrim(), entity_id)
            material = UsdShade.Material.Define(stage, f"/World/RRMMarkers/{entity_id}_material")
            shader = UsdShade.Shader.Define(stage, f"/World/RRMMarkers/{entity_id}_shader")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI(marker.GetPrim()).Bind(material)
        print("[rrm-office-eval] authored blue_marker and orange_marker")

    def post_spawn(self, stage):
        """Opt-in bounded observation of actual stage prims; never navigation authority."""
        directory = os.environ.get("ISAAC_SIM_TRUTH_DIR", "").strip()
        if not directory:
            return
        import hashlib
        import json
        import time
        import uuid
        from pathlib import Path
        from pxr import UsdGeom

        output = Path(directory) / "office-marker-observation.json"
        output.parent.mkdir(parents=True, exist_ok=True)
        try:
            from office_camera_teacher import OfficeCameraTeacher
            self.office_camera_teacher = OfficeCameraTeacher(directory, stage,
                launcher_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        except Exception as exc:
            print("[office-camera-teacher] initialization disabled: " + str(exc))
        episode = uuid.uuid4().hex
        count, elapsed, last = 0, 0.0, -1.0

        def observe(dt):
            nonlocal count, elapsed, last
            elapsed += float(dt)
            if count >= 600 or elapsed - last < 1.0:
                return
            last = elapsed
            try:
                cache = UsdGeom.XformCache()
                markers = []
                for entity in ("blue_marker", "orange_marker"):
                    prim = stage.GetPrimAtPath("/World/RRMMarkers/" + entity)
                    valid = bool(prim.IsValid() and prim.IsActive())
                    markers.append(dict(entity_id=entity, prim_path=str(prim.GetPath()),
                        exists=valid, observed_entity_id=prim.GetAttribute("rrm:entity_id").Get() if valid else None,
                        world_position_stage_units=list(cache.GetLocalToWorldTransform(prim).ExtractTranslation()) if valid else None))
                record = dict(schema_version="rrm-office-stage-observation/v1", episode_id=episode,
                    sample_index=count, receipt_monotonic_s=time.monotonic(),
                    physics_callback_elapsed_s=elapsed, engine_time_s=float(self.world.current_time),
                    stage_meters_per_unit=float(UsdGeom.GetStageMetersPerUnit(stage)),
                    environment_ref=self.env_url, stage_scale=self.stage_scale, markers=markers,
                    launcher_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    camera_frame_bound=False, map_alignment_verified=False, execution_dispatch=False)
                temporary = output.with_suffix(".json.tmp")
                temporary.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
                temporary.replace(output)
                count += 1
            except Exception as exc:
                count = 600
                print("[rrm-office-eval] stage observation disabled: " + str(exc))

        self.world.add_physics_callback("rrm_office_stage_observation", observe)

    def post_step(self):
        # OmniGraph attach/detach mutates the graph; never do it during PhysX work.
        teacher = getattr(self, "office_camera_teacher", None)
        if teacher is not None:
            teacher.poll()


def main():
    env_url, stage_scale = resolve_scene_from_env(SIMULATION_ENVIRONMENTS)
    print(f"[rrm-office-eval] Scene: {env_url} (stage_scale={stage_scale})")
    RrmOfficeVisualEval(
        env_url=env_url,
        stage_scale=stage_scale,
        drone_configs=[{
            "domain_id": 1, "x_m": 0.0, "y_m": 0.0, "z_m": 0.07,
            "prim": "/World/base_link", "node_name": "PX4Multirotor",
        }],
        enable_lidar=True,
    ).run()


if __name__ == "__main__":
    main()

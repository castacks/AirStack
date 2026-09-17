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
            material = UsdShade.Material.Define(stage, f"/World/RRMMarkers/{entity_id}_material")
            shader = UsdShade.Shader.Define(stage, f"/World/RRMMarkers/{entity_id}_shader")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI(marker.GetPrim()).Bind(material)
        print("[rrm-office-eval] authored blue_marker and orange_marker")


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

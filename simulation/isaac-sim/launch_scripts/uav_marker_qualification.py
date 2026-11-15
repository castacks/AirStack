#!/usr/bin/env python3
"""Passive red/blue visual fixture scene; one drone, no automatic flight commands."""
import hashlib
import json
import math
import os
from pathlib import Path
import sys

DIAGNOSTIC_PITCH=float(os.environ.get('UAV_DIAGNOSTIC_PITCH_RAD','0'))
if not math.isfinite(DIAGNOSTIC_PITCH) or abs(DIAGNOSTIC_PITCH)>.1:
    raise ValueError('diagnostic initial pitch must be finite within +/-0.1 rad')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pegasus_app import create_simulation_app
simulation_app = create_simulation_app()
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus_app import PegasusApp


class MarkerQualification(PegasusApp):
    def post_step(self):
        directory=os.environ.get('UAV_CONTACT_DIR','')
        if directory:
            if not hasattr(self,'contact_capture'):
                from uav_contact_capture import UavContactCapture
                self.contact_capture=UavContactCapture(directory)
            import omni.usd
            self.contact_capture.step(omni.usd.get_context().get_stage(),float(self.world.current_time))

    def post_scene_prep(self, stage):
        from pxr import Gf, UsdGeom, UsdPhysics, UsdShade
        fixtures = [dict(id='red_panel',position=[4.,0.,1.2],color=[1.,.02,.02]),
                    dict(id='blue_distractor',position=[4.,-1.5,1.2],color=[.02,.1,1.])]
        for item in fixtures:
            path='/World/UAVFixtures/'+item['id'];cube=UsdGeom.Cube.Define(stage,path)
            cube.CreateSizeAttr(.6);cube.AddTranslateOp().Set(Gf.Vec3d(*item['position']))
            cube.CreateDisplayColorAttr([Gf.Vec3f(*item['color'])]);UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            material=UsdShade.Material.Define(stage,path+'Material');shader=UsdShade.Shader.Define(stage,path+'Shader')
            shader.CreateIdAttr('UsdPreviewSurface')
            from pxr import Sdf
            for name in ('diffuseColor','emissiveColor'):shader.CreateInput(name,Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*item['color']))
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
        output=os.environ.get('UAV_FIXTURE_MANIFEST','')
        if output:
            p=Path(output);p.parent.mkdir(parents=True,exist_ok=True)
            p.write_text(json.dumps(dict(fixtures=fixtures,size_m=.6,static_colliders=True,diagnostic_initial_pitch_rad=DIAGNOSTIC_PITCH,
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2))


def main():
    MarkerQualification(env_url=SIMULATION_ENVIRONMENTS['Default Environment'],
        drone_configs=[dict(domain_id=1,x_m=0.,y_m=0.,z_m=.10 if DIAGNOSTIC_PITCH else .07,orient=[0.,math.sin(DIAGNOSTIC_PITCH/2),0.,math.cos(DIAGNOSTIC_PITCH/2)],prim='/World/base_link',node_name='PX4Multirotor')],
        enable_lidar=True).run()

if __name__=='__main__':main()

#!/usr/bin/env python3
"""Passive red/blue visual fixture scene; one drone, no automatic flight commands."""
import hashlib
import math
import os
from pathlib import Path
import sys

DIAGNOSTIC_PITCH=float(os.environ.get('UAV_DIAGNOSTIC_PITCH_RAD','0'))
if not math.isfinite(DIAGNOSTIC_PITCH) or abs(DIAGNOSTIC_PITCH)>.1:
    raise ValueError('diagnostic initial pitch must be finite within +/-0.1 rad')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uav_marker_fixtures import fixture_spec, author_fixtures, write_manifest
FIXTURE_SPEC=fixture_spec(os.environ.get('UAV_MARKER_SCENARIO','nominal'))
if FIXTURE_SPEC['scenario']!='nominal' and DIAGNOSTIC_PITCH!=0:
    raise ValueError('controlled fixture variants require zero diagnostic pitch')
if os.environ.get('UAV_FIXTURE_MANIFEST') and Path(os.environ['UAV_FIXTURE_MANIFEST']).exists():
    raise ValueError('fixture manifest output must be new')
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
        author_fixtures(stage,FIXTURE_SPEC)
        output=os.environ.get('UAV_FIXTURE_MANIFEST','')
        if output:
            write_manifest(output,dict(FIXTURE_SPEC,diagnostic_initial_pitch_rad=DIAGNOSTIC_PITCH,
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                fixture_source_sha256=hashlib.sha256(Path(__file__).with_name('uav_marker_fixtures.py').read_bytes()).hexdigest()))


def main():
    MarkerQualification(env_url=SIMULATION_ENVIRONMENTS['Default Environment'],
        drone_configs=[dict(domain_id=1,x_m=0.,y_m=0.,z_m=.10 if DIAGNOSTIC_PITCH else .07,orient=[0.,math.sin(DIAGNOSTIC_PITCH/2),0.,math.cos(DIAGNOSTIC_PITCH/2)],prim='/World/base_link',node_name='PX4Multirotor')],
        enable_lidar=True).run()

if __name__=='__main__':main()

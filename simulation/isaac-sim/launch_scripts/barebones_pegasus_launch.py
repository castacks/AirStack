#!/usr/bin/env python
"""
Minimal PegasusApp launcher that:
 - Starts Isaac Sim (honoring ISAAC_SIM_HEADLESS / ISAAC_SIM_LIVESTREAM)
 - Enables required extensions
 - Creates a Pegasus world with a simple environment and no drones
 - Starts the timeline and steps until closed

Use as the smallest template for a new launch script: copy, add drone configs
(see example_one_px4_pegasus_launch_script.py) or hooks (see pegasus_app.py).
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pegasus_app import create_simulation_app

# Must be created before any omni/pegasus imports.
simulation_app = create_simulation_app()

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS  # noqa: E402
from pegasus_app import PegasusApp  # noqa: E402


class BarebonesApp(PegasusApp):

    def post_scene_prep(self, stage):
        import omni.graph.core as og

        print("=" * 60)
        print("Registered OmniGraph Nodes (Pegasus/Ascent):")
        print("=" * 60)
        for node_name in og.get_registered_nodes():
            if any(k in node_name for k in ("Pegasus", "pegasus", "Ascent", "ascent")):
                print(f" - {node_name}")

        # Reset so physics/articulations are ready
        self.world.reset()


def main():
    BarebonesApp(
        env_url=SIMULATION_ENVIRONMENTS["Curved Gridroom"],
        drone_configs=[],
        dome_light=False,
    ).run()


if __name__ == "__main__":
    main()

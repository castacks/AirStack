#!/usr/bin/env python
"""
Minimal PegasusApp launcher that:
 - Starts Isaac Sim
 - Enables required extensions
 - Creates a Pegasus world
 - Starts the timeline and steps until closed
"""

import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script starts here
# -----------------------------------
import omni.kit.app
import omni.timeline
from omni.isaac.core.world import World

# Pegasus imports
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from omni.isaac.core.utils.stage import add_reference_to_stage

# Explicitly enable required extensions
ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in [
    # "airlab.airstack",
    "omni.physx.forcefields",
    "omni.graph.core",  # Core runtime for OmniGraph engine
    "omni.graph.action",  # Action Graph framework
    "omni.graph.action_nodes",  # Built-in Action Graph node library
    "omni.graph.ui",  # UI scaffolding for graph tools
    "omni.graph.visualization.nodes",  # Visualization helper nodes
    "omni.graph.scriptnode",  # Python script node support
    "omni.graph.window.action",  # Action Graph editor window
    "omni.graph.window.generic",  # Generic graph UI tools
    "omni.graph.ui_nodes",  # UI node building helpers
    "airlab.pegasus",  # Airlab extension Pegasus core extension
    "pegasus.simulator",
]:
    if not ext_manager.is_extension_enabled(ext):
        ext_manager.set_extension_enabled(ext, True)


class PegasusApp:
    """
    Minimal PegasusApp: just loads a Pegasus world and steps simulation
    """

    def __init__(self):
        # Timeline for controlling play/stop
        self.timeline = omni.timeline.get_timeline_interface()

        # Start Pegasus interface + world
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # self.world.scene.add_default_ground_plane()

        add_reference_to_stage(
            usd_path="omniverse://airlab-storage.andrew.cmu.edu:8443/Users/ajong/pegasus_scene.usd",
            prim_path="/World/layout",
        )

        # Reset so physics/articulations are ready
        self.world.reset()

        self.stop_sim = False

    def run(self):
        # Start sim timeline
        self.timeline.play()

        # Main loop
        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)

        # Cleanup
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()

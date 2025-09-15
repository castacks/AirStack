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
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from scipy.spatial.transform import Rotation

# Explicitly enable required extensions
ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in [
    # "airlab.airstack",
    "omni.physx.forcefields",
    "omni.graph.core",                  # Core runtime for OmniGraph engine
    "omni.graph.action",                # Action Graph framework
    "omni.graph.action_nodes",          # Built-in Action Graph node library
    "omni.graph.ui",                    # UI scaffolding for graph tools
    "omni.graph.visualization.nodes",   # Visualization helper nodes
    "omni.graph.scriptnode",            # Python script node support
    "omni.graph.window.action",         # Action Graph editor window
    "omni.graph.window.generic",        # Generic graph UI tools
    "omni.graph.ui_nodes",              # UI node building helpers
    "airlab.pegasus",                   # Airlab extension Pegasus core extension
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

        # Load default environment
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        
        import omni.graph.core as og

        print("="*60)
        print("Registered OmniGraph Nodes:")
        print("="*60)

        # Iterate all registered node types
        for node_name in og.get_registered_nodes():
            if "Pegasus" in node_name or "pegasus" in node_name or "Ascent" in node_name or "ascent" in node_name:
                print(f" - {node_name}")

        # import omni.graph.core as og
        # print(og.get_node_type("action"))
        spawn_px4_multirotor_node(
            node_name="PX4Multirotor",
            drone_prim="/World/Quadrotor",
            vehicle_id=0,
            usd_file="/root/Documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd",
        )
        
        #########################################################################################
        # Create the vehicle using the old method (directly through Pegasus API)
        #########################################################################################
        # # Create the vehicle
        # # Try to spawn the selected robot in the world to the specified namespace
        # config_multirotor = MultirotorConfig()
        # # Create the multirotor configuration
        # mavlink_config = PX4MavlinkBackendConfig({
        #     "vehicle_id": 0,
        #     "px4_autolaunch": True,
        #     "px4_dir": self.pg.px4_path,
        #     "px4_vehicle_model": self.pg.px4_default_airframe # CHANGE this line to 'iris' if using PX4 version bellow v1.14
        # })
        # config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]

        # Multirotor(
        #     "/World/quadrotor",
        #     ROBOTS['Iris'],
        #     0,
        #     [0.0, 0.0, 0.07],
        #     Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        #     config=config_multirotor,
        # )
        #########################################################################################

        print("Spawned PX4 multirotor node")
        print("!" * 60)

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

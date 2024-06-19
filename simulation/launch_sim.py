# Imports to start Isaac Sim from this script
import sys
import numpy as np
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

from omni.isaac.core.prims import RigidPrim, GeometryPrim
from omni.isaac.core.utils import stage

# Auxiliary scipy and numpy modules
import os.path
from scipy.spatial.transform import Rotation

import math

import dronekit


class Drone:
    def __init__(self, vehicle_id, init_pos, world, usd_path):
        self._usd_path = usd_path
        self._init_pos = init_pos
        self._world = world
        self._drone_prim = None
        self.scale = 1.

        self._create(vehicle_id, usd_path, init_pos)
        self.set_world_position(init_pos)

    def _create(self, vehicle_id, usd_path, position = [0.,0.,0.], orientation = None):
        model_prim_path = f'/World/drone{vehicle_id}'
        stage.add_reference_to_stage(usd_path=usd_path, prim_path=model_prim_path)
        self._drone_prim = self._world.scene.add(GeometryPrim(
            prim_path=model_prim_path, 
            name=f'drone_{vehicle_id}', 
            position=position, 
            orientation=orientation, 
            scale=[self.scale, self.scale, self.scale],
            collision=True
        ))

    def get_world_pose(self):
        return self._drone_prim.get_world_pose()

    def set_world_pose(self, position, orientation):
        self._drone_prim.set_world_pose(position, orientation)        

    def set_world_position(self, position):
        _, orientation = self._drone_prim.get_world_pose()
        self._drone_prim.set_world_pose(position, orientation)
    
    def set_orientation_z_angle(self, angle):
        position, _ = self._drone_prim.get_world_pose()
        o = Rotation.from_euler("XYZ", [0.0, 0.0, angle]).as_quat()
        orienatation = [o[3], o[0], o[1], o[2]]
        self._drone_prim.set_world_pose(position, orienatation)

if __name__ == "__main__":

    vehicle = dronekit.connect('127.0.0.1:14553', wait_ready=True, timeout=999999, rate=120)

    # Locate Isaac Sim assets folder to load sample
    from omni.isaac.nucleus import is_file

    usd_path = "omniverse://nucleusserver.andrew.cmu.edu/Projects/DSTA/Scenes/Tokyo_Spirit_Drone/Tokyo_Spirit_Drone_Sim.usd"

    # make sure the file exists before we try to open it
    try:
        result = is_file(usd_path)
    except:
        result = False

    if result:
        omni.usd.get_context().open_stage(usd_path)
    else:
        carb.log_error(
            f"the usd path {usd_path} could not be opened, please make sure that {usd_path} is a valid usd file"
        )
        simulation_app.close()
        sys.exit()
    # Wait two frames so that stage starts loading
    simulation_app.update()
    simulation_app.update()

    print("Loading stage...")
    from omni.isaac.core.utils.stage import is_stage_loading

    while is_stage_loading():
        simulation_app.update()
    print("Loading Complete")

    world = World(stage_units_in_meters=1.0)

    drone_usd = "omniverse://nucleusserver.andrew.cmu.edu/Library/Assets/Ascent_Aerosystems/Spirit_UAV/spirit_uav_red_yellow.usd"
    drone = Drone("spirit_0", [0.,0.,0.7], world, usd_path=drone_usd)

    last_time = omni.timeline.get_timeline_interface().get_current_time()

    position = 0

    def update_state(args):

        global position

        current_time = omni.timeline.get_timeline_interface().get_current_time()
        r = vehicle._roll + np.pi/2
        p = vehicle._pitch
        y = vehicle._yaw

        rot = Rotation.from_euler('xyz', [r, p, y], degrees=False)
        # quaternion: xyzw
        q = rot.as_quat()

        o = [
            q[3], q[0], q[1], q[2]
        ]
        p = [
            vehicle.location.local_frame.east,
            vehicle.location.local_frame.north,
            -vehicle.location.local_frame.down
        ]

        # quaternion: wxyz
        drone.set_world_pose(p, o)


    world.add_physics_callback("update_drone_state", update_state)


    omni.timeline.get_timeline_interface().play()
    # Run in test mode, exit after a fixed number of steps
    while simulation_app.is_running():

        # Update the UI of the app and perform the physics step
        # world.step(render=True)
        simulation_app.update()

    omni.timeline.get_timeline_interface().stop()
    simulation_app.close()


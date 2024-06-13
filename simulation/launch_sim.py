# Imports to start Isaac Sim from this script
import sys
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

from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.utils import stage

# Auxiliary scipy and numpy modules
import os.path
from scipy.spatial.transform import Rotation

import math

import dronekit


class Drone:
    def __init__(self, vehicle_id, init_pos, world, usd_path, waypoints = None, speed = 10.):
        self._usd_path = usd_path
        # self._init_pos = init_pos
        self._world = world
        self._rigid_body = None
        self.speed = speed
        self.z_level = 1.06
        self.scale = 1.


        self._create(vehicle_id, usd_path, init_pos)
        self.set_world_position(init_pos)

    def _create(self, vehicle_id, usd_path, position = [0.,0.,0.], orientation = None):
        model_prim_path = f'/World/drone{vehicle_id}'
        stage.add_reference_to_stage(usd_path=usd_path, prim_path=model_prim_path)
        self._rigid_body = self._world.scene.add(RigidPrim(
            prim_path=model_prim_path, 
            name=f'drone{vehicle_id}', 
            position=position, 
            orientation=orientation, 
            scale=[self.scale, self.scale, self.scale],
            mass=None,
            linear_velocity = [0.,0.,0.],
        ))

    def get_world_pose(self):
        return self._rigid_body.get_world_pose()

    def set_world_pose(self, position, orientation):
        self._rigid_body.set_world_pose(position, orientation)        

    def set_world_position(self, position):
        _, orientation = self._rigid_body.get_world_pose()
        self._rigid_body.set_world_pose(position, orientation)

    def zero_velocity(self):
        self._rigid_body.set_linear_velocity([0.,0.,0.])
    
    def set_orientation_z_angle(self, angle):
        position, _ = self._rigid_body.get_world_pose()
        o = Rotation.from_euler("XYZ", [0.0, 0.0, angle]).as_quat()
        orienatation = [o[3], o[0], o[1], o[2]]
        self._rigid_body.set_world_pose(position, orienatation)

    def set_orientation_next_waypoint(self):
        position, orientation = self._rigid_body.get_world_pose()
        dx = self.waypoints[self.current_waypoint][0] - position[0]
        dy = self.waypoints[self.current_waypoint][1] - position[1]
        angle = math.atan2(dy, dx) + math.pi/2
        self.set_orientation_z_angle(angle)
    
    def set_next_waypoint(self):
        self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)
        self.set_orientation_next_waypoint()
    
    def move(self, dt = 0.025):
        self.zero_velocity()
        position, orientation = self._rigid_body.get_world_pose()
        dx = self.waypoints[self.current_waypoint][0] - position[0]
        dy = self.waypoints[self.current_waypoint][1] - position[1]
        dz = 0. #self.waypoints[self.current_waypoint][2] - position[2]
        d = (dx**2 + dy**2 + dz**2)**0.5
        if d < 0.26:
            self.set_next_waypoint()
        else:
            dx = dx/d
            dy = dy/d
            dz = dz/d
            position[0] += self.speed * dx * dt
            position[1] += self.speed * dy * dt
            position[2] =  self.z_level
            self.set_world_position(position)
        return position, orientation


if __name__ == "__main__":

    vehicle = dronekit.connect('127.0.0.1:14553', wait_ready=True, timeout=999999)

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
    drone = Drone(0, [0.,0.,0.7], world, usd_path=drone_usd)


    omni.timeline.get_timeline_interface().play()
    # Run in test mode, exit after a fixed number of steps
    while simulation_app.is_running():
        # Run in realtime mode, we don't specify the step size
        r = vehicle._roll
        p = vehicle._pitch
        y = vehicle._yaw

        # xyzw
        rot = Rotation.from_euler('xyz', [r, p, y], degrees=False)

        q = rot.as_quat()

        o = [
            q[3], q[0], q[1], q[2]
        ]
        p = [
            vehicle.location.local_frame.east,
            vehicle.location.local_frame.north,
            -vehicle.location.local_frame.down
        ]

        drone.zero_velocity()
        #wxyz
        drone.set_world_pose(p, o)

        # Update the UI of the app and perform the physics step
        # world.step(render=True)
        simulation_app.update()

    omni.timeline.get_timeline_interface().stop()
    simulation_app.close()


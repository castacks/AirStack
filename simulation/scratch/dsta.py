#!/usr/bin/env python
"""
| File: 1_px4_single_vehicle.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to build an app that makes use of the Pegasus API to run a simulation with a single vehicle, controlled using the MAVLink control backend.
"""

# Imports to start Isaac Sim from this script
import carb
from omni.isaac.kit import SimulationApp

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

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend, MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
# Auxiliary scipy and numpy modules
import os.path
from scipy.spatial.transform import Rotation

import math

import dronekit

vehicle = dronekit.connect('127.0.0.1:14552', timeout=999999)

class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        # self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        # self.pg.load_environment("omniverse://nucleusserver.andrew.cmu.edu/Projects/MultiDrone/Environments/ModUrbanCity/Content/Demo.usd")
        # self.pg.load_environment("omniverse://nucleusserver.andrew.cmu.edu/Library/Stages/AbandonedFactory/AbandonedFactory.usd")
        # self.pg.load_environment("omniverse://localhost/Library/Stages/AbandonedFactory/AbandonedFactory.usd")
        self.pg.load_environment("omniverse://nucleusserver.andrew.cmu.edu/Projects/DSTA/Scenes/Tokyo_Spirit_Drone/Tokyo_Spirit_Drone_Sim.usd")

        # Create the vehicle
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor = MultirotorConfig()
        # Create the multirotor configuration
        # mavlink_config = MavlinkBackendConfig({
        #     "vehicle_id": 0,
        #     "px4_autolaunch": False,
        #     "px4_dir": self.pg.px4_path,
        #     "px4_vehicle_model": self.pg.px4_default_airframe # CHANGE this line to 'iris' if using PX4 version bellow v1.14
        # })
        # config_multirotor.backends = [MavlinkBackend(mavlink_config)]

        # Multirotor(
        #     "/World/quadrotor",
        #     ROBOTS['Iris'],
        #     0,
        #     [0.0, 0.0, 0.07],
        #     Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        #     config=config_multirotor,
        # )
        drone_usd = "omniverse://nucleusserver.andrew.cmu.edu/Library/Assets/Ascent_Aerosystems/Spirit_UAV/spirit_uav_red_yellow.usd"

        self.drone = Drone(0, [0.,0.,0.], self.world, usd_path=drone_usd)

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:

            # vehicle.simple_goto(dronekit.LocationGlobal(0,0,100))

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

            self.drone.zero_velocity()
            #wxyz
            self.drone.set_world_pose(p, o)

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


class Drone:
    def __init__(self, vehicle_id, init_pos, world, waypoints = None, speed = 10., usd_path = ROBOTS['Iris']):
        self._usd_path = usd_path
        # self._init_pos = init_pos
        self._world = world
        self._rigid_body = None
        self.speed = speed
        self.z_level = 1.06
        self.scale = 1.


        self._create(vehicle_id, usd_path, init_pos)
        self.set_world_position(init_pos)

    def _create(self, vehicle_id, usd_path, position = [0.,0.,0.], orientation = [0.,0.,0.,1.]):
        model_prim_path = f'/World/drone{vehicle_id}'
        stage.add_reference_to_stage(usd_path=usd_path, prim_path=model_prim_path)
        self._rigid_body = self._world.scene.add(RigidPrim(
            prim_path=model_prim_path, 
            name=f'drone{vehicle_id}', 
            position=position, 
            orientation=orientation, 
            scale=[self.scale, self.scale, self.scale],
            mass=0.0,
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











def main():

    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()

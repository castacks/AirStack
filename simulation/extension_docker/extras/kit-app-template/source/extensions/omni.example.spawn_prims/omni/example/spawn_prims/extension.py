# Copyright (c) 2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited. 

import omni.ext
import omni.ui as ui
import omni.kit.commands

from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path

import sys
import dronekit
import os
import numpy as np
from scipy.spatial.transform import Rotation
import carb
import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.timeline
import usdrt.Sdf
from omni.isaac.core.prims import GeometryPrim, RigidPrim
from omni.isaac.core.utils import extensions, stage
from omni.isaac.core.world import World
from pxr import Gf, Usd, UsdGeom
from .AscentAeroSystems.ascent_sitl_launch_tool import AscentSitlLaunchTool
import time
import threading
import struct
import socket

import asyncio


vehicle_num = 0
# enable ROS bridge extension
#extensions.enable_extension("omni.isaac.ros2_bridge")

#simulation_app.update()

current_sim_time = 0.
def get_sim_time():
    return current_sim_time

def handle_client(conn, addr):
    print(f'Connected by {addr}')

    initial_sitl_time = -1.
    initial_sim_time = -1.
    
    with conn:
        while True:
            data = conn.recv(8)
            #print('data', data)
            if not data:
                break

            t = struct.unpack('Q', data)[0]
            s = get_sim_time()

            if initial_sitl_time < 0:
                print('initial')
                initial_sitl_time = t
                initial_sim_time = s

            sitl_time = t - initial_sitl_time
            sim_time = (s - initial_sim_time)*1000000
            time_to_sleep = int(sitl_time - sim_time)
            #print('time to sleep', time_to_sleep, sitl_time, sim_time)
            
            #print(f'Received {data.decode()} from {addr}')
            #time.sleep(0.01)
            #conn.sendall(b'Hello from server')
            conn.sendall(struct.pack('i', time_to_sleep))

def start_server(host='127.0.0.1', port=65432):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f'Server listening on {host}:{port}')
        while True:
            conn, addr = s.accept()
            client_thread = threading.Thread(target=handle_client, args=(conn, addr))
            client_thread.start()
            print(f'Active connections: {threading.active_count() - 1}')



class Drone:
    def __init__(self, vehicle_id, vehicle_num, init_pos, world, usd_path):
        print(world)
        self.vehicle_id = vehicle_id
        self.vehicle_num = vehicle_num
        self._usd_path = usd_path
        self._init_pos = init_pos
        self._world = world
        self._drone_prim = None
        self.scale = 1.0
        
        script_dir = os.path.dirname(os.path.realpath(__file__)) + "/AscentAeroSystems"
        self.sitl_tool = AscentSitlLaunchTool(script_dir, vehicle_num)
        
        self.sitl_tool.launch()
        self.thread = None
        
        if world._physics_context == None:
            print('starting thread')
            self.thread = threading.Thread(target=start_server)
            self.thread.start()
            print('done starting thread')
        async def init_physics():
            if world._physics_context == None:
                await world.initialize_simulation_context_async()
            world.add_physics_callback("update_drone_state_" + str(self.vehicle_num), self.update_state_from_mavlink)
            world.play()
        print('1')
        asyncio.ensure_future(init_physics())
        print('2')
        
        print('3')
        self._create_prim(vehicle_id, usd_path, init_pos)
        print('4')
        self.set_world_position(init_pos)
        print('5')
        self.last_time = time.time()
        print('6')
        def f():
            self._dronekit_connection = dronekit.connect(self.sitl_tool.get_dronekit_address(), wait_ready=True, timeout=999999, rate=120)
        threading.Thread(target=f).start()
        print('7')
        
        #world.add_physics_callback("update_drone_state", self.update_state_from_mavlink)

    def _create_prim(self, vehicle_id, usd_path, position=[0.0, 0.0, 0.0], orientation=None):
        self.stage_path = f"/World/drone{vehicle_id}"
        stage.add_reference_to_stage(usd_path=usd_path, prim_path=self.stage_path)
        
        self._drone_prim = self._world.scene.add(
            GeometryPrim(
                prim_path=self.stage_path,
                name=f"drone_{vehicle_id}",
                position=position,
                orientation=orientation,
                scale=[self.scale, self.scale, self.scale],
                collision=True,
            )
        )
    
    def update_state_from_mavlink(self, args):
        print(self._drone_prim.GetPropertyNames())
        global current_sim_time
        current_sim_time += args
        
        current_time = time.time()
        elapsed = self.last_time - current_time
        self.last_time = current_time
        print(self.vehicle_id, args, elapsed)
        args  # is required function definition for the physics callback
        
        r = self._dronekit_connection._roll + np.pi / 2
        p = self._dronekit_connection._pitch
        y = self._dronekit_connection._yaw

        rot = Rotation.from_euler("xyz", [r, p, y], degrees=False)
        # quaternion: xyzw
        q = rot.as_quat()

        o = [q[3], q[0], q[1], q[2]]

        n, e, d = (
            self._dronekit_connection.location.local_frame.east,
            self._dronekit_connection.location.local_frame.north,
            self._dronekit_connection.location.local_frame.down,
        )
        if d is not None:
            # quaternion: wxyz
            p = (e, n, -d)  # enu
            self.set_world_pose(p, o)
        else:
            print("Drone location from dronekit is None")

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
        
# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class MyExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        """
         Called when MyExtension starts. 
         
         Args:
            ext_id : id of the extension that is
        """
        print("[omni.example.spawn_prims] MyExtension startup")

        self._window = ui.Window("Spawn Primitives", width=300, height=300)
        self._window.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
        with self._window.frame:
            # VStack which will layout UI elements vertically
            with ui.VStack():
                def on_click(prim_type):
                    """
                     Creates a mesh primitive of the given type. 
                     
                     Args:
                       prim_type : The type of primitive to
                    """
                    # omni.kit.commands.execute will execute the given command that is passed followed by the commands arguments
                    omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
                        prim_type=prim_type,
                        above_ground=True)

                def spawn_ascent():
                    global vehicle_num
                    print('ascent')
                    prim_path = '/World/drone_0'
                    usd_path = 'omniverse://nucleusserver.andrew.cmu.edu/Library/Assets/Ascent_Aerosystems/Spirit_UAV/spirit_uav_red_yellow.prop.usd'
                    world = World()
                    print(world)
                    drone = Drone("spirit_" + str(vehicle_num), vehicle_num, [0.0, 0.0, 0.7], world, usd_path=usd_path)
                    vehicle_num += 1
                    '''
                    prim = get_prim_at_path(prim_path)
                    if not prim.IsValid():
                        prim = define_prim(prim_path, 'Xform')
                        prim.GetReferences().AddReference(usd_path)
                    '''

                    
                    
                    
                # Button UI Elements
                ui.Button("Spawn Ascent", clicked_fn=lambda: spawn_ascent())

    def on_shutdown(self):
        """
         Called when the extension is shutting down. 
        """
        print("[omni.example.spawn_prims] MyExtension shutdown")

from omni.kit.scripting import BehaviorScript
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
from pxr.Gf import Quatf
from .ascent_sitl_launch_tool import AscentSitlLaunchTool
import time
import threading
import struct
import socket

import asyncio

#==========================================================================================
# ------------------------------------- Time Sync Server ----------------------------------
#==========================================================================================

mutex = threading.Lock()
client_count = 1
current_sim_time = 0.
def get_sim_time():
    return current_sim_time

def handle_client(conn, addr):
    global client_count
    print(f'Connected by {addr}')

    initial_sitl_time = -1.
    initial_sim_time = -1.
    
    with conn:
        while True:
            data = conn.recv(16)
            if not data:
                break

            #print('data', len(data), data)
            message_type, t = struct.unpack('cQ', data)
            #print('message type', message_type, t)
            if message_type == b't':
                s = get_sim_time()

                if initial_sitl_time < 0:
                    initial_sitl_time = t
                    initial_sim_time = s

                sitl_time = t - initial_sitl_time
                sim_time = (s - initial_sim_time)*1000000
                time_to_sleep = int(sitl_time - sim_time)

                conn.sendall(struct.pack('i', time_to_sleep))
            elif message_type == b'n':
                print('message type', message_type, t)
                with mutex:
                    client_count += 1
                conn.sendall(struct.pack('i', client_count))

def start_server(host='127.0.0.1', port=65432):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((host, port))
        s.listen()
        print(f'Server listening on {host}:{port}')
        def f():
            while True:
                conn, addr = s.accept()
                client_thread = threading.Thread(target=handle_client, args=(conn, addr))
                client_thread.start()
        threading.Thread(target=f).start()
        return 1
    except:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect((host, port))
        message = struct.pack('cQ', b'n', 0)
        client.sendall(message)
        drone_count = struct.unpack('i', client.recv(4))[0]
        return drone_count
        

#==========================================================================================
# ------------------------------------- Behavior Script -----------------------------------
#==========================================================================================
            
class TestScript(BehaviorScript):
    def on_init(self):
        print('on init')
        #print('PRIM PATH', self.prim_path)
        self.p = self.stage.GetPrimAtPath(self.prim_path)
        self.prop_prim = self.stage.GetPrimAtPath(str(self.prim_path) + '/spirit_uav/meshes/mesh_17')
        #print('YOOOOOOOOOOOO', str(self.prim_path) + '/spirit_uav/meshes/mesh_17', self.prop_prim)
        #print(self.p.GetAttributes())
        self.p.GetAttribute('xformOp:translate').Set((0,0, 0))
        self.q_type = type(self.p.GetAttribute("xformOp:orient").Get())
        self.p.GetAttribute('xformOp:orient').Set(self.p.GetAttribute("xformOp:orient").Get())
        #print(self.p.GetAttribute("xformOp:translate").Get())
        self.dronekit_connection = None
        self.initialized = False
        self.drone_count = 0
        #'''
        self.prop_rotation_q = Quatf(0.9999619, 0, 0.0087265, 0) # 1 degree
        for i in range(7):
            self.prop_rotation_q *= self.prop_rotation_q
        #print(self.prop_prim.GetAttribute('xformOp:orient'))
        self.prop_prim.GetAttribute('xformOp:orient').Set(self.prop_prim.GetAttribute("xformOp:orient").Get()*
                                                          self.prop_rotation_q)
        #'''
        print('on init done')

    def on_update(self, current_time, delta_time):
        #print('update', current_time, delta_time)
        global current_sim_time
        current_sim_time = current_time
        
        if self.dronekit_connection == None:
            if not self.initialized:
                self.initialized = True
                script_dir = os.path.dirname(os.path.realpath(__file__)) + '/'
                self.drone_count = start_server()
                self.sitl_tool = AscentSitlLaunchTool(script_dir, self.drone_count)
                self.sitl_tool.launch()
                #self.thread = threading.Thread(target=start_server)
                #self.thread.start()
                def f():
                    self.dronekit_connection = dronekit.connect(self.sitl_tool.get_dronekit_address(),
                                                                wait_ready=True, timeout=999999, rate=120)
                threading.Thread(target=f).start()
        else:
            r = self.dronekit_connection._roll# + np.pi / 2
            p = self.dronekit_connection._pitch
            y = self.dronekit_connection._yaw

            rot = Rotation.from_euler("xyz", [r, p, y], degrees=False)
            # quaternion: xyzw
            q = rot.as_quat()

            o = self.q_type(q[3], q[0], q[1], q[2])

            n, e, d = (
                self.dronekit_connection.location.local_frame.east,
                self.dronekit_connection.location.local_frame.north,
                self.dronekit_connection.location.local_frame.down,
            )
            if d is not None:
                # quaternion: wxyz
                p = (e, n, -d)  # enu
                #self.p.set_world_pose(p, o)
                self.p.GetAttribute('xformOp:translate').Set(p)
                self.p.GetAttribute('xformOp:orient').Set(o)
                #'''
                self.prop_prim.GetAttribute('xformOp:orient').Set(self.prop_prim.GetAttribute("xformOp:orient").Get()*
                                                                  self.prop_rotation_q)
                #'''
            else:
                #print("Drone location from dronekit is None")
                pass
    
    def on_destroy(self):
        print('destroy')

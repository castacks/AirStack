#!/usr/bin/env python3

import sys
import time 
import dronekit  
import os 
import math
import gz.math7

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.header_pb2 import Header
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.quaternion_pb2 import Quaternion
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.transport13 import Node

from AscentAeroSystems.ascent_sitl_launch_tool import AscentSitlLaunchTool

script_dir = os.path.dirname(os.path.realpath(__file__)) + "/AscentAeroSystems"
sitl_tool = AscentSitlLaunchTool(script_dir)
sitl_tool.launch()

world_name = "dron"
model_name = "drone"
#world_name = "empty"
#model_name = "box"
connection = "udp:127.0.0.1:14552"
timeout_ms = 9999
update_rate_hz = 30.0
pose_offset_x = 0.0
pose_offset_y = 0.0
pose_offset_z = 0.17
def transform_ardupilot_to_gazebo(pose):
  wldAToWldG = gz.math7.Pose3d(0, 0, 0, -math.pi, 0, 0)
  bdyAToBdyG = gz.math7.Pose3d(0, 0, 0, -math.pi, 0, 0)
  wldGToBdyG = wldAToWldG.inverse() * pose * bdyAToBdyG
  return wldGToBdyG
def main():
    connection_string = "udp:127.0.0.1:14552"
    print("Connecting to vehicle on: {}".format(connection_string))
    vehicle = dronekit.connect(connection_string, wait_ready=True, baud=57600)
    print("Vehicle connected: {}".format(vehicle))
    try:
        # create a transport node
        node = Node()
        # set service details
        service = "/world/{}/set_pose".format(world_name)
        reqtype = "gz.msgs10.Pose" 
        reptype = "gz.msgs10.Boolean" 
        timeout = 30 
	# configure update loop
        now_s = time.time()
        start_time_s = now_s
        # update_rate_hz = 1.0
        update_period_s = 1.0 / update_rate_hz
        last_update_time_s = now_s
        sim_time_s = now_s - start_time_s
        while True:
            now_s = time.time()
            time_s = now_s - start_time_s
            if now_s - last_update_time_s >= update_period_s:
                last_update_time_s = now_s
                # check we have valid data
                if ((vehicle.location.local_frame.north is not None)
                  and (vehicle.attitude.roll is not None)):
                    # ardupilot pose
                    wldAToBdyA = gz.math7.Pose3d(
                        vehicle.location.local_frame.north,
                        vehicle.location.local_frame.east,
                        vehicle.location.local_frame.down,
                        vehicle.attitude.roll,
                        vehicle.attitude.pitch,
                        vehicle.attitude.yaw)
                    # ignition pose
                    wldGToBdyG = transform_ardupilot_to_gazebo(wldAToBdyA)
                    print("[{}] ned_xyz: {:.3f} {:.3f} {:.3f}, ned_rpy {:.3f} {:.3f} {:.3f}".format(
                        sim_time_s, wldAToBdyA.x(), wldAToBdyA.y(), wldAToBdyA.z(),
                        wldAToBdyA.roll(), wldAToBdyA.pitch(), wldAToBdyA.yaw()))
                    # create request message
                    vector3d_msg = Vector3d()
                    vector3d_msg.x = wldGToBdyG.x() + pose_offset_x
                    vector3d_msg.y = wldGToBdyG.y() + pose_offset_y
                    vector3d_msg.z = wldGToBdyG.z() + pose_offset_z
                    quat_msg = Quaternion()
                    quat_msg.x = wldGToBdyG.rot().x()
                    quat_msg.y = wldGToBdyG.rot().y()
                    quat_msg.z = wldGToBdyG.rot().z()
                    quat_msg.w = wldGToBdyG.rot().w()
                    pose_msg = Pose()
                    pose_msg.name = model_name
                    pose_msg.position.CopyFrom(vector3d_msg)
                    pose_msg.orientation.CopyFrom(quat_msg)
                    # submit request (blocking)
                    result = node.request(service, pose_msg, Pose, Boolean, timeout_ms)
                    if not result:
                        print("[{:.1f}] update failed".format(time_s))
    except KeyboardInterrupt:
        pass
    vehicle.close()
if __name__ == "__main__":
    main()

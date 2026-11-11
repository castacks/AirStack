#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class OdomMeshMarker(Node):
    def __init__(self):
        super().__init__('odom_mesh_marker')

        # ---- params ----
        # self.declare_parameter('odom_topic', '/ov/odom')
        self.declare_parameter('odom_topic', '/mocap_odom_processed')
        self.declare_parameter('marker_topic', '/drone_mesh')
        self.declare_parameter('frame_id', '/map')  # 留空表示用 odom.header.frame_id
        self.declare_parameter('mesh_resource', 'file:///home/robot/AirStack/robot/ros_ws/model_ai_mesh_rviz.stl')
        self.declare_parameter('mesh_use_embedded_materials', False)  # STL 没材质；DAE 可设 True
        self.declare_parameter('scale', [0.01, 0.01, 0.01])
        self.declare_parameter('color', [0.8, 0.8, 0.8, 1.0])  # RGBA; STL 需要颜色
        self.declare_parameter('ns', 'drone')
        self.declare_parameter('id', 0)

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.frame_id_override = self.get_parameter('frame_id').get_parameter_value().string_value
        self.mesh_resource = self.get_parameter('mesh_resource').get_parameter_value().string_value
        self.mesh_use_embedded_materials = self.get_parameter('mesh_use_embedded_materials').get_parameter_value().bool_value

        scale = self.get_parameter('scale').get_parameter_value().double_array_value
        color = self.get_parameter('color').get_parameter_value().double_array_value
        self.scale = [float(scale[0]), float(scale[1]), float(scale[2])]
        self.color = [float(color[0]), float(color[1]), float(color[2]), float(color[3])]

        self.ns = self.get_parameter('ns').get_parameter_value().string_value
        self.mid = int(self.get_parameter('id').get_parameter_value().integer_value)

        self.pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.sub = self.create_subscription(Odometry,
                                            self.odom_topic,
                                            self.odom_cb,
                                            qos_profile_sensor_data)

        self.get_logger().info(f"Subscribing: {self.odom_topic}")
        self.get_logger().info(f"Publishing : {self.marker_topic}")
        self.get_logger().info(f"Mesh       : {self.mesh_resource}")

    def odom_cb(self, msg: Odometry):
        m = Marker()
        m.header.stamp = msg.header.stamp
        m.header.frame_id = self.frame_id_override if self.frame_id_override else msg.header.frame_id

        m.ns = self.ns
        m.id = self.mid
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD

        # 让 marker 跟着 odom.pose 走
        m.pose = msg.pose.pose

        m.mesh_resource = self.mesh_resource
        m.mesh_use_embedded_materials = self.mesh_use_embedded_materials

        m.scale.x, m.scale.y, m.scale.z = self.scale
        m.color.r, m.color.g, m.color.b, m.color.a = self.color

        # 可选：避免 RViz 每帧残影（一般不需要，但保险）
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0

        self.pub.publish(m)

def main():
    rclpy.init()
    node = OdomMeshMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

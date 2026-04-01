"""Minimal AirSim-to-ROS 2 bridge: publishes depth image and camera_info.

Control is handled by PX4 SITL (connected to AirSim via TCP lockstep)
and MAVROS (in the robot container). This node only bridges sensor data.
"""

import math
import threading
import queue

import airsim
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class AirSimDepthBridge(Node):

    def __init__(self):
        super().__init__('airsim_depth_bridge')

        self.declare_parameter('airsim_ip', '127.0.0.1')
        self.declare_parameter('camera_name', 'front_camera')
        self.declare_parameter('vehicle_name', 'drone1')
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('robot_name', 'robot_1')

        ip = self.get_parameter('airsim_ip').value
        self.camera_name = self.get_parameter('camera_name').value
        self.vehicle_name = self.get_parameter('vehicle_name').value
        rate = self.get_parameter('publish_rate').value
        robot_name = self.get_parameter('robot_name').value

        # Connect to AirSim
        self.client = airsim.MultirotorClient(ip=ip)
        self.client.confirmConnection()
        self.get_logger().info(f'Connected to AirSim at {ip}')

        # Get camera intrinsics from FOV
        cam_info = self.client.simGetCameraInfo(
            self.camera_name, vehicle_name=self.vehicle_name
        )
        self.width = 480
        self.height = 300
        fov_rad = math.radians(cam_info.fov)
        self.fx = self.width / (2.0 * math.tan(fov_rad / 2.0))
        self.fy = self.fx
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        # Publishers — generic sensor topics, remapped in launch files
        prefix = f'/{robot_name}/sensors/front_camera'
        self.depth_pub = self.create_publisher(Image, f'{prefix}/depth', 1)
        self.info_pub = self.create_publisher(CameraInfo, f'{prefix}/camera_info', 1)

        # Static TF: base_link -> camera optical frame
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'front_camera_optical_frame'
        t.transform.translation.x = 0.2
        t.transform.translation.z = -0.1
        # Rotate from body (FLU) to optical (RDF): 90deg pitch down, 90deg yaw
        t.transform.rotation.x = 0.5
        t.transform.rotation.y = -0.5
        t.transform.rotation.z = 0.5
        t.transform.rotation.w = 0.5
        self.tf_broadcaster.sendTransform(t)

        # Background image fetcher (workaround for slow simGetImages)
        self._queue = queue.Queue(maxsize=2)
        self._shutdown = threading.Event()
        self._thread = threading.Thread(target=self._fetcher, daemon=True)
        self._thread.start()

        # Publish timer
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'Bridge running: depth @ {rate}Hz, '
            f'fx={self.fx:.1f}, image={self.width}x{self.height}'
        )

    def _fetcher(self):
        """Background thread: polls AirSim for depth images."""
        req = airsim.ImageRequest(
            self.camera_name, airsim.ImageType.DepthPlanar,
            pixels_as_float=True, compress=False
        )
        while not self._shutdown.is_set():
            try:
                responses = self.client.simGetImages(
                    [req], vehicle_name=self.vehicle_name
                )
                if responses and responses[0].height > 0:
                    if not self._queue.full():
                        self._queue.put(responses[0])
            except Exception as e:
                self.get_logger().warn(f'Image fetch failed: {e}', throttle_duration_sec=5.0)

    def _publish(self):
        """Timer callback: publish latest depth + camera_info."""
        try:
            resp = self._queue.get_nowait()
        except queue.Empty:
            return

        now = self.get_clock().now().to_msg()

        # Depth image
        depth = np.array(resp.image_data_float, dtype=np.float32)
        depth = depth.reshape(resp.height, resp.width)

        msg = Image()
        msg.header.stamp = now
        msg.header.frame_id = 'front_camera_optical_frame'
        msg.height = resp.height
        msg.width = resp.width
        msg.encoding = '32FC1'
        msg.is_bigendian = False
        msg.step = resp.width * 4
        msg.data = depth.tobytes()
        self.depth_pub.publish(msg)

        # Camera info
        info = CameraInfo()
        info.header = msg.header
        info.height = resp.height
        info.width = resp.width
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0,
        ]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self.info_pub.publish(info)

    def destroy_node(self):
        self._shutdown.set()
        self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AirSimDepthBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

"""AirSim-to-ROS 2 bridge: publishes stereo RGB, depth, and camera_info.

Matches the topic contract used by simple-sim and Isaac Sim so the
robot autonomy stack works without modification.

Control is handled by PX4 SITL + MAVROS. This node only bridges sensor data.
"""

import math
import threading
import queue

import airsim
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rosgraph_msgs.msg import Clock


class AirSimRosBridge(Node):

    def __init__(self):
        super().__init__('airsim_ros_bridge')

        self.declare_parameter('airsim_ip', '127.0.0.1')
        self.declare_parameter('vehicle_name', 'drone1')
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('baseline', 0.12)

        ip = self.get_parameter('airsim_ip').value
        self.vehicle_name = self.get_parameter('vehicle_name').value
        rate = self.get_parameter('publish_rate').value
        robot_name = self.get_parameter('robot_name').value
        self.baseline = self.get_parameter('baseline').value

        # Connect to AirSim (retry until UE4 binary is ready)
        import time
        self.client = airsim.MultirotorClient(ip=ip)
        while True:
            try:
                self.client.confirmConnection()
                break
            except Exception:
                self.get_logger().info(
                    f'Waiting for AirSim at {ip}...', throttle_duration_sec=5.0
                )
                time.sleep(2)
                self.client = airsim.MultirotorClient(ip=ip)
        self.get_logger().info(f'Connected to AirSim at {ip}')

        # Get camera intrinsics from FOV (both cameras share same settings)
        cam_info = self.client.simGetCameraInfo(
            'front_left', vehicle_name=self.vehicle_name
        )
        self.width = 480
        self.height = 300
        fov_rad = math.radians(cam_info.fov)
        self.fx = self.width / (2.0 * math.tan(fov_rad / 2.0))
        self.fy = self.fx
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        # Publishers — match simple-sim topic contract
        prefix = f'/{robot_name}/sensors/front_stereo'
        self.left_image_pub = self.create_publisher(Image, f'{prefix}/left/image_rect', 1)
        self.right_image_pub = self.create_publisher(Image, f'{prefix}/right/image_rect', 1)
        self.left_info_pub = self.create_publisher(CameraInfo, f'{prefix}/left/camera_info', 1)
        self.right_info_pub = self.create_publisher(CameraInfo, f'{prefix}/right/camera_info', 1)
        self.depth_pub = self.create_publisher(Image, f'{prefix}/left/depth_ground_truth', 1)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)

        # Background image fetcher (workaround for slow simGetImages)
        self._queue = queue.Queue(maxsize=2)
        self._shutdown = threading.Event()
        self._thread = threading.Thread(target=self._fetcher, daemon=True)
        self._thread.start()

        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'Bridge running: stereo RGB + depth @ {rate}Hz, '
            f'fx={self.fx:.1f}, baseline={self.baseline}m, '
            f'image={self.width}x{self.height}'
        )

    def _fetcher(self):
        """Background thread: polls AirSim for stereo RGB + depth."""
        requests = [
            airsim.ImageRequest('front_left', airsim.ImageType.Scene, False, True),
            airsim.ImageRequest('front_left', airsim.ImageType.DepthPlanar, True, False),
            airsim.ImageRequest('front_right', airsim.ImageType.Scene, False, True),
        ]
        while not self._shutdown.is_set():
            try:
                responses = self.client.simGetImages(
                    requests, vehicle_name=self.vehicle_name
                )
                if len(responses) == 3 and responses[0].height > 0:
                    if not self._queue.full():
                        self._queue.put(responses)
            except Exception as e:
                self.get_logger().warn(
                    f'Image fetch failed: {e}', throttle_duration_sec=5.0
                )

    def _publish(self):
        """Timer callback: publish clock, stereo images, depth, and camera_info."""
        try:
            responses = self._queue.get_nowait()
        except queue.Empty:
            return

        # Publish /clock from the image response timestamp (avoids extra API call)
        sim_ts = responses[0].time_stamp
        clock_msg = Clock()
        clock_msg.clock.sec = int(sim_ts // 1_000_000_000)
        clock_msg.clock.nanosec = int(sim_ts % 1_000_000_000)
        self.clock_pub.publish(clock_msg)

        now = clock_msg.clock
        left_resp, depth_resp, right_resp = responses

        # Left RGB
        left_rgb = self._decode_compressed(left_resp)
        self.left_image_pub.publish(
            self._make_image_msg(left_rgb, now, 'camera_left', 'rgb8')
        )

        # Right RGB
        right_rgb = self._decode_compressed(right_resp)
        self.right_image_pub.publish(
            self._make_image_msg(right_rgb, now, 'camera_right', 'rgb8')
        )

        # Depth (float32)
        depth = np.array(depth_resp.image_data_float, dtype=np.float32)
        depth = depth.reshape(depth_resp.height, depth_resp.width)
        depth_msg = Image()
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = 'camera_left'
        depth_msg.height = depth_resp.height
        depth_msg.width = depth_resp.width
        depth_msg.encoding = '32FC1'
        depth_msg.is_bigendian = False
        depth_msg.step = depth_resp.width * 4
        depth_msg.data = depth.tobytes()
        self.depth_pub.publish(depth_msg)

        # Camera info
        self.left_info_pub.publish(self._make_cam_info(now, 'left', 0.0))
        self.right_info_pub.publish(
            self._make_cam_info(now, 'right', -self.fx * self.baseline)
        )

    def _decode_compressed(self, response):
        """Decode a compressed PNG response to RGB numpy array."""
        import cv2
        img = cv2.imdecode(
            np.frombuffer(response.image_data_uint8, dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    def _make_image_msg(self, img_array, stamp, frame_id, encoding):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = img_array.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = msg.width * img_array.shape[2]
        msg.data = img_array.tobytes()
        return msg

    def _make_cam_info(self, stamp, side, tx):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = f'camera_{side}'
        info.height = self.height
        info.width = self.width
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0,
        ]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            self.fx, 0.0, self.cx, tx,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return info

    def destroy_node(self):
        self._shutdown.set()
        self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AirSimRosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

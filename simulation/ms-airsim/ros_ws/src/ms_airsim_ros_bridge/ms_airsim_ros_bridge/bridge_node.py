"""Microsoft AirSim (legacy)-to-ROS 2 bridge: publishes stereo RGB, depth, and camera_info.

One process per robot, each on its own ROS_DOMAIN_ID (matching Isaac Sim convention).
Lazy: only polls AirSim for images that have active ROS subscribers.
Clock is published at high rate from a dedicated thread/client.
"""

import math
import threading
import queue
import time

import airsim
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rosgraph_msgs.msg import Clock


class MsAirSimRosBridge(Node):

    def __init__(self):
        super().__init__('ms_airsim_ros_bridge')

        self.declare_parameter('ms_airsim_ip', '127.0.0.1')
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('clock_rate', 50.0)

        ip = self.get_parameter('ms_airsim_ip').value
        rate = self.get_parameter('publish_rate').value
        robot_name = self.get_parameter('robot_name').value
        clock_rate = self.get_parameter('clock_rate').value
        self.vehicle_name = robot_name

        # Connect to AirSim (retry until ready)
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

        # Get camera intrinsics from AirSim
        cam_info = self.client.simGetCameraInfo(
            'front_left', vehicle_name=self.vehicle_name
        )
        test_img = self.client.simGetImages(
            [airsim.ImageRequest('front_left', airsim.ImageType.Scene, False, True)],
            vehicle_name=self.vehicle_name,
        )[0]
        self.width = test_img.width
        self.height = test_img.height
        fov_rad = math.radians(cam_info.fov)
        self.fx = self.width / (2.0 * math.tan(fov_rad / 2.0))
        self.fy = self.fx
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        # Derive baseline from camera Y positions in settings.json
        # simGetCameraInfo returns world-frame poses, not body-relative,
        # so we read the config directly instead.
        import json
        settings_path = '/home/ms-airsim/Documents/AirSim/settings.json'
        with open(settings_path) as f:
            settings = json.load(f)
        cams = settings['Vehicles'][self.vehicle_name]['Cameras']
        left_y = cams['front_left']['Y']
        right_y = cams['front_right']['Y']
        self.baseline = abs(right_y - left_y)

        # Publishers
        prefix = f'/{robot_name}/sensors/front_stereo'
        self.left_image_pub = self.create_publisher(Image, f'{prefix}/left/image_rect', 1)
        self.right_image_pub = self.create_publisher(Image, f'{prefix}/right/image_rect', 1)
        self.left_info_pub = self.create_publisher(CameraInfo, f'{prefix}/left/camera_info', 1)
        self.right_info_pub = self.create_publisher(CameraInfo, f'{prefix}/right/camera_info', 1)
        self.left_depth_pub = self.create_publisher(Image, f'{prefix}/left/depth_ground_truth', 1)
        self.right_depth_pub = self.create_publisher(Image, f'{prefix}/right/depth_ground_truth', 1)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)

        self._shutdown = threading.Event()

        # Dedicated clock thread with its own AirSim client
        self._clock_client = airsim.MultirotorClient(ip=ip)
        self._clock_client.confirmConnection()
        self._clock_interval = 1.0 / clock_rate
        self._clock_thread = threading.Thread(target=self._clock_loop, daemon=True)
        self._clock_thread.start()

        # Background image fetcher with its own AirSim client
        self._image_client = airsim.MultirotorClient(ip=ip)
        self._image_client.confirmConnection()
        self._queue = queue.Queue(maxsize=2)
        self._image_thread = threading.Thread(target=self._fetcher, daemon=True)
        self._image_thread.start()

        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'Bridge running: {robot_name} (lazy) @ {rate}Hz, '
            f'clock @ {clock_rate}Hz, '
            f'fx={self.fx:.1f}, baseline={self.baseline:.3f}m, '
            f'image={self.width}x{self.height}'
        )

    def _clock_loop(self):
        while not self._shutdown.is_set():
            try:
                state = self._clock_client.getMultirotorState(
                    vehicle_name=self.vehicle_name
                )
                sim_ts = state.timestamp
                clock_msg = Clock()
                clock_msg.clock.sec = int(sim_ts // 1_000_000_000)
                clock_msg.clock.nanosec = int(sim_ts % 1_000_000_000)
                self.clock_pub.publish(clock_msg)
            except Exception:
                pass
            time.sleep(self._clock_interval)

    def _has_subscribers(self, *publishers):
        return any(p.get_subscription_count() > 0 for p in publishers)

    def _build_requests(self):
        requests = []
        keys = []

        need_left_rgb = self._has_subscribers(self.left_image_pub)
        need_right_rgb = self._has_subscribers(self.right_image_pub)
        need_left_depth = self._has_subscribers(self.left_depth_pub)
        need_right_depth = self._has_subscribers(self.right_depth_pub)
        self._need_left_info = self._has_subscribers(self.left_info_pub) or need_left_rgb or need_left_depth
        self._need_right_info = self._has_subscribers(self.right_info_pub) or need_right_rgb or need_right_depth

        if need_left_rgb:
            requests.append(airsim.ImageRequest('front_left', airsim.ImageType.Scene, False, True))
            keys.append('left_rgb')
        if need_left_depth:
            requests.append(airsim.ImageRequest('front_left', airsim.ImageType.DepthPlanar, True, False))
            keys.append('left_depth')
        if need_right_rgb:
            requests.append(airsim.ImageRequest('front_right', airsim.ImageType.Scene, False, True))
            keys.append('right_rgb')
        if need_right_depth:
            requests.append(airsim.ImageRequest('front_right', airsim.ImageType.DepthPlanar, True, False))
            keys.append('right_depth')

        return requests, keys

    def _fetcher(self):
        while not self._shutdown.is_set():
            requests, keys = self._build_requests()
            if not requests:
                time.sleep(0.05)
                continue
            try:
                responses = self._image_client.simGetImages(
                    requests, vehicle_name=self.vehicle_name
                )
                if len(responses) == len(keys) and responses[0].height > 0:
                    if not self._queue.full():
                        self._queue.put((keys, responses))
            except Exception as e:
                self.get_logger().warn(
                    f'Image fetch failed: {e}', throttle_duration_sec=5.0
                )

    def _publish(self):
        try:
            keys, responses = self._queue.get_nowait()
        except queue.Empty:
            return

        data = dict(zip(keys, responses))
        sim_ts = responses[0].time_stamp
        now = Clock().clock
        now.sec = int(sim_ts // 1_000_000_000)
        now.nanosec = int(sim_ts % 1_000_000_000)

        if 'left_rgb' in data:
            self.left_image_pub.publish(
                _make_image_msg(_decode_compressed(data['left_rgb']), now, 'camera_left', 'rgb8')
            )
        if 'right_rgb' in data:
            self.right_image_pub.publish(
                _make_image_msg(_decode_compressed(data['right_rgb']), now, 'camera_right', 'rgb8')
            )
        if 'left_depth' in data:
            self.left_depth_pub.publish(_make_depth_msg(data['left_depth'], now, 'camera_left'))
        if 'right_depth' in data:
            self.right_depth_pub.publish(_make_depth_msg(data['right_depth'], now, 'camera_right'))
        if self._need_left_info:
            self.left_info_pub.publish(self._make_cam_info(now, 'left', 0.0))
        if self._need_right_info:
            self.right_info_pub.publish(
                self._make_cam_info(now, 'right', -self.fx * self.baseline)
            )

    def _make_cam_info(self, stamp, side, tx):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = f'camera_{side}'
        info.height = self.height
        info.width = self.width
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, tx, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def destroy_node(self):
        self._shutdown.set()
        self._clock_thread.join(timeout=2.0)
        self._image_thread.join(timeout=2.0)
        super().destroy_node()


def _decode_compressed(response):
    import cv2
    img = cv2.imdecode(
        np.frombuffer(response.image_data_uint8, dtype=np.uint8), cv2.IMREAD_COLOR
    )
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def _make_image_msg(img_array, stamp, frame_id, encoding):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = img_array.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = msg.width * img_array.shape[2]
    msg.data = img_array.tobytes()
    return msg


def _make_depth_msg(response, stamp, frame_id):
    depth = np.array(response.image_data_float, dtype=np.float32)
    depth = depth.reshape(response.height, response.width)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = response.height
    msg.width = response.width
    msg.encoding = '32FC1'
    msg.is_bigendian = False
    msg.step = response.width * 4
    msg.data = depth.tobytes()
    return msg


def main(args=None):
    rclpy.init(args=args)
    node = MsAirSimRosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

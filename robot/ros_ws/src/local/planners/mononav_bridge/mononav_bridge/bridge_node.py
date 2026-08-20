#!/usr/bin/env python3
"""Serve synchronized AirStack camera/GT-pose samples and accept MonoNav paths."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import struct
import threading
import time
import zlib

import cv2
import numpy as np
import rclpy
from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
from airstack_msgs.srv import TrajectoryMode
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def _transform_matrix(transform):
    q = transform.rotation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    rotation = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
    return matrix


class _BridgeHttpServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, node):
        self.node = node
        super().__init__(address, _BridgeRequestHandler)


class _BridgeRequestHandler(BaseHTTPRequestHandler):
    server_version = "AirStackMonoNav/0.1"

    def log_message(self, _format, *_args):
        return

    def _json_response(self, status, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            self._json_response(200, self.server.node.health())
            return
        if self.path != "/frame":
            self._json_response(404, {"error": "unknown endpoint"})
            return
        sample = self.server.node.latest_sample()
        if sample is None:
            self._json_response(503, {"error": "no synchronized camera/pose sample yet"})
            return
        metadata, jpeg, depth = sample
        metadata = dict(metadata)
        metadata["jpeg_bytes"] = len(jpeg)
        metadata["depth_bytes"] = len(depth)
        metadata_bytes = json.dumps(metadata, separators=(",", ":")).encode("utf-8")
        body = struct.pack("!I", len(metadata_bytes)) + metadata_bytes + jpeg + depth
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path == "/pause":
            self._json_response(200, self.server.node.pause_trajectory())
            return
        if self.path != "/trajectory":
            self._json_response(404, {"error": "unknown endpoint"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(length))
            result = self.server.node.accept_trajectory(payload)
            self._json_response(200 if result["accepted"] else 409, result)
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
            self._json_response(400, {"accepted": False, "error": str(exc)})


class MonoNavBridge(Node):
    def __init__(self):
        super().__init__("mononav_bridge")
        self.declare_parameter("server_address", "0.0.0.0")
        self.declare_parameter("server_port", 8765)
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("max_frame_rate", 3.0)
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("execute_commands", False)
        self.declare_parameter("command_velocity", 0.35)
        self.declare_parameter("anchor_trajectory_to_odometry", True)

        self._target_frame = self.get_parameter("target_frame").value
        self._max_frame_rate = float(self.get_parameter("max_frame_rate").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self._execute_commands = bool(self.get_parameter("execute_commands").value)
        self._command_velocity = float(self.get_parameter("command_velocity").value)
        self._anchor_to_odometry = bool(
            self.get_parameter("anchor_trajectory_to_odometry").value
        )
        self._lock = threading.Lock()
        self._camera_info = None
        self._depth_sample = None
        self._sample = None
        self._sequence = 0
        self._last_frame_wall_time = 0.0
        self._last_odom_stamp = None
        self._last_odom_position = None
        self._last_command = None

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(CameraInfo, "camera_info", self._camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, "image", self._image_callback, qos_profile_sensor_data)
        self.create_subscription(Image, "depth_image", self._depth_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odometry", self._odometry_callback, qos_profile_sensor_data)

        self._trajectory_publisher = self.create_publisher(
            TrajectoryXYZVYaw, "trajectory_segment", 1
        )
        self._trajectory_override_publisher = self.create_publisher(
            TrajectoryXYZVYaw, "trajectory_override", 1
        )
        self._marker_publisher = self.create_publisher(MarkerArray, "trajectory_markers", 1)
        self._status_publisher = self.create_publisher(String, "status", 10)
        self._mode_client = self.create_client(TrajectoryMode, "set_trajectory_mode")
        self.create_timer(1.0, self._publish_status)

        address = self.get_parameter("server_address").value
        port = int(self.get_parameter("server_port").value)
        self._http_server = _BridgeHttpServer((address, port), self)
        self._http_thread = threading.Thread(target=self._http_server.serve_forever, daemon=True)
        self._http_thread.start()
        self.get_logger().info(
            f"MonoNav bridge listening on {address}:{port}; execute_commands={self._execute_commands}"
        )

    def destroy_node(self):
        self._http_server.shutdown()
        self._http_server.server_close()
        return super().destroy_node()

    def _camera_info_callback(self, msg):
        with self._lock:
            self._camera_info = {
                "width": msg.width,
                "height": msg.height,
                "k": list(msg.k),
                "frame_id": msg.header.frame_id,
            }

    def _odometry_callback(self, msg):
        with self._lock:
            self._last_odom_stamp = msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec
            self._last_odom_position = np.array(
                [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
                dtype=np.float64,
            )

    @staticmethod
    def _image_to_bgr(msg):
        channels = 3 if msg.encoding.lower() in ("rgb8", "bgr8") else None
        if channels is None:
            raise ValueError(f"unsupported image encoding: {msg.encoding}")
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        image = raw[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        if msg.encoding.lower() == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image

    @staticmethod
    def _depth_to_meters(msg):
        encoding = msg.encoding.lower()
        if encoding == "32fc1":
            item_size = np.dtype(np.float32).itemsize
            raw = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // item_size)
            depth = raw[:, : msg.width].copy()
        elif encoding in ("16uc1", "mono16"):
            item_size = np.dtype(np.uint16).itemsize
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // item_size)
            depth = raw[:, : msg.width].astype(np.float32) / 1000.0
        else:
            raise ValueError(f"unsupported depth encoding: {msg.encoding}")
        depth[~np.isfinite(depth)] = 0.0
        return depth

    def _depth_callback(self, msg):
        try:
            depth = self._depth_to_meters(msg)
        except ValueError as exc:
            self.get_logger().warn(f"Skipping depth frame: {exc}", throttle_duration_sec=2.0)
            return
        stamp = msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec
        compressed = zlib.compress(depth.astype("<f4", copy=False).tobytes(), level=1)
        with self._lock:
            self._depth_sample = (stamp, msg.width, msg.height, compressed)

    def _image_callback(self, msg):
        now = time.monotonic()
        if now - self._last_frame_wall_time < 1.0 / max(self._max_frame_rate, 0.1):
            return
        with self._lock:
            camera_info = self._camera_info
            depth_sample = self._depth_sample
        if camera_info is None:
            return
        try:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    msg.header.frame_id,
                    Time.from_msg(msg.header.stamp),
                    timeout=Duration(seconds=0.03),
                )
            except TransformException:
                transform = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    msg.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=0.03),
                )
            image = self._image_to_bgr(msg)
            ok, encoded = cv2.imencode(
                ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
            )
            if not ok:
                raise ValueError("JPEG encoding failed")
        except (TransformException, ValueError) as exc:
            self.get_logger().warn(f"Skipping camera frame: {exc}", throttle_duration_sec=2.0)
            return

        stamp = msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec
        metadata = {
            "sequence": self._sequence,
            "stamp": stamp,
            "target_frame": self._target_frame,
            "camera_frame": msg.header.frame_id,
            "width": msg.width,
            "height": msg.height,
            "k": camera_info["k"],
            "t_world_camera": _transform_matrix(transform.transform).reshape(-1).tolist(),
        }
        depth_bytes = b""
        if depth_sample is not None:
            depth_stamp, depth_width, depth_height, depth_bytes = depth_sample
            metadata.update(
                {
                    "depth_stamp": depth_stamp,
                    "depth_width": depth_width,
                    "depth_height": depth_height,
                    "depth_dtype": "float32_le_m",
                }
            )
        with self._lock:
            self._sample = (metadata, encoded.tobytes(), depth_bytes)
            self._sequence += 1
        self._last_frame_wall_time = now

    def latest_sample(self):
        with self._lock:
            return self._sample

    def health(self):
        with self._lock:
            sample = self._sample
            odom_stamp = self._last_odom_stamp
            last_command = self._last_command
            depth_sample = self._depth_sample
        return {
            "ready": sample is not None,
            "sequence": None if sample is None else sample[0]["sequence"],
            "image_stamp": None if sample is None else sample[0]["stamp"],
            "odometry_stamp": odom_stamp,
            "depth_stamp": None if depth_sample is None else depth_sample[0],
            "execute_commands": self._execute_commands,
            "last_command": last_command,
        }

    def accept_trajectory(self, payload):
        primitive_index = int(payload["primitive_index"])
        execute_requested = bool(payload.get("execute", False))
        points = payload["waypoints"]
        if not isinstance(points, list) or not 2 <= len(points) <= 200:
            raise ValueError("waypoints must contain between 2 and 200 points")

        trajectory = TrajectoryXYZVYaw()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = self._target_frame
        marker = Marker()
        marker.header = trajectory.header
        marker.ns = "mononav_selected_primitive"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.06
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.color.a = 1.0

        with self._lock:
            odom_position = None if self._last_odom_position is None else self._last_odom_position.copy()
        offset = np.zeros(3, dtype=np.float64)
        if self._anchor_to_odometry and odom_position is not None:
            offset = odom_position - np.asarray(points[0][:3], dtype=np.float64)

        for item in points:
            if len(item) < 3 or not all(math.isfinite(float(value)) for value in item[:3]):
                raise ValueError("waypoint coordinates must be finite xyz triples")
            waypoint = WaypointXYZVYaw()
            waypoint.position.x = float(item[0]) + float(offset[0])
            waypoint.position.y = float(item[1]) + float(offset[1])
            waypoint.position.z = float(item[2]) + float(offset[2])
            waypoint.velocity = float(payload.get("velocity", self._command_velocity))
            waypoint.yaw = float(item[3]) if len(item) > 3 else 0.0
            trajectory.waypoints.append(waypoint)
            marker.points.append(
                Point(x=waypoint.position.x, y=waypoint.position.y, z=waypoint.position.z)
            )

        self._marker_publisher.publish(MarkerArray(markers=[marker]))
        with self._lock:
            self._last_command = {
                "primitive_index": primitive_index,
                "waypoint_count": len(points),
                "stamp": time.time(),
            }

        if not self._execute_commands or not execute_requested:
            return {
                "accepted": False,
                "visualized": True,
                "reason": (
                    "bridge execute_commands is false"
                    if not self._execute_commands
                    else "worker did not request execution"
                ),
            }

        if self._mode_client.service_is_ready():
            request = TrajectoryMode.Request()
            request.mode = TrajectoryMode.Request.ADD_SEGMENT
            self._mode_client.call_async(request)
        else:
            self.get_logger().warn("Trajectory mode service is not ready; publishing segment anyway")
        replace_trajectory = bool(payload.get("replace", True))
        if replace_trajectory:
            self._trajectory_override_publisher.publish(trajectory)
        else:
            self._trajectory_publisher.publish(trajectory)
        self.get_logger().info(
            f"Published MonoNav primitive {primitive_index} with {len(points)} waypoints "
            f"({'override' if replace_trajectory else 'segment'})"
        )
        return {
            "accepted": True,
            "visualized": True,
            "waypoint_count": len(points),
            "trajectory_mode": "override" if replace_trajectory else "segment",
        }

    def pause_trajectory(self):
        if not self._mode_client.service_is_ready():
            return {"success": False, "reason": "trajectory mode service is not ready"}
        request = TrajectoryMode.Request()
        request.mode = TrajectoryMode.Request.PAUSE
        self._mode_client.call_async(request)
        self.get_logger().info("Requested trajectory PAUSE from MonoNav worker")
        return {"success": True}

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps(self.health(), separators=(",", ":"))
        self._status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MonoNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

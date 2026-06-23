"""FPV + LVLM navigation baseline.

A per-robot ROS 2 node that drives the drone purely from first-person-view
images using an InternVL3-2B vision-language model. Each tick it asks the model
for one of {move forward, turn left, turn right} given the recent FPV frames and
the target query, turns that into a waypoint in the robot's local 'map' frame,
and sends it to the robot's real global planner (droan_gl) as a NavigateTask
action goal — so the actual planner flies there with obstacle avoidance, rather
than the drone blindly following a raw /global_plan path.

This is the standalone FPV+LVLM baseline for RAVEN, integrated into the robot
container (no separate DDS-interop container). It is namespaced by ROBOT_NAME so
it runs unchanged on robot_1..robot_N.

Topics (robot = ROBOT_NAME):
  sub  /{robot}/sensors/front_stereo/left/image_rect   sensor_msgs/Image
  sub  /{robot}/odometry_conversion/odometry           nav_msgs/Odometry
  sub  /input_prompt                                    std_msgs/String  (query)
  sub  /{robot}/lvlm_baseline/search_area              geometry_msgs/PolygonStamped (optional clamp)
  action client /{robot}/tasks/navigate                task_msgs/action/NavigateTask
"""

import math
import os
import threading
import time
from collections import deque

import numpy as np
import rclpy
import rclpy.executors
import torch
import torchvision.transforms as T
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from PIL import Image as PIL_Image
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PolygonStamped
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from torchvision.transforms.functional import InterpolationMode
from transformers import (AutoConfig, AutoModel, AutoTokenizer,
                          BitsAndBytesConfig)
from cv_bridge import CvBridge

from task_msgs.action import NavigateTask

IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


def _point_in_polygon_xy(x: float, y: float, poly: list) -> bool:
    """Ray-cast point-in-polygon in XY. <3 vertices ⇒ unbounded (always inside)."""
    n = len(poly)
    if n < 3:
        return True
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and \
                (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-9) + xi):
            inside = not inside
        j = i
    return inside


def _nearest_point_in_polygon_xy(x: float, y: float, poly: list,
                                 inset_m: float = 1.0) -> tuple:
    """Closest point on the polygon boundary to (x, y), nudged inset_m toward the
    centroid so the result lands inside. Returns (x, y) if poly has < 3 vertices."""
    n = len(poly)
    if n < 3:
        return x, y
    best = (x, y)
    best_d2 = float('inf')
    for i in range(n):
        ax, ay = poly[i - 1]
        bx, by = poly[i]
        ex, ey = bx - ax, by - ay
        seg2 = ex * ex + ey * ey
        t = 0.0 if seg2 < 1e-9 else ((x - ax) * ex + (y - ay) * ey) / seg2
        t = max(0.0, min(1.0, t))
        px, py = ax + t * ex, ay + t * ey
        d2 = (px - x) ** 2 + (py - y) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = (px, py)
    cx = sum(p[0] for p in poly) / n
    cy = sum(p[1] for p in poly) / n
    dx, dy = cx - best[0], cy - best[1]
    d = (dx * dx + dy * dy) ** 0.5
    if d > 1e-6:
        f = min(inset_m, d) / d
        best = (best[0] + dx * f, best[1] + dy * f)
    return best


class LVLMBaseline(Node):
    def __init__(self):
        super().__init__('lvlm_baseline')

        self._robot_name = os.getenv('ROBOT_NAME', 'robot_1')
        self._prefix = f'/{self._robot_name}'

        # ── parameters ─────────────────────────────────────────────────────────
        self._model_path = self.declare_parameter(
            'model_path', 'OpenGVLab/InternVL3-2B').value
        target_param = self.declare_parameter('target_objects', ['']).value
        targets = [t.strip().lower() for t in target_param if t and t.strip()]
        self._target_objects = targets if targets else None

        self._min_altitude = float(self.declare_parameter('min_altitude_agl', 5.0).value)
        self._max_altitude = float(self.declare_parameter('max_altitude_agl', 20.0).value)
        self._forward_dist = float(self.declare_parameter('forward_dist_m', 10.0).value)
        self._turn_angle = float(self.declare_parameter('turn_angle_rad', math.pi / 3.0).value)
        self._goal_tolerance_m = float(self.declare_parameter('goal_tolerance_m', 4.0).value)
        # Camera mount pitch (rad) applied to odometry before the FLU→RDF transform.
        self._camera_pitch = float(self.declare_parameter('camera_pitch_rad', 0.1745).value)
        self._plan_period_s = float(self.declare_parameter('plan_period_s', 1.0).value)
        # Max time to let droan_gl fly to one LVLM waypoint before replanning.
        self._nav_goal_timeout_s = float(self.declare_parameter('nav_goal_timeout_s', 45.0).value)
        self._max_new_tokens = int(self.declare_parameter('max_new_tokens', 32).value)

        self._cbg = ReentrantCallbackGroup()
        self._bridge = CvBridge()
        # Guards the deques + pose/target/polygon shared between the executor
        # callbacks (image/odom/query) and the control worker thread.
        self._state_lock = threading.Lock()
        self._frame_memory = deque(maxlen=5)
        self._pose_memory = deque(maxlen=200)
        self._cur_pose_np = None
        self._search_poly: list = []
        self._last_warn_ts = 0.0
        self._src2rdf_transform = self.mat_3x3_to_4x4(
            self.get_coord_system_transform('flu', 'rdf'))

        self._stop = False
        self._model_ready = False
        self.model = None
        self.tokenizer = None
        self.generation_config = dict(max_new_tokens=self._max_new_tokens, do_sample=True)

        # ── ROS I/O ────────────────────────────────────────────────────────────
        self.create_subscription(
            Image, f'{self._prefix}/sensors/front_stereo/left/image_rect',
            self.image_callback, 10, callback_group=self._cbg)
        self.create_subscription(
            Odometry, f'{self._prefix}/odometry_conversion/odometry',
            self.odom_callback, 10, callback_group=self._cbg)
        # Global query topic (shared, matches raven_nav + the standalone baseline).
        self.create_subscription(
            String, '/input_prompt', self.target_object_callback, 10,
            callback_group=self._cbg)
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            PolygonStamped, f'{self._prefix}/lvlm_baseline/search_area',
            self.search_area_callback, latched_qos, callback_group=self._cbg)

        self._nav_client = ActionClient(
            self, NavigateTask, f'{self._prefix}/tasks/navigate',
            callback_group=self._cbg)

        self.get_logger().info(
            f'lvlm_baseline starting | robot={self._robot_name} | '
            f'model={self._model_path} | targets={self._target_objects} | '
            f'altitude=[{self._min_altitude}, {self._max_altitude}]')

        # Load the model off the ROS thread so subscriptions/feedback stay live.
        threading.Thread(target=self._load_model, daemon=True).start()
        # Drive navigation from a worker thread: model.chat() blocks for ~seconds,
        # which would stall a timer callback on the executor. The worker only
        # touches shared state through self._state_lock (snapshots), and ROS
        # futures (action client) are thread-safe while the executor spins.
        threading.Thread(target=self._control_loop, daemon=True).start()

    # ── model ──────────────────────────────────────────────────────────────────

    def _load_model(self):
        self.get_logger().info('Loading InternVL3-2B model...')
        device_map = self.split_model(self._model_path)
        quant_config = BitsAndBytesConfig(load_in_8bit=True)
        self.model = AutoModel.from_pretrained(
            self._model_path,
            torch_dtype=torch.bfloat16,
            quantization_config=quant_config,
            low_cpu_mem_usage=True,
            trust_remote_code=True,
            device_map=device_map).eval()
        self.tokenizer = AutoTokenizer.from_pretrained(
            self._model_path, trust_remote_code=True, use_fast=False)
        self._model_ready = True
        self.get_logger().info('InternVL3-2B model loaded!')

    # ── callbacks ────────────────────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        # Only move frames to the GPU once the model (and its CUDA context) is up.
        if not self._model_ready:
            return
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            pil_image = PIL_Image.fromarray(cv_image)
            pixel_values = self.load_image(pil_image, max_num=1).to(torch.bfloat16).cuda()
            with self._state_lock:
                self._frame_memory.append(pixel_values)
        except Exception as e:
            self.get_logger().error(f'Image preprocessing failed: {e}')

    def target_object_callback(self, msg: String):
        targets = [t.strip().lower() for t in msg.data.split(',') if t.strip()]
        with self._state_lock:
            self._target_objects = targets if targets else None
        self.get_logger().info(f'Target objects set: {targets if targets else None}')

    def search_area_callback(self, msg: PolygonStamped):
        poly = [(p.x, p.y) for p in msg.polygon.points]
        with self._state_lock:
            self._search_poly = poly
        if len(poly) >= 3:
            self.get_logger().info(
                f'search_area: {len(poly)} vertices (clamping waypoints)')

    def odom_callback(self, msg: Odometry):
        try:
            src_pose_4x4 = self.pose_to_numpy(msg.pose.pose)

            pitch = self._camera_pitch
            R_pitch = np.array([
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)]
            ], dtype=np.float32)
            T_base_to_cam = np.eye(4, dtype=np.float32)
            T_base_to_cam[:3, :3] = R_pitch
            src_pose_4x4 = src_pose_4x4 @ T_base_to_cam

            rdf_pose_4x4 = self.transform_pose_4x4(src_pose_4x4, self._src2rdf_transform)

            cur_pose_np = np.array([
                float(rdf_pose_4x4[2, 3]),
                float(-rdf_pose_4x4[0, 3]),
                float(-rdf_pose_4x4[1, 3]),
            ])
            with self._state_lock:
                self._pose_memory.append(cur_pose_np)
                self._cur_pose_np = cur_pose_np
        except Exception as e:
            self.get_logger().error(f'Odometry processing failed: {e}')

    # ── control loop ─────────────────────────────────────────────────────────────

    def _control_loop(self):
        while rclpy.ok() and not self._model_ready and not self._stop:
            time.sleep(0.5)
        # Let the device_map GPU contexts settle before the first inference.
        time.sleep(1.0)
        self.get_logger().info('lvlm_baseline control loop active')
        while rclpy.ok() and not self._stop:
            # Snapshot all shared state once, under the lock, so the executor
            # callbacks can keep mutating the deques during inference/navigation.
            with self._state_lock:
                targets = self._target_objects
                frames = list(self._frame_memory)
                poses = list(self._pose_memory)
                cur = self._cur_pose_np
                poly = list(self._search_poly)
            if targets is None:
                time.sleep(self._plan_period_s)
                continue
            if not frames or cur is None:
                now = time.time()
                if now - self._last_warn_ts > 5.0:
                    self.get_logger().warn('Not enough data to plan yet')
                    self._last_warn_ts = now
                time.sleep(self._plan_period_s)
                continue
            action = self._infer_action(frames, targets)
            path_msg = self._compute_waypoint_path(action, cur, poses, poly)
            if path_msg is None:
                time.sleep(self._plan_period_s)
                continue
            self.get_logger().info(
                f"Planned action '{action}' -> NavigateTask "
                f"({len(path_msg.poses)} poses)")
            self._send_navigate_and_wait(path_msg, self._nav_goal_timeout_s)

    def _infer_action(self, frames: list, targets: list) -> str:
        recent_frames = self.sample_periodic_frames(frames, num_samples=5, step=10)
        pixel_values = torch.cat(recent_frames, dim=0).to(torch.bfloat16).cuda()
        prompt = (
            "<image>\n"
            f"Find {targets}. "
            "Based on the current first-person view images, "
            "decide the next action for the robot. "
            "Choose only one of the following actions: 'move forward', 'turn left', 'turn right'. "
            "Return ONLY the action as plain text, no explanation."
        )
        try:
            response = self.model.chat(
                self.tokenizer, pixel_values, prompt, self.generation_config)
            self.get_logger().info(f'Model response: {response}')
        except Exception as e:
            self.get_logger().error(f'Model chat failed: {e}')
            response = 'move forward'
        action = response.strip().lower()
        if action not in ('move forward', 'turn left', 'turn right'):
            action = 'move forward'
        return action

    def _compute_waypoint_path(self, action: str, cur_pose, poses: list, poly: list):
        cur = np.array(cur_pose, dtype=np.float32)
        z_height = float(min(max(cur[2], self._min_altitude), self._max_altitude))
        forward_dist = self._forward_dist
        turn_angle = self._turn_angle

        lookback_idx = -10
        if len(poses) >= abs(lookback_idx):
            prev = np.array(poses[lookback_idx], dtype=np.float32)
            heading_vec = cur - prev
            heading_vec[2] = 0.0
            norm = np.linalg.norm(heading_vec)
            heading_unit = heading_vec / norm if norm > 0 else np.array([1.0, 0.0, 0.0])
        else:
            heading_unit = np.array([1.0, 0.0, 0.0])

        if action == 'turn left':
            c, s = np.cos(turn_angle), np.sin(turn_angle)
            heading_unit[:2] = np.array([c * heading_unit[0] - s * heading_unit[1],
                                         s * heading_unit[0] + c * heading_unit[1]])
        elif action == 'turn right':
            c, s = np.cos(-turn_angle), np.sin(-turn_angle)
            heading_unit[:2] = np.array([c * heading_unit[0] - s * heading_unit[1],
                                         s * heading_unit[0] + c * heading_unit[1]])

        goal = cur + heading_unit * forward_dist
        goal[2] = z_height
        extended_point = goal + heading_unit * forward_dist
        extended_point[2] = z_height

        # Keep waypoints inside the search polygon when one is set. Inset only the
        # XY projection; the Z altitude set above is preserved.
        if len(poly) >= 3:
            for pt in (goal, extended_point):
                if not _point_in_polygon_xy(pt[0], pt[1], poly):
                    nx, ny = _nearest_point_in_polygon_xy(pt[0], pt[1], poly)
                    pt[:2] = np.array([nx, ny], dtype=np.float32)

        planned_points = [goal, extended_point]
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for pt in planned_points:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.position.z = float(pt[2])
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        return path_msg

    def _send_navigate_and_wait(self, path_msg: Path, timeout_s: float) -> bool:
        """Send the LVLM waypoint to droan_gl as a NavigateTask goal and block until
        it completes (reached), times out, or is preempted. droan_gl runs the real
        local planner to fly there with obstacle avoidance."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('NavigateTask server (droan_gl) not available')
            return False
        goal = NavigateTask.Goal()
        goal.global_plan = path_msg
        goal.goal_tolerance_m = self._goal_tolerance_m
        send_future = self._nav_client.send_goal_async(goal)
        deadline = time.time() + 5.0
        while not send_future.done() and time.time() < deadline \
                and rclpy.ok() and not self._stop:
            time.sleep(0.05)
        handle = send_future.result() if send_future.done() else None
        if handle is None or not getattr(handle, 'accepted', False):
            self.get_logger().warn('NavigateTask goal not accepted by droan_gl')
            time.sleep(1.0)
            return False
        result_future = handle.get_result_async()
        deadline = time.time() + timeout_s
        while not result_future.done() and time.time() < deadline \
                and rclpy.ok() and not self._stop:
            time.sleep(0.1)
        if not result_future.done():
            self.get_logger().info('NavigateTask timed out — cancelling to replan')
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
            return False
        return True

    # ── coordinate helpers (preserved from the standalone baseline) ──────────────

    def vector3_to_numpy(self, msg):
        return np.array([msg.x, msg.y, msg.z])

    def quat_to_numpy(self, msg):
        return np.array([msg.x, msg.y, msg.z, msg.w])

    def quat_to_rot_matrix(self, q, scalar_first=False):
        q = np.array(q, dtype=np.float32)
        if not scalar_first:
            q = np.array([q[3], q[0], q[1], q[2]], dtype=np.float32)
        w, x, y, z = q
        return np.array([
            [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)]
        ], dtype=np.float32)

    def pose_to_numpy(self, ros_pose):
        pose_t = self.vector3_to_numpy(ros_pose.position)
        pose_q = self.quat_to_numpy(ros_pose.orientation)
        pose_R = self.quat_to_rot_matrix(pose_q, scalar_first=False)
        pose_Rt_3x4 = np.concatenate((pose_R, pose_t.reshape(3, 1)), axis=1)
        row = np.array([[0, 0, 0, 1]])
        return np.concatenate((pose_Rt_3x4, row), axis=-2)

    def transform_pose_4x4(self, pose_4x4, transform_mat_4x4):
        return transform_mat_4x4 @ pose_4x4 @ np.linalg.inv(transform_mat_4x4)

    def get_coord_system_transform(self, src: str, tgt: str):
        axes = dict(r=0, l=0, u=1, d=1, f=2, b=2)
        T = np.zeros((3, 3), dtype=np.float32)
        for i, tgt_dir in enumerate(tgt.lower()):
            a = axes[tgt_dir]
            for j, src_dir in enumerate(src.lower()):
                b = axes[src_dir]
                if a == b:
                    T[i, j] = 1 if src_dir == tgt_dir else -1
                    break
        return T

    def mat_3x3_to_4x4(self, mat):
        mat = np.asarray(mat)
        zeros = np.zeros((*mat.shape[:-2], 3, 1), dtype=mat.dtype)
        mat_3x4 = np.concatenate((mat, zeros), axis=-1)
        return self.mat_3x4_to_4x4(mat_3x4)

    def mat_3x4_to_4x4(self, mat):
        mat = np.asarray(mat)
        batch_shape = mat.shape[:-2]
        row = np.zeros((*batch_shape, 1, 4), dtype=mat.dtype)
        row[..., 0, 3] = 1.0
        return np.concatenate((mat, row), axis=-2)

    def sample_periodic_frames(self, memory, num_samples=5, step=10):
        if not memory:
            return []
        memory_len = len(memory)
        indices = [max(memory_len - 1 - i * step, 0) for i in range(num_samples)]
        indices = sorted(indices)
        return [memory[i] for i in indices]

    # ── InternVL3 image preprocessing (preserved from the standalone baseline) ───

    def build_transform(self, input_size):
        return T.Compose([
            T.Lambda(lambda img: img.convert('RGB') if img.mode != 'RGB' else img),
            T.Resize((input_size, input_size), interpolation=InterpolationMode.BICUBIC),
            T.ToTensor(),
            T.Normalize(mean=IMAGENET_MEAN, std=IMAGENET_STD),
        ])

    def find_closest_aspect_ratio(self, aspect_ratio, target_ratios, width, height, image_size):
        best_ratio_diff = float('inf')
        best_ratio = (1, 1)
        area = width * height
        for ratio in target_ratios:
            target_aspect_ratio = ratio[0] / ratio[1]
            ratio_diff = abs(aspect_ratio - target_aspect_ratio)
            if ratio_diff < best_ratio_diff:
                best_ratio_diff = ratio_diff
                best_ratio = ratio
            elif ratio_diff == best_ratio_diff:
                if area > 0.5 * image_size * image_size * ratio[0] * ratio[1]:
                    best_ratio = ratio
        return best_ratio

    def dynamic_preprocess(self, image, min_num=1, max_num=12, image_size=448, use_thumbnail=False):
        orig_width, orig_height = image.size
        aspect_ratio = orig_width / orig_height
        target_ratios = set(
            (i, j) for n in range(min_num, max_num + 1) for i in range(1, n + 1)
            for j in range(1, n + 1) if i * j <= max_num and i * j >= min_num)
        target_ratios = sorted(target_ratios, key=lambda x: x[0] * x[1])
        target_aspect_ratio = self.find_closest_aspect_ratio(
            aspect_ratio, target_ratios, orig_width, orig_height, image_size)
        target_width = image_size * target_aspect_ratio[0]
        target_height = image_size * target_aspect_ratio[1]
        blocks = target_aspect_ratio[0] * target_aspect_ratio[1]
        resized_img = image.resize((target_width, target_height))
        processed_images = []
        for i in range(blocks):
            box = (
                (i % (target_width // image_size)) * image_size,
                (i // (target_width // image_size)) * image_size,
                ((i % (target_width // image_size)) + 1) * image_size,
                ((i // (target_width // image_size)) + 1) * image_size,
            )
            processed_images.append(resized_img.crop(box))
        assert len(processed_images) == blocks
        if use_thumbnail and len(processed_images) != 1:
            processed_images.append(image.resize((image_size, image_size)))
        return processed_images

    def split_model(self, model_name):
        device_map = {}
        world_size = max(torch.cuda.device_count(), 1)
        config = AutoConfig.from_pretrained(model_name, trust_remote_code=True)
        num_layers = config.llm_config.num_hidden_layers
        num_layers_per_gpu = math.ceil(num_layers / (world_size - 0.5))
        num_layers_per_gpu = [num_layers_per_gpu] * world_size
        num_layers_per_gpu[0] = math.ceil(num_layers_per_gpu[0] * 0.5)
        layer_cnt = 0
        for i, num_layer in enumerate(num_layers_per_gpu):
            for _ in range(num_layer):
                device_map[f'language_model.model.layers.{layer_cnt}'] = i
                layer_cnt += 1
        device_map['vision_model'] = 0
        device_map['mlp1'] = 0
        device_map['language_model.model.tok_embeddings'] = 0
        device_map['language_model.model.embed_tokens'] = 0
        device_map['language_model.output'] = 0
        device_map['language_model.model.norm'] = 0
        device_map['language_model.model.rotary_emb'] = 0
        device_map['language_model.lm_head'] = 0
        device_map[f'language_model.model.layers.{num_layers - 1}'] = 0
        return device_map

    def load_image(self, image, input_size=448, max_num=1):
        transform = self.build_transform(input_size=input_size)
        images = self.dynamic_preprocess(
            image, image_size=input_size, use_thumbnail=True, max_num=max_num)
        pixel_values = [transform(img) for img in images]
        return torch.stack(pixel_values)


def main(args=None):
    rclpy.init(args=args)
    node = LVLMBaseline()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop = True
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

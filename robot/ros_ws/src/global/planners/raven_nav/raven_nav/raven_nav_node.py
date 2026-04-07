import os
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from raven_nav.behavior_manager import BehaviorManager


class RavenNavNode(Node):
    def __init__(self):
        super().__init__('raven_nav')

        robot_name = os.getenv('ROBOT_NAME', 'robot')
        self._prefix = f'/{robot_name}'

        # Ray data parsed from rays_sim/all (all in FLU world frame after conversion)
        self._ray_origins = None    # (N, 3) FLU
        self._ray_dirs = None       # (N, 3) FLU
        self._ray_scores = None     # (N, M) softmax scores

        self._frontiers = None      # (N, 3) FLU — published already in FLU
        self._cur_pose = None       # (3,)

        self._waypoint_locked = False
        self._target_waypoint = None
        self._target_waypoint2 = None
        self._behavior_mode = 'Frontier-based'

        # Parameters
        self._score_threshold = self.declare_parameter('score_threshold', 0.95).value
        query_labels_param = self.declare_parameter(
            'query_labels', ['red building', 'water tower', 'radio tower']).value
        self._query_labels = list(query_labels_param)
        self._target_objects = self._query_labels[:]

        timer_period = self.declare_parameter('timer_period', 0.5).value

        self._path_pub = self.create_publisher(
            Path, f'{self._prefix}/global_plan', 10)
        self._filtered_rays_pub = self.create_publisher(
            MarkerArray, f'{self._prefix}/filtered_rays', 10)
        self._viewpoint_pub = self.create_publisher(
            PointCloud2, f'{self._prefix}/frontier_viewpoints', 10)
        self._current_target_pub = self.create_publisher(
            String, f'{self._prefix}/current_target', 10)

        self._publisher_dict = {
            'path': self._path_pub,
            'filtered_rays': self._filtered_rays_pub,
            'viewpoint': self._viewpoint_pub,
            'current_target': self._current_target_pub,
        }

        self._behavior_manager = BehaviorManager(
            get_clock=self.get_clock,
            publisher_dict=self._publisher_dict,
            score_threshold=self._score_threshold,
        )

        self.create_subscription(
            PointCloud2,
            f'{self._prefix}/rayfronts/msg_serv/rays_sim/all',
            self._rays_cb, 10)
        self.create_subscription(
            PointCloud2,
            f'{self._prefix}/rayfronts/msg_serv/frontiers',
            self._frontiers_cb, 10)
        self.create_subscription(
            Odometry,
            f'{self._prefix}/odometry',
            self._odometry_cb, 10)
        self.create_subscription(
            String,
            '/input_prompt',
            self._input_prompt_cb, 10)

        self.create_timer(timer_period, self._timer_cb)

        self.get_logger().info(
            f'raven_nav started | robot={robot_name} | '
            f'timer={timer_period:.2f}s | '
            f'query_labels={self._query_labels} | '
            f'score_threshold={self._score_threshold}')

    def _rays_cb(self, msg: PointCloud2):
        """Parse rays_sim/all PointCloud2.

        Fields: x, y, z (RDF origin), theta, phi (degrees), sim_0, sim_1, ...
        Converts origin + direction to world FLU frame.
        """
        field_names = [f.name for f in msg.fields]
        pts = list(point_cloud2.read_points(msg, field_names=field_names,
                                            skip_nans=True))
        if not pts:
            self._ray_origins = None
            self._ray_dirs = None
            self._ray_scores = None
            return

        pts_arr = np.array([list(p) for p in pts], dtype=np.float32)

        x_idx = field_names.index('x')
        y_idx = field_names.index('y')
        z_idx = field_names.index('z')
        theta_idx = field_names.index('theta')
        phi_idx = field_names.index('phi')

        rdf_origins = pts_arr[:, [x_idx, y_idx, z_idx]]
        theta = np.deg2rad(pts_arr[:, theta_idx])
        phi = np.deg2rad(pts_arr[:, phi_idx])

        # Spherical → Cartesian in RDF frame
        # Matches geometry3d.spherical_to_cartesian / cartesian_to_spherical:
        #   theta = azimuthal in xy-plane, phi = polar from +z (forward)
        # x=cos(theta)*sin(phi), y=sin(theta)*sin(phi), z=cos(phi)
        dx_rdf = np.cos(theta) * np.sin(phi)
        dy_rdf = np.sin(theta) * np.sin(phi)
        dz_rdf = np.cos(phi)
        rdf_dirs = np.stack([dx_rdf, dy_rdf, dz_rdf], axis=1)

        # RDF → FLU: world_x=rdf_z, world_y=-rdf_x, world_z=-rdf_y
        self._ray_origins = np.stack([
            rdf_origins[:, 2], -rdf_origins[:, 0], -rdf_origins[:, 1]], axis=1)
        self._ray_dirs = np.stack([
            rdf_dirs[:, 2], -rdf_dirs[:, 0], -rdf_dirs[:, 1]], axis=1)

        # Collect sim_* columns in order
        sim_cols = [i for i, n in enumerate(field_names) if n.startswith('sim_')]
        sim_cols.sort(key=lambda i: int(field_names[i].split('_')[1]))
        if sim_cols:
            self._ray_scores = pts_arr[:, sim_cols]  # (N, M)
        else:
            self._ray_scores = None

    def _frontiers_cb(self, msg: PointCloud2):
        """Parse msg_serv/frontiers — already in FLU world frame."""
        pts = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                            skip_nans=True))
        if pts:
            self._frontiers = np.array([[p[0], p[1], p[2]] for p in pts],
                                       dtype=np.float32)
        else:
            self._frontiers = None

    def _odometry_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._cur_pose = np.array([p.x, p.y, p.z], dtype=np.float64)

    def _input_prompt_cb(self, msg: String):
        targets = [t.strip().lower() for t in msg.data.split(',') if t.strip()]
        if targets:
            self._target_objects = targets
            self.get_logger().info(f'target objects updated: {self._target_objects}')

    def _timer_cb(self):
        if self._cur_pose is None:
            self.get_logger().warn('waiting for odometry...', throttle_duration_sec=5.0)
            return

        prev_mode = self._behavior_mode
        self._behavior_manager.mode_select(
            ray_origins=self._ray_origins,
            ray_dirs=self._ray_dirs,
            ray_scores=self._ray_scores,
            query_labels=self._query_labels,
            target_objects=self._target_objects,
        )
        self._behavior_mode = self._behavior_manager.behavior_mode

        if self._behavior_mode != prev_mode:
            self.get_logger().info(f'behavior mode: {prev_mode} -> {self._behavior_mode}')
            self._waypoint_locked = False
            self._target_waypoint = None
            self._target_waypoint2 = None

        self._waypoint_locked, self._target_waypoint, self._target_waypoint2 = \
            self._behavior_manager.behavior_execute(
                behavior_mode=self._behavior_mode,
                frontiers=self._frontiers,
                cur_pose_np=self._cur_pose,
                waypoint_locked=self._waypoint_locked,
                target_waypoint=self._target_waypoint,
                target_waypoint2=self._target_waypoint2,
                publisher_dict=self._publisher_dict,
            )

        # Throttled status summary
        n_frontiers = len(self._frontiers) if self._frontiers is not None else 0
        n_rays = len(self._ray_origins) if self._ray_origins is not None else 0
        ray_beh = self._behavior_manager.ray_behavior
        n_filtered = len(ray_beh._filtered_indices) if ray_beh._filtered_indices is not None else 0
        publishing = self._target_waypoint is not None
        wp = (f'({self._target_waypoint[0]:.1f}, {self._target_waypoint[1]:.1f}, '
              f'{self._target_waypoint[2]:.1f})') if publishing else 'none'
        self.get_logger().info(
            f'[{self._behavior_mode}] '
            f'frontiers={n_frontiers} | rays={n_rays} filtered={n_filtered} | '
            f'target={self._behavior_manager.ray_behavior.current_target} | '
            f'publishing={publishing} wp={wp}',
            throttle_duration_sec=2.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = RavenNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

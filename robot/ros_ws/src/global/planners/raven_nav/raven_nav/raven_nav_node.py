import json
import os
import re
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from coordination_bringup.frame_utils import gps_to_enu
from coordination_msgs.msg import PeerProfile as PeerProfileMsg

from raven_nav.behavior_manager import BehaviorManager
from raven_nav.peer_state import PeerState


GOSSIP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Latched: late-joining raven_nav still gets the most recent search_area.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_NAV_MODE_TAG = {
    'Frontier-based': 'frontier',
    'Ray-based':      'ray',
    'Voxel-based':    'voxel',
}


class RavenNavNode(Node):
    def __init__(self):
        super().__init__('raven_nav')

        self._robot_name = os.getenv('ROBOT_NAME', 'robot')
        self._prefix = f'/{self._robot_name}'
        # rayfronts topics are under /robot_${ROS_DOMAIN_ID}/rayfronts/...
        ros_domain = os.getenv('ROS_DOMAIN_ID', '0')
        self._rf_prefix = f'/robot_{ros_domain}/rayfronts/msg_serv'

        # Numeric id for the tiebreak; lower id wins contested frontiers.
        try:
            self._my_id = int(ros_domain)
        except (TypeError, ValueError):
            m = re.search(r'(\d+)$', self._robot_name)
            self._my_id = int(m.group(1)) if m else 0

        # Ray/voxel data from rayfronts (converted to FLU)
        self._ray_origins = None
        self._ray_dirs = None
        self._ray_scores = None
        self._vox_xyz = None
        self._vox_scores = None

        self._frontiers = None
        self._cur_pose = None

        self._waypoint_locked = False
        self._target_waypoint = None
        self._target_waypoint2 = None
        self._behavior_mode = 'Frontier-based'
        self._last_completed = []

        # Set on first GPS fix.
        self._boot_enu: 'np.ndarray | None' = None
        self._alt_ground: 'float | None' = None
        self._peer_state = PeerState()

        # Polygon constraint in local 'map' frame. None = unconstrained.
        self._search_area_xy: 'np.ndarray | None' = None
        self._warned_polygon_degenerate = False

        self._score_threshold = self.declare_parameter('score_threshold', 0.95).value
        query_labels_param = self.declare_parameter(
            'query_labels', ['red building', 'water tower', 'radio tower']).value
        self._query_labels = list(query_labels_param)
        # Subset of query_labels to navigate toward; empty = all.
        target_labels_param = self.declare_parameter('target_labels', ['']).value
        target_labels = [t for t in target_labels_param if t]
        self._target_objects = target_labels if target_labels else self._query_labels[:]

        self._min_altitude = self.declare_parameter('min_altitude_agl', 1.5).value
        self._max_altitude = self.declare_parameter('max_altitude_agl', 100.0).value

        timer_period = self.declare_parameter('timer_period', 0.5).value
        # Coordination debug lines are tagged "[coord]".
        self._debug_coord = self.declare_parameter('debug_coordination', True).value

        self._path_pub = self.create_publisher(
            Path, f'{self._prefix}/global_plan', 10)
        self._filtered_rays_pub = self.create_publisher(
            MarkerArray, f'{self._prefix}/filtered_rays', 10)
        self._viewpoint_pub = self.create_publisher(
            PointCloud2, f'{self._prefix}/frontier_viewpoints', 10)
        # Shared with peers via gossip; receivers apply their own filters.
        self._raw_frontiers_pub = self.create_publisher(
            PointCloud2, f'{self._prefix}/raw_frontiers', 10)
        self._current_target_pub = self.create_publisher(
            String, f'{self._prefix}/current_target', 10)
        self._voxel_bbox_pub = self.create_publisher(
            MarkerArray, f'{self._prefix}/voxel_clusters', 10)
        self._completed_targets_pub = self.create_publisher(
            String, f'{self._prefix}/completed_targets', 10)
        self._nav_mode_pub = self.create_publisher(
            String, f'{self._prefix}/navigation_mode', 10)

        self._publisher_dict = {
            'path': self._path_pub,
            'filtered_rays': self._filtered_rays_pub,
            'viewpoint': self._viewpoint_pub,
            'raw_frontiers': self._raw_frontiers_pub,
            'current_target': self._current_target_pub,
            'voxel_bbox': self._voxel_bbox_pub,
        }

        self._behavior_manager = BehaviorManager(
            get_clock=self.get_clock,
            publisher_dict=self._publisher_dict,
            score_threshold=self._score_threshold,
            min_altitude=self._min_altitude,
            max_altitude=self._max_altitude,
        )

        # rayfronts publishes _sim/all topics only when something subscribes.
        self.create_subscription(
            PointCloud2,
            f'{self._rf_prefix}/rays_sim/all',
            self._ray_all_cb, 10)
        self.create_subscription(
            PointCloud2,
            f'{self._rf_prefix}/voxels_sim/all',
            self._vox_all_cb, 10)

        self.create_subscription(
            PointCloud2,
            f'{self._rf_prefix}/frontiers',
            self._frontiers_cb, 10)
        self.create_subscription(
            Odometry,
            f'{self._prefix}/odometry',
            self._odometry_cb, 10)
        self.create_subscription(
            String,
            '/input_prompt',
            self._input_prompt_cb, 10)

        # mavros publishes raw/fix as BEST_EFFORT; matching it here is required.
        navsat_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            NavSatFix,
            f'{self._prefix}/interface/mavros/global_position/raw/fix',
            self._navsat_cb, navsat_qos)
        self.create_subscription(
            PeerProfileMsg, '/gossip/peers',
            self._on_peer_profile, GOSSIP_QOS)
        self.create_subscription(
            PolygonStamped,
            f'{self._prefix}/raven_nav/search_area',
            self._search_area_cb, LATCHED_QOS)

        self.create_timer(timer_period, self._timer_cb)

        self.get_logger().info(
            f'raven_nav started | robot={self._robot_name} (id={self._my_id}) | '
            f'timer={timer_period:.2f}s | '
            f'query_labels={self._query_labels} | '
            f'score_threshold={self._score_threshold} | '
            f'altitude=[{self._min_altitude}, {self._max_altitude}]')

    def _ray_all_cb(self, msg: PointCloud2):
        """Ray PointCloud2 fields: x,y,z,theta,phi,sim_0,sim_1,..."""
        Q = len(self._query_labels)
        if Q == 0:
            return
        msg_field_names = [f.name for f in msg.fields]
        sim_fields = sorted([f for f in msg_field_names if f.startswith('sim_')],
                            key=lambda s: int(s.split('_', 1)[1]))
        if not sim_fields:
            self._ray_origins = None
            self._ray_dirs = None
            self._ray_scores = None
            return
        fields = ('x', 'y', 'z', 'theta', 'phi') + tuple(sim_fields)
        pts = list(point_cloud2.read_points(msg, field_names=fields, skip_nans=True))
        if not pts:
            self._ray_origins = None
            self._ray_dirs = None
            self._ray_scores = None
            return
        arr = np.array([list(p) for p in pts], dtype=np.float32)
        rdf_orig = arr[:, :3]
        theta = np.deg2rad(arr[:, 3])
        phi = np.deg2rad(arr[:, 4])
        sim_all = arr[:, 5:]
        dx = np.cos(theta) * np.sin(phi)
        dy = np.sin(theta) * np.sin(phi)
        dz = np.cos(phi)
        rdf_dirs = np.stack([dx, dy, dz], axis=1)
        # RDF → FLU
        flu_orig = np.stack([rdf_orig[:, 2], -rdf_orig[:, 0], -rdf_orig[:, 1]], axis=1)
        flu_dirs = np.stack([rdf_dirs[:, 2], -rdf_dirs[:, 0], -rdf_dirs[:, 1]], axis=1)
        self._ray_origins = flu_orig
        self._ray_dirs = flu_dirs
        self._ray_scores = sim_all

    def _vox_all_cb(self, msg: PointCloud2):
        """Voxel PointCloud2 fields: x,y,z,sim_0,sim_1,..."""
        Q = len(self._query_labels)
        if Q == 0:
            return
        msg_field_names = [f.name for f in msg.fields]
        sim_fields = sorted([f for f in msg_field_names if f.startswith('sim_')],
                            key=lambda s: int(s.split('_', 1)[1]))
        if not sim_fields:
            self._vox_xyz = None
            self._vox_scores = None
            return
        fields = ('x', 'y', 'z') + tuple(sim_fields)
        pts = list(point_cloud2.read_points(msg, field_names=fields, skip_nans=True))
        if not pts:
            self._vox_xyz = None
            self._vox_scores = None
            return
        arr = np.array([list(p) for p in pts], dtype=np.float32)
        rdf_xyz = arr[:, :3]
        sim_all = arr[:, 3:]
        # RDF → FLU
        flu_xyz = np.stack([rdf_xyz[:, 2], -rdf_xyz[:, 0], -rdf_xyz[:, 1]], axis=1)
        self._vox_xyz = flu_xyz
        self._vox_scores = sim_all

    def _frontiers_cb(self, msg: PointCloud2):
        pts = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                            skip_nans=True))
        self._frontiers = (
            np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)
            if pts else None)

    def _odometry_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._cur_pose = np.array([p.x, p.y, p.z], dtype=np.float64)

    def _input_prompt_cb(self, msg: String):
        targets = [t.strip() for t in msg.data.split(',') if t.strip()]
        if targets:
            self._target_objects = targets
            if targets != self._query_labels:
                # Column count changed; clear cached arrays + visited state.
                self._query_labels = targets[:]
                self._vox_xyz = None
                self._vox_scores = None
                self._ray_origins = None
                self._ray_dirs = None
                self._ray_scores = None
                self._behavior_manager.voxel_behavior.reset()
                self._last_completed = []
            self.get_logger().info(f'target objects updated: {self._target_objects}')

    def _navsat_cb(self, msg: NavSatFix):
        """Capture boot ENU + ground altitude on first valid fix."""
        if self._boot_enu is not None:
            return
        if msg.status.status < 0:
            return
        self._alt_ground = msg.altitude
        self._boot_enu = np.array(
            gps_to_enu(msg.latitude, msg.longitude, msg.altitude),
            dtype=np.float64,
        )
        self.get_logger().info(
            f'boot GPS captured: alt_ground={self._alt_ground:.2f}m, '
            f'boot_enu=({self._boot_enu[0]:.2f}, {self._boot_enu[1]:.2f}, '
            f'{self._boot_enu[2]:.2f})')

    def _on_peer_profile(self, msg: PeerProfileMsg):
        # Skip own profile (gossip publishes self too) and pre-boot messages.
        if msg.robot_name == self._robot_name:
            return
        if self._boot_enu is None:
            if self._debug_coord:
                self.get_logger().info(
                    f'[coord] dropped peer profile from {msg.robot_name}: '
                    'own boot GPS not received yet',
                    throttle_duration_sec=5.0)
            return
        new_peer = msg.robot_name not in self._peer_state.peer_last_seen
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._peer_state.update(msg, self._boot_enu, self._alt_ground, now_sec)
        if new_peer and self._debug_coord:
            wp = self._peer_state.peer_waypoints.get(msg.robot_name)
            pos = self._peer_state.peer_positions.get(msg.robot_name)
            wp_s = (f'wp=({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f})'
                    if wp is not None else 'wp=none')
            pos_s = (f'pos=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})'
                     if pos is not None else 'pos=none')
            self.get_logger().info(
                f'[coord] peer FIRST SEEN: {msg.robot_name} '
                f'(id={self._peer_state.peer_ids.get(msg.robot_name)}) {pos_s} {wp_s}')

    def _search_area_cb(self, msg: PolygonStamped):
        """Empty / <3 vertex polygon clears the constraint."""
        pts = msg.polygon.points
        if len(pts) < 3:
            self._search_area_xy = None
            if not self._warned_polygon_degenerate and len(pts) > 0:
                self.get_logger().warn(
                    f'search_area has {len(pts)} vertex/vertices (<3); '
                    'treating as unconstrained.')
                self._warned_polygon_degenerate = True
            return
        self._warned_polygon_degenerate = False
        self._search_area_xy = np.array(
            [[p.x, p.y] for p in pts], dtype=np.float64)
        self.get_logger().info(
            f'search_area updated: {self._search_area_xy.shape[0]} vertices.')

    def _timer_cb(self):
        if self._cur_pose is None:
            self.get_logger().warn('waiting for odometry...', throttle_duration_sec=5.0)
            self._nav_mode_pub.publish(String(data='idle'))
            return

        prev_mode = self._behavior_mode
        self._behavior_manager.mode_select(
            ray_origins=self._ray_origins,
            ray_dirs=self._ray_dirs,
            ray_scores=self._ray_scores,
            query_labels=self._query_labels,
            target_objects=self._target_objects,
            vox_xyz=self._vox_xyz,
            vox_scores=self._vox_scores,
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
                vox_xyz=self._vox_xyz,
                vox_scores=self._vox_scores,
                query_labels=self._query_labels,
                peer_state=self._peer_state,
                my_id=self._my_id,
                search_area_xy=self._search_area_xy,
                debug_logger=(self.get_logger() if self._debug_coord else None),
            )

        self._nav_mode_pub.publish(
            String(data=_NAV_MODE_TAG.get(self._behavior_mode, 'idle')))

        completed = sorted(self._behavior_manager.completed_queries)
        if completed != self._last_completed:
            self._completed_targets_pub.publish(String(data=json.dumps(completed)))
            self._last_completed = completed

        n_frontiers = len(self._frontiers) if self._frontiers is not None else 0
        n_rays = len(self._ray_origins) if self._ray_origins is not None else 0
        n_voxels = len(self._vox_xyz) if self._vox_xyz is not None else 0
        ray_beh = self._behavior_manager.ray_behavior
        n_filtered = (len(ray_beh._filtered_indices)
                      if ray_beh._filtered_indices is not None else 0)
        wp = (f'({self._target_waypoint[0]:.1f}, {self._target_waypoint[1]:.1f}, '
              f'{self._target_waypoint[2]:.1f})'
              if self._target_waypoint is not None else 'none')
        self.get_logger().info(
            f'[{self._behavior_mode}] '
            f'frontiers={n_frontiers} rays={n_rays} filtered={n_filtered} '
            f'voxels={n_voxels} | '
            f'target={ray_beh.current_target} | '
            f'completed={completed} | wp={wp}',
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

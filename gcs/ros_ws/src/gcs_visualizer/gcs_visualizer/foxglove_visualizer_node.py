"""
foxglove_visualizer_node.py — Standard GCS visualization node.

Handles data common to every project: robot mesh markers, name labels,
body-frame axes, local trajectory, global plan, and VDB occupancy map.
All markers are published to /gcs/robot_markers in the global ENU 'map' frame.
"""

import os
import re

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, Image, PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from std_msgs.msg import ColorRGBA, Float32, Float64
from tf2_ros import StaticTransformBroadcaster

from gcs_visualizer.gcs_utils import (
    gps_to_enu, multiply_quaternions, rotate_vector, transform_marker_array,
    transform_point_cloud2, translate_occupancy_grid, translate_point_cloud2,
    voxel_sim_cloud_to_scene_update,
    voxel_sim_cloud_to_cube_marker, fluorescent,
    ORIGIN_LAT, ORIGIN_LON, ROBOT_COLORS,
)

# foxglove_msgs enters the GCS image via Dockerfile.gcs; older images predate
# it, so fall back to CUBE_LIST markers (no cube outlines) when absent.
try:
    from foxglove_msgs.msg import SceneUpdate
except ImportError:
    SceneUpdate = None

# Per-query voxel gradient high color = fluorescent robot wheel color, except
# robot 3 — its wheel blue fluoresces to a dark #0040ff, so override with a
# brighter blue. Keyed by robot number; keep in sync with render_layout.py.
VOXEL_HIGH_OVERRIDE = {3: (0.20, 0.53, 1.0)}  # bright blue #3387ff

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

GPS_SUFFIX  = '/interface/mavros/global_position/global'
ODOM_SUFFIX = '/odometry_conversion/odometry'
TRAJ_SUFFIX = '/trajectory_controller/trajectory_vis'
PLAN_SUFFIX = '/global_plan'
VDB_SUFFIX  = '/vdb_mapping/vdb_map_visualization'
RAY_GROUPS_SUFFIX = '/ray_groups_viz'

# rayfronts per-query voxel heatmap topics, bridged from the robot domain by the
# DDS router (see onboard_all/config/dds_router.yaml). Names: /<robot>/rayfronts/
# msg_serv/voxels_sim/{all|q{q}_{label}} — the robot segment is robot_${DOMAIN}.
VOXELS_INFIX = '/rayfronts/msg_serv/'
# rayfronts rgb voxel map — the ROS visualizer's voxel_rgb layer (x, y, z, rgb),
# bridged the same way. Unlike voxels_sim (raw RDF), the visualizer already
# applies rdf2flu at the source, so this cloud is FLU (boot-translation only).
RGB_VOXELS_SUFFIX = '/rayfronts/voxel_rgb'
# rayfronts publishes voxels_sim clouds in RDF (right-down-forward, camera-
# optical); raven converts to FLU as [z, -x, -y] (raven_nav_node._vox_all_cb).
# That swap is a pure rotation — this is its quaternion (x, y, z, w), the same
# one gossip_node applies to gossiped voxel clouds. Applied before the boot-ENU
# translation so the republished cloud lands in the global 'map' frame.
RDF_TO_FLU_QUAT = (-0.5, 0.5, -0.5, 0.5)

# search_baselines (search_planner) overlay topics, bridged from the robot
# domain by the DDS router. EVERY ONE OF THEM IS IN THE ROBOT'S LOCAL `map`,
# which is anchored at that robot's TAKEOFF POINT — the same frame as its
# odometry, `/global_plan` and `trajectory_vis`, all of which this node
# already translates by the robot's boot ENU before drawing. Drawn raw on the
# GCS (whose `map` is global ENU: sim ground, GT boxes, robot meshes) they
# land at the WORLD ORIGIN instead of next to the drone, offset by exactly the
# spawn point — 100 m on the 250 m suburb, 340 m on the wildfire plat. Each is
# republished translated under `/gcs/<robot><suffix>`; the rendered layout
# points at those, never at the raw names.
SEARCH_OVERLAY_TOPICS = {
    '/occupancy':      OccupancyGrid,
    '/value_map':      OccupancyGrid,
    '/frontiers':      MarkerArray,
    '/search/markers': MarkerArray,
    '/frontier_cloud': PointCloud2,
    '/voxel_map':      PointCloud2,
}
_SEARCH_OVERLAY_TYPE_NAMES = {
    OccupancyGrid: 'nav_msgs/msg/OccupancyGrid',
    MarkerArray:   'visualization_msgs/msg/MarkerArray',
    PointCloud2:   'sensor_msgs/msg/PointCloud2',
}
# BEST_EFFORT, because that is what the DDS router RE-OFFERS on this domain
# for the two PointCloud2 topics (measured with `ros2 topic info -v` on the
# GCS: voxel_map / frontier_cloud arrive BEST_EFFORT+VOLATILE, the MarkerArray
# and OccupancyGrid ones RELIABLE) — whatever the planner itself publishes
# (RELIABLE+TRANSIENT_LOCAL). A RELIABLE reader is INCOMPATIBLE with a
# best-effort writer and silently gets nothing ("New publisher discovered ...
# offering incompatible QoS"), which is how the voxel map and frontier cloud
# went missing on the GCS while occupancy and the markers drew fine. A
# best-effort reader matches both kinds; the clouds are ~1 Hz and latched by
# the republish below, so a dropped fragment costs one tick, not the layer.
SEARCH_OVERLAY_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)
# How many GPS/odom pairs to average before the map origin is LOCKED. One
# sample is noise; ten is what coordination_bringup/map_anchor_node uses.
MAP_ORIGIN_SAMPLES = 10

# OBJ mesh axis correction quaternion (belly -Z, nose +X)
AXIS_CORRECTION = (-0.5, -0.5, 0.5, 0.5)


def _translate_marker(msg: Marker, bx: float, by: float, bz: float) -> Marker:
    """Return a new Marker with all points translated by (bx, by, bz) via numpy."""
    m = Marker()
    m.type = msg.type
    m.action = msg.action
    m.pose = msg.pose
    m.scale = msg.scale
    m.color = msg.color
    m.colors = list(msg.colors)
    m.header.frame_id = 'map'
    n = len(msg.points)
    if n > 0:
        xyz = np.array([(pt.x, pt.y, pt.z) for pt in msg.points], dtype=np.float64)
        xyz[:, 0] += bx
        xyz[:, 1] += by
        xyz[:, 2] += bz
        m.points = [Point(x=float(r[0]), y=float(r[1]), z=float(r[2])) for r in xyz]
    return m


class FoxgloveVisualizerNode(Node):
    def __init__(self):
        super().__init__('foxglove_visualizer_node')

        self.declare_parameter('robot_name_prefix', 'robot')
        self._prefix = self.get_parameter('robot_name_prefix').value
        self._gps_pattern  = re.compile(rf'^/({re.escape(self._prefix)}_\w+){re.escape(GPS_SUFFIX)}$')
        self._odom_pattern = re.compile(rf'^/({re.escape(self._prefix)}_\w+){re.escape(ODOM_SUFFIX)}$')
        self._traj_pattern = re.compile(rf'^/({re.escape(self._prefix)}_\w+){re.escape(TRAJ_SUFFIX)}$')
        self._plan_pattern = re.compile(rf'^/({re.escape(self._prefix)}_\w+){re.escape(PLAN_SUFFIX)}$')
        self._vdb_pattern  = re.compile(rf'^/({re.escape(self._prefix)}_\w+){re.escape(VDB_SUFFIX)}$')
        self._ray_groups_pattern = re.compile(rf'^/({re.escape(self._prefix)}_\w+){re.escape(RAY_GROUPS_SUFFIX)}$')
        # Captures (robot_name, voxel-topic suffix), e.g.
        # /robot_1/rayfronts/msg_serv/voxels_sim/q0_car -> ('robot_1', 'voxels_sim/q0_car')
        self._voxels_pattern = re.compile(
            rf'^/({re.escape(self._prefix)}_\w+){re.escape(VOXELS_INFIX)}(voxels_sim/.+)$')
        # Captures robot_name from /robot_1/rayfronts/voxel_rgb -> 'robot_1'
        self._rgb_voxels_pattern = re.compile(
            rf'^/({re.escape(self._prefix)}_\w+){re.escape(RGB_VOXELS_SUFFIX)}$')
        # Captures (robot_name, suffix) for the search_planner overlays, e.g.
        # /robot_1/search/markers -> ('robot_1', '/search/markers').
        self._search_pattern = re.compile(
            rf'^/({re.escape(self._prefix)}_\w+)('
            + '|'.join(re.escape(s) for s in SEARCH_OVERLAY_TOPICS) + r')$')

        self._gps_positions  = {}
        # robot_name -> the robot's `map` ORIGIN in global ENU (x, y, z). The
        # takeoff point, i.e. what every robot-local message has to be shifted
        # by to land on the GCS canvas. Named "boot" historically because it
        # used to be the first GPS fix this node happened to hear — which is
        # the takeoff point only if the GCS was up before the drone moved.
        # Now MEASURED as enu(fix) - odom at the same instant, like
        # map_anchor_node, so a GCS (re)started mid-flight gets the same
        # answer; the first-fix value remains the fallback until odom arrives.
        self._gps_boot       = {}
        self._boot_samples: dict = {}   # robot_name -> [(dx, dy, dz), ...] until locked
        self._odom_xyz: dict = {}       # robot_name -> last odom position (map frame)
        self._orientations   = {}
        self._trajectories   = {}
        self._global_plans   = {}
        self._vdb_markers    = {}
        self._vdb_global     = {}  # pre-translated global-frame VDB, keyed by robot_name
        # Per-robot NavSatFix re-publisher (frame_id rewritten to 'map' so
        # Foxglove's Map panel will accept it as a location source).
        self._location_pubs: dict = {}
        # /gcs/<robot>/map_origin: the locked boot ENU, LATCHED. A bag of the
        # raw /robot_N/odometry_conversion/odometry and /global_plan (each in
        # that robot's own takeoff-anchored `map`) is unplaceable without it;
        # the number used to live only in this node's log line.
        self._origin_pubs: dict = {}
        self._subscribed_gps  = set()
        self._subscribed_odom = set()
        self._subscribed_traj = set()
        self._subscribed_plan = set()
        self._subscribed_search = set()
        self._search_pubs: dict = {}    # source topic -> republish publisher

        # Make 'map' a known frame on the GCS domain so Foxglove resolves it.
        #
        # On the GCS, `map` IS global ENU — everything this package publishes
        # (robot meshes, sim ground, GT boxes, the translated overlays) is in
        # world coordinates stamped `map` — so world->map is identity HERE by
        # definition, and this is the ONLY transform the GCS tree should ever
        # hold. Robot TF is deliberately not bridged (dds_router.yaml): each
        # robot's `map` is its own takeoff-anchored frame under the same
        # unprefixed name, so bridging N of them (plus N measured world->maps
        # from map_anchor_node) onto this tree makes every `map`-stamped layer
        # follow whichever robot's transform arrived last. This node places
        # robots from their GPS fix and translates their local topics itself.
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'map'
        t.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(t)

        # Publish a stationary "map origin" location at the configured
        # ORIGIN_LAT/LON so the Foxglove Map panel always has a fixed
        # reference (otherwise the panel auto-centers on the robot, making
        # the robot appear at the visible map's center). 1 Hz so any newly
        # opened Map layer picks it up quickly.
        self._origin_pub = self.create_publisher(
            NavSatFix, '/gcs/map_origin/location', 10)
        self.create_timer(1.0, self._publish_map_origin)

        # Sim-mode "textured ground" pipeline. The simulator publishes a
        # raw overhead Image of its static scene plus three Float32 specs
        # (coverage_m, center_x_m, center_y_m) on separate topics so the
        # GCS doesn't need to be configured manually. We wait until image +
        # all three specs have been received, then build a TRIANGLE_LIST
        # marker with per-vertex colors and publish it once on a latched
        # topic for Foxglove's 3D panel. Subscriptions are torn down after
        # the first build to free up bandwidth.
        self.declare_parameter('overhead_image_topic', '/sim/overhead/image')
        self.declare_parameter('overhead_spec_topic', '/sim/overhead/spec')
        self.declare_parameter('overhead_center_x_topic', '/sim/overhead/center_x')
        self.declare_parameter('overhead_center_y_topic', '/sim/overhead/center_y')
        self.declare_parameter('overhead_coverage_m', 0.0)        # 0 → wait for spec topic
        # Center overrides: NaN sentinel ("nan") → wait for sim topic; any
        # finite value → use directly and ignore the topic.
        self.declare_parameter('overhead_center_x_m', float('nan'))
        self.declare_parameter('overhead_center_y_m', float('nan'))
        self.declare_parameter('overhead_grid_resolution', 0)     # 0 → derive from coverage
        self.declare_parameter('overhead_grid_per_m', 0.8)
        self.declare_parameter('overhead_max_grid_resolution', 384)
        self._overhead_topic = self.get_parameter(
            'overhead_image_topic').value
        self._overhead_spec_topic = self.get_parameter(
            'overhead_spec_topic').value
        self._overhead_cx_topic = self.get_parameter(
            'overhead_center_x_topic').value
        self._overhead_cy_topic = self.get_parameter(
            'overhead_center_y_topic').value
        # 0 → unknown / wait for spec; >0 → manual override.
        cov_param = float(self.get_parameter('overhead_coverage_m').value)
        self._overhead_coverage_m = cov_param if cov_param > 0 else None
        cx_param = float(self.get_parameter('overhead_center_x_m').value)
        cy_param = float(self.get_parameter('overhead_center_y_m').value)
        # math.isnan-safe via != self comparison (NaN != NaN).
        self._overhead_center_x_m = None if cx_param != cx_param else cx_param
        self._overhead_center_y_m = None if cy_param != cy_param else cy_param
        grid_param = int(self.get_parameter('overhead_grid_resolution').value)
        self._overhead_grid_n_override = grid_param if grid_param > 0 else None
        self._overhead_grid_per_m = float(self.get_parameter(
            'overhead_grid_per_m').value)
        self._overhead_max_grid_n = int(self.get_parameter(
            'overhead_max_grid_resolution').value)
        self._ground_published = False
        # OPAQUE by default. This is the BOTTOM layer — the scene itself — and
        # anything drawn over it (an occupancy grid, frontier markers, GT boxes)
        # is what should be translucent. At the old 0.7 the ground washed out
        # against the viewport background and every overlay on top of it read as
        # muddy rather than as a layer.
        self._ground_alpha = float(
            self.declare_parameter('ground_alpha', 1.0).value)
        self._pending_image = None    # cache image until all specs arrive
        self._ground_pub = self.create_publisher(
            Marker, '/gcs/sim_ground', LATCHED_QOS)
        self._overhead_sub = self.create_subscription(
            Image, self._overhead_topic,
            self._on_overhead_image, SENSOR_QOS)
        self._overhead_spec_sub = self.create_subscription(
            Float32, self._overhead_spec_topic,
            self._on_overhead_spec, SENSOR_QOS)
        self._overhead_cx_sub = self.create_subscription(
            Float32, self._overhead_cx_topic,
            self._on_overhead_center_x, SENSOR_QOS)
        self._overhead_cy_sub = self.create_subscription(
            Float32, self._overhead_cy_topic,
            self._on_overhead_center_y, SENSOR_QOS)
        self._subscribed_vdb  = set()
        self._subscribed_ray_groups = set()
        self._ray_groups_pubs: dict = {}
        self._subscribed_voxels = set()
        self._subscribed_rgb_voxels = set()
        self._voxels_pubs: dict = {}   # source topic -> republish publisher
        # Gradient min for the per-query voxel cubes — raven_nav's
        # voxel_score_threshold sim gate. Env default so the mission runner can
        # set it alongside the layout's (see gcs-base-docker-compose.yaml).
        self._voxel_sim_min = float(self.declare_parameter(
            'voxel_score_threshold',
            float(os.environ.get('VOXEL_SCORE_THRESHOLD') or 0.65)).value)
        # Per-query debug clouds are capped to the top-N voxels by sim (0 =
        # off): building per-cube messages in Python is the node's heaviest
        # work and clouds reach ~55k voxels late in a mission.
        self._max_debug_voxels = int(self.declare_parameter(
            'max_debug_voxels', 20000).value)
        if SceneUpdate is None:
            self.get_logger().warn(
                'foxglove_msgs not installed — per-query voxels fall back to '
                'CUBE_LIST markers without cube outlines (rebuild the GCS '
                'image to get ros-jazzy-foxglove-msgs)')

        # Ground reference altitude (MSL of map z=0). Set once when we have
        # both GPS and odom from the same robot — see _try_lock_ground. Until
        # then, GPS callbacks defer; otherwise gps_to_enu would fail with None.
        self._alt_ground          = None
        self._alt_ground_anchor   = None
        self._gps_msl: dict       = {}   # robot_name -> last MSL altitude (m)
        self._odom_z: dict        = {}   # robot_name -> last odom z (AGL above takeoff, m)
        # Latched so payload_visualizer_node and any other consumer get the
        # most recent value immediately on subscribe, regardless of startup
        # order relative to this node.
        self._ground_msl_pub = self.create_publisher(
            Float64, '/gcs/map_origin/ground_msl', LATCHED_QOS)

        self._pub = self.create_publisher(MarkerArray, '/gcs/robot_markers', 10)

        self.create_timer(5.0, self._discover_robots)
        self.create_timer(0.1, self._publish_markers)
        self._discover_robots()

    def _discover_robots(self):
        for topic, type_list in self.get_topic_names_and_types():
            if topic not in self._subscribed_gps:
                m = self._gps_pattern.match(topic)
                if m and 'sensor_msgs/msg/NavSatFix' in type_list:
                    name = m.group(1)
                    self.create_subscription(
                        NavSatFix, topic,
                        lambda msg, n=name: self._gps_callback(msg, n),
                        SENSOR_QOS,
                    )
                    self._subscribed_gps.add(topic)
                    self.get_logger().info(f'Subscribed to GPS: {topic}')

            if topic not in self._subscribed_odom:
                m = self._odom_pattern.match(topic)
                if m and 'nav_msgs/msg/Odometry' in type_list:
                    name = m.group(1)
                    self.create_subscription(
                        Odometry, topic,
                        lambda msg, n=name: self._odom_callback(msg, n),
                        SENSOR_QOS,
                    )
                    self._subscribed_odom.add(topic)
                    self.get_logger().info(f'Subscribed to odom: {topic}')

            if topic not in self._subscribed_traj:
                m = self._traj_pattern.match(topic)
                if m and 'visualization_msgs/msg/MarkerArray' in type_list:
                    name = m.group(1)
                    self.create_subscription(
                        MarkerArray, topic,
                        lambda msg, n=name: self._traj_callback(msg, n),
                        SENSOR_QOS,
                    )
                    self._subscribed_traj.add(topic)
                    self.get_logger().info(f'Subscribed to trajectory_vis: {topic}')

            if topic not in self._subscribed_plan:
                m = self._plan_pattern.match(topic)
                if m and 'nav_msgs/msg/Path' in type_list:
                    name = m.group(1)
                    self.create_subscription(
                        Path, topic,
                        lambda msg, n=name: self._plan_callback(msg, n),
                        SENSOR_QOS,
                    )
                    self._subscribed_plan.add(topic)
                    self.get_logger().info(f'Subscribed to global_plan: {topic}')

            if topic not in self._subscribed_vdb:
                m = self._vdb_pattern.match(topic)
                if m and 'visualization_msgs/msg/Marker' in type_list:
                    name = m.group(1)
                    self.create_subscription(
                        Marker, topic,
                        lambda msg, n=name: self._vdb_callback(msg, n),
                        SENSOR_QOS,
                    )
                    self._subscribed_vdb.add(topic)
                    self.get_logger().info(f'Subscribed to vdb_map_visualization: {topic}')

            if topic not in self._subscribed_ray_groups:
                m = self._ray_groups_pattern.match(topic)
                if m and 'visualization_msgs/msg/MarkerArray' in type_list:
                    name = m.group(1)
                    out_topic = f'/gcs/{name}/ray_groups_viz'
                    self._ray_groups_pubs[name] = self.create_publisher(
                        MarkerArray, out_topic, 10)
                    self.create_subscription(
                        MarkerArray, topic,
                        lambda msg, n=name: self._ray_groups_callback(msg, n),
                        10,
                    )
                    self._subscribed_ray_groups.add(topic)
                    self.get_logger().info(f'Subscribed to ray_groups_viz: {topic}')

            if topic not in self._subscribed_voxels:
                m = self._voxels_pattern.match(topic)
                if m and 'sensor_msgs/msg/PointCloud2' in type_list:
                    name, suffix = m.group(1), m.group(2)
                    out_topic = f'/rayfronts_debug/{name}/{suffix}'
                    # 'all' passes through as PointCloud2 (styled by the
                    # rendered layout — its name is static). Per-query topic
                    # names embed the mission query, so they carry their
                    # styling in-data instead: scene-entity cubes (outlined by
                    # Foxglove), or CUBE_LIST markers without foxglove_msgs.
                    if suffix == 'voxels_sim/all':
                        out_type = PointCloud2
                    else:
                        out_type = SceneUpdate if SceneUpdate else Marker
                    self._voxels_pubs[topic] = self.create_publisher(
                        out_type, out_topic, 5)
                    self.create_subscription(
                        PointCloud2, topic,
                        lambda msg, n=name, t=topic: self._voxels_callback(msg, n, t),
                        SENSOR_QOS,
                    )
                    self._subscribed_voxels.add(topic)
                    self.get_logger().info(
                        f'Subscribed to rayfronts voxels: {topic} -> {out_topic}')

            if topic not in self._subscribed_rgb_voxels:
                m = self._rgb_voxels_pattern.match(topic)
                if m and 'sensor_msgs/msg/PointCloud2' in type_list:
                    name = m.group(1)
                    out_topic = f'/rayfronts_debug/{name}/voxel_rgb'
                    self._voxels_pubs[topic] = self.create_publisher(
                        PointCloud2, out_topic, 5)
                    self.create_subscription(
                        PointCloud2, topic,
                        lambda msg, n=name, t=topic: self._rgb_voxels_callback(msg, n, t),
                        SENSOR_QOS,
                    )
                    self._subscribed_rgb_voxels.add(topic)
                    self.get_logger().info(
                        f'Subscribed to rayfronts rgb voxels: {topic} -> {out_topic}')

            if topic not in self._subscribed_search:
                m = self._search_pattern.match(topic)
                if m:
                    name, suffix = m.group(1), m.group(2)
                    msg_type = SEARCH_OVERLAY_TOPICS[suffix]
                    if _SEARCH_OVERLAY_TYPE_NAMES[msg_type] in type_list:
                        out_topic = f'/gcs/{name}{suffix}'
                        # Latched, like the planner's own: a Foxglove panel
                        # that connects mid-run draws the current map at once.
                        self._search_pubs[topic] = self.create_publisher(
                            msg_type, out_topic, LATCHED_QOS)
                        self.create_subscription(
                            msg_type, topic,
                            lambda msg, n=name, t=topic: self._search_overlay_callback(msg, n, t),
                            SEARCH_OVERLAY_SUB_QOS,
                        )
                        self._subscribed_search.add(topic)
                        self.get_logger().info(
                            f'Subscribed to search_planner overlay: {topic} -> {out_topic}')

    def _search_overlay_callback(self, msg, robot_name: str, src_topic: str):
        """Republish a search_planner overlay shifted into the GCS `map`.

        Pure translation by this robot's map origin (its takeoff point in
        global ENU) — the robot's `map` and the GCS's `map` share axes and
        differ only by that offset. Held back, not passed through, until the
        origin is known: an untranslated grid on the canvas is exactly the
        "everything is at the world origin" picture this exists to prevent.
        """
        pub = self._search_pubs.get(src_topic)
        if pub is None:
            return
        boot = self._gps_boot.get(robot_name)
        if boot is None:
            self.get_logger().warn(
                f'awaiting GPS + odom for {robot_name} to place {src_topic}',
                throttle_duration_sec=10.0)
            return
        bx, by, bz = float(boot[0]), float(boot[1]), float(boot[2])
        if isinstance(msg, MarkerArray):
            out = transform_marker_array(msg, bx, by, bz)
        elif isinstance(msg, PointCloud2):
            out = translate_point_cloud2(msg, bx, by, bz)
        elif isinstance(msg, OccupancyGrid):
            out = translate_occupancy_grid(msg, bx, by, bz)
        else:
            return
        pub.publish(out)

    def _update_map_origin(self, robot_name: str, enu_xyz) -> None:
        """Refine `_gps_boot[robot]` = enu(fix) - odom, then lock it.

        The first fix alone is kept as the estimate until odom is available
        (the historical behaviour, and still right when the drone has not
        moved). Once both are in hand the difference IS the map origin
        regardless of where the drone is now, so a GCS started mid-flight
        agrees with one started before takeoff. Averaged over
        MAP_ORIGIN_SAMPLES pairs, then frozen — a moving origin would make
        every overlay swim.
        """
        samples = self._boot_samples.get(robot_name)
        if samples is None:
            return                           # locked (or never started)
        od = self._odom_xyz.get(robot_name)
        if od is None:
            return
        samples.append((enu_xyz[0] - od[0], enu_xyz[1] - od[1], enu_xyz[2] - od[2]))
        n = len(samples)
        dx = sum(s[0] for s in samples) / n
        dy = sum(s[1] for s in samples) / n
        dz = sum(s[2] for s in samples) / n
        self._gps_boot[robot_name] = (dx, dy, dz)
        if n >= MAP_ORIGIN_SAMPLES:
            spread = max(max(abs(s[0] - dx), abs(s[1] - dy)) for s in samples)
            self._boot_samples[robot_name] = None
            self.get_logger().info(
                f'{robot_name} map origin locked at ENU ({dx:.2f}, {dy:.2f}, '
                f'{dz:.2f}) [{n} samples, spread {spread:.2f} m]')
            self._publish_map_origin_of(robot_name, dx, dy, dz)
            if robot_name in self._vdb_markers:
                self._vdb_global[robot_name] = _translate_marker(
                    self._vdb_markers[robot_name], dx, dy, dz)

    def _publish_map_origin_of(self, robot_name: str, dx: float, dy: float,
                               dz: float) -> None:
        """Latch this robot's map origin (its takeoff point, global ENU) on
        /gcs/<robot>/map_origin, so a recording can place the robot's raw
        map-frame topics: world = local + this point."""
        pub = self._origin_pubs.get(robot_name)
        if pub is None:
            pub = self.create_publisher(
                PointStamped, f'/gcs/{robot_name}/map_origin', LATCHED_QOS)
            self._origin_pubs[robot_name] = pub
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x, msg.point.y, msg.point.z = float(dx), float(dy), float(dz)
        pub.publish(msg)

    def _try_lock_ground(self, robot_name: str) -> None:
        """Lock _alt_ground = msl - odom_z for the first robot with both signals.

        Locking once (no recompute) keeps visualizations stable; using
        msl - odom_z makes the ground reference invariant to whether the
        triggering robot is on the ground or already in the air.
        """
        if self._alt_ground is not None:
            return
        msl = self._gps_msl.get(robot_name)
        agl = self._odom_z.get(robot_name)
        if msl is None or agl is None:
            return
        ground = msl - agl
        self._alt_ground = ground
        self._alt_ground_anchor = robot_name
        self._ground_msl_pub.publish(Float64(data=float(ground)))
        self.get_logger().info(
            f'ground reference locked: {robot_name} at MSL {ground:.2f} m '
            f'(msl={msl:.2f}, agl={agl:.2f})')

    def _gps_callback(self, msg: NavSatFix, robot_name: str):
        if msg.status.status < 0:
            return
        # Cache the raw MSL before any ENU conversion so _try_lock_ground can
        # compute ground = msl - odom_z without re-reading the message.
        self._gps_msl[robot_name] = msg.altitude
        self._try_lock_ground(robot_name)
        if self._alt_ground is None:
            # Still waiting for an odom message from any robot — defer.
            self.get_logger().warn(
                'awaiting odom from any robot to anchor ground plane',
                throttle_duration_sec=10.0)
            return
        pos = gps_to_enu(msg.latitude, msg.longitude, msg.altitude, self._alt_ground)
        self._gps_positions[robot_name] = pos
        if robot_name not in self._gps_boot:
            # First estimate of this robot's map origin: where it is now.
            # Exact only if it has not moved yet — refined below.
            self._gps_boot[robot_name] = pos
            self._boot_samples.setdefault(robot_name, [])
            if robot_name in self._vdb_markers:
                bx, by, bz = pos
                self._vdb_global[robot_name] = _translate_marker(
                    self._vdb_markers[robot_name], bx, by, bz)
        self._update_map_origin(robot_name, pos)

        # Re-publish on /gcs/<robot>/location with frame_id='map' so the
        # Foxglove Map panel will accept it as a location source. lat/lon
        # are passed through unchanged.
        if robot_name not in self._location_pubs:
            self._location_pubs[robot_name] = self.create_publisher(
                NavSatFix, f'/gcs/{robot_name}/location', 10)
        out = NavSatFix()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'
        out.status = msg.status
        out.latitude = msg.latitude
        out.longitude = msg.longitude
        out.altitude = msg.altitude
        out.position_covariance = msg.position_covariance
        out.position_covariance_type = msg.position_covariance_type
        self._location_pubs[robot_name].publish(out)

    def _odom_callback(self, msg: Odometry, robot_name: str):
        o = msg.pose.pose.orientation
        self._orientations[robot_name] = (o.x, o.y, o.z, o.w)
        # odom is in 'map' frame anchored to the robot's takeoff point, so
        # position.z is altitude above takeoff (≈ AGL of the actual ground).
        p = msg.pose.pose.position
        self._odom_z[robot_name] = p.z
        self._odom_xyz[robot_name] = (p.x, p.y, p.z)
        self._try_lock_ground(robot_name)

    def _traj_callback(self, msg: MarkerArray, robot_name: str):
        self._trajectories[robot_name] = msg

    def _plan_callback(self, msg: Path, robot_name: str):
        self._global_plans[robot_name] = msg

    def _vdb_callback(self, msg: Marker, robot_name: str):
        self._vdb_markers[robot_name] = msg
        boot = self._gps_boot.get(robot_name)
        if boot is not None:
            bx, by, bz = boot
            self._vdb_global[robot_name] = _translate_marker(msg, bx, by, bz)

    def _ray_groups_callback(self, msg: MarkerArray, robot_name: str):
        """Translate each marker into the global frame via the robot's boot offset."""
        boot = self._gps_boot.get(robot_name)
        out = MarkerArray()
        if boot is None:
            out.markers = list(msg.markers)
        else:
            bx, by, bz = boot
            ns_prefix = f'{robot_name}_'
            for m in msg.markers:
                tm = _translate_marker(m, bx, by, bz)
                tm.header.stamp = m.header.stamp
                tm.ns = ns_prefix + m.ns
                tm.id = m.id
                tm.type = m.type
                tm.action = m.action
                tm.text = m.text
                # _translate_marker preserves pose; offset it for non-points markers.
                if not m.points:
                    tm.pose.position.x = m.pose.position.x + bx
                    tm.pose.position.y = m.pose.position.y + by
                    tm.pose.position.z = m.pose.position.z + bz
                    tm.pose.orientation = m.pose.orientation
                out.markers.append(tm)
        pub = self._ray_groups_pubs.get(robot_name)
        if pub is not None:
            pub.publish(out)

    def _voxel_high_color(self, robot_name: str):
        """High end of this robot's voxel gradient: fluorescent wheel color,
        with a per-robot override (see VOXEL_HIGH_OVERRIDE)."""
        m = re.search(r'(\d+)$', robot_name)
        idx = int(m.group(1)) if m else 1
        if idx in VOXEL_HIGH_OVERRIDE:
            return VOXEL_HIGH_OVERRIDE[idx]
        return fluorescent(ROBOT_COLORS[(idx - 1) % len(ROBOT_COLORS)])

    def _rgb_voxels_callback(self, msg: PointCloud2, robot_name: str, src_topic: str):
        """Republish a bridged rayfronts rgb voxel cloud in the global ENU 'map'
        frame under /rayfronts_debug/<robot>/voxel_rgb.

        Unlike voxels_sim (raw RDF), voxel_rgb comes from the rayfronts ROS
        visualizer, which already applies rdf2flu at the source — so apply ONLY
        this robot's GPS boot-ENU translation (no rotation). The packed 'rgb'
        field passes through untouched so Foxglove colours it by rgb."""
        boot = self._gps_boot.get(robot_name)
        pub = self._voxels_pubs.get(src_topic)
        if pub is None:
            return
        if boot is None:
            self.get_logger().warn(
                f'awaiting GPS boot for {robot_name} to place rgb voxels',
                throttle_duration_sec=10.0)
            return
        bx, by, bz = float(boot[0]), float(boot[1]), float(boot[2])
        out = transform_point_cloud2(msg, bx, by, bz)  # identity rot: translate only
        out.header.stamp = self.get_clock().now().to_msg()
        pub.publish(out)

    def _voxels_callback(self, msg: PointCloud2, robot_name: str, src_topic: str):
        """Republish a bridged rayfronts voxel heatmap in the global ENU 'map' frame.

        The bridged cloud is in RDF (camera-optical), robot-local. Apply the same
        transform gossip_node uses for gossiped voxels — RDF→FLU rotation then
        this robot's GPS boot-ENU translation — and republish under
        /rayfronts_debug/<robot>/voxels_sim/... .

        voxels_sim/all passes through as a PointCloud2 (xyz moved, sim_K fields
        untouched) so the rendered layout can color it by any query's
        similarity. Per-query clouds become 0.5 m cubes pre-colored on a
        dark-purple → per-robot-color gradient over [voxel_score_threshold, 1.0],
        capped to the top max_debug_voxels by sim. The entity id is distinct per
        robot AND per query so enabling one cloud never replaces another's.

        Note: every path here loops per point in Python; these clouds can be
        large and there is one per query, so this is the heaviest handler here.
        If GCS CPU becomes a concern, throttle per topic or lower the cap."""
        boot = self._gps_boot.get(robot_name)
        pub = self._voxels_pubs.get(src_topic)
        if pub is None:
            return
        if boot is None:
            # No GPS boot offset yet → can't place the cloud globally. Skip;
            # rayfronts keeps publishing, so we resume once the boot ENU is known.
            self.get_logger().warn(
                f'awaiting GPS boot for {robot_name} to place voxels',
                throttle_duration_sec=10.0)
            return
        bx, by, bz = float(boot[0]), float(boot[1]), float(boot[2])
        now = self.get_clock().now().to_msg()
        if src_topic.endswith('/voxels_sim/all'):
            out = transform_point_cloud2(msg, bx, by, bz, q=RDF_TO_FLU_QUAT)
            out.header.stamp = now
            pub.publish(out)
            return
        color = self._voxel_high_color(robot_name)
        # Unique per robot AND per query (the topic segment after voxels_sim/,
        # e.g. 'q0_red_building') so enabling one cloud never replaces another's.
        query = src_topic.rsplit('/voxels_sim/', 1)[-1]
        ent_id = f'{robot_name}_voxels_{query}'
        if SceneUpdate:
            res = voxel_sim_cloud_to_scene_update(
                msg, bx, by, bz, q=RDF_TO_FLU_QUAT,
                entity_id=ent_id, stamp=now, robot_color=color,
                sim_min=self._voxel_sim_min, scale=0.5,
                max_voxels=self._max_debug_voxels)
        else:
            res = voxel_sim_cloud_to_cube_marker(
                msg, bx, by, bz, q=RDF_TO_FLU_QUAT,
                ns=ent_id, stamp=now, robot_color=color,
                sim_min=self._voxel_sim_min, scale=0.5,
                max_voxels=self._max_debug_voxels)
        if res is None:
            return
        out, total = res
        if self._max_debug_voxels and total > self._max_debug_voxels:
            self.get_logger().info(
                f'{src_topic}: showing top {self._max_debug_voxels} of '
                f'{total} voxels by sim', throttle_duration_sec=30.0)
        pub.publish(out)

    def _publish_map_origin(self):
        m = NavSatFix()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.status.status = 0  # STATUS_FIX
        m.status.service = 1  # SERVICE_GPS
        m.latitude = ORIGIN_LAT
        m.longitude = ORIGIN_LON
        m.altitude = 0.0
        self._origin_pub.publish(m)

    def _specs_ready(self) -> bool:
        """All three sim-published specs (coverage, center_x, center_y) have
        been received or set explicitly."""
        return (self._overhead_coverage_m is not None
                and self._overhead_center_x_m is not None
                and self._overhead_center_y_m is not None)

    def _try_build_pending(self):
        """If we have a pending image and all specs, finish the build."""
        if self._ground_published or self._pending_image is None:
            return
        if not self._specs_ready():
            return
        img = self._pending_image
        self._pending_image = None
        self._build_sim_ground_marker(img)

    def _on_overhead_spec(self, msg: Float32):
        """Cache the sim-published coverage_m."""
        if self._ground_published:
            return
        if self._overhead_coverage_m is None and float(msg.data) > 0:
            self._overhead_coverage_m = float(msg.data)
            self.get_logger().info(
                f'Sim overhead spec: coverage_m = {self._overhead_coverage_m:.1f}')
        self._try_build_pending()

    def _on_overhead_center_x(self, msg: Float32):
        """Cache the sim-published camera center X (world meters)."""
        if self._ground_published:
            return
        if self._overhead_center_x_m is None:
            self._overhead_center_x_m = float(msg.data)
            self.get_logger().info(
                f'Sim overhead spec: center_x_m = {self._overhead_center_x_m:.1f}')
        self._try_build_pending()

    def _on_overhead_center_y(self, msg: Float32):
        """Cache the sim-published camera center Y (world meters)."""
        if self._ground_published:
            return
        if self._overhead_center_y_m is None:
            self._overhead_center_y_m = float(msg.data)
            self.get_logger().info(
                f'Sim overhead spec: center_y_m = {self._overhead_center_y_m:.1f}')
        self._try_build_pending()

    def _on_overhead_image(self, msg: Image):
        """Cache or process the first valid sim-overhead Image."""
        if self._ground_published:
            return
        if not self._specs_ready():
            # Hold onto the image until all specs arrive.
            self._pending_image = msg
            return
        self._build_sim_ground_marker(msg)

    def _build_sim_ground_marker(self, msg: Image):
        """Build and publish the TRIANGLE_LIST sim_ground marker, then tear
        down both overhead subscriptions so we stop pulling traffic."""
        if self._ground_published:
            return

        # Decode raw Image: encoding can be rgb8 / rgba8 / bgr* — Isaac Sim's
        # RGB camera helper publishes rgba8.
        try:
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            arr = arr.reshape((msg.height, msg.width, -1))
            enc = (msg.encoding or '').lower()
            if enc.startswith('rgba'):
                pix = arr[..., :3]
            elif enc.startswith('bgra'):
                pix = arr[..., [2, 1, 0]]
            elif enc.startswith('bgr'):
                pix = arr[..., ::-1]
            else:
                pix = arr[..., :3]
        except Exception as e:
            self.get_logger().warn(f'Failed to decode overhead Image: {e}')
            return

        coverage = float(self._overhead_coverage_m)
        cx = float(self._overhead_center_x_m or 0.0)
        cy = float(self._overhead_center_y_m or 0.0)
        # Grid resolution: explicit override > coverage × density (capped).
        if self._overhead_grid_n_override is not None:
            N = self._overhead_grid_n_override
        else:
            N = int(round(coverage * self._overhead_grid_per_m))
        N = max(8, min(N, int(self._overhead_max_grid_n)))

        # Downsample to N×N grid via simple nearest-neighbor stride.
        H, W = pix.shape[:2]
        ys = (np.linspace(0, H - 1, N)).astype(np.int32)
        xs = (np.linspace(0, W - 1, N)).astype(np.int32)
        pixels = pix[ys[:, None], xs[None, :], :]  # (N, N, 3)

        cell = coverage / N

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'sim_ground'
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0  # per-vertex colors override .color, but a must be set

        # Build the grid. Image row 0 is the +Y edge (image-up = world-north).
        # World x = cx + (col / N - 0.5) * coverage;
        # world y = cy + (0.5 - row / N) * coverage.
        for row in range(N):
            y_top = cy + (0.5 - row / N) * coverage
            y_bot = y_top - cell
            for col in range(N):
                x_left = cx + (col / N - 0.5) * coverage
                x_right = x_left + cell
                r, g, b = pixels[row, col]
                color = ColorRGBA(r=r/255.0, g=g/255.0, b=b/255.0,
                                  a=self._ground_alpha)

                p_tl = Point(x=x_left,  y=y_top, z=-0.01)
                p_tr = Point(x=x_right, y=y_top, z=-0.01)
                p_bl = Point(x=x_left,  y=y_bot, z=-0.01)
                p_br = Point(x=x_right, y=y_bot, z=-0.01)

                # CCW winding (looking down +Z): top-left, bottom-left, bottom-right
                # then top-left, bottom-right, top-right.
                marker.points.extend([p_tl, p_bl, p_br,
                                      p_tl, p_br, p_tr])
                marker.colors.extend([color] * 6)

        self._ground_pub.publish(marker)
        self._ground_published = True
        # Stop pulling raw image bytes (and the now-redundant specs) off the wire.
        for sub_attr in ('_overhead_sub', '_overhead_spec_sub',
                         '_overhead_cx_sub', '_overhead_cy_sub'):
            try:
                self.destroy_subscription(getattr(self, sub_attr))
            except Exception:
                pass
        self.get_logger().info(
            f'Published sim_ground marker: {N}x{N} cells, '
            f'{len(marker.points)} verts, coverage {coverage:.1f} m, '
            f'center ({cx:.1f}, {cy:.1f}) m')

    def _publish_markers(self):
        if not self._gps_positions:
            return

        array = MarkerArray()
        now = self.get_clock().now().to_msg()
        lifetime = Duration(sec=1, nanosec=0)

        for i, robot_name in enumerate(sorted(self._gps_positions.keys())):
            x, y, z = self._gps_positions[robot_name]
            orientation = self._orientations.get(robot_name)

            mesh = Marker()
            mesh.header.frame_id = 'map'
            mesh.header.stamp = now
            mesh.ns = 'robot_meshes'
            mesh.id = i * 10
            mesh.type = Marker.MESH_RESOURCE
            mesh.action = Marker.ADD
            mesh.mesh_resource = 'package://robot_descriptions/iris/meshes/base_link_body_body.obj'
            mesh.pose.position.x = x
            mesh.pose.position.y = y
            mesh.pose.position.z = z
            if orientation:
                qx, qy, qz, qw = multiply_quaternions(orientation, AXIS_CORRECTION)
            else:
                qx, qy, qz, qw = AXIS_CORRECTION
            mesh.pose.orientation.x = qx
            mesh.pose.orientation.y = qy
            mesh.pose.orientation.z = qz
            mesh.pose.orientation.w = qw
            mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0
            mesh.color.r = 0.6
            mesh.color.g = 0.6
            mesh.color.b = 0.6
            mesh.color.a = 1.0
            mesh.lifetime = lifetime
            array.markers.append(mesh)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = now
            label.ns = 'robot_labels'
            label.id = i * 10 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 1.0
            label.pose.orientation.w = 1.0
            label.scale.z = 0.2
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = robot_name
            label.lifetime = lifetime
            array.markers.append(label)

            axes = [
                ((1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                ((0.0, 1.0, 0.0), (0.0, 1.0, 0.0)),
                ((0.0, 0.0, 1.0), (0.0, 0.0, 1.0)),
            ]
            q = orientation if orientation else (0.0, 0.0, 0.0, 1.0)
            axis_len = 0.6
            for j, (unit_vec, axis_color) in enumerate(axes):
                tip = rotate_vector(unit_vec, q)
                arrow = Marker()
                arrow.header.frame_id = 'map'
                arrow.header.stamp = now
                arrow.ns = f'{robot_name}_axes'
                arrow.id = i * 10 + 2 + j
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                start = Point(x=x, y=y, z=z)
                end = Point(x=x + tip[0] * axis_len,
                            y=y + tip[1] * axis_len,
                            z=z + tip[2] * axis_len)
                arrow.points = [start, end]
                arrow.scale.x = 0.04
                arrow.scale.y = 0.08
                arrow.scale.z = 0.0
                arrow.color.r = axis_color[0]
                arrow.color.g = axis_color[1]
                arrow.color.b = axis_color[2]
                arrow.color.a = 1.0
                arrow.lifetime = lifetime
                array.markers.append(arrow)

            boot = self._gps_boot.get(robot_name)

            traj = self._trajectories.get(robot_name)
            if traj is not None and boot is not None:
                bx, by, bz = boot
                transformed = transform_marker_array(traj, bx, by, bz)
                traj_color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
                for k, m in enumerate(transformed.markers):
                    m.ns = f'{robot_name}_traj'
                    m.id = i * 10000 + k
                    m.header.stamp = now
                    m.lifetime = lifetime
                    m.color.r = traj_color[0]
                    m.color.g = traj_color[1]
                    m.color.b = traj_color[2]
                    m.color.a = 1.0
                    m.colors = []
                    if m.type in (Marker.LINE_STRIP, Marker.LINE_LIST):
                        m.scale.x = 0.30
                    needs_points = m.type in (Marker.LINE_STRIP, Marker.LINE_LIST,
                                              Marker.POINTS, Marker.ARROW)
                    if needs_points and len(m.points) == 0:
                        continue
                    array.markers.append(m)

            plan = self._global_plans.get(robot_name)
            if plan is not None and boot is not None:
                bx, by, bz = boot
                poses = plan.poses
                n_poses = len(poses)
                if n_poses >= 2:
                    color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
                    xyz = np.array(
                        [(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
                         for ps in poses], dtype=np.float64)
                    xyz[:, 0] += bx
                    xyz[:, 1] += by
                    xyz[:, 2] += bz
                    line = Marker()
                    line.header.frame_id = 'map'
                    line.header.stamp = now
                    line.ns = f'{robot_name}_global_plan'
                    line.id = i * 10000 + 9999
                    line.type = Marker.LINE_STRIP
                    line.action = Marker.ADD
                    line.pose.orientation.w = 1.0
                    line.scale.x = 0.30
                    line.color.r = float(np.sqrt(color[0]))
                    line.color.g = float(np.sqrt(color[1]))
                    line.color.b = float(np.sqrt(color[2]))
                    line.color.a = 1.0
                    line.lifetime = Duration(sec=2, nanosec=0)
                    line.points = [Point(x=float(r[0]), y=float(r[1]), z=float(r[2]))
                                   for r in xyz]
                    array.markers.append(line)

            vdb = self._vdb_global.get(robot_name)
            if vdb is not None:
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = now
                m.ns = f'{robot_name}_vdb'
                m.id = i * 10000 + 9998
                m.type = vdb.type
                m.action = vdb.action
                m.pose = vdb.pose
                m.scale = vdb.scale
                m.color = vdb.color
                m.colors = vdb.colors
                m.points = vdb.points  # pre-translated, safe to share read-only
                m.lifetime = Duration(sec=2, nanosec=0)
                array.markers.append(m)

        self._pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = FoxgloveVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

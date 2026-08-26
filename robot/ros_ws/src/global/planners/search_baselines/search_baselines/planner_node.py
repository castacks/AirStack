"""Co-NavGPT2 as an AirStack global planner.

This is the ROS 2 wrapper around the UPSTREAM implementation (vendored under
`conavgpt2/vendor/`, see VENDORED.md). Upstream builds its own occupancy +
explored map and its own frontiers straight from RGBD — no semantic map, no
rayfronts — merges every robot's cloud into one grid, renders a top view with one
numbered frontier per candidate image, and asks a VLM which frontier each robot
should take.

Three things are ours rather than upstream's, and only these three:

  * Actuation. Upstream PUBs `speedctl` strings over ZMQ to hardcoded Unitree Go2
    IPs. Here the selected goal becomes a `task_msgs/action/NavigateTask` goal on
    `/{robot}/tasks/navigate`, so droan_gl flies it with obstacle avoidance, and
    the same path is published on `/{robot}/global_plan` for Foxglove/gossip.
  * Visualisation. Upstream runs an open3d GUI window in a second process. That
    cannot exist in a headless container, so the top view (the very image the VLM
    is shown) is published as `sensor_msgs/Image` instead.
  * The VLM endpoint. Upstream constructs `OpenAI()` at import against
    api.openai.com. Here base_url/model/api_key are parameters pointing at any
    OpenAI-compatible server (default: a locally served Qwen2.5-VL).

Everything between the pixels and the goal point is upstream's code.
"""

import json
import math
import os
import signal
import threading
import time
from argparse import Namespace

import numpy as np
import rclpy
import rclpy.executors
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image, NavSatFix, PointCloud2, PointField
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

import message_filters

try:
    from coordination_bringup.frame_utils import gps_to_enu
except ImportError:   # same flat-earth constants the rest of the stack uses
    def gps_to_enu(lat, lon, alt, origin_lat=38.736832, origin_lon=-9.137977,
                   origin_alt=90.0):
        return ((lon - origin_lon) * 111320.0 * math.cos(math.radians(origin_lat)),
                (lat - origin_lat) * 111320.0,
                alt - origin_alt)

try:
    from task_msgs.action import NavigateTask
    _TASK_MSGS_OK = True
except ImportError as _exc:  # built by colcon, absent from the bare image
    NavigateTask = None
    _TASK_MSGS_OK = False
    _TASK_MSGS_ERR = _exc

try:
    import cv2
    import open3d as o3d

    from search_baselines import vlm_client
    from search_baselines import itm_client as vlfm_scorer
    from search_baselines import lawnmower as lm
    from search_baselines import sector as sect
    from search_baselines.value_map import ValueMap
    from search_baselines.visit_cost import VisitCost
    from search_baselines.voxel_map import VoxelMap, OCCUPIED as VOX_OCCUPIED
    from search_baselines.airstack_agent import AirStackAgent, o3d_xz_to_map_xy
    from conavgpt2.vendor import system_prompt
    from conavgpt2.vendor.arguments import get_args
    from conavgpt2.vendor.utils import chat_utils
    from conavgpt2.vendor.utils import explored_map_utils
    from conavgpt2.vendor.utils import visualization as vu
    from conavgpt2.vendor.utils.explored_map_utils import Global_Map_Proc, detect_frontier
    _VENDOR_OK = True
except ImportError as _exc:
    _VENDOR_OK = False
    _VENDOR_ERR = _exc


# FLU body -> RDF camera optical. Columns are the optical axes written in body
# coordinates: right = -y_body, down = -z_body, forward = +x_body.
R_FLU_TO_RDF = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])

DEFAULT_CLASSES = [
    'person', 'car', 'truck', 'house', 'building', 'tree', 'smoke', 'fire',
]


def _quat_to_rot(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class _NavGoal:
    """Per-robot NavigateTask bookkeeping. droan_gl rejects a second goal while one
    is active, so a new endpoint has to cancel the old goal before it can be sent."""

    def __init__(self):
        self.handle = None
        self.result_future = None
        self.endpoint = None
        self.sent_at = 0.0


class CoNavGPT2Node(Node):

    def __init__(self):
        super().__init__('search_planner')

        # ── robots and topics ─────────────────────────────────────────────────
        # Upstream's ros_multi_nav.py is one process for the whole team, because a
        # single merged grid and a single VLM call are the method. Kept that way:
        # this node subscribes to every robot and commands every robot.
        # AirStack runs one container per robot on its OWN ROS_DOMAIN_ID, so a
        # single process normally sees exactly one robot. num_robots > 1 is only
        # meaningful where the other robots' sensor topics are actually visible
        # (a bridged domain); see README.
        self._num_robots = int(self._p('num_robots', int(os.environ.get('NUM_ROBOTS', '1'))))
        names = [n for n in self._p('robot_names', ['']) if n]
        if not names:
            template = self._p('robot_name_template', 'robot_{i}')
            base = int(self._p('robot_index_base', 1))
            names = [template.format(i=base + k) for k in range(self._num_robots)]
            if self._num_robots == 1 and os.environ.get('ROBOT_NAME'):
                names = [os.environ['ROBOT_NAME']]
        self._robots = names
        self._num_robots = len(self._robots)

        self._rgb_tpl = self._p(
            'rgb_topic_template', '/{robot}/sensors/front_stereo/left/image_rect')
        self._depth_tpl = self._p(
            'depth_topic_template',
            '/{robot}/sensors/front_stereo/left/depth_ground_truth')
        self._info_tpl = self._p(
            'camera_info_topic_template',
            '/{robot}/sensors/front_stereo/left/camera_info')
        self._odom_tpl = self._p(
            'odom_topic_template', '/{robot}/odometry_conversion/odometry')
        self._nav_tpl = self._p('navigate_action_template', '/{robot}/tasks/navigate')
        self._plan_tpl = self._p('global_plan_topic_template', '/{robot}/global_plan')
        self._agent_image_tpl = self._p(
            'agent_image_topic_template', '/{robot}/search/agent_image')
        # Templates, formatted against the FIRST robot: one node drives the
        # whole team, so its map and round table are the team's, but they still
        # live under a robot namespace so the rest of the stack (dds_router,
        # keepalive, Foxglove layouts) can address them the usual way.
        _r0 = self._robots[0]
        self._map_image_topic = self._p(
            'map_image_topic', '/{robot}/search/map_image').format(robot=_r0)
        self._vlm_image_topic = self._p(
            'vlm_image_topic', '/{robot}/search/vlm_prompt_image').format(robot=_r0)
        # The same grid and the same frontiers as the rendered map image, but as
        # real message types, so Foxglove's Map/3D panels can draw them over the
        # sim ground instead of showing a picture of them.
        self._occupancy_topic = self._p(
            'occupancy_topic', '/{robot}/occupancy').format(robot=_r0)
        self._frontier_topic = self._p(
            'frontier_marker_topic', '/{robot}/frontiers').format(robot=_r0)
        self._voxel_cloud_topic = self._p(
            'voxel_cloud_topic', '/{robot}/voxel_map').format(robot=_r0)
        # Cap on cubes per frontier marker. A frontier region on a coarse grid
        # can be thousands of cells; beyond a few hundred the shape is already
        # readable and the rest is bandwidth.
        self._frontier_max_cells = int(self._p('frontier_marker_max_cells', 400))
        # What the frontier marker topic draws.
        #   'centroids' just the goal point per frontier — the single point a
        #               robot is actually sent to, which is what upstream's
        #               `Goal_point` is. Least clutter.
        #   'cells'     the frontier REGION, one cube per grid cell.
        #   'both'      region plus its centroid.
        self._frontier_style = str(self._p('frontier_marker_style', 'centroids'))
        # Height the OccupancyGrid is drawn at, above the map frame's z=0. The
        # sim's ground texture is a flat plane at world z=0 and the grid is a
        # flat plane too, so at the same height they Z-FIGHT and the overlay
        # flickers between the two. Lift it clear.
        self._occupancy_z = float(self._p('occupancy_z_offset_m', 0.5))

        self._map_frame = self._p('map_frame', 'map')
        # TF is exact (it walks the real URDF chain) but the frames are bare, i.e.
        # unprefixed, so it only resolves for robots on this ROS domain. The odom
        # path is the fallback and works for any robot whose odometry is visible.
        self._pose_source = self._p('pose_source', 'tf')
        self._cam_frame_tpl = self._p('camera_optical_frame_template', 'camera_left')
        self._navsat_tpl = self._p(
            'navsat_topic_template',
            '/{robot}/interface/mavros/global_position/raw/fix')
        # DOWNWARD pitch of the camera relative to what the URDF models, in
        # radians. It describes a PHYSICAL MOUNT, not a preference: set it
        # without a matching mount and every point unprojects to the wrong
        # bearing, silently.
        #
        # The URDF chain is plain FLU->RDF with no pitch, so this is the whole
        # mount rotation on the odometry path AND the correction TF needs on the
        # tf path — TF walks that same pitchless chain, so a camera the launch
        # script tilted is a camera TF does not know about.
        #
        # It therefore defaults to ZED_PITCH_DEG, the same environment variable
        # the launch script reads to tilt the camera, so the two cannot drift
        # apart. An explicit parameter still wins.
        self._camera_pitch = float(self._p('camera_pitch_rad', 0.0))
        if self._camera_pitch == 0.0:
            try:
                self._camera_pitch = math.radians(
                    float(os.environ.get('ZED_PITCH_DEG', '0') or 0.0))
            except ValueError:
                self._camera_pitch = 0.0
        # base_link -> camera_left from iris_with_sensors.pegasus.robot.urdf, with
        # the +90 deg base_link->body yaw already folded in. Only used when TF is
        # unavailable; the rotation is the plain FLU->RDF, which that chain equals.
        self._camera_offset = [float(v) for v in self._p(
            'camera_offset_xyz', [0.209, 0.060, -0.037])]
        # Robots do NOT share a map origin: each robot's 'map' is anchored on its
        # own boot GPS. Merging several robots into one grid therefore has to
        # happen in global ENU, reached by adding each robot's boot_enu offset.
        self._frame_mode = self._p('frame_mode', 'local')
        # Centre of the occupancy grid, in the merge frame. In 'local' mode the
        # merge frame is the robot's own map and 0,0 (its takeoff point) is right.
        # In 'global_enu' it is sim-world ENU, where 0,0 is the GPS origin and the
        # drones spawn hundreds of metres away — set this to the middle of the
        # search area or the whole cloud is cropped away.
        self._map_origin_xy = [float(v) for v in self._p('map_origin_xy', [0.0, 0.0])]

        # ── sensor conditioning ───────────────────────────────────────────────
        self._sync_slop = float(self._p('sync_slop_s', 0.10))
        self._sync_queue = int(self._p('sync_queue_size', 10))
        # 1.0 for 32FC1 metres, 0.001 for 16UC1 millimetres (upstream's RealSense).
        self._depth_scale = float(self._p('depth_encoding_scale', 1.0))
        self._depth_min_m = float(self._p('depth_min_m', 0.5))
        self._depth_max_m = float(self._p('depth_max_m', 60.0))
        self._depth_border_px = int(self._p('depth_border_px', 0))
        # Upstream's render composites the FPV at exactly 640x480; intrinsics are
        # rescaled to match, so this is a resize, not a crop.
        self._frame_w = int(self._p('frame_width', 640))
        self._frame_h = int(self._p('frame_height', 480))

        # ── map geometry ──────────────────────────────────────────────────────
        # Upstream ships map_size_cm=2400 — a 24x24 m map, sized for a quadruped
        # in a house. Our scenes are 500-1600 m, where a 24 m map is smaller than
        # one block and every frontier is a wall of the map. Expressed in metres
        # and cells here so it can be set per scene.
        #
        # 480 cells is not arbitrary: write_number() stamps the robot markers on
        # the VLM's candidate images using 480-pixel coordinates, so the grid has
        # to be 480 across for those markers to land on the right place.
        self._map_cells = int(self._p('map_cells', 480))
        self._map_extent_m = float(self._p('map_extent_m', 500.0))
        # Explicit overrides, in upstream's own units; 0 means "derive from the
        # extent above" and keeps map_size_cm an exact multiple of the resolution.
        self._map_resolution = int(self._p('map_resolution_cm', 0))
        if self._map_resolution <= 0:
            self._map_resolution = max(
                1, int(round(self._map_extent_m * 100.0 / self._map_cells)))
        self._map_size_cm = int(self._p('map_size_cm', 0))
        if self._map_size_cm <= 0:
            self._map_size_cm = self._map_resolution * self._map_cells

        # Which slab of altitude counts as an obstacle, in map-frame metres (z is
        # AGL, because each robot's 'map' is anchored at its takeoff point on the
        # ground). Upstream's map_height_cm=130 is a 1.3 m band centred on a
        # quadruped's camera: "what I would walk into". From 15-40 m AGL that
        # distinction is useless — everything observed is below it. The aerial
        # equivalent is GROUND vs STRUCTURE: bare terrain near z=0 is free space,
        # anything standing up out of it is an obstacle. Buildings and canopy here
        # top out around 15 m, so [2, 15] separates the two. Ground must stay OUT
        # of the band: a cell that is seen but has nothing in the band is what
        # upstream calls free, and free cells are what frontiers grow from.
        # NEEDS TUNING against a real run.
        self._obst_min_z = float(self._p('obstacle_min_z_m', 2.0))
        self._obst_max_z = float(self._p('obstacle_max_z_m', 15.0))
        self._scene_voxel = float(self._p('scene_voxel_m', 0.25))
        self._scene_eps = float(self._p('scene_dbscan_eps_m', 0.0))
        self._scene_min_pts = int(self._p('scene_dbscan_min_points', 0))
        self._frontier_threshold = int(self._p('frontier_threshold_points', 20))
        # How many frontiers get offered to the VLM. Upstream hardcodes 6, which
        # is a PROMPT-SIZE bound (one BEV image each), not a mapping one — and
        # six candidates over a 240 m bench is a very different proposition from
        # six over a 1200 m plat covering 25x the area.
        #
        # 0 derives it from the extent, so a bigger world automatically gets more
        # candidates without anyone remembering to change it: 6 at 240 m, scaling
        # linearly, capped by max_frontiers_limit because every extra frontier is
        # another ~290 prompt tokens and more time in prefill.
        self._max_frontiers = int(self._p('max_frontiers', 0))
        self._max_frontiers_limit = int(self._p('max_frontiers_limit', 12))
        if self._max_frontiers <= 0:
            self._max_frontiers = int(min(
                self._max_frontiers_limit,
                max(6, round(6.0 * self._map_extent_m / 240.0))))
        self._frontier_order = str(self._p('frontier_order', 'smallest'))

        # ── where frontiers come from ────────────────────────────────────────
        #   'slab2d'  upstream Co-NavGPT2: the cloud is rasterised into ONE
        #             altitude slab and frontiers are contours of the free
        #             region. Every frontier is at the same height by
        #             construction, because the map has no height.
        #   'voxel3d' rayfronts' definition: a three-state voxel map built by
        #             RAY CARVING (empty / occupied / unobserved), and a frontier
        #             is an EMPTY voxel with unobserved neighbours. Frontiers
        #             appear at every height the camera has actually seen
        #             through, and each carries its own z.
        #
        # vdb_mapping is not used for this even though it also carves: it
        # publishes occupied voxels only (vdb_map_pointcloud /
        # vdb_map_visualization), and FREE-vs-UNOBSERVED is the whole basis of a
        # frontier. It would also require the lidar, which costs ~3x the sim
        # rate and sees 360 degrees where VLFM's value is camera-FOV bound.
        self._frontier_source = str(self._p('frontier_source', 'slab2d'))
        self._vox_size = float(self._p('voxel_size_m', 2.0))
        # Past this, stereo depth is not a surface measurement. Matches the
        # range rayfronts trusts.
        self._vox_max_range = float(self._p('voxel_max_range_m', 20.0))
        self._vox_carve_samples = int(self._p('voxel_carve_samples', 24))
        self._vox_max_points = int(self._p('voxel_max_points_per_tick', 20000))
        self._vox_neigh_r = int(self._p('voxel_neighborhood_r', 1))
        self._vox_min_unobs = int(self._p('voxel_min_unobserved', 4))
        self._vox_min_empty = int(self._p('voxel_min_empty', 2))
        self._vox_min_occ = int(self._p('voxel_min_occupied', 0))
        self._vox_subsample = int(self._p('voxel_frontier_subsample', 2))
        # Height span of the voxel MAP, in map-frame metres. Distinct from the
        # flight band: the map has to contain the ground and the buildings or
        # nothing is ever carved as empty. 0 on the max derives it from the
        # obstacle band and the altitude ceiling.
        self._vox_z_min = float(self._p('voxel_z_min_m', 0.0))
        self._vox_z_max = float(self._p('voxel_z_max_m', 0.0))
        self._vox_cloud_max_points = int(self._p('voxel_cloud_max_points', 120000))
        # WHERE FRONTIERS MAY BE, which is not the same question as where the
        # drone may fly. Conflating them is a footgun in both directions: a
        # frontier band of 3-20 m over a suburb is sensible, and flying at 3 m
        # between 10 m houses is not. 0 falls back to the flight band.
        # rayfronts frontier_vdb_map defaults.
        self._fronti_subsampling = int(self._p('frontier_subsampling', 4))
        self._fronti_min_cells = int(
            self._p('frontier_subsampling_min_cells', 10))
        self._fr_z_stratify = bool(
            self._p('frontier_z_stratify', True))
        self._fr_z_min = float(self._p('frontier_z_min_m', 0.0))
        self._fr_z_max = float(self._p('frontier_z_max_m', 0.0))
        self._voxel_map = None
        self._frontier_z = []
        self._frontier_candidates = []

        # ── VLFM (nav_mode 'vlfm') ───────────────────────────────────────────
        # Score the CURRENT view against the target, paint it over the ground the
        # camera can see, pick the frontier standing in the highest-value cell.
        # Same map, same frontiers and same actuation as nav_mode 'gpt' — only
        # the SELECTION differs, which is what makes the two comparable.
        #
        # keyframe_period_s is the whole cost/benefit: canonical VLFM scores at
        # camera rate with a ~50 ms ITM model, this scores with a generative VLM
        # at ~2.5 s a call. Too short and calls queue behind each other and every
        # score is stale; too long and the value map is nearly empty.
        self._vlfm_keyframe_s = float(self._p('vlfm_keyframe_period_s', 3.0))
        self._vlfm_max_range = float(self._p('vlfm_value_range_m', 0.0)) \
            or self._depth_max_m
        self._vlfm_min_conf = float(self._p('vlfm_min_confidence', 0.02))
        # Frontier score = value - distance_penalty_per_m * metres. 0 makes it
        # pure argmax value, which will happily send the drone across the map
        # for a marginally better cell.
        self._vlfm_dist_penalty = float(self._p('vlfm_distance_penalty', 0.002))
        self._vlfm_img_max_side = int(self._p('vlfm_image_max_side', 512))
        # ── commitment ───────────────────────────────────────────────────────
        # Argmax every tick makes the drone thrash: the value map moves as it
        # flies, two frontiers trade places, and it turns around having reached
        # neither. Same hysteresis raven_nav's frontier_behavior uses, and the
        # same three knobs:
        #   a target is held for at least lock_s,
        #   a rival must beat it by swap_margin_frac of the current score,
        #   and it is released on arrival within unlock_radius_m.
        # NOTE this is OURS, not the paper's — I do not have VLFM's reference
        # implementation to say what it does about oscillation.
        self._lock_s = float(self._p('frontier_lock_s', 6.0))
        self._swap_frac = float(self._p('frontier_swap_margin_frac', 0.35))
        self._unlock_radius = float(self._p('frontier_unlock_radius_m', 8.0))
        self._locked_goal = None
        self._locked_goal_xy = None   # map-frame xy of the committed goal
        self._locked_score = None
        self._locked_at = 0.0

        # ── frontier arm: visit cost, not a blacklist ────────────────────────
        # Blacklisting a visited frontier removes it permanently, which ends
        # coverage the moment the map is explored once. The detector is
        # imperfect, so repeated passes are wanted. Cost instead of removal.
        self._visit_weight = float(self._p('visit_cost_weight', 1.0))
        # NOT _visit_radius: that name is the target-revisit geofence set
        # further down, which would otherwise silently clobber this.
        self._visit_cost_radius = float(self._p('visit_cost_radius_m', 25.0))
        self._visit_decay = float(self._p('visit_cost_decay_per_s', 0.0))
        self._visit_cost = None
        self._last_visit_t = None

        # ── lawnmower arm ────────────────────────────────────────────────────
        self._lm_spacing = float(self._p('lawnmower_spacing_m', 0.0))
        self._lm_overlap = float(self._p('lawnmower_overlap_frac', 0.2))
        self._lm_axis = str(self._p('lawnmower_axis', 'auto'))
        self._lm_reach = float(self._p('lawnmower_reach_radius_m', 8.0))
        self._lm_path = None
        self._lm_idx = 0
        # The BLIP-2 ITM service. Separate from the generative endpoint on
        # purpose: they are different models doing different jobs, and only this
        # one is fast enough to share across a fleet.
        self._vlfm_url = str(self._p('vlfm_itm_url', 'http://localhost:8100'))
        self._vlfm_prompt = str(self._p('vlfm_prompt', ''))
        # Tile grid for scoring. [1, 1] is canonical whole-frame VLFM; anything
        # larger exists because BLIP-2 resizes to 224 and destroys small aerial
        # targets — see vlfm_scorer.score_view_tiled for the measurements.
        tiles = list(self._p('vlfm_tiles', [2, 3]) or [2, 3])
        self._vlfm_rows = max(1, int(tiles[0]))
        self._vlfm_cols = max(1, int(tiles[1] if len(tiles) > 1 else 1))

        # ── search area + altitude band ──────────────────────────────────────
        # A frontier outside the polygon is not a candidate at all. Flat list
        # [x0,y0,x1,y1,...] because ROS 2 parameters have no nested arrays.
        # Default is [0.0], not []: rclpy infers a parameter's type from its
        # default and an EMPTY list has no inferable type, so declare_parameter
        # raises. One element is the smallest thing that types as DOUBLE_ARRAY,
        # and anything under 3 points (6 numbers) is not a polygon anyway.
        poly = [float(v) for v in (self._p('search_area_xy', [0.0]) or [])]
        self._search_poly = (np.asarray(poly, dtype=float).reshape(-1, 2)
                             if len(poly) >= 6 and len(poly) % 2 == 0 else None)
        self._min_alt = float(self._p('min_altitude_agl_m', 0.0))
        self._max_alt = float(self._p('max_altitude_agl_m', 0.0))

        # ── sector ───────────────────────────────────────────────────────────
        # Grow the region by a margin, split it N ways, take this robot's slice.
        # Every robot computes the SAME partition from the same inputs and reads
        # off its own index, so no messages are exchanged — which is the whole
        # point of a static-sector arm. At n=1 this is a no-op returning the
        # padded region, so the machinery exists without changing single-drone
        # behaviour.
        self._sector_mode = str(self._p('sector_partition', 'none'))
        self._sector_axis = str(self._p('sector_axis', 'auto'))
        self._sector_pad = float(self._p('search_area_pad_m', 0.0))
        self._sector_index = int(self._p('sector_index', -1))
        if self._search_poly is not None and self._sector_mode != 'none':
            idx = (self._sector_index if self._sector_index >= 0
                   else max(0, self._robot_index_of_self()))
            try:
                sub = sect.sector_for(self._search_poly, self._num_robots, idx,
                                      mode=self._sector_mode,
                                      axis=self._sector_axis,
                                      margin_m=self._sector_pad)
                # An EMPTY sector would be read as "degenerate" downstream, and
                # this module's convention is that a degenerate polygon means
                # UNBOUNDED. That is right for a mistyped geofence and very wrong
                # here: it would silently remove this robot's confinement instead
                # of confining it. Refuse rather than fly unbounded.
                if sub is None or len(sub) < 3:
                    raise ValueError(
                        f'sector {idx} of {self._num_robots} came back empty; '
                        'refusing to run unbounded')
                self._search_poly = np.asarray(sub, dtype=float)
                self.get_logger().info(
                    f'sector {idx + 1}/{self._num_robots} ({self._sector_mode}, '
                    f'pad {self._sector_pad:.0f} m): {len(self._search_poly)} pts, '
                    f'{sect.polygon_area(self._search_poly):.0f} m2')
            except Exception as exc:
                raise RuntimeError(f'sector partition failed: {exc}') from exc
        elif self._search_poly is not None and self._sector_pad > 0.0:
            self._search_poly = np.asarray(
                sect.pad_polygon(self._search_poly, self._sector_pad), dtype=float)
        self._vlfm_scores = 0
        self._vlfm_score_fail = 0
        self._vlfm_server_s = 0.0
        self._last_keyframe = 0.0
        self._value_map = None
        self._incremental = bool(self._p('incremental_mapping', True))

        # ── planning cadence ──────────────────────────────────────────────────
        self._num_local_steps = int(self._p('num_local_steps', 25))
        self._warmup_steps = int(self._p('warmup_steps', 0))
        self._plan_period_s = float(self._p('plan_period_s', 1.0))
        # One VLM call assigns the WHOLE team, so cost scales with FRONTIER count
        # (one top-view image per candidate), not with robot count. If a call takes
        # longer than this, every robot is flying to a stale assignment — which is
        # why an overrun is warned about by name below. 0 falls back to upstream's
        # step-based cadence (every num_local_steps ticks).
        self._round_period_s = float(self._p('round_period_s', 30.0))
        self._results_dir = str(self._p('results_dir', ''))
        # SIM seconds of flight before the run is declared over, matching how
        # every other baseline here is bounded (semantic_search_task's
        # max_sim_seconds). 0 = unbounded. Wall clock is the wrong budget: RTF
        # varies with scene and card, so two runs of the same wall duration
        # cover different amounts of ground.
        self._max_sim_seconds = float(self._p('max_sim_seconds', 0.0))
        self._nav_mode = self._p('nav_mode', 'gpt')
        self._goal_name = self._p('goal_name', 'person')
        classes = [c for c in self._p('detection_classes', DEFAULT_CLASSES) if c]
        if self._goal_name not in classes:
            classes = classes + [self._goal_name]
        self._classes = classes
        self._sem_threshold = float(self._p('sem_threshold', 0.5))

        # ── actuation ─────────────────────────────────────────────────────────
        self._altitude = float(self._p('flight_altitude_m', 15.0))
        self._goal_tolerance = float(self._p('goal_tolerance_m', 6.0))
        self._nav_timeout = float(self._p('nav_goal_timeout_s', 60.0))
        self._goal_change_m = float(self._p('goal_change_threshold_m', 5.0))
        self._path_extension_m = float(self._p('path_extension_m', 2.0))
        # How droan_gl is driven. 'activator' sends ONE empty NavigateTask and
        # steers by the /global_plan topic this node already publishes every
        # tick; 'goal_per_round' sends a fresh goal carrying the path. See
        # _ensure_activator() for why the first is the default.
        self._nav_activation = self._p('nav_activation', 'activator')
        self._activator_retry_s = float(self._p('activator_retry_s', 3.0))
        # What goes on /global_plan.
        #   'waypoints' two poses — the goal, and one path_extension_m beyond it
        #              along the approach — and droan's local planner finds its
        #              own way there. This is what raven_nav publishes
        #              (frontier_behavior.py: target_waypoint + 2 m * direction),
        #              and it is the pattern the rest of this stack is tuned for.
        #   'fmm_path' upstream's full FMM waypoint list through its own
        #              occupancy grid. Higher fidelity to Co-NavGPT, which plans
        #              around the map it built — but that grid is 0.6 m per cell
        #              here and 3.5 m on the plat, so the path it produces fights
        #              droan's own avoidance at a resolution droan does better.
        self._plan_output = self._p('plan_output', 'waypoints')
        # Altitude to drop to once the target has actually been DETECTED, in m
        # AGL. 0 disables it and the drone holds flight_altitude_m for the whole
        # run, which is upstream's behaviour — Co-NavGPT is a 2D method on a
        # quadruped and has no altitude concept at all, so cruise height is
        # something this port had to invent.
        #
        # Search happens from cruise, where the camera covers ground fastest;
        # this is for the confirmation pass, when `found_goal` is set and
        # `nearest_point` gives the detection's real height. A person is on the
        # ground and a 20 m standoff is not a look.
        self._inspect_altitude = float(self._p('inspect_altitude_m', 0.0))

        # ── target INSTANCES ─────────────────────────────────────────────────
        # Upstream has no concept of one: `object_pcd` is a single merged cloud
        # of every detected goal-class point, `found_goal` is a bool, and
        # `nearest_point` is the nearest sample in that cloud. So it flies to the
        # closest instance and stops — object-GOAL navigation, where the episode
        # ends on arrival. Nothing marks anything complete, so on a scene with
        # several instances it latches onto the first and never moves on.
        #
        # This clusters the cloud into instances, remembers which have been
        # visited, and hands the planner the nearest UNVISITED one. When they run
        # out, found_goal is released and frontier exploration resumes — which is
        # what turns "find a house" into "find the houses".
        self._track_instances = bool(self._p('target_instances', True))
        self._inst_eps = float(self._p('target_cluster_eps_m', 4.0))
        self._inst_min_pts = int(self._p('target_cluster_min_points', 10))
        self._visit_radius = float(self._p('target_visit_radius_m', 12.0))
        self._det_marker_max = int(self._p('detection_marker_max_points', 1500))
        self._lm_needs_anchor = False    # re-anchor lanes on first pick
        self._lm_spacing_used = 0.0
        self._max_target_marker = -1     # highest target id ever drawn
        self._trail = []                 # map-frame (x, y, z) breadcrumbs
        self._trail_max = int(self._p('trail_max_points', 4000))
        self._trail_min_step_m = float(self._p('trail_min_step_m', 1.0))
        self._visited_targets = []       # map-frame (x, y) of visited instances
        self._target_instances = []      # map-frame (x, y) of every instance seen
        self._active_target = None       # the one currently being flown to

        # ── perception backend ────────────────────────────────────────────────
        self._device = self._p('device', 'cuda:0')
        self._gpu_id = int(self._p('gpu_id', 0))
        self._dump_location = self._p('dump_location', '/tmp/conavgpt2')
        self._save_debug_images = bool(self._p('save_debug_images', False))
        self._publish_vis = bool(self._p('publish_vis', True))
        self._publish_agent_images = bool(self._p('publish_agent_images', True))
        yolo_w = self._p('yolo_world_weights', '')
        sam_w = self._p('mobile_sam_weights', '')
        if yolo_w:
            os.environ['CONAVGPT2_YOLO_WORLD_WEIGHTS'] = yolo_w
        if sam_w:
            os.environ['CONAVGPT2_SAM_WEIGHTS'] = sam_w
        # ultralytics' WEIGHTS_DIR defaults to the literal string 'weights',
        # which is RELATIVE — it resolves against the CWD, and the CWD under
        # `ros2 launch` is the colcon workspace root. YOLO-World's set_classes()
        # then downloads CLIP ViT-B/32 (338 MB) into robot/ros_ws/weights/ on
        # every fresh container. Pin it to somewhere mounted and absolute.
        # SETTINGS and the already-imported constant both, because
        # ultralytics.nn.text_model does `from ultralytics.utils import
        # WEIGHTS_DIR` and reads the attribute, not the setting.
        ultra_dir = str(self._p('ultralytics_weights_dir', ''))
        if ultra_dir:
            try:
                import ultralytics.utils as _uu
                from pathlib import Path as _Path
                os.makedirs(ultra_dir, exist_ok=True)
                _uu.SETTINGS.update({'weights_dir': ultra_dir})
                _uu.WEIGHTS_DIR = _Path(ultra_dir)
            except Exception as exc:
                self.get_logger().warn(
                    f'could not pin ultralytics weights_dir to {ultra_dir}: {exc}')

        # ── VLM backend ───────────────────────────────────────────────────────
        base_url, model, api_key = vlm_client.resolve(
            self._p('vlm_base_url', ''), self._p('vlm_model', ''),
            self._p('vlm_api_key', ''))
        self._vlm_timeout = float(self._p('vlm_timeout_s', 60.0))
        self._vlm_preflight = bool(self._p('vlm_preflight', True))
        vlm_client.configure(base_url, model, api_key, self._num_robots,
                             self._vlm_timeout)

        # ── upstream's argparse Namespace, built without touching sys.argv ────
        self._args = self._build_args()
        explored_map_utils.SCENE_VOXEL_M = self._scene_voxel
        explored_map_utils.SCENE_DBSCAN_EPS_M = self._scene_eps
        explored_map_utils.SCENE_DBSCAN_MIN_POINTS = self._scene_min_pts
        explored_map_utils.MAX_FRONTIERS = self._max_frontiers
        explored_map_utils.FRONTIER_ORDER = self._frontier_order

        # Body +y (left) is optical -x, so a DOWNWARD body pitch p is a rotation
        # of -p about the optical x axis. Applied on the right of a
        # camera->map pose, which rotates the camera's own axes.
        _c, _s = math.cos(self._camera_pitch), math.sin(self._camera_pitch)
        self._T_mount_pitch = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, _c, _s, 0.0],
            [0.0, -_s, _c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

        map_size = self._map_size_cm // self._map_resolution
        if map_size != 480:
            self.get_logger().warn(
                f'grid is {map_size} cells, not 480. Upstream stamps the robot '
                'markers on the VLM candidate images in 480-pixel coordinates, so '
                'they will land in the wrong place. Set map_cells:=480 (and leave '
                'map_size_cm/map_resolution_cm at 0 to derive from map_extent_m).')

        # ── state ─────────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._obs = [{} for _ in self._robots]
        self._stamp = [None] * self._num_robots
        self._nav = [_NavGoal() for _ in self._robots]
        self._boot_enu = [None] * self._num_robots
        self._goal_points = []
        self._round = 0
        self._last_round_start = None
        self._stop = False
        self._sim_t0 = None
        self._run_complete = False
        # Identifies this process's rows in an appended results file.
        self._run_id = f'{int(time.time())}-{os.getpid()}'
        self._bridge = CvBridge()
        self._agents = None
        self._map_process = None
        self._agents_ready = threading.Event()

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self._cbg = ReentrantCallbackGroup()
        # Isaac's camera/odometry streams are BEST_EFFORT; a RELIABLE subscriber
        # gets a QoS mismatch and silently receives nothing.
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)
        self._syncs = []
        self._plan_pubs = []
        self._agent_image_pubs = []
        self._nav_clients = []
        for i, robot in enumerate(self._robots):
            rgb_sub = message_filters.Subscriber(
                self, Image, self._rgb_tpl.format(robot=robot), qos_profile=sensor_qos)
            depth_sub = message_filters.Subscriber(
                self, Image, self._depth_tpl.format(robot=robot), qos_profile=sensor_qos)
            sync = message_filters.ApproximateTimeSynchronizer(
                [rgb_sub, depth_sub], queue_size=self._sync_queue, slop=self._sync_slop)
            sync.registerCallback(
                lambda rgb, depth, idx=i: self._rgbd_callback(idx, rgb, depth))
            self._syncs.append(sync)

            self.create_subscription(
                CameraInfo, self._info_tpl.format(robot=robot),
                lambda msg, idx=i: self._camera_info_callback(idx, msg),
                sensor_qos, callback_group=self._cbg)
            self.create_subscription(
                Odometry, self._odom_tpl.format(robot=robot),
                lambda msg, idx=i: self._odom_callback(idx, msg),
                sensor_qos, callback_group=self._cbg)
            if self._frame_mode == 'global_enu':
                self.create_subscription(
                    NavSatFix, self._navsat_tpl.format(robot=robot),
                    lambda msg, idx=i: self._navsat_callback(idx, msg),
                    sensor_qos, callback_group=self._cbg)

            self._plan_pubs.append(self.create_publisher(
                Path, self._plan_tpl.format(robot=robot), 10))
            self._agent_image_pubs.append(self.create_publisher(
                Image, self._agent_image_tpl.format(robot=robot), 1))
            self._nav_clients.append(ActionClient(
                self, NavigateTask, self._nav_tpl.format(robot=robot),
                callback_group=self._cbg))

        self._map_image_pub = self.create_publisher(Image, self._map_image_topic, 1)
        self._vlm_image_pub = self.create_publisher(Image, self._vlm_image_topic, 1)
        # TRANSIENT_LOCAL: a Foxglove panel that connects mid-run should draw the
        # map it already has rather than stay blank until the next tick.
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self._occupancy_pub = self.create_publisher(
            OccupancyGrid, self._occupancy_topic, latched)
        # VLFM's value field, as an OccupancyGrid because that is the message a
        # viewer already knows how to colour: 0..100 is the value, -1 is "never
        # scored" and draws transparent.
        self._voxel_pub = self.create_publisher(
            PointCloud2, self._voxel_cloud_topic, latched)
        self._value_pub = self.create_publisher(
            OccupancyGrid, self._p('value_map_topic',
                                   '/{robot}/value_map').format(robot=_r0),
            latched)
        self._frontier_pub = self.create_publisher(
            MarkerArray, self._frontier_topic, latched)
        # Where the robot is, where it has been, and every target instance with
        # its visit state — in the SAME map frame as the frontiers and the voxel
        # cloud. The GCS visualiser draws its robot markers in a GPS-anchored
        # global ENU frame instead, which does not line up with this one.
        # Frontiers as a POINT CLOUD as well as markers: a viewer controls point
        # size on a cloud, so the candidates can be made small enough to read
        # against the map without republishing. Colour carries the same meaning
        # as the markers — green is the one being flown to.
        self._frontier_cloud_pub = self.create_publisher(
            PointCloud2, self._p('frontier_cloud_topic',
                                 '/{robot}/frontier_cloud').format(robot=_r0),
            latched)
        self._search_marker_pub = self.create_publisher(
            MarkerArray, self._p('search_marker_topic',
                                 '/{robot}/search/markers').format(robot=_r0),
            latched)
        # Depth 10 rather than latched: every round must survive into the mcap, and
        # TRANSIENT_LOCAL depth 1 would keep only the last one.
        self._round_stats_pub = self.create_publisher(
            String, self._p('round_stats_topic',
                            '/{robot}/search/round_stats').format(robot=_r0), 10)
        # Latched, because it is a one-shot edge a mission step polls for with
        # `ros2 topic echo --once` — which would otherwise have to be listening
        # at the exact moment the budget ran out.
        self._complete_pub = self.create_publisher(
            Bool, self._p('run_complete_topic',
                          '/{robot}/search/run_complete').format(robot=_r0),
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self._stats_path = os.path.join(
            self._results_dir or self._dump_location, 'search_planner_rounds.jsonl')

        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info(
            f'search_planner starting | robots={self._robots} | goal="{self._goal_name}" | '
            f'nav_mode={self._nav_mode} | grid={map_size}x{map_size} @ '
            f'{self._map_resolution} cm = {self._map_size_cm / 100.0:.0f} m across | '
            f'obstacle band z=[{self._obst_min_z}, {self._obst_max_z}] m AGL | '
            f'camera pitch {math.degrees(self._camera_pitch):.1f} deg down | '
            f'<= {self._max_frontiers} frontiers ({self._frontier_order} kept) | '
            f'vlm={model} @ {base_url}')

        # YOLO-World + MobileSAM load takes tens of seconds; keep callbacks live.
        threading.Thread(target=self._init_agents, daemon=True).start()
        threading.Thread(target=self._plan_loop, daemon=True).start()

    # ── parameters ────────────────────────────────────────────────────────────

    def _robot_index_of_self(self):
        """This container's index in `robot_names`, from ROBOT_NAME. Each robot
        runs its own planner, so 'which slice is mine' is answered locally."""
        me = os.environ.get('ROBOT_NAME', '')
        if me in self._robots:
            return self._robots.index(me)
        return 0

    def _p(self, name, default):
        return self.declare_parameter(name, default).value

    def _build_args(self):
        """Upstream reads its configuration from argparse. `get_args([])` gives the
        upstream defaults without consuming --ros-args, then ROS parameters win."""
        args = get_args([])
        args.num_agents = self._num_robots
        args.num_local_steps = self._num_local_steps
        args.map_resolution = self._map_resolution
        args.map_size_cm = self._map_size_cm
        # Upstream centres the obstacle slab on the camera height and takes its
        # thickness from map_height_cm; expressed here as an explicit z range.
        args.map_height_cm = int(round((self._obst_max_z - self._obst_min_z) * 100))
        args.sem_threshold = self._sem_threshold
        args.nav_mode = self._nav_mode
        args.dump_location = self._dump_location
        args.frame_width = self._frame_w
        args.frame_height = self._frame_h
        args.device = self._device
        args.gpu_id = self._gpu_id
        args.cuda = str(self._device).startswith('cuda')
        args.visualize = 0                       # no cv2.imshow / open3d GUI
        # Upstream gates computing the annotated FPV and the per-agent map render
        # on print_images; AirStackAgent._visualize suppresses the file writes
        # unless save_debug_images is on.
        args.print_images = 1 if (self._publish_vis and self._publish_agent_images) else 0
        args.rank = 0
        return args

    @property
    def _slab_center_z(self):
        return 0.5 * (self._obst_min_z + self._obst_max_z)

    # ── perception init ───────────────────────────────────────────────────────

    def _init_agents(self):
        try:
            # ONLY nav_mode 'gpt' talks to the generative endpoint. 'vlfm' scores
            # with BLIP-2 ITM and 'nearest' uses no model at all, so preflighting
            # a chat server they will never call turns an absent (and
            # unnecessary) 6 GB service into a fatal startup error.
            if self._vlm_preflight and self._nav_mode == 'gpt':
                vlm_client.preflight(self.get_logger())
            elif self._nav_mode == 'vlfm':
                try:
                    h = vlfm_scorer.health(self._vlfm_url)
                    self.get_logger().info(
                        f'vlfm: ITM endpoint OK ({h.get("model")} on '
                        f'{h.get("device")}, {h.get("gpu_gib")} GiB) @ {self._vlfm_url}')
                except Exception as exc:
                    raise RuntimeError(
                        f'vlfm: no BLIP-2 ITM server at {self._vlfm_url} ({exc}). '
                        'Start it with: ros2 run search_baselines itm_server') from exc
            agents = []
            for i in range(self._num_robots):
                agent = AirStackAgent(
                    self._args, i, self._goal_name, classes=self._classes,
                    depth_min_m=self._depth_min_m, depth_max_m=self._depth_max_m,
                    depth_border_px=self._depth_border_px)
                agent.save_debug_images = self._save_debug_images
                agents.append(agent)
            self._map_process = Global_Map_Proc(self._args)
            if self._frontier_source == 'voxel3d':
                half = self._map_size_cm / 100.0 / 2.0
                # The MAP must span the height the WORLD occupies, not the
                # height the drone is allowed to fly at. Sizing it to the flight
                # band was wrong and showed up immediately: with the band at
                # [8, 25] m the map covered only empty air, the ground and the
                # houses (0-12 m) fell outside it entirely, and every frontier
                # came back at 9 m — the bottom layer — because that was the
                # only place carved rays clipped the volume.
                #
                # The flight band still applies, but as a filter on CANDIDATES
                # (`z_range` in frontiers()), which is a different question:
                # where geometry is, versus where the drone may go.
                zlo = self._vox_z_min
                zhi = (self._vox_z_max if self._vox_z_max > 0.0 else
                       max(self._obst_max_z, self._max_alt, self._fr_z_max)
                       + 2.0 * self._vox_size)
                zlo, zhi = min(zlo, zhi), max(zlo, zhi)
                self._voxel_map = VoxelMap((-half, -half, zlo - self._vox_size),
                                           (half, half, zhi + self._vox_size),
                                           self._vox_size)
                d = self._voxel_map.dims
                self.get_logger().info(
                    f'voxel3d: {d[0]}x{d[1]}x{d[2]} voxels @ {self._vox_size:.1f} m, '
                    f'z [{zlo:.1f}, {zhi:.1f}] m, '
                    f'frontier = empty voxel with >= {self._vox_min_unobs} '
                    f'unobserved neighbours in r={self._vox_neigh_r}')
            size0 = self._map_size_cm // self._map_resolution
            origin0 = (int(agents[0].origins_grid[0]), int(agents[0].origins_grid[1]))
            if self._nav_mode == 'frontier':
                self._visit_cost = VisitCost(
                    size0, self._map_resolution / 100.0, origin0,
                    radius_m=self._visit_cost_radius,
                    decay_per_s=self._visit_decay, weight=self._visit_weight)
                self.get_logger().info(
                    f'frontier: visit cost {size0}x{size0} @ '
                    f'{self._map_resolution / 100.0:.2f} m, deposit radius '
                    f'{self._visit_cost_radius:.0f} m, weight {self._visit_weight:.2f}, '
                    f'decay {self._visit_decay:g}/s')
            if self._nav_mode == 'lawnmower':
                if self._search_poly is None:
                    raise RuntimeError(
                        'lawnmower needs a sector: set search_area_xy (a lawnmower '
                        'over an unbounded region has no lanes to fly)')
                spacing = self._lm_spacing
                if spacing <= 0.0:
                    with self._lock:
                        hfov = self._obs[0].get('hfov_rad')
                    if not hfov:
                        raise RuntimeError(
                            'lawnmower spacing is derived from the camera footprint '
                            'but no camera_info has arrived yet')
                    spacing = lm.spacing_for_footprint(
                        self._altitude, hfov, self._lm_overlap)
                self._lm_path = lm.boustrophedon(
                    self._search_poly, spacing, axis=self._lm_axis)
                self._lm_idx = 0
                self._lm_spacing_used = spacing
                # The lanes are known here but the entry point is not: agents
                # have no pose at init. Re-anchor on the first pick so the
                # sweep starts at the lane end nearest the drone — otherwise
                # the first waypoint is whichever corner the partition emitted,
                # which on the preview scene is a 170 m transit before any
                # coverage begins.
                self._lm_needs_anchor = True
                self.get_logger().info(
                    f'lawnmower: {len(self._lm_path)} waypoints, lane spacing '
                    f'{spacing:.1f} m ({self._lm_overlap:.0%} overlap at '
                    f'{self._altitude:.0f} m), axis={self._lm_axis}')
            if self._nav_mode == 'vlfm':
                size = self._map_size_cm // self._map_resolution
                self._value_map = ValueMap(
                    size, self._map_resolution / 100.0,
                    (int(agents[0].origins_grid[0]), int(agents[0].origins_grid[1])))
                self.get_logger().info(
                    f'vlfm: value map {size}x{size} @ '
                    f'{self._map_resolution / 100.0:.2f} m, keyframe every '
                    f'{self._vlfm_keyframe_s:.1f} s, range {self._vlfm_max_range:.0f} m'
                    + (f', search area {len(self._search_poly)} pts'
                       if self._search_poly is not None else ', NO search area'))
            self._agents = agents
            self._agents_ready.set()
            self.get_logger().info('search_planner: agents ready')
        except Exception as exc:
            self.get_logger().fatal(f'search_planner failed to start: {exc}')
            self._stop = True
            raise

    # ── subscriptions ─────────────────────────────────────────────────────────

    def _rgbd_callback(self, i, rgb_msg, depth_msg):
        try:
            rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth = np.asarray(depth, dtype=np.float32) * self._depth_scale

            sw = self._frame_w / float(rgb.shape[1])
            sh = self._frame_h / float(rgb.shape[0])
            if (rgb.shape[1], rgb.shape[0]) != (self._frame_w, self._frame_h):
                rgb = cv2.resize(rgb, (self._frame_w, self._frame_h),
                                 interpolation=cv2.INTER_AREA)
            if (depth.shape[1], depth.shape[0]) != (self._frame_w, self._frame_h):
                depth = cv2.resize(depth, (self._frame_w, self._frame_h),
                                   interpolation=cv2.INTER_NEAREST)
            with self._lock:
                self._obs[i]['rgb'] = np.ascontiguousarray(rgb)
                self._obs[i]['depth'] = depth
                self._obs[i]['rgb_scale'] = (sw, sh)
                self._stamp[i] = depth_msg.header.stamp
        except Exception as exc:
            self.get_logger().warn(f'[{self._robots[i]}] rgbd callback failed: {exc}')

    def _camera_info_callback(self, i, msg):
        with self._lock:
            first = 'cam_K_raw' not in self._obs[i]
            self._obs[i]['cam_K_raw'] = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            if msg.k[0] > 0.0:
                # VLFM needs the real horizontal FOV to paint its value cone;
                # a wrong one either spills value onto ground the camera never
                # saw or starves the cells it did.
                self._obs[i]['hfov_rad'] = 2.0 * math.atan(0.5 * msg.width / msg.k[0])
        if first and msg.k[0] > 0.0:
            # Upstream hardcodes hfov=79 (the RealSense D455). Nothing here uses
            # that: the intrinsics come off the wire. Logged so the geometry can be
            # sanity-checked against the ZED without reading the USD.
            hfov = math.degrees(2.0 * math.atan(0.5 * msg.width / msg.k[0]))
            vfov = math.degrees(2.0 * math.atan(0.5 * msg.height / msg.k[4]))
            self.get_logger().info(
                f'[{self._robots[i]}] camera_info {msg.width}x{msg.height} '
                f'fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} -> hfov={hfov:.1f} deg, '
                f'vfov={vfov:.1f} deg, frame={msg.header.frame_id}')

    def _odom_callback(self, i, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        T = np.eye(4)
        T[:3, :3] = _quat_to_rot(q.x, q.y, q.z, q.w)
        T[:3, 3] = [p.x, p.y, p.z]
        with self._lock:
            self._obs[i]['odom'] = T

    def _navsat_callback(self, i, msg):
        """boot_enu anchors this robot's local 'map' origin in global ENU.

        The node can start long after takeoff, so gps_to_enu(fix) alone would
        anchor wherever the drone happens to be; subtracting the concurrent odom
        pose recovers the origin. Same derivation as raven_nav/lvlm_baseline.
        """
        if self._boot_enu[i] is not None:
            return
        with self._lock:
            odom = self._obs[i].get('odom')
        if odom is None:
            return
        enu = np.array(gps_to_enu(msg.latitude, msg.longitude, msg.altitude))
        self._boot_enu[i] = enu - odom[:3, 3]
        self.get_logger().info(
            f'[{self._robots[i]}] boot_enu = {self._boot_enu[i][:2].round(2).tolist()}')

    def _offset(self, i):
        """XY translation from this robot's local `map` frame into the grid frame.

        grid = local + boot_enu - map_origin. Z stays AGL, matching how the rest of
        the stack lifts local -> global.
        """
        origin = np.array([self._map_origin_xy[0], self._map_origin_xy[1], 0.0])
        if self._frame_mode != 'global_enu':
            return -origin
        b = self._boot_enu[i]
        if b is None:
            return None
        return np.array([b[0], b[1], 0.0]) - origin

    # ── pose ──────────────────────────────────────────────────────────────────

    def _camera_pose(self, i, odom_T, stamp):
        """4x4 pose of the camera OPTICAL frame (RDF) expressed in the map frame."""
        if self._pose_source == 'tf':
            frame = self._cam_frame_tpl.format(robot=self._robots[i])
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._map_frame, frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2))
                t = tf.transform.translation
                r = tf.transform.rotation
                T = np.eye(4)
                T[:3, :3] = _quat_to_rot(r.x, r.y, r.z, r.w)
                T[:3, 3] = [t.x, t.y, t.z]
                # TF walks the URDF, which models no mount pitch. A launch script
                # that tilted the camera produces a real camera TF cannot see, so
                # rotate the frame TF returned by that known mount angle. Position
                # is untouched: the tilt is about the camera's own centre.
                return T @ self._T_mount_pitch
            except tf2_ros.TransformException as exc:
                # TF frames are unprefixed, so a peer robot on another ROS domain
                # will never resolve; fall through to its odometry instead.
                self.get_logger().warn(
                    f'[{self._robots[i]}] TF {self._map_frame}->{frame} failed '
                    f'({exc}); using odometry + static camera extrinsics',
                    throttle_duration_sec=10.0)

        if odom_T is None:
            return None
        pitch = self._camera_pitch
        R_pitch = np.array([
            [math.cos(pitch), 0.0, math.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-math.sin(pitch), 0.0, math.cos(pitch)],
        ])
        T_body_cam = np.eye(4)
        T_body_cam[:3, :3] = R_pitch @ R_FLU_TO_RDF
        T_body_cam[:3, 3] = self._camera_offset
        return odom_T @ T_body_cam

    # ── main loop ─────────────────────────────────────────────────────────────

    def _plan_loop(self):
        self._agents_ready.wait()
        while rclpy.ok() and not self._stop:
            started = time.time()
            try:
                self._tick()
            except Exception as exc:
                self.get_logger().error(f'search_planner tick failed: {exc}', throttle_duration_sec=5.0)
            sleep_for = self._plan_period_s - (time.time() - started)
            if sleep_for > 0:
                time.sleep(sleep_for)

    def _snapshot(self):
        with self._lock:
            out = []
            for i in range(self._num_robots):
                o = self._obs[i]
                if 'rgb' not in o or 'depth' not in o or 'cam_K_raw' not in o:
                    return None
                if 'odom' not in o and self._pose_source != 'tf':
                    return None
                sw, sh = o['rgb_scale']
                fx, fy, cx, cy = o['cam_K_raw']
                out.append({
                    'rgb': o['rgb'],
                    'depth': o['depth'],
                    # Intrinsics must follow the resize applied in the callback.
                    'cam_K': Namespace(fx=fx * sw, fy=fy * sh, cx=cx * sw, cy=cy * sh),
                    'odom': o.get('odom'),
                    'stamp': self._stamp[i],
                })
            return out

    def _sim_budget_spent(self):
        """True once max_sim_seconds of SIM time have passed since the first tick
        that had data. use_sim_time is on, so get_clock() is /clock — which only
        advances while the sim timeline plays, and stops when it is paused."""
        if self._max_sim_seconds <= 0.0:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        if now <= 0.0:                      # /clock has not arrived yet
            return False
        if self._sim_t0 is None:
            self._sim_t0 = now
            return False
        return (now - self._sim_t0) >= self._max_sim_seconds

    def _finish_run(self):
        """Sim budget spent: stop commanding, hold position, say so once.

        The node deliberately stays alive — the map, the frontier markers and
        the round table are what a run is read from afterwards, and they are all
        latched topics that would disappear with the process."""
        self._run_complete = True
        for i, nav in enumerate(self._nav):
            if nav.handle is not None:
                try:
                    self._await(nav.handle.cancel_goal_async(), 2.0)
                except Exception:
                    pass
                nav.handle = None
                nav.result_future = None
        msg = Bool()
        msg.data = True
        self._complete_pub.publish(msg)
        self.get_logger().info(
            f'search_planner: sim budget spent ({self._max_sim_seconds:.0f} s) after '
            f'{self._round} VLM rounds — planning stopped, node left up for the '
            'final map')

    def _tick(self):
        if self._run_complete:
            return
        snap = self._snapshot()
        if snap is None:
            self.get_logger().info('search_planner: waiting for rgb/depth/camera_info/odometry',
                                   throttle_duration_sec=10.0)
            return
        if self._sim_budget_spent():
            self._finish_run()
            return

        merged = o3d.geometry.PointCloud()
        visited_vis, vis_pose_pred, grid_pose, o3d_pose = [], [], [], []
        found_goal = False

        offsets = []
        for i, agent in enumerate(self._agents):
            pose = self._camera_pose(i, snap[i]['odom'], snap[i]['stamp'])
            if pose is None:
                return
            offset = self._offset(i)
            if offset is None:
                self.get_logger().info(
                    f'[{self._robots[i]}] waiting for the first GPS fix to anchor '
                    'its map in global ENU (frame_mode=global_enu)',
                    throttle_duration_sec=10.0)
                return
            offsets.append(offset)
            pose = pose.copy()
            pose[:3, 3] = pose[:3, 3] + offset
            obs = {'rgb': snap[i]['rgb'], 'depth': snap[i]['depth'],
                   'cam_K': snap[i]['cam_K'], 'pose': pose}

            agent.mapping(obs)
            if self._nav_mode == 'vlfm':
                self._vlfm_keyframe(i, agent, pose)
            if self._visit_cost is not None:
                now = time.time()
                if self._last_visit_t is not None and self._visit_decay > 0.0:
                    self._visit_cost.decay(now - self._last_visit_t)
                self._last_visit_t = now
                self._visit_cost.visit(self._agent_xy(agent))

            if self._incremental:
                # Upstream re-rasterises every point ever seen on every tick, which
                # is unbounded in both memory and time. Global_Map_Proc already
                # accumulates into its own grids, so feeding it only this tick's
                # cloud is equivalent and bounded.
                contribution = agent.point_sum
                agent.point_sum = o3d.geometry.PointCloud()
            else:
                contribution = agent.point_sum
            merged += contribution

            visited_vis.append(agent.visited_vis)
            vis_pose_pred.append([
                agent.current_grid_pose[1] * 480.0 / agent.map_size,
                int((agent.map_size - agent.current_grid_pose[0]) * 480.0 / agent.map_size),
                np.deg2rad(agent.relative_angle)])
            grid_pose.append(agent.current_grid_pose)
            o3d_pose.append(agent.camera_position)
            found_goal = found_goal or agent.found_goal

        for agent in self._agents:
            agent.open3d_pose = o3d_pose

        merged = self._crop_to_map(merged)
        obstacle_map, explored_map, top_view_map = self._map_process.Map_Extraction(
            merged, self._slab_center_z)

        if self._frontier_source == 'voxel3d' and self._voxel_map is not None:
            target_edge_map, target_point_list = self._voxel_frontiers(merged)
        else:
            _, target_edge_map, target_point_list = self._map_process.Frontier_Det(
                threshold_point=self._frontier_threshold)
            self._frontier_z = []

        # Instance bookkeeping BEFORE the assignment gate, because whether any
        # unvisited target remains is exactly what decides between "fly to it"
        # and "ask the VLM for a frontier".
        if self._track_instances:
            self._update_targets(self._agents[0])
            found_goal = self._active_target is not None
        self._publish_search_markers(self._agents[0])

        step = self._agents[0].l_step
        if self._round_period_s > 0.0:
            assignment_due = (not self._goal_points
                              or self._last_round_start is None
                              or (time.time() - self._last_round_start) >= self._round_period_s)
        else:
            assignment_due = (not self._goal_points
                              or step % self._num_local_steps == self._num_local_steps - 1)
        self._frontier_candidates = list(target_point_list)
        if assignment_due and not found_goal:
            self._assign(target_edge_map, target_point_list, top_view_map,
                         vis_pose_pred, obstacle_map, explored_map, grid_pose, step)

        while len(self._goal_points) < self._num_robots:
            self._goal_points.append(list(self._agents[len(self._goal_points)]
                                          .current_grid_pose))

        goal_maps = []
        for i, agent in enumerate(self._agents):
            agent.obstacle_map = obstacle_map
            agent.explored_map = explored_map
            agent.act(self._goal_points[i], grid_pose)
            agent.found_goal = found_goal
            goal_maps.append(agent.goal_map)
            self._command(i, agent, offsets[i])

        if self._publish_vis:
            self._publish_map_image(step, vis_pose_pred, obstacle_map, explored_map,
                                    visited_vis, target_edge_map, goal_maps,
                                    top_view_map)
            self._publish_occupancy(obstacle_map, explored_map)
            self._publish_frontiers(target_edge_map, target_point_list)
            if self._value_map is not None:
                self._publish_value_map()
            if self._voxel_map is not None:
                self._publish_voxel_cloud()
            if self._publish_agent_images:
                self._publish_agent_images_now()

    def _voxel_frontiers(self, merged):
        """Integrate this tick's cloud and extract 3D frontiers.

        Returns the same two things upstream's `Frontier_Det` does — a labelled
        edge map and a list of [i, j] grid cells — so everything downstream (the
        BEV render the VLM is shown, the value lookup, the goal) is unchanged.
        The difference is that each candidate ALSO has a height, kept in
        `self._frontier_z` and used for the waypoint altitude, which is what a
        2D slab can never provide.
        """
        agent = self._agents[0]
        size = self._map_size_cm // self._map_resolution
        edge = np.zeros((size, size), dtype=np.float32)
        self._frontier_z = []
        try:
            pts = np.asarray(merged.points)
            if pts.shape[0]:
                # merged is in upstream's open3d frame; the voxel map works in
                # the map frame, so convert before integrating.
                world = np.stack(
                    [pts[:, 0], -pts[:, 2], pts[:, 1]], axis=1)
                cam = agent.camera_position
                cam_map = np.array([cam[0], -cam[2], cam[1]], dtype=float)
                self._voxel_map.integrate(
                    cam_map, world, carve_samples=self._vox_carve_samples,
                    max_points=self._vox_max_points,
                    max_range_m=self._vox_max_range)
                if world.shape[0]:
                    d = np.linalg.norm(world - cam_map[None, :], axis=1)
                    self.get_logger().info(
                        f'voxel3d IN: {world.shape[0]} pts | cam z '
                        f'{cam_map[2]:.1f} | pt z '
                        f'[{world[:, 2].min():.1f}, {world[:, 2].max():.1f}] '
                        f'p99 {np.percentile(world[:, 2], 99):.1f} | range '
                        f'[{d.min():.1f}, {d.max():.1f}] p99 '
                        f'{np.percentile(d, 99):.1f} | above cam '
                        f'{100.0 * (world[:, 2] > cam_map[2]).mean():.1f}%',
                        throttle_duration_sec=15.0)

            # Candidate band: the frontier band when set, else the flight band.
            zlo = self._fr_z_min if self._fr_z_min > 0 else (
                self._min_alt if self._min_alt > 0 else None)
            zhi = self._fr_z_max if self._fr_z_max > 0 else (
                self._max_alt if self._max_alt > 0 else None)
            # rayfronts' scheme: the set ACCUMULATES and only the active
            # window (the bbox this observation wrote) is re-evaluated. A
            # recompute-everything-every-tick set has unstable identity — the
            # committed frontier is usually gone from the next tick's set, so
            # no goal can be held.
            fr, gain = self._voxel_map.frontiers_persistent(
                neighborhood_r=self._vox_neigh_r,
                min_unobserved=self._vox_min_unobs,
                min_empty=self._vox_min_empty,
                min_occupied=self._vox_min_occ,
                subsampling=self._fronti_subsampling,
                subsampling_min_fronti=self._fronti_min_cells,
                z_range=(zlo, zhi) if (zlo is not None and zhi is not None) else None)
            if fr.shape[0] == 0:
                return edge, []

            # Inside the search area, if one is set — cheaper here than after
            # they have been turned into candidates.
            keep = self._points_in_polygon(fr[:, :2], self._search_poly)
            fr, gain = fr[keep], gain[keep]
            if fr.shape[0] == 0:
                return edge, []

            order = self._rank_frontiers(fr, gain)
            pts_out = []
            for rank, k in enumerate(order):
                i, j = agent.map_xy_to_grid(float(fr[k, 0]), float(fr[k, 1]))
                pts_out.append([i, j])
                self._frontier_z.append(float(fr[k, 2]))
                # Label the cell (and a small disc) so the BEV render and the
                # marker path show something at this candidate's location.
                r = 2
                edge[max(0, i - r):min(size, i + r + 1),
                     max(0, j - r):min(size, j + r + 1)] = rank + 1
            return edge, pts_out
        except Exception as exc:
            self.get_logger().warn(f'voxel3d frontier extraction failed: {exc}',
                                   throttle_duration_sec=10.0)
            return edge, []

    def _rank_frontiers(self, fr, gain):
        """Indices of the frontiers to offer, best first.

        Unobserved-neighbour count IS the information gain, so ranking by it
        alone is unambiguous — but it is also degenerate: the least-observed
        height wins every slot, all six candidates come back at one z, and the
        3D map buys nothing over the 2D slab. Take the best frontier at each
        height before taking a second one anywhere, so the offered set spans
        the band. Levels are visited in order of their best frontier, so the
        single best candidate overall is still rank 0.
        """
        order = np.argsort(-gain)
        if not self._fr_z_stratify:
            return order[:self._max_frontiers]

        levels = {}
        for k in order:
            # floor, not round: voxel centres sit at odd multiples of half
            # the voxel size, and round()'s ties-to-even merges adjacent
            # layers in pairs — which silently drops every other height.
            levels.setdefault(
                int(float(fr[k, 2]) // self._vox_size), []).append(k)
        # Best-first across levels, not lowest-first: otherwise the ground
        # layer would always be offered as frontier_0.
        by_gain = sorted(levels, key=lambda z: -gain[levels[z][0]])

        picked = []
        for rnd in range(max(len(v) for v in levels.values())):
            for z in by_gain:
                if rnd < len(levels[z]):
                    picked.append(levels[z][rnd])
                    if len(picked) >= self._max_frontiers:
                        return np.asarray(picked, dtype=int)
        return np.asarray(picked, dtype=int)

    # ── VLFM ──────────────────────────────────────────────────────────────────

    @staticmethod
    def _points_in_polygon(pts_xy, poly_xy):
        """Vectorised even-odd ray cast. Same routine raven_nav uses to enforce
        its geofence, so a frontier that one method calls out of bounds is out of
        bounds for the other too."""
        pts_xy = np.asarray(pts_xy, dtype=float)
        if pts_xy.size == 0 or poly_xy is None or poly_xy.shape[0] < 3:
            return np.ones(len(pts_xy), dtype=bool)
        x, y = pts_xy[:, 0], pts_xy[:, 1]
        inside = np.zeros(len(pts_xy), dtype=bool)
        M = poly_xy.shape[0]
        for i in range(M):
            x1, y1 = poly_xy[i]
            x2, y2 = poly_xy[(i + 1) % M]
            straddles = (y1 > y) != (y2 > y)
            with np.errstate(divide='ignore', invalid='ignore'):
                xint = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            inside ^= straddles & (x < xint)
        return inside

    def _vlfm_keyframe(self, i, agent, pose):
        """Score this view and paint it, if a keyframe is due.

        Deliberately SYNCHRONOUS in the plan loop: the score describes the view
        at this pose, and running it in the background would paint it at
        whatever pose the drone had drifted to by the time the reply arrived.
        The cost is that the tick blocks for the call, which is exactly the
        bottleneck `vlfm_keyframe_period_s` trades against.
        """
        now = time.time()
        if self._value_map is None or (now - self._last_keyframe) < self._vlfm_keyframe_s:
            return
        self._last_keyframe = now
        with self._lock:
            rgb = self._obs[i].get('rgb')
            hfov = self._obs[i].get('hfov_rad')
        if rgb is None or not hfov:
            return

        score = vlfm_scorer.score_view_tiled(
            rgb, self._goal_name, rows=self._vlfm_rows, cols=self._vlfm_cols,
            base_url=self._vlfm_url, timeout=self._vlm_timeout,
            max_side=self._vlfm_img_max_side,
            text=(self._vlfm_prompt or None))
        info = dict(vlfm_scorer.LAST_SCORE)
        self._vlfm_server_s += float(info.get('round_trip_s') or 0.0)
        if score is None:
            # NOT zero: a failed call is no information. Painting 0 would tell
            # the map this view is actively unpromising.
            self._vlfm_score_fail += 1
            self.get_logger().warn(
                f'vlfm: view score failed ({info.get("error") or info.get("response")})',
                throttle_duration_sec=15.0)
            return
        self._vlfm_scores += 1

        cam_xy = (float(pose[0, 3]), float(pose[1, 3]))
        # Optical +z is forward; its ground projection is where the camera looks.
        fwd = pose[:3, 2]
        heading = math.atan2(float(fwd[1]), float(fwd[0]))
        n = self._value_map.observe(cam_xy, heading, hfov, self._vlfm_max_range,
                                    score)
        self.get_logger().info(
            f'vlfm: score {max(score):.2f} '
            f'cols={[round(v, 2) for v in score]} for "{self._goal_name}" | '
            f'{1000.0 * (info.get("round_trip_s") or 0.0):.0f} ms '
            f'| painted {n} cells | {self._vlfm_scores} scored, '
            f'{self._vlfm_score_fail} failed', throttle_duration_sec=10.0)

    def _vlfm_pick(self, target_point_list, agent):
        """Frontier standing in the highest-value cell, inside the search area.

        Falls back to the NEAREST in-bounds frontier while the value map is still
        empty — at t=0 nothing has been scored, and argmax over a field of zeros
        is an arbitrary choice dressed up as a decision.
        """
        if not target_point_list:
            return None
        xy = np.array([agent.grid_to_map_xy(p[0], p[1]) for p in target_point_list])
        keep = self._points_in_polygon(xy, self._search_poly)
        if not keep.any():
            self.get_logger().warn(
                f'vlfm: all {len(xy)} frontiers are outside the search area',
                throttle_duration_sec=15.0)
            return None
        idx = np.flatnonzero(keep)
        here = np.array(self._agent_xy(agent))

        best, best_score, scored_any = None, -1e9, False
        for k in idx:
            i_c, j_c = int(target_point_list[k][0]), int(target_point_list[k][1])
            v = self._value_map.value_at(i_c, j_c) if self._value_map else 0.0
            c = (self._value_map.conf[i_c, j_c]
                 if self._value_map and 0 <= i_c < self._value_map.size
                 and 0 <= j_c < self._value_map.size else 0.0)
            if c > self._vlfm_min_conf:
                scored_any = True
            d = float(np.linalg.norm(xy[k] - here))
            sc = v - self._vlfm_dist_penalty * d
            if sc > best_score:
                best_score, best = sc, k
        if not scored_any:
            best = int(idx[np.argmin(np.linalg.norm(xy[idx] - here, axis=1))])
            best_score = 0.0
            self.get_logger().info(
                'vlfm: value map still unscored — nearest in-bounds frontier',
                throttle_duration_sec=15.0)
        return self._commit(list(target_point_list[best]), best_score, xy[best], here)

    def _frontier_pick(self, target_point_list, agent):
        """Frontier maximising information gain MINUS accumulated visit cost.

        Nothing is ever removed from consideration. As coverage completes the
        visit costs equalise, their differences stop deciding anything, and
        selection reverts to information gain — so the drone sweeps the sector
        again instead of declaring it done. That repeat pass is the point: the
        detector misses people, and a second look is the cheapest way to find
        them.
        """
        if not target_point_list:
            return None
        xy = np.array([agent.grid_to_map_xy(p[0], p[1]) for p in target_point_list])
        keep = self._points_in_polygon(xy, self._search_poly)
        if not keep.any():
            self.get_logger().warn(
                f'frontier: all {len(xy)} candidates are outside the sector',
                throttle_duration_sec=15.0)
            return None
        idx = np.flatnonzero(keep)
        here = np.array(self._agent_xy(agent))

        # Information gain: the voxel path already ranks candidates by
        # unobserved-neighbour count, so position in the list IS the ranking.
        # Normalised to [0, 1] because visit cost is, and `weight` only means
        # something if the two are commensurate.
        n = len(target_point_list)
        best, best_score, best_xy = None, -1e9, None
        for k in idx:
            gain = 1.0 - (float(k) / max(1, n - 1)) if n > 1 else 1.0
            d = float(np.linalg.norm(xy[k] - here))
            gain -= self._vlfm_dist_penalty * d
            i_c, j_c = int(target_point_list[k][0]), int(target_point_list[k][1])
            sc = (self._visit_cost.score(gain, i_c, j_c)
                  if self._visit_cost is not None else gain)
            if sc > best_score:
                best_score, best, best_xy = sc, k, xy[k]
        if best is None:
            return None
        # The frontier arm's whole behaviour is gain-minus-visit-cost, and none
        # of it is observable from the trajectory alone. Record the decision:
        # coverage rising while scores converge is what a completed sweep looks
        # like, and is what distinguishes a second pass from being stuck.
        if self._visit_cost is not None:
            st = self._visit_cost.stats()
            self.get_logger().info(
                f'frontier: picked {best} of {n} '
                f'(score {best_score:.3f}) | coverage {st["coverage"]*100:.1f}% '
                f'| visit max {st["max"]:.2f} mean {st["mean"]:.3f}',
                throttle_duration_sec=10.0)
        return self._commit(list(target_point_list[best]), best_score, best_xy, here)

    def _lawnmower_pick(self, agent):
        """The next lane waypoint, as a grid cell.

        Wraps at the end of the path — the caller runs until the sim budget is
        spent, so the lanes are a loop and a second pass is expected, not an
        overrun.
        """
        if self._lm_path is None or len(self._lm_path) == 0:
            return None
        here = np.array(self._agent_xy(agent))
        if self._lm_needs_anchor:
            self._lm_needs_anchor = False
            self._lm_path = lm.boustrophedon(
                self._search_poly, self._lm_spacing_used,
                axis=self._lm_axis, start_xy=(float(here[0]), float(here[1])))
            self._lm_idx = 0
            self.get_logger().info(
                f'lawnmower: anchored at ({here[0]:.0f}, {here[1]:.0f}); '
                f'first waypoint ({self._lm_path[0][0]:.0f}, '
                f'{self._lm_path[0][1]:.0f})')
        self._lm_idx, wp = lm.next_waypoint(
            self._lm_path, here, self._lm_idx, self._lm_reach)
        return list(agent.map_xy_to_grid(float(wp[0]), float(wp[1])))

    def _commit(self, cand, cand_score, cand_xy, here):
        """Hold the current target unless the challenger is clearly better.

        Without this the drone re-argmaxes every tick, and because the value map
        shifts as it flies, two frontiers swap places repeatedly and it arrives
        at neither.
        """
        now = time.time()
        if self._locked_goal is None:
            self._locked_goal, self._locked_score, self._locked_at = \
                cand, cand_score, now
            self._locked_goal_xy = tuple(cand_xy)
            return list(cand)

        # ARRIVED -> release and take the challenger.
        #
        # Measured against the LOCKED GOAL'S OWN POSITION, not the challenger's.
        # The old test also required `cand == self._locked_goal` — the same grid
        # CELL — but the candidate set is re-extracted every tick and the cell
        # the lock was taken on is rarely in it again, so the equality almost
        # never held: the drone flew to its frontier, sat on it, and never
        # released. That is the hover.
        if (self._locked_goal_xy is not None
                and math.dist(tuple(here), self._locked_goal_xy)
                <= self._unlock_radius):
            self.get_logger().info(
                f'reached goal ({self._locked_goal_xy[0]:.0f}, '
                f'{self._locked_goal_xy[1]:.0f}) — releasing for the next',
                throttle_duration_sec=5.0)
            self._locked_goal, self._locked_score, self._locked_at = \
                cand, cand_score, now
            self._locked_goal_xy = tuple(cand_xy)
            return list(cand)

        held_for = now - self._locked_at
        margin = self._swap_frac * (abs(self._locked_score or 0.0) + 1e-6)
        if held_for >= self._lock_s and cand_score > (self._locked_score or 0.0) + margin:
            self.get_logger().info(
                f'vlfm: swap after {held_for:.1f} s — {cand_score:.3f} beats '
                f'{self._locked_score:.3f} by more than {margin:.3f}',
                throttle_duration_sec=5.0)
            self._locked_goal, self._locked_score, self._locked_at = \
                cand, cand_score, now
            self._locked_goal_xy = tuple(cand_xy)
        return list(self._locked_goal)

    def _crop_to_map(self, pcd):
        """Map_Extraction indexes the grid without clamping, so a point sitting
        exactly on the +halfsize boundary lands one cell past the end."""
        if len(pcd.points) == 0:
            return pcd
        half = self._map_size_cm / 100.0 / 2.0 - self._map_resolution / 100.0
        box = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(-half, -1e6, -half), max_bound=(half, 1e6, half))
        return pcd.crop(box)

    # ── frontier assignment ───────────────────────────────────────────────────

    def _assign(self, target_edge_map, target_point_list, top_view_map,
                vis_pose_pred, obstacle_map, explored_map, grid_pose, step):
        self._goal_points = []
        map_size = obstacle_map.shape[0]

        if self._nav_mode == 'gpt':
            if len(target_point_list) > 0 and step >= self._warmup_steps:
                round_start = time.time()
                since_last = (None if self._last_round_start is None
                              else round_start - self._last_round_start)
                self._last_round_start = round_start
                self._round += 1

                # Request build: rendering one top-view per frontier and base64ing
                # them is the part that grows with frontier count on OUR side.
                t_build = time.time()
                candidate_maps = chat_utils.get_all_candidate_maps(
                    target_edge_map, top_view_map, vis_pose_pred)
                message = chat_utils.message_prepare(
                    system_prompt.system_prompt, candidate_maps, self._goal_name)
                build_s = time.time() - t_build
                payload_bytes = sum(len(b.getvalue()) for b in candidate_maps)

                t_call = time.time()
                assignment = chat_utils.chat_with_gpt4v(message)
                call_s = time.time() - t_call

                self._record_round(assignment, len(target_point_list), candidate_maps,
                                   payload_bytes, build_s, call_s, since_last)
                if self._publish_vis:
                    self._publish_vlm_image(candidate_maps)
                for i in range(self._num_robots):
                    self._goal_points.append(self._resolve_frontier(
                        assignment, i, target_point_list, map_size))
            else:
                for _ in range(self._num_robots):
                    self._goal_points.append(self._random_goal(map_size))

        elif self._nav_mode == 'vlfm':
            for i in range(self._num_robots):
                pick = self._vlfm_pick(target_point_list, self._agents[i])
                self._goal_points.append(
                    pick if pick is not None else self._random_goal(map_size))

        elif self._nav_mode == 'frontier':
            for i in range(self._num_robots):
                self._goal_points.append(
                    self._frontier_pick(target_point_list, self._agents[i])
                    or self._random_goal(map_size))

        elif self._nav_mode == 'lawnmower':
            # No frontier is consulted at all: the lane path IS the plan. The
            # occupancy map still builds and publishes, so the run is readable
            # against the other arms, but nothing reads it for selection.
            for i in range(self._num_robots):
                self._goal_points.append(
                    self._lawnmower_pick(self._agents[i])
                    or self._random_goal(map_size))

        elif self._nav_mode == 'nearest':
            for i in range(self._num_robots):
                _, _, points = detect_frontier(
                    explored_map, obstacle_map, grid_pose[i],
                    threshold_point=self._frontier_threshold)
                self._goal_points.append(
                    points[0] if points else self._random_goal(map_size))
        else:
            raise ValueError(f'unknown nav_mode {self._nav_mode!r}')

    def _record_round(self, assignment, num_frontiers, candidate_maps,
                      payload_bytes, build_s, call_s, since_last):
        """Per-round telemetry: JSONL under the results dir, a String topic so it
        lands in the mcap, and one INFO line."""
        call = dict(chat_utils.LAST_CALL)
        server_s = call.get('server_s') or 0.0
        total_s = build_s + call_s
        budget = (self._round_period_s if self._round_period_s > 0.0
                  else self._num_local_steps * self._plan_period_s)
        overran = budget > 0.0 and total_s > budget

        stats = {
            # Which RUN this row belongs to. The file is opened 'a' (never
            # clobber a previous run's results) and `round` restarts at 1 every
            # process, so without this two runs in the same results dir — an
            # iteration retry, a relaunch after a crash — interleave into one
            # file that reads as a single long run. Group by run_id.
            'run_id': self._run_id,
            'round': self._round,
            'stamp': time.time(),
            'sim_time': self.get_clock().now().nanoseconds * 1e-9,
            'num_robots': self._num_robots,
            'num_frontiers': num_frontiers,
            'num_images': len(candidate_maps),
            'image_payload_bytes': int(payload_bytes),
            'build_s': round(build_s, 4),
            'call_s': round(call_s, 4),
            'server_s': round(server_s, 4),
            'total_s': round(total_s, 4),
            'attempts': call.get('attempts'),
            'model': call.get('model'),
            'prompt_tokens': call.get('prompt_tokens'),
            'completion_tokens': call.get('completion_tokens'),
            'total_tokens': call.get('total_tokens'),
            'parse': call.get('parse'),
            'invalid_ids': call.get('invalid_ids'),
            # Whether upstream's six-frontier cap dropped anything this round,
            # and what it dropped. `n_offered` is what the VLM saw;
            # `n_above_threshold` is what existed. See LAST_DET's note: the cap
            # keeps the SMALLEST regions, so a non-empty `areas_dropped` means
            # the biggest frontiers were the ones withheld.
            'frontier_det': dict(explored_map_utils.LAST_DET),
            'errors': call.get('errors'),
            'response_text': call.get('response_text'),
            'assignment': {k: str(v) for k, v in (assignment or {}).items()},
            'round_period_s': round(since_last, 4) if since_last is not None else None,
            'round_budget_s': budget,
            'overran': bool(overran),
            'overrun_s': round(total_s - budget, 4) if overran else 0.0,
        }

        try:
            os.makedirs(os.path.dirname(self._stats_path), exist_ok=True)
            with open(self._stats_path, 'a') as fh:
                fh.write(json.dumps(stats) + '\n')
        except OSError as exc:
            self.get_logger().warn(f'[search_planner] round log write failed: {exc}',
                                   throttle_duration_sec=30.0)
        msg = String()
        msg.data = json.dumps(stats)
        self._round_stats_pub.publish(msg)

        tok = (f"{stats['prompt_tokens']}+{stats['completion_tokens']} tok"
               if stats['prompt_tokens'] is not None else 'tok n/a')
        self.get_logger().info(
            f"[search_planner] round {self._round} | {num_frontiers} frontiers, "
            f"{self._num_robots} robot{'s' if self._num_robots != 1 else ''} | "
            f"{total_s:.1f} s (server {server_s:.1f}) | {tok} | {stats['parse']}")
        if overran:
            self.get_logger().warn(
                f"[search_planner] round {self._round} OVERRAN its period by "
                f"{stats['overrun_s']:.1f} s ({total_s:.1f} s vs {budget:.1f} s budget, "
                f"{num_frontiers} frontier images, {payload_bytes / 1024:.0f} KiB) "
                '— every robot is now flying to a stale assignment')
        if call.get('invalid_ids'):
            self.get_logger().warn(
                f"[search_planner] round {self._round}: model returned frontier ids that "
                f"were not offered: {call['invalid_ids']}")

    def _resolve_frontier(self, assignment, i, target_point_list, map_size):
        try:
            idx = int(str(assignment[f'robot_{i}']).split('_')[1])
            return target_point_list[idx]
        except (KeyError, ValueError, IndexError):
            self.get_logger().warn(
                f'conavgpt2: unusable assignment for robot_{i}: {assignment!r}')
            return target_point_list[0] if target_point_list else self._random_goal(map_size)

    def _random_goal(self, map_size):
        action = np.random.rand(2) * (map_size - 1)
        return [int(action[0]), int(action[1])]

    # ── actuation ─────────────────────────────────────────────────────────────

    def _instance_centroids(self, agent):
        """Cluster `object_pcd` into instances, as map-frame (x, y) centroids.

        DBSCAN rather than connected components: the cloud is unstructured
        detection samples, not a raster, and instances are separated by real gaps
        (52 m between houses on this bench). `target_cluster_eps_m` is that gap
        scale — too large and two neighbouring houses merge into one instance,
        too small and one house splits into several.
        """
        pcd = getattr(agent, 'object_pcd', None)
        if pcd is None or len(pcd.points) < self._inst_min_pts:
            return []
        pts = np.asarray(pcd.points)
        try:
            labels = np.array(pcd.cluster_dbscan(
                eps=self._inst_eps, min_points=self._inst_min_pts,
                print_progress=False))
        except Exception as exc:
            self.get_logger().warn(f'target clustering failed: {exc}',
                                   throttle_duration_sec=30.0)
            return []
        out = []
        for lab in set(int(x) for x in labels if x >= 0):
            m = labels == lab
            c = pts[m].mean(axis=0)
            out.append(o3d_xz_to_map_xy(c[0], c[2]))
        return out

    def release_nav_goals(self, timeout_s=2.0):
        """Cancel every outstanding NavigateTask before exiting.

        droan_gl guards its action server with a plain `task_active_` bool that
        is only cleared when a goal finishes or is cancelled (droan_gl_node.cpp,
        handle_navigate_goal). It is not tied to the client that set it, so a
        planner that dies without cancelling leaves the flag latched and EVERY
        later goal — from this or any other node — is rejected with "task
        already active" until droan itself is restarted. The drone then hovers
        with a planner that looks healthy, which is expensive to diagnose in a
        mission log. Cancelling on the way out is what keeps a planner restart
        survivable.
        """
        deadline = time.time() + timeout_s
        for i, nav in enumerate(getattr(self, '_nav', []) or []):
            h = getattr(nav, 'handle', None)
            if h is None:
                continue
            try:
                h.cancel_goal_async()
                self.get_logger().info(
                    f'[{self._robots[i]}] cancelled NavigateTask on shutdown')
            except Exception as exc:
                self.get_logger().warn(f'nav cancel failed: {exc}')
            nav.handle = None
        # Give the cancels a moment to reach droan; the executor is already
        # stopping, so spin them out by hand rather than relying on it.
        while time.time() < deadline:
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception:
                break

    def _publish_search_markers(self, agent):
        """Robot, trail and target instances, in the map frame.

        This is the answer to "where is it, where has it been, and what has it
        already dealt with". Target colour IS the visit state, so a glance says
        whether an instance still owes a visit:

            RED     seen, not yet visited — still a candidate
            YELLOW  the instance being flown to right now
            GREEN   visited; inside target_visit_radius_m it is never chosen
                    again, which is what stops the repeated-visit loop

        The green discs are drawn at the geofence radius rather than at some
        arbitrary size, so what is on screen is literally the region that
        suppresses a revisit.
        """
        try:
            here = self._agent_xy(agent)
            # camera_position is open3d (x, y, z); map z is its y, the same
            # convention the voxel integration uses.
            z = float(agent.camera_position[1])
            if (not self._trail
                    or math.dist(self._trail[-1][:2], here)
                    >= self._trail_min_step_m):
                self._trail.append((float(here[0]), float(here[1]), z))
                if len(self._trail) > self._trail_max:
                    del self._trail[0]

            stamp = self.get_clock().now().to_msg()
            ma = MarkerArray()

            def _mk(ns, mid, mtype):
                m = Marker()
                m.header.stamp = stamp
                m.header.frame_id = self._map_frame
                m.ns, m.id, m.type = ns, mid, mtype
                m.action = Marker.ADD
                m.pose.orientation.w = 1.0
                return m

            # The robot. Sized to be legible over a 240 m map, not to scale.
            r = _mk('search_robot', 0, Marker.SPHERE)
            r.pose.position.x, r.pose.position.y, r.pose.position.z = (
                float(here[0]), float(here[1]), z)
            r.scale.x = r.scale.y = r.scale.z = 4.0
            r.color = ColorRGBA(r=0.1, g=0.6, b=1.0, a=0.95)
            ma.markers.append(r)

            lbl = _mk('search_robot_id', 0, Marker.TEXT_VIEW_FACING)
            lbl.pose.position.x, lbl.pose.position.y = float(here[0]), float(here[1])
            lbl.pose.position.z = z + 4.0
            lbl.scale.z = 3.0
            lbl.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            lbl.text = f'{self._robots[0]} @ {z:.0f}m'
            ma.markers.append(lbl)

            if len(self._trail) > 1:
                t = _mk('search_trail', 0, Marker.LINE_STRIP)
                t.scale.x = 0.6
                t.color = ColorRGBA(r=0.1, g=0.6, b=1.0, a=0.55)
                t.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self._trail]
                ma.markers.append(t)

            # RAW detector output, before clustering. Instances (the discs
            # below) only appear once the cloud is dense enough to cluster, so
            # a real but sparse detection is otherwise invisible — which is
            # exactly the case that looks like "it stopped for no reason".
            pcd = getattr(agent, 'object_pcd', None)
            n_det = len(pcd.points) if pcd is not None else 0
            if n_det:
                pts = np.asarray(pcd.points)
                if pts.shape[0] > self._det_marker_max:
                    pts = pts[::max(1, pts.shape[0] // self._det_marker_max)]
                dm = _mk('search_detection', 0, Marker.CUBE_LIST)
                dm.scale.x = dm.scale.y = dm.scale.z = 1.0
                dm.color = ColorRGBA(r=1.0, g=0.1, b=0.85, a=0.8)
                for q in pts:
                    mx, my = o3d_xz_to_map_xy(float(q[0]), float(q[2]))
                    dm.points.append(Point(x=mx, y=my, z=float(q[1])))
                ma.markers.append(dm)

                dl = _mk('search_detection_id', 0, Marker.TEXT_VIEW_FACING)
                c = np.asarray(pcd.points).mean(axis=0)
                cx, cy = o3d_xz_to_map_xy(float(c[0]), float(c[2]))
                dl.pose.position.x, dl.pose.position.y = cx, cy
                dl.pose.position.z = float(c[1]) + 8.0
                dl.scale.z = 2.5
                dl.color = ColorRGBA(r=1.0, g=0.4, b=0.95, a=0.95)
                dl.text = (f'{self._goal_name} DETECTED: {n_det} pts, '
                           f'{len(self._target_instances)} instance(s)')
                ma.markers.append(dl)

            active = self._active_target
            for k, xy in enumerate(self._target_instances):
                done = self._is_visited(xy)
                is_active = (active is not None
                             and math.dist(active, xy) < 1e-6)
                d = _mk('search_target', k, Marker.CYLINDER)
                d.pose.position.x, d.pose.position.y = float(xy[0]), float(xy[1])
                d.pose.position.z = 0.5
                # Visited discs show the geofence itself; unvisited ones are
                # drawn small, because their extent is not yet meaningful.
                rad = self._visit_radius if done else 4.0
                d.scale.x = d.scale.y = 2.0 * rad
                d.scale.z = 1.0
                d.color = (ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.35) if done
                           else ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.75) if is_active
                           else ColorRGBA(r=1.0, g=0.15, b=0.1, a=0.75))
                ma.markers.append(d)

                tl = _mk('search_target_id', k, Marker.TEXT_VIEW_FACING)
                tl.pose.position.x, tl.pose.position.y = float(xy[0]), float(xy[1])
                tl.pose.position.z = 6.0
                tl.scale.z = 2.5
                tl.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
                state = 'VISITED' if done else ('VISITING' if is_active else 'FOUND')
                tl.text = f'{self._goal_name} {k} {state}'
                ma.markers.append(tl)

            # Retire ids that no longer exist. DELETEALL every publish would
            # clear the array first and read as a flicker, so delete only what
            # actually went away.
            n = len(self._target_instances)
            for k in range(n, self._max_target_marker + 1):
                for ns in ('search_target', 'search_target_id'):
                    d = Marker()
                    d.header.stamp = stamp
                    d.header.frame_id = self._map_frame
                    d.ns, d.id, d.action = ns, k, Marker.DELETE
                    ma.markers.append(d)
            self._max_target_marker = max(self._max_target_marker, n - 1)

            self._search_marker_pub.publish(ma)
        except Exception as exc:
            self.get_logger().warn(f'search marker publish failed: {exc}',
                                   throttle_duration_sec=10.0)

    def _is_visited(self, xy):
        return any(math.dist(xy, v) <= self._visit_radius
                   for v in self._visited_targets)

    def _update_targets(self, agent):
        """Refresh instances, retire any the drone has now reached, and choose
        the nearest unvisited one. Returns that target, or None."""
        pcd = getattr(agent, 'object_pcd', None)
        self._target_instances = self._instance_centroids(agent)
        self.get_logger().info(
            f'targets: object_pcd {len(pcd.points) if pcd is not None else 0} pts '
            f'| found_goal {bool(getattr(agent, "found_goal", False))} '
            f'| instances {len(self._target_instances)} '
            f'| visited {len(self._visited_targets)} '
            f'| detector {getattr(agent, "last_detection", {})}',
            throttle_duration_sec=10.0)
        if not self._target_instances:
            self._active_target = None
            return None

        here = self._agent_xy(agent)
        # Retire on ARRIVAL, not on detection — the point of the blacklist is
        # "we have been there", and a target retired on sight would let the
        # drone tick instances off from across the map without ever visiting.
        for xy in self._target_instances:
            if math.dist(here, xy) <= self._visit_radius and not self._is_visited(xy):
                self._visited_targets.append(xy)
                self.get_logger().info(
                    f'target VISITED at ({xy[0]:.1f}, {xy[1]:.1f}) — '
                    f'{len(self._visited_targets)} done, blacklisted; '
                    f'{sum(1 for t in self._target_instances if not self._is_visited(t))} '
                    'unvisited in view')

        unvisited = [t for t in self._target_instances if not self._is_visited(t)]
        if not unvisited:
            self._active_target = None
            return None
        self._active_target = min(unvisited, key=lambda t: math.dist(here, t))
        return self._active_target

    def _goal_xy(self, i, agent):
        """Where this robot should go, in map-frame metres."""
        if self._track_instances:
            # The nearest UNVISITED instance, not upstream's nearest point of a
            # merged cloud — that would keep re-selecting a target already flown
            # to, because nothing in upstream retires one.
            if self._active_target is not None:
                return self._active_target
        elif agent.found_goal and agent.nearest_point is not None:
            p = np.asarray(agent.nearest_point)
            return o3d_xz_to_map_xy(p[0], p[2])
        return agent.grid_to_map_xy(*self._goal_points[i])

    def _build_path(self, agent, goal_xy, offset):
        """The Path droan_gl steers by.

        'waypoints' (default) is raven_nav's shape: the goal, then one point
        past it along the approach direction so droan has a heading THROUGH the
        final pose rather than stopping dead on it. Two poses, and the local
        planner owns everything between here and there.

        'fmm_path' prepends upstream's own FMM waypoints. See `plan_output`.
        """
        if self._plan_output == 'waypoints':
            cur = self._agent_xy(agent)
            d = np.array(goal_xy) - np.array(cur)
            n = float(np.linalg.norm(d))
            direction = d / n if n > 1e-6 else np.array([1.0, 0.0])
            pts = [tuple(goal_xy),
                   tuple(np.array(goal_xy) + direction * self._path_extension_m)]
            return self._path_msg(pts, offset, self._waypoint_z(agent))

        pts = [p for p in agent.plan_path_map_xy()]
        pts.append(goal_xy)

        # Drop waypoints that double back onto the goal, and de-duplicate.
        cleaned = []
        for p in pts:
            if not cleaned or math.dist(p, cleaned[-1]) > 0.5:
                cleaned.append(p)
        if len(cleaned) >= 2:
            a, b = np.array(cleaned[-2]), np.array(cleaned[-1])
            d = b - a
            n = np.linalg.norm(d)
            direction = d / n if n > 1e-6 else np.array([1.0, 0.0])
        else:
            direction = np.array([1.0, 0.0])
        cleaned.append(tuple(np.array(cleaned[-1]) + direction * self._path_extension_m))
        return self._path_msg(cleaned, offset, self._waypoint_z(agent))

    def _agent_xy(self, agent):
        """Where this agent currently is, in map-frame metres."""
        return agent.grid_to_map_xy(*agent.current_grid_pose)

    def _frontier_altitude(self, i):
        """Height of the frontier this robot was assigned, when the frontier
        source supplies one. A 2D slab cannot, so this returns None there and
        the caller falls back to the configured cruise altitude."""
        if self._frontier_source != 'voxel3d' or not self._frontier_z:
            return None
        try:
            goal = self._goal_points[i]
        except (IndexError, TypeError):
            return None
        # _goal_points holds grid cells; match it back to the candidate it came
        # from so the z belongs to THIS frontier and not to whichever was last.
        for k, p in enumerate(self._frontier_candidates):
            if int(p[0]) == int(goal[0]) and int(p[1]) == int(goal[1]):
                if k < len(self._frontier_z):
                    return self._clamp_alt(self._frontier_z[k])
        return None

    def _waypoint_z(self, agent):
        """Altitude for this tick's waypoints, in map-frame metres.

        Cruise, unless the target has been detected and inspect_altitude_m is
        set. Upstream has no z at all — its goal is a 2D grid cell — so this is
        the one place a 2D method has to be told what "there" means in 3D.
        """
        if self._inspect_altitude <= 0.0 or not getattr(agent, 'found_goal', False):
            fz = self._frontier_altitude(self._agents.index(agent)
                                         if agent in self._agents else 0)
            # _frontier_altitude already clamps to the FLIGHT band, so a
            # frontier at 3 m does not fly the drone at 3 m.
            return fz if fz is not None else self._clamp_alt(self._altitude)
        if getattr(agent, 'nearest_point', None) is None:
            return self._clamp_alt(self._inspect_altitude)
        # open3d axis 1 is height, and T_MAP_TO_O3D maps map z -> o3d y, so this
        # IS the detection's height above the takeoff plane. Descend to the
        # standoff above whatever was actually seen rather than to a fixed AGL,
        # so a target on a roof is not approached at ground height.
        try:
            target_z = float(np.asarray(agent.nearest_point)[1])
        except (TypeError, ValueError, IndexError):
            return self._clamp_alt(self._inspect_altitude)
        return self._clamp_alt(max(self._inspect_altitude,
                                   target_z + self._inspect_altitude))

    def _clamp_alt(self, z):
        """Hold the waypoint inside the mission's altitude band. 0 on either
        bound leaves that side unconstrained, so a scene that does not care about
        a floor or a ceiling does not have to invent one."""
        if self._min_alt > 0.0:
            z = max(z, self._min_alt)
        if self._max_alt > 0.0:
            z = min(z, self._max_alt)
        return z

    def _path_msg(self, pts, offset, z=None):
        z = self._altitude if z is None else z
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._map_frame
        for x, y in pts:
            ps = PoseStamped()
            ps.header = path.header
            # The grid lives in the merge frame; droan_gl wants this robot's own
            # local 'map', so drop the boot_enu offset back off on the way out.
            ps.pose.position.x = float(x - offset[0])
            ps.pose.position.y = float(y - offset[1])
            ps.pose.position.z = float(z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    def _command(self, i, agent, offset):
        goal_xy = self._goal_xy(i, agent)
        path = self._build_path(agent, goal_xy, offset)
        self._plan_pubs[i].publish(path)

        if self._nav_activation == 'activator':
            self._ensure_activator(i)
            return

        nav = self._nav[i]
        endpoint = np.array([path.poses[-1].pose.position.x,
                             path.poses[-1].pose.position.y,
                             path.poses[-1].pose.position.z])

        active = nav.handle is not None and nav.result_future is not None \
            and not nav.result_future.done()
        if active:
            moved = nav.endpoint is None or \
                float(np.linalg.norm(endpoint - nav.endpoint)) > self._goal_change_m
            timed_out = (time.time() - nav.sent_at) > self._nav_timeout
            if not moved and not timed_out:
                return
            try:
                cancel_future = nav.handle.cancel_goal_async()
                # droan_gl's execute loop runs at rclcpp::Rate(1.0) and only
                # notices a cancel on a tick, so the goal stays ACTIVE for up to
                # a second after the ack. Re-sending inside that window is
                # rejected with "task already active" and the drone silently
                # keeps flying the previous round's assignment.
                self._await(cancel_future, 2.0)
                self._await(nav.result_future, 2.0)
            except Exception:
                pass
            nav.handle = None
            nav.result_future = None

        if not self._nav_clients[i].wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                f'[{self._robots[i]}] NavigateTask server not available '
                f'({self._nav_tpl.format(robot=self._robots[i])})',
                throttle_duration_sec=10.0)
            return

        goal = NavigateTask.Goal()
        goal.global_plan = path
        goal.goal_tolerance_m = self._goal_tolerance
        send_future = self._nav_clients[i].send_goal_async(goal)
        deadline = time.time() + 3.0
        while not send_future.done() and time.time() < deadline and rclpy.ok() \
                and not self._stop:
            time.sleep(0.02)
        handle = send_future.result() if send_future.done() else None
        if handle is None or not getattr(handle, 'accepted', False):
            self.get_logger().warn(f'[{self._robots[i]}] NavigateTask goal not accepted',
                                   throttle_duration_sec=10.0)
            return
        nav.handle = handle
        nav.result_future = handle.get_result_async()
        nav.endpoint = endpoint
        nav.sent_at = time.time()

    def _await(self, future, timeout_s):
        """Block this planning thread until `future` resolves. The node spins on
        its own executor thread, so waiting here does not deadlock it."""
        deadline = time.time() + timeout_s
        while future is not None and not future.done() and time.time() < deadline \
                and rclpy.ok() and not self._stop:
            time.sleep(0.02)
        return future is not None and future.done()

    def _ensure_activator(self, i):
        """Keep ONE empty NavigateTask goal alive on droan_gl.

        droan_gl treats a goal whose `global_plan` is empty as a pure activator:
        it enters ADD_SEGMENT and steers by the `global_plan` TOPIC until
        cancelled, rather than flying a path baked into the goal
        (`droan_gl_node.cpp` `execute_navigate`). Since this node republishes the
        FMM path every tick anyway, that makes the topic the single steering
        source and a new VLM assignment takes effect within one plan tick.

        A fresh goal per round cannot do that: it has to cancel the previous one
        first, droan notices a cancel only on its 1 Hz tick, and the re-send lands
        inside that window and is rejected with "task already active" — leaving
        the drone flying round 1's assignment for the rest of the run.
        """
        # Shutting down: release_nav_goals() spins this node to flush the
        # cancels, which keeps timers running. Without this guard the tick
        # re-sends an activator milliseconds after cancelling one and can
        # re-latch droan on the way out.
        if getattr(self, '_stop', False):
            return
        nav = self._nav[i]
        if nav.handle is not None and nav.result_future is not None \
                and not nav.result_future.done():
            return
        # Rejected or finished: back off before retrying, so a droan that is
        # still winding a cancelled goal down is not hammered every tick.
        if nav.sent_at and (time.time() - nav.sent_at) < self._activator_retry_s:
            return
        nav.sent_at = time.time()
        robot = self._robots[i]
        if not self._nav_clients[i].wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                f'[{robot}] NavigateTask server not available '
                f'({self._nav_tpl.format(robot=robot)})',
                throttle_duration_sec=10.0)
            return
        goal = NavigateTask.Goal()          # empty global_plan == activator
        goal.goal_tolerance_m = self._goal_tolerance
        send_future = self._nav_clients[i].send_goal_async(goal)
        self._await(send_future, 3.0)
        handle = send_future.result() if send_future.done() else None
        if handle is None or not getattr(handle, 'accepted', False):
            self.get_logger().warn(
                f'[{robot}] droan_gl activator not accepted — retrying in '
                f'{self._activator_retry_s:.0f} s', throttle_duration_sec=10.0)
            return
        nav.handle = handle
        nav.result_future = handle.get_result_async()
        nav.endpoint = None
        self.get_logger().info(
            f'[{robot}] droan_gl activated — steering by '
            f'{self._plan_tpl.format(robot=robot)}')

    # ── visualisation ─────────────────────────────────────────────────────────

    def _grid_origin_cells(self):
        """`origins_grid` is the cell the map frame's origin sits in. Map_Extraction
        offsets by int(map_size/2), and grid_to_map_xy reads origins_grid; they
        agree, but read the agent's so the two can never drift apart."""
        agent = self._agents[0]
        return float(int(agent.origins_grid[0])), float(int(agent.origins_grid[1]))

    def _publish_occupancy(self, obstacle_map, explored_map):
        """Upstream's own grid as a nav_msgs/OccupancyGrid.

        Values follow the ROS convention rather than upstream's two float
        planes:

            -1  UNKNOWN   never observed. Foxglove draws this transparent, so
                          the sim ground shows through and the grid reads as
                          coverage.
             0  FREE      observed, and every point in the cell fell OUTSIDE
                          the obstacle altitude band. Frontiers grow from the
                          boundary of THIS, which is why the band must exclude
                          the ground.
           100  OCCUPIED  something stands inside [obstacle_min_z_m,
                          obstacle_max_z_m] here.

        That is the map the method actually reasons over, not a re-derivation
        of it. Note it is NOT the same as the greyscale in
        the map_image topic, which is upstream's own render: there white is
        unknown, light grey is free and dark grey is obstacle, with the red
        frontier boundary drawn on top.
        """
        try:
            size = int(obstacle_map.shape[0])
            res = self._map_resolution / 100.0
            grid = np.full((size, size), -1, dtype=np.int8)
            grid[explored_map > 0.1] = 0
            grid[obstacle_map > 0.1] = 100

            # TARGET cells, stamped over the obstacle layer — a detected house IS
            # an obstacle, and which of the two you want to see is the target.
            # 101 is deliberately OUTSIDE the ROS 0..100 range so a viewer paints
            # it with its "invalid" colour, giving a fourth distinct class in a
            # message type that only defines three. See the layout's
            # invalidColor.
            for xy in self._target_instances:
                self._stamp_target(grid, xy, size)

            # Upstream indexes [i, j] with i along +x and j along +o3d_z, and
            # map_y = -o3d_z. An OccupancyGrid is row-major in (y, x) with +y
            # increasing by row, so transpose and flip the row order.
            data = np.flipud(grid.T)

            ox, oy = self._grid_origin_cells()
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._map_frame
            msg.info.resolution = res
            msg.info.width = size
            msg.info.height = size
            # Lower-left CORNER of cell (0, 0), not its centre: upstream floors
            # into the grid, so cell i spans [(i - ox) * res, (i - ox + 1) * res).
            msg.info.origin.position.x = -ox * res
            msg.info.origin.position.y = -(size - oy) * res
            msg.info.origin.position.z = self._occupancy_z
            msg.info.origin.orientation.w = 1.0
            msg.data = data.reshape(-1).tolist()
            self._occupancy_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'occupancy publish failed: {exc}',
                                   throttle_duration_sec=10.0)

    def _publish_value_map(self):
        """The value field on the SAME grid geometry as the occupancy topic, so
        the two overlay cell-for-cell in Foxglove with no resampling."""
        try:
            vm = self._value_map
            data = np.flipud(vm.as_occupancy_bytes().T)
            res = self._map_resolution / 100.0
            ox, oy = self._grid_origin_cells()
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._map_frame
            msg.info.resolution = res
            msg.info.width = vm.size
            msg.info.height = vm.size
            msg.info.origin.position.x = -ox * res
            msg.info.origin.position.y = -(vm.size - oy) * res
            # Just above the occupancy layer so the two do not z-fight.
            msg.info.origin.position.z = self._occupancy_z + 0.25
            msg.info.origin.orientation.w = 1.0
            msg.data = data.reshape(-1).tolist()
            self._value_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'value map publish failed: {exc}',
                                   throttle_duration_sec=10.0)

    def _publish_voxel_cloud(self):
        """The OCCUPIED voxels, as a coloured XYZRGB cloud.

        Occupied only. The map is three-state internally and the frontier
        predicate needs all three, but drawing the free voxels renders open air
        as solid blocks — the map reads as though everything were an obstacle.
        Free and unobserved stay in the grid; they just are not published.
        """
        try:
            xyz, rgb = self._voxel_map.as_points(
                states=(VOX_OCCUPIED,),
                max_points=self._vox_cloud_max_points)
            if xyz.shape[0] == 0:
                return
            packed = ((np.clip(rgb[:, 0] * 255, 0, 255).astype(np.uint32) << 16)
                      | (np.clip(rgb[:, 1] * 255, 0, 255).astype(np.uint32) << 8)
                      | np.clip(rgb[:, 2] * 255, 0, 255).astype(np.uint32))
            arr = np.zeros(xyz.shape[0], dtype=[
                ('x', np.float32), ('y', np.float32), ('z', np.float32),
                ('rgb', np.uint32)])
            arr['x'], arr['y'], arr['z'] = (xyz[:, 0], xyz[:, 1], xyz[:, 2])
            arr['rgb'] = packed
            msg = PointCloud2()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._map_frame
            msg.height = 1
            msg.width = arr.shape[0]
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
            msg.is_bigendian = False
            msg.point_step = 16
            msg.row_step = 16 * arr.shape[0]
            msg.is_dense = True
            msg.data = arr.tobytes()
            self._voxel_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'voxel cloud publish failed: {exc}',
                                   throttle_duration_sec=10.0)

    def _stamp_target(self, grid, xy, size):
        """Paint a disc of TARGET cells at a map-frame instance centroid.

        A disc rather than the detected cells themselves: the detection cloud is
        sparse and speckled, and what matters visually is "there is a target
        here", at the radius the blacklist actually uses.
        """
        res = self._map_resolution / 100.0
        ox, oy = self._grid_origin_cells()
        ci = int(np.floor(xy[0] / res) + ox)
        cj = int(np.floor(-xy[1] / res) + oy)
        r = max(1, int(round(self._visit_radius / res)))
        i0, i1 = max(0, ci - r), min(size, ci + r + 1)
        j0, j1 = max(0, cj - r), min(size, cj + r + 1)
        if i0 >= i1 or j0 >= j1:
            return
        ii, jj = np.ogrid[i0:i1, j0:j1]
        grid[i0:i1, j0:j1][(ii - ci) ** 2 + (jj - cj) ** 2 <= r * r] = 101

    def _publish_frontiers(self, target_edge_map, target_point_list):
        """The candidate frontiers, as `frontier_marker_style` asks for.

        The CENTROID is upstream's `Goal_point` — the single point a robot is
        actually sent to — and is what the VLM's `frontier_N` id refers to. The
        REGION is the connected run of free-space boundary that centroid came
        from; it reads as a line rather than a dot because that is what a
        frontier is. Default is centroids only, because the regions are the
        noisier half and the decision lives in the centroid.

        Green is the frontier assigned this round, orange one that was offered
        and not taken. There are never more than SIX — `Frontier_Det` breaks at
        `i == 5`, which is what bounds the VLM prompt to six images.
        """
        try:
            agent = self._agents[0]
            res = self._map_resolution / 100.0
            assigned = {(int(g[0]), int(g[1])) for g in self._goal_points}
            edge = np.asarray(target_edge_map)
            stamp = self.get_clock().now().to_msg()

            ma = MarkerArray()
            # DELETEALL first: the count shrinks as the map closes, and without
            # it the previous round's extra markers stay on screen forever.
            clear = Marker()
            clear.header.frame_id = self._map_frame
            clear.action = Marker.DELETEALL
            ma.markers.append(clear)

            for k, p in enumerate(target_point_list):
                is_goal = (int(p[0]), int(p[1])) in assigned
                colour = (ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.9) if is_goal
                          else ColorRGBA(r=0.9, g=0.25, b=0.1, a=0.7))
                if self._frontier_style in ('cells', 'both'):
                    self._add_region(ma, edge, k, stamp, colour, res, agent)
                cx, cy = agent.grid_to_map_xy(p[0], p[1])
                # The frontier's OWN height when the source supplies one. Drawing
                # 3D frontiers at a single altitude would hide the only thing
                # that distinguishes them from the 2D slab.
                cz = (self._frontier_z[k]
                      if k < len(self._frontier_z) else self._altitude)
                self._add_centroid(ma, k, cx, cy, cz, stamp, is_goal, res)

            self._frontier_pub.publish(ma)
            self._publish_frontier_cloud(target_point_list, agent, res)
        except Exception as exc:
            self.get_logger().warn(f'frontier marker publish failed: {exc}',
                                   throttle_duration_sec=10.0)

    def _publish_frontier_cloud(self, target_point_list, agent, res):
        """The same candidates as an XYZRGB cloud.

        A MarkerArray fixes point size at publish time; a cloud lets the viewer
        scale it, which is the difference between frontiers that read as dots
        over a 278 m plat and frontiers that swallow it.

        Green is the committed goal, orange a candidate. The goal is matched by
        DISTANCE, not by grid cell: the candidate set is re-extracted every
        tick and the committed cell usually is not in it any more, which is why
        nothing ever drew green.
        """
        try:
            if not target_point_list:
                return
            goal_xy = getattr(self, '_locked_goal_xy', None)
            pts = []
            for k, p in enumerate(target_point_list):
                cx, cy = agent.grid_to_map_xy(p[0], p[1])
                cz = (self._frontier_z[k] if k < len(self._frontier_z)
                      else self._altitude)
                is_goal = (goal_xy is not None
                           and math.dist((cx, cy), goal_xy) <= self._goal_tolerance)
                rgb = (0x19E533 if is_goal else 0xE64019)
                pts.append((float(cx), float(cy), float(cz), rgb))

            arr = np.zeros(len(pts), dtype=[('x', np.float32), ('y', np.float32),
                                           ('z', np.float32), ('rgb', np.uint32)])
            for i, (x, y, z, c) in enumerate(pts):
                arr[i] = (x, y, z, c)
            msg = PointCloud2()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._map_frame
            msg.height, msg.width = 1, arr.shape[0]
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
            msg.is_bigendian = False
            msg.point_step = 16
            msg.row_step = 16 * arr.shape[0]
            msg.is_dense = True
            msg.data = arr.tobytes()
            self._frontier_cloud_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'frontier cloud publish failed: {exc}',
                                   throttle_duration_sec=10.0)

    def _add_region(self, ma, edge, k, stamp, colour, res, agent):
        """One cube per grid cell of frontier k, so the drawn region IS the
        region — no radius is invented."""
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self._map_frame
        m.ns = 'conavgpt2_frontier'
        m.id = k
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = res
        m.scale.z = res * 0.5
        m.color = colour
        cells = np.argwhere(edge == (k + 1))
        if not len(cells):
            return
        # THIN rather than truncate: truncating would draw only one end of the
        # region and misrepresent where it is.
        stride = max(1, len(cells) // max(1, self._frontier_max_cells))
        zk = (self._frontier_z[k] if k < len(self._frontier_z)
              else self._altitude)
        for ci, cj in cells[::stride]:
            gx, gy = agent.grid_to_map_xy(int(ci), int(cj))
            m.points.append(Point(x=float(gx), y=float(gy), z=float(zk)))
        if m.points:
            ma.markers.append(m)

    def _add_centroid(self, ma, k, cx, cy, cz, stamp, is_goal, res):
        """The goal point, sized to goal_tolerance_m — so what is drawn is the
        disc the drone has to reach, not an arbitrary blob — plus the id the VLM
        was shown, so an assignment in the round table can be pointed at."""
        c = Marker()
        c.header.stamp = stamp
        c.header.frame_id = self._map_frame
        c.ns = 'conavgpt2_frontier_goal'
        c.id = k
        c.type = Marker.SPHERE
        c.action = Marker.ADD
        c.pose.position.x = float(cx)
        c.pose.position.y = float(cy)
        c.pose.position.z = float(cz)
        c.pose.orientation.w = 1.0
        c.scale.x = c.scale.y = c.scale.z = 2.0 * self._goal_tolerance
        c.color = (ColorRGBA(r=0.1, g=1.0, b=0.3, a=0.85) if is_goal
                   else ColorRGBA(r=1.0, g=0.6, b=0.1, a=0.55))
        ma.markers.append(c)

        t = Marker()
        t.header.stamp = stamp
        t.header.frame_id = self._map_frame
        t.ns = 'conavgpt2_frontier_id'
        t.id = k
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = float(cx)
        t.pose.position.y = float(cy)
        # Floats a fixed number of CELLS above the point, so it stays legible
        # whether a cell is 0.6 m or 3.5 m.
        t.pose.position.z = float(cz) + 6.0 * res
        t.text = f'frontier_{k} @ {cz:.0f}m'
        t.pose.orientation.w = 1.0
        t.scale.z = max(1.5, 8.0 * res)
        t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        ma.markers.append(t)

    def _publish_image(self, pub, bgr_or_rgb, frame_id, rgb=False):
        try:
            img = bgr_or_rgb if rgb else bgr_or_rgb[:, :, ::-1]
            msg = self._bridge.cv2_to_imgmsg(
                np.ascontiguousarray(img.astype(np.uint8)), encoding='rgb8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'image publish failed: {exc}',
                                   throttle_duration_sec=10.0)

    def _publish_map_image(self, step, vis_pose_pred, obstacle_map, explored_map,
                           visited_vis, target_edge_map, goal_maps, top_view_map):
        saved = self._args.print_images
        self._args.print_images = 1 if self._save_debug_images else 0
        try:
            image = vu.Visualize(
                self._args, step, vis_pose_pred, obstacle_map, explored_map,
                self._goal_name, visited_vis, target_edge_map, goal_maps,
                # Visualize() drops top_view_map straight into a BGR canvas, but it
                # is filled from RGB colours; pre-swap so the published image reads
                # correctly. The VLM's candidate images go through PIL and are
                # already right, so they are left alone.
                top_view_map[:, :, ::-1])
        finally:
            self._args.print_images = saved
        if image is not None:
            self._publish_image(self._map_image_pub, image, self._map_frame)

    def _publish_vlm_image(self, candidate_maps):
        """The exact frames handed to the VLM, tiled — the single most useful thing
        to look at when an assignment makes no sense."""
        tiles = []
        for buf in candidate_maps:
            arr = cv2.imdecode(np.frombuffer(buf.getvalue(), np.uint8), cv2.IMREAD_COLOR)
            if arr is not None:
                tiles.append(cv2.resize(arr, (480, 480), interpolation=cv2.INTER_NEAREST))
        if tiles:
            self._publish_image(self._vlm_image_pub, np.hstack(tiles), self._map_frame)

    def _publish_agent_images_now(self):
        for i, agent in enumerate(self._agents):
            if agent.annotated_image is not None:
                self._publish_image(self._agent_image_pubs[i], agent.annotated_image,
                                    self._robots[i])


def main(args=None):
    if not _VENDOR_OK:
        raise SystemExit(
            f'conavgpt2: vendored dependencies unavailable ({_VENDOR_ERR}). '
            'It needs scikit-fmm, open3d, ultralytics, supervision==0.19.0 and '
            'openai; see the package README for the venv install.')
    if not _TASK_MSGS_OK:
        raise SystemExit(
            f'conavgpt2: task_msgs unavailable ({_TASK_MSGS_ERR}). Build the '
            'workspace (bws) and source it (sws) first.')

    rclpy.init(args=args)
    node = CoNavGPT2Node()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    def _on_term(_sig, _frm):
        node._stop = True
        raise KeyboardInterrupt

    for sig in (signal.SIGTERM, signal.SIGINT):
        try:
            signal.signal(sig, _on_term)
        except ValueError:
            pass        # not the main thread

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop = True
        # Before destroy_node(): once the node is destroyed the action clients
        # are gone and droan stays latched.
        try:
            node.release_nav_goals()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

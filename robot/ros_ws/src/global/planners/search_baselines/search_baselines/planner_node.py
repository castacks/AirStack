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

import colorsys
import json
import math
import os
import re
import signal
import threading
import time
import traceback
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
from sensor_msgs.msg import (CameraInfo, CompressedImage, Image, NavSatFix,
                             PointCloud2, PointField)
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
    from search_baselines import clearance as clr
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

# PER-ROBOT COLOUR — THE GCS's, NOT OUR OWN. The GCS visualizer draws every
# robot (mesh, trajectory, global plan) in `gcs_utils.ROBOT_COLORS[N-1]`,
# passed through `fluorescent()`; a sector outline, trail or lane set drawn
# in a different palette does not read as belonging to that drone. Mirrored
# here because gcs_utils is a GCS package the robot image does not carry.
# KEEP IN SYNC with gcs_utils.ROBOT_COLORS / fluorescent and
# render_layout._robot_color_hex.
ROBOT_COLORS = [
    (0.90, 0.10, 0.10),   # robot_1
    (0.10, 0.70, 0.20),   # robot_2
    (0.20, 0.40, 1.00),   # robot_3
    (1.00, 0.55, 0.00),
    (0.70, 0.30, 0.90),
    (0.00, 0.80, 0.85),
    (1.00, 0.85, 0.10),
    (1.00, 0.40, 0.70),
    (0.40, 0.80, 0.40),
    (0.55, 0.27, 0.07),
    (0.30, 0.30, 0.30),
    (0.95, 0.95, 0.95),
]


def _fluorescent(color):
    """gcs_utils.fluorescent: full value, saturation boosted 1.25x."""
    h, s_, _ = colorsys.rgb_to_hsv(*color)
    return colorsys.hsv_to_rgb(h, min(1.0, s_ * 1.25), 1.0)


def robot_rgb(index0):
    """The GCS colour of robot `index0 + 1` (robot_N -> ROBOT_COLORS[N-1])."""
    return _fluorescent(ROBOT_COLORS[int(index0) % len(ROBOT_COLORS)])


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
        # THE TEAM is not the robots this process drives. AirStack runs one
        # planner per robot container on its own ROS domain, so outside
        # team_mode this process drives ONE robot (its own) while `num_robots`
        # — NUM_ROBOTS from the launch — is the TEAM size, used to cut the
        # search area into that many sectors. team_mode is the old
        # one-process-for-the-whole-team layout (every robot's topics on one
        # domain). launch_xml cannot switch a <param> on a condition, so the
        # node does it.
        self._team_mode = bool(self._p('team_mode', False))
        team_size = max(1, self._num_robots)
        if not self._team_mode:
            self._num_robots = 1
        names = [n for n in self._p('robot_names', ['']) if n]
        if not names:
            template = self._p('robot_name_template', 'robot_{i}')
            base = int(self._p('robot_index_base', 1))
            names = [template.format(i=base + k) for k in range(self._num_robots)]
            if self._num_robots == 1 and os.environ.get('ROBOT_NAME'):
                names = [os.environ['ROBOT_NAME']]
        self._robots = names
        self._num_robots = len(self._robots)
        # THE TEAM is not the robots this process drives. AirStack runs one
        # planner per robot container on its own ROS domain, so `num_robots`
        # is 1 there — but the sector partition has to be cut into as many
        # slices as there are DRONES, or every container computes "1 of 1"
        # and all three sweep the whole area. `sector_count` is the team size
        # (the launch passes NUM_ROBOTS); 0 falls back to num_robots.
        self._sector_count = int(self._p('sector_count', 0)) or team_size

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
        # THE EVIDENCE for a detection: the annotated FPV, JPEG, published
        # ONLY on a tick where the goal class scored >= detector_log_conf
        # (0.5) — the frames the "detector SEEN"/"detector PASS" log lines
        # describe, mapped or not. Event-driven, so it is cheap to bridge
        # and record (hundreds of frames a run, not tens of thousands), and
        # it is the only way to tell a false positive from a mis-projected
        # person after the fact: the 2026-08-27 fleet run committed to 15
        # targets, every one 78-226 m from any GT person, with no image in
        # the bag to say what YOLO had actually seen.
        self._det_image_tpl = self._p(
            'detection_image_topic_template', '/{robot}/search/detection_image')
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
        # UPSTREAM'S RENDER CANVAS IS FIXED AT 640x480. `Global_Map_Proc`
        # composites the FPV into a hardcoded 480x640x3 array, so any other
        # size dies EVERY TICK with
        #   could not broadcast input array from shape (H,W,3) into (480,640,3)
        # which reads as a camera fault rather than a config error. Fail here,
        # once, with the reason. Raising the camera's own resolution is the
        # supported way to get more pixels on target: the planner resizes the
        # incoming frame to this, so a 720x450 ZED still delivers ~640x450 of
        # real detail against 480x300's upscaled ~480x300.
        if (self._frame_w, self._frame_h) != (640, 480):
            raise ValueError(
                f'frame_width/frame_height must be 640x480, got '
                f'{self._frame_w}x{self._frame_h}: upstream Global_Map_Proc '
                'renders into a hardcoded 480x640x3 canvas and any other size '
                'fails on every tick. Raise ZED_WIDTH/ZED_HEIGHT instead — the '
                'camera resolution is what sets pixels on target.')

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
        # A NAVIGATION map, not a survey. Voxels last written more than this
        # many seconds ago go back to unobserved (and the frontiers standing
        # in them are dropped), so the map stays local to the drone instead
        # of accumulating the whole scene. 0 keeps everything.
        self._vox_forget_s = float(self._p('voxel_forget_after_s', 60.0))
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
        # robot index -> (goal cell, z) the last time the goal's height was
        # resolved from the candidate list; see _frontier_altitude.
        self._goal_z_cache = {}

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
        self._marker_offset = None    # grid -> map, same as the published path
        self._locked_score = None
        self._locked_at = 0.0
        # ── unreachable frontiers: stall -> blacklist -> release ──────────
        # A voxel frontier can sit inside a house or under a canopy: droan_gl
        # sees every approach in collision, publishes hover, and the lock/swap
        # rule above never lets go — a rival has to beat the held score by
        # 35 %, and the held one keeps scoring well precisely because nobody
        # has looked there. Measured on the 250 m suburb: both people found and
        # visited in 60 s, then parked for the rest of the run 16 m from a
        # frontier at (15.5, 23.6). The lawnmower arm already had a stall
        # timer; this is the same idea for the frontier arms, with a memory so
        # the same cell cannot be picked again the next tick. This is DIFFERENT
        # from the visit cost below: that discourages re-covering explored
        # ground, this removes a frontier the drone physically cannot reach.
        self._stall_s = float(self._p('frontier_stall_s', 20.0))
        self._stall_dist = float(self._p('frontier_stall_dist_m', 1.5))
        self._blacklist_r = float(self._p('frontier_blacklist_radius_m', 8.0))
        self._blacklist_ttl = float(self._p('frontier_blacklist_ttl_s', 0.0))  # 0 = never expires
        self._frontier_blacklist = []   # [(x, y, t)] in GRID-frame metres, like _locked_goal_xy
        self._progress_xy = None
        self._progress_t = 0.0
        # The same watchdog for TARGET visits: an instance the drone cannot
        # reach (droan sees every approach in collision) is marked visited —
        # which is already the target blacklist — instead of being held
        # forever. Measured: 7 of 8 instances visited in 3 min, then parked
        # 40 m short of the 8th for the rest of the run.
        self._tgt_progress_xy = None
        self._tgt_progress_t = 0.0
        self._tgt_active_since = 0.0

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
        # The path is NEVER sent whole, and neither are its lane ends: the
        # lanes are cut into legs of at most this many metres and handed to
        # droan ONE AT A TIME, the way a frontier is, after a transit of the
        # same legs from wherever the drone is to the entry point of the
        # sweep. See lawnmower.Sweep for why a lane end alone is not enough.
        self._lm_leg_m = float(self._p('lawnmower_leg_m', 25.0))
        # No progress toward the current goal for this long -> skip it. The
        # frontier arms have lock/swap to get off an unreachable goal; a
        # lane leg inside a tree canopy would otherwise stall the whole run.
        self._lm_stall_s = float(self._p('lawnmower_stall_s', 30.0))
        # ── the 3D lawnmower: known obstacles and the clearance surface ─────
        # A lane goal at cruise height inside a tree canopy is a goal droan
        # can never reach; the sweep used to sit on it for `lawnmower_stall_s`
        # and then leave that ground unswept. A coverage path is planned over
        # a KNOWN map, so this arm reads the layout generator's own ground
        # truth (the scene's annotation JSON — house / tree / car boxes, the
        # same file the GCS draws) and flies each leg at
        # max(cruise, tallest box along the leg + clearance), clamped to the
        # flight band. See search_baselines/clearance.py. The frontier arms
        # do NOT get this: their map is what they have seen.
        self._obst_file = str(self._p('known_obstacles_file', ''))
        self._obst_scene = str(self._p('known_obstacles_scene', ''))
        self._obst_classes = [str(c) for c in self._p(
            'known_obstacle_classes', list(clr.OBSTACLE_CLASSES)) if c]
        self._obst_clear = float(self._p('obstacle_clearance_m', 3.0))
        self._obst_inflate = float(self._p('obstacle_inflate_m', 2.0))
        # Stall handling, up before out: a goal with no progress for
        # lawnmower_stall_s is flown `lawnmower_stall_lift_m` higher first
        # (something unlisted is in the way), `lawnmower_stall_lifts` times,
        # and only then skipped.
        self._lm_lift_m = float(self._p('lawnmower_stall_lift_m', 5.0))
        self._lm_lifts = int(self._p('lawnmower_stall_lifts', 2))
        self._obstacles_world = None     # KnownObstacles as authored (world)
        self._obstacles = None           # ... in this robot's MAP frame
        self._lm_leg_z = None            # (goal z, through z) of the current leg
        self._lm_path = None
        self._lm_idx = 0
        self._lm_sweep = None
        # (goal, through) of the current leg in GRID-frame metres, for
        # _build_path: the second pose is the next lane point, not a 2 m
        # extension, so droan has the lane's heading and progress along it.
        self._lm_leg = None
        self._lm_last_pick_t = None
        self._lm_phase = None
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
        # WHICH FRAME `search_area_xy` IS AUTHORED IN. 'map' is this robot's
        # local map, anchored at ITS TAKEOFF POINT — so a polygon written as
        # scene/world coordinates but read as map coordinates is displaced by
        # the spawn offset, and the drone confines itself to the wrong square
        # (16.5 m x 18 m off on the 250 m suburb, 340 m on the wildfire plat).
        # 'world' converts the polygon ONCE, the first tick the map origin is
        # known — measured as enu(GPS fix) - odom at the same instant, the same
        # derivation map_anchor_node and the GCS use — and the sector split /
        # pad above are translation-invariant, so they were applied in world
        # and simply move with it. Lawnmower lanes are shifted with it too.
        self._search_area_frame = str(self._p('search_area_frame', 'map')).lower()
        if self._search_area_frame not in ('map', 'world'):
            raise ValueError(
                f"search_area_frame must be 'map' or 'world', got "
                f"{self._search_area_frame!r}")
        # WHERE THE POLYGON COMES FROM. 'config' is `search_area_xy` above.
        # 'scene' reads the DISASTER-AFFECTED AREA the launcher wrote for
        # this scene — `<RESULTS_SCENE>_region.json`, beside the people GT and
        # the obstacle boxes (scene_gen/disaster/region.py): the fire-front
        # ellipse at the scene's burn time plus the evacuation band the
        # survivors are staged in ('affected'), or the burnt ground alone
        # ('burn'). The search then covers the ground the disaster touched
        # rather than the whole plat, and it follows the scene instead of a
        # number typed into a config. The file is authored in WORLD, so the
        # frame is forced to 'world'; `search_area_pad_m` grows it (the scar
        # fingers ~80 m past the front on a 1 km plat) and the sector split
        # cuts it, exactly as for a config polygon. A MISSING FILE IS FATAL:
        # a run that quietly fell back to the whole map would be
        # indistinguishable from one that searched the affected area, only
        # slower — the same reason an unreachable detector is fatal.
        self._search_area_source = str(
            self._p('search_area_source', 'config')).lower()
        self._search_area_scene_key = str(
            self._p('search_area_scene_key', 'affected'))
        if self._search_area_source not in ('config', 'scene'):
            raise ValueError(
                f"search_area_source must be 'config' or 'scene', got "
                f"{self._search_area_source!r}")
        if self._search_area_source == 'scene':
            path, scene_poly = self._load_scene_region(self._search_area_scene_key)
            self._search_poly = np.asarray(scene_poly, dtype=float)
            if self._search_area_frame != 'world':
                self.get_logger().warn(
                    f"search_area_source=scene: the region file is authored in "
                    f"WORLD; overriding search_area_frame={self._search_area_frame!r}")
                self._search_area_frame = 'world'
            lo = self._search_poly.min(axis=0)
            hi = self._search_poly.max(axis=0)
            self.get_logger().info(
                f"search area from the SCENE ({self._search_area_scene_key!r} in "
                f"{path}): {len(self._search_poly)} pts, "
                f"{sect.polygon_area(self._search_poly):.0f} m2, world bbox "
                f"x [{lo[0]:.0f}, {hi[0]:.0f}] y [{lo[1]:.0f}, {hi[1]:.0f}]")
        self._search_poly_converted = (self._search_area_frame != 'world')
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
                sub = sect.sector_for(self._search_poly, self._sector_count, idx,
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
                        f'sector {idx} of {self._sector_count} came back empty; '
                        'refusing to run unbounded')
                self._search_poly = np.asarray(sub, dtype=float)
                self.get_logger().info(
                    f'sector {idx + 1}/{self._sector_count} ({self._sector_mode}, '
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
        # DETECTOR BOOKKEEPING, per robot, so the run's log answers "what did
        # YOLO score the goal class, how often, and how often above the gate"
        # without a sample-throttled line: every gate pass is logged, every
        # near miss is logged (throttled), and a summary with the run max and
        # a confidence histogram goes out every detector_log_period_s of SIM
        # time and once more when the budget ends. (The 2026-08-27 review had
        # only a 10 s-throttled sample to read the max off.)
        self._detector_log_period = float(self._p('detector_log_period_s', 30.0))
        # LOGGING-ONLY floor: every tick with a goal-class box at or above it
        # is logged UNTHROTTLED with each box's score and pixel rect (and the
        # annotated frame goes out as a detection_image), whether or not it
        # cleared the mapping gate. What the detector is SEEING at 0.5-0.65
        # is exactly what a threshold decision needs and what no gate-only
        # log can show. Does not touch sem_threshold.
        self._detector_log_conf = float(self._p('detector_log_conf', 0.5))
        self._det_stats = {}

        # ── actuation ─────────────────────────────────────────────────────────
        self._altitude = float(self._p('flight_altitude_m', 15.0))
        self._goal_tolerance = float(self._p('goal_tolerance_m', 6.0))
        self._nav_timeout = float(self._p('nav_goal_timeout_s', 60.0))
        self._goal_change_m = float(self._p('goal_change_threshold_m', 5.0))
        self._path_extension_m = float(self._p('path_extension_m', 2.0))
        # FLIGHT SPEED BY INTENT. droan_gl's rollouts are capped by its
        # `max_velocity` parameter (2 m/s as shipped). Crossing the sector to
        # a frontier or a lane goal is transit — nothing is being inspected —
        # so it goes at `explore_speed_mps`; the moment a detected person
        # becomes the goal the cap drops to `target_speed_mps` so the
        # approach and the look are flown slowly. Set on droan's parameter
        # service the tick the intent changes, and never for the gpt arm,
        # whose paper flies one speed (0 on either leaves droan alone).
        self._explore_speed = float(self._p('explore_speed_mps', 1.5))
        self._target_speed = float(self._p('target_speed_mps', 1.5))
        # A THIRD GEAR FOR THE WAY TO THE SECTOR. A fleet spawns in one
        # cluster and a drone's own strip can start hundreds of metres away
        # (robot_1 on the 1 km plat: 355 m to its centroid = 2 min at 3 m/s
        # of a 10 min budget, flown over ground it was told not to search).
        # While the drone is OUTSIDE its sector and not on a target it is
        # only travelling, so it may go faster; droan_gl caps at 15 m/s.
        # 0 = no third gear, explore_speed_mps applies everywhere.
        self._transit_speed = float(self._p('transit_speed_mps', 7.0))
        # HOW THE CAP REACHES droan_gl: on the NavigateTask goal itself
        # (task_msgs NavigateTask.Goal.max_speed_mps). The activator is
        # re-sent with the new cap the tick the intent changes and droan
        # PREEMPTS the running goal with it, so the change is one goal
        # round-trip, not a parameter service. (The parameter path is gone:
        # measured 2026-08-27, it never moved the drone anyway.)
        self._speed_set = {}             # robot index -> cap on the live goal
        # MEASURED ground speed, logged every speed_log_period_s SIM seconds
        # per robot with the cap and the intent, so the next run's log says
        # whether the cap took ("ground speed 6.4 m/s ... cap 7.0").
        self._speed_log_period = float(self._p('speed_log_period_s', 10.0))
        self._speed_hist = {}            # robot index -> (sim_t, xy)
        # How droan_gl is driven. 'activator' sends ONE empty NavigateTask and
        # steers by the /global_plan topic this node already publishes every
        # tick; 'goal_per_round' sends a fresh goal carrying the path. See
        # _ensure_activator() for why the first is the default.
        self._nav_activation = str(self._p('nav_activation', 'activator'))
        # 'auto' follows LOCAL_PLANNER (the same variable local.launch.xml
        # picks the local planner with): droan_gl wants the empty activator
        # goal kept alive; MIGHTY's bridge follows /global_plan on its own
        # and takes an empty goal only as a speed cap ('follower').
        if self._nav_activation == 'auto':
            lp = os.environ.get('LOCAL_PLANNER', '').strip().lower()
            self._nav_activation = 'follower' if lp == 'mighty' else 'activator'
            self.get_logger().info(
                f"nav_activation auto -> '{self._nav_activation}' "
                f"(LOCAL_PLANNER={lp or 'unset'})")
        if self._nav_activation not in ('activator', 'follower', 'goal_per_round'):
            raise ValueError(
                f'nav_activation must be activator | follower | goal_per_round | '
                f'auto, got {self._nav_activation!r}')
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
        # WHERE THE DETECTOR RUNS. 'server' calls the shared YOLO+MobileSAM
        # service and loads NOTHING here; 'local' is the old behaviour, one
        # model pair per planner process. Detection is stateless — one frame in,
        # boxes and masks out, no history — which is what makes one instance
        # safe for a fleet, the same argument as the ITM scorer. Three robots
        # each loading their own copy is 3x the VRAM for identical answers.
        # There is NO silent fallback: 'server' with an unreachable server is
        # fatal at startup, because a run that quietly detects nothing looks
        # exactly like a search that found nobody.
        self._detector_mode = str(self._p('detector_mode', 'local')).strip().lower()
        self._detector_url = str(self._p(
            'detector_url', 'http://offboard-compute:8200')).rstrip('/')
        self._detector_timeout = float(self._p('detector_timeout_s', 30.0))

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
        # An in-flight VLM round (gpt arm): the tick never waits for it.
        self._pending_round = None
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
        self._det_image_pubs = []
        # IMAGES ARE BEST-EFFORT. A RELIABLE image writer blocks in publish()
        # when a matched reliable reader stops acking — measured 2026-08-28:
        # the DDS router's reader for map_image went stale across a GCS
        # restart and the plan loop sat inside Publisher.publish for minutes
        # (py-spy: _publish_map_image -> rclpy/publisher.py:70), the node
        # vanished from `ros2 node list`, the drone parked. Nothing downstream
        # needs every frame; a dropped image costs one tick of a picture.
        self._image_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE,
                                     history=HistoryPolicy.KEEP_LAST, depth=1)
        self._nav_clients = []
        for i, robot in enumerate(self._robots):
            # ON THE REENTRANT GROUP, like every other subscription here.
            # Left on the node's DEFAULT group these two were the only
            # callbacks that never fired: `camera_info` (Reentrant) arrived
            # 15 times while `rgbd callback fired` stayed at 0, and the same
            # subscription code standalone delivered 564 syncs in 20 s against
            # the same live topics. The default group is MUTUALLY EXCLUSIVE and
            # `use_sim_time` puts the high-rate /clock subscription in it, so
            # one callback at a time meant the images never got a turn.
            rgb_sub = message_filters.Subscriber(
                self, Image, self._rgb_tpl.format(robot=robot),
                qos_profile=sensor_qos, callback_group=self._cbg)
            depth_sub = message_filters.Subscriber(
                self, Image, self._depth_tpl.format(robot=robot),
                qos_profile=sensor_qos, callback_group=self._cbg)
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
            if self._frame_mode == 'global_enu' or self._search_area_frame == 'world':
                self.create_subscription(
                    NavSatFix, self._navsat_tpl.format(robot=robot),
                    lambda msg, idx=i: self._navsat_callback(idx, msg),
                    sensor_qos, callback_group=self._cbg)

            self._plan_pubs.append(self.create_publisher(
                Path, self._plan_tpl.format(robot=robot), 10))
            self._agent_image_pubs.append(self.create_publisher(
                Image, self._agent_image_tpl.format(robot=robot), self._image_qos))
            self._det_image_pubs.append(self.create_publisher(
                CompressedImage, self._det_image_tpl.format(robot=robot),
                QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE,
                           history=HistoryPolicy.KEEP_LAST, depth=5)))
            self._nav_clients.append(ActionClient(
                self, NavigateTask, self._nav_tpl.format(robot=robot),
                callback_group=self._cbg))

        self._map_image_pub = self.create_publisher(Image, self._map_image_topic, self._image_qos)
        self._vlm_image_pub = self.create_publisher(Image, self._vlm_image_topic, self._image_qos)
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

        self._load_known_obstacles()

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

        self.get_logger().info(
            f'detector gate: a "{self._goal_name}" box is mapped only when its '
            f'confidence > sem_threshold {self._sem_threshold:.2f} (the server '
            f'proposes boxes from conf 0.1 up). Logging floor '
            f'{self._detector_log_conf:.2f}: every tick with a box at or above '
            f'it is logged as "detector SEEN" (below the gate) or "detector '
            f'PASS" (mapped) with each box\'s score and pixel rect; "detector '
            f'below gate" (throttled) covers proposals under the floor; '
            f'"detector summary" every {self._detector_log_period:.0f} s sim '
            f'and at the end')

        # YOLO-World + MobileSAM load takes tens of seconds; keep callbacks live.
        threading.Thread(target=self._init_agents, daemon=True).start()
        threading.Thread(target=self._plan_loop, daemon=True).start()

    # ── parameters ────────────────────────────────────────────────────────────

    def _robot_index_of_self(self):
        """This container's index in the TEAM, from ROBOT_NAME. Each robot
        runs its own planner, so 'which slice is mine' is answered locally.

        With num_robots=1 the names list is just [ROBOT_NAME], whose index is
        always 0 — so robot_2 and robot_3 would take slice 0 too. Read the
        trailing number off the name instead (robot_N -> N - robot_index_base)
        and fall back to the list only when that fails."""
        me = os.environ.get('ROBOT_NAME', '')
        m = re.search(r'(\d+)$', me)
        if m:
            base = int(self._p_cached('robot_index_base', 1))
            idx = int(m.group(1)) - base
            if 0 <= idx < max(1, self._sector_count):
                return idx
        if me in self._robots:
            return self._robots.index(me)
        return 0

    def _p_cached(self, name, default):
        """A parameter already declared, or `default`."""
        try:
            return self.get_parameter(name).value
        except Exception:
            return default

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
            # The DETECTOR, shared or local. Preflighted exactly like the ITM
            # scorer above and fatal for the same reason: the alternative to a
            # loud death is a run whose detector silently answers "nobody
            # here", which is indistinguishable from a search that failed.
            # Imported here rather than in the module header because it is only
            # reachable from this method, and the header's import block is what
            # decides _VENDOR_OK.
            det_health = None
            if self._detector_mode == 'server':
                from search_baselines import detector_client
                try:
                    det_health = detector_client.health(
                        self._detector_url, timeout=self._detector_timeout)
                except Exception as exc:
                    raise RuntimeError(
                        f'detector_mode=server but no detector service at '
                        f'{self._detector_url} ({exc}). Start ONE for the whole '
                        f'fleet on the offboard-compute container:\n'
                        f'  ./airstack.sh up offboard-compute\n'
                        f'  (or, by hand) docker exec -d offboard-compute bash -lc '
                        f"'ros2 run search_baselines detector_server'\n"
                        f'  then check: curl -s {self._detector_url}/health\n'
                        f'Set detector_mode:=local to load YOLO+SAM in this '
                        f'process instead.') from exc
                self.get_logger().info(
                    f'detector: shared service @ {self._detector_url} '
                    f'({det_health.get("yolo_weights")} + '
                    f'{det_health.get("sam_weights")} on '
                    f'{det_health.get("device")}, '
                    f'{"open-vocab" if det_health.get("open_vocab") else "closed-set"}, '
                    f'{det_health.get("gpu_gib")} GiB) — nothing loaded locally')
                # THE CHECKPOINT IS THE SERVER'S, NOT THE SCENE'S. Scene layers
                # pin `yolo_world_weights` (suburb_mini asks for closed-set
                # yolov8x, the wildfire plat for open-vocab yolov8l-world), and
                # in server mode that parameter configures nothing — the server
                # was started with whatever it was started with. Silently flying
                # a different detector than the scene names would change what
                # the arm can see, so say so loudly.
                served = str(det_health.get('yolo_weights') or '')
                # Already declared in the parameter block; read back rather than
                # threading a new attribute through it.
                want = str(self._p_cached('yolo_world_weights', '') or '')
                if want and os.path.basename(want) != os.path.basename(served):
                    self.get_logger().warn(
                        f'DETECTOR MISMATCH: this scene asks for '
                        f'{want}, the shared server at {self._detector_url} '
                        f'is running {served}. In server mode the SERVER wins. '
                        f'Restart it with --yolo-weights {want} (compose: '
                        f'CONAVGPT2_YOLO_WORLD_WEIGHTS) or set '
                        f'detector_mode:=local.')
            elif self._detector_mode != 'local':
                raise RuntimeError(
                    f"detector_mode must be 'server' or 'local', got "
                    f'{self._detector_mode!r}')
            agents = []
            for i in range(self._num_robots):
                detector = None
                if self._detector_mode == 'server':
                    detector = detector_client.RemoteDetector(
                        classes=self._classes, base_url=self._detector_url,
                        timeout=self._detector_timeout, health_info=det_health)
                agent = AirStackAgent(
                    self._args, i, self._goal_name, classes=self._classes,
                    depth_min_m=self._depth_min_m, depth_max_m=self._depth_max_m,
                    depth_border_px=self._depth_border_px, detector=detector)
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
                    # WAIT FOR camera_info, do not demand it. With the detector
                    # served remotely this init finishes in well under a
                    # second — before the first CameraInfo has been received
                    # — where the local YOLO load used to hide the race behind
                    # 30 s of weights loading. All three fleet planners died
                    # here on the first three-drone run.
                    hfov = None
                    for _ in range(120):                       # <= 60 s
                        with self._lock:
                            hfov = self._obs[0].get('hfov_rad')
                        if hfov or self._stop:
                            break
                        time.sleep(0.5)
                    if not hfov:
                        raise RuntimeError(
                            'lawnmower spacing is derived from the camera footprint '
                            'but no camera_info arrived within 60 s — is the '
                            f'camera publishing on {self._info_tpl}?')
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
                    f'lawnmower: {len(self._lm_path)} lane points, lane spacing '
                    f'{spacing:.1f} m ({self._lm_overlap:.0%} overlap at '
                    f'{self._altitude:.0f} m), axis={self._lm_axis}; flown as '
                    f'legs <= {self._lm_leg_m:.0f} m, one goal at a time')
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
        # Fires-or-not is the one thing the "waiting for rgb/depth" message
        # cannot distinguish: a callback that never runs and one that runs and
        # throws look identical from _snapshot(). Counted so the log says which.
        self._rgbd_hits = getattr(self, '_rgbd_hits', 0) + 1
        if self._rgbd_hits in (1, 10, 100) or self._rgbd_hits % 500 == 0:
            self.get_logger().info(
                f'[{self._robots[i]}] rgbd callback fired x{self._rgbd_hits}')
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

    def _place_search_area(self):
        """world -> map for `search_area_xy`, once. False while it cannot yet.

        Only does anything for `search_area_frame: world`. The map origin is
        `_boot_enu[0]` (enu(fix) - odom, set by `_navsat_callback`); until a
        fix has arrived the tick is held, because a sector applied in the
        wrong frame is exactly the failure this exists to remove.
        """
        if self._search_poly_converted or self._search_poly is None:
            return True
        b = self._boot_enu[0]
        if b is None:
            self.get_logger().info(
                'search_area_frame=world: waiting for a GPS fix to place the '
                "search area in this robot's map", throttle_duration_sec=10.0)
            return False
        shift = np.array([float(b[0]), float(b[1])])
        self._search_poly = np.asarray(self._search_poly, dtype=float) - shift
        lanes = getattr(self, '_lm_path', None)
        if lanes is not None and len(lanes):
            self._lm_path = np.asarray(lanes, dtype=float).reshape(-1, 2) - shift
        # The known obstacles are authored in the same WORLD frame as the
        # polygon, so they move by the same vector — z included, so a box top
        # is compared against the takeoff-anchored altitude the legs fly at.
        if self._obstacles_world is not None:
            self._obstacles = self._obstacles_world.shifted(
                float(b[0]), float(b[1]), float(b[2]) if len(b) > 2 else 0.0)
        self._search_poly_converted = True
        self.get_logger().info(
            f'search area authored in WORLD: map origin is at world '
            f'({b[0]:.1f}, {b[1]:.1f}), polygon shifted by ({-b[0]:.1f}, '
            f'{-b[1]:.1f}) into map; {len(self._search_poly)} pts, '
            f'x[{self._search_poly[:, 0].min():.0f}, {self._search_poly[:, 0].max():.0f}] '
            f'y[{self._search_poly[:, 1].min():.0f}, {self._search_poly[:, 1].max():.0f}]')
        return True

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
                self.get_logger().error(
                    f'search_planner tick failed: {exc}\n'
                    + traceback.format_exc(), throttle_duration_sec=5.0)
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
        for i in range(len(self._robots)):
            self._log_detector_summary(i, final=True)
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
        if not self._place_search_area():
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
            # THE ONE TRANSFORM. `_build_path` publishes `grid - offset`, and
            # the drone flies that correctly, so `grid - offset` IS the map
            # frame. Every marker must use the SAME transform or the picture
            # disagrees with the plan: markers were drawn from raw
            # `grid_to_map_xy` and rendered offset from the drone by exactly
            # this vector. Stored per-tick so the marker pass cannot silently
            # use a different one.
            if i == 0:
                self._marker_offset = offset
            pose = pose.copy()
            pose[:3, 3] = pose[:3, 3] + offset
            obs = {'rgb': snap[i]['rgb'], 'depth': snap[i]['depth'],
                   'cam_K': snap[i]['cam_K'], 'pose': pose}

            agent.mapping(obs)
            self._track_detector(i, agent)
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
        # THE GOAL MUST STILL BE FREE. A frontier is chosen where nothing has
        # been observed yet; by the time the drone is on its way the camera
        # has seen that ground, and if what it saw is a canopy or a wall the
        # goal is now INSIDE occupied voxels and no local planner will ever
        # arrive. Re-checked every tick against the live map, for every arm.
        if self._recheck_goals():
            assignment_due = True
        pend = self._pending_round
        if pend is not None and pend.get('done'):
            assignment_due = True          # the VLM's answer is in: apply it now
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
            self._log_ground_speed(i, agent)

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
            self._publish_detection_images()

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
            if self._vox_forget_s > 0.0:
                n_forgot, n_dropped = self._voxel_map.forget_older_than(
                    max(1, int(round(self._vox_forget_s / max(self._plan_period_s, 1e-3)))))
                if n_forgot:
                    self.get_logger().info(
                        f'voxel3d: forgot {n_forgot} voxels older than '
                        f'{self._vox_forget_s:.0f} s ({n_dropped} frontiers with them)',
                        throttle_duration_sec=30.0)
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
            # they have been turned into candidates. IN THE MAP FRAME: `fr` is
            # grid-frame metres (the voxel map is fed the offset poses) and
            # `search_area_xy` is authored in the map frame, so the same
            # `_to_map_arr` the pick functions apply goes here too. Without it
            # the polygon is displaced by `_marker_offset` at the source, and
            # the gpt arm — which has no pick-side filter of its own — is
            # offered the wrong sector.
            keep = self._points_in_polygon(self._to_map_arr(fr[:, :2]),
                                           self._search_poly)
            # And not blacklisted as unreachable (see _stall_watchdog).
            keep &= ~self._blacklisted_mask(fr[:, :2])
            fr, gain = fr[keep], gain[keep]
            if fr.shape[0] == 0:
                return edge, []

            # Rank EVERYTHING, then offer the first max_frontiers DISTINCT
            # cells. Stratified candidates at two heights over the same
            # ground project to the same grid cell, and the gpt arm's
            # candidate images are cut from this edge map BY LABEL
            # (`target_edge_map == i+1`, chat_utils.get_all_candidate_maps):
            # a later disc written over an earlier one erased that label, so
            # the VLM was shown a blank map for "frontier N" — and could still
            # pick it. Dropping a 2D duplicate is lossless for the other arms:
            # value lookup, rank gain and visit cost are all functions of the
            # cell, so the better-ranked twin would have won anyway.
            order = self._rank_frontiers(fr, gain, cap=fr.shape[0])
            pts_out = []
            r = 2
            for k in order:
                if len(pts_out) >= self._max_frontiers:
                    break
                i, j = agent.map_xy_to_grid(float(fr[k, 0]), float(fr[k, 1]))
                disc = edge[max(0, i - r):min(size, i + r + 1),
                            max(0, j - r):min(size, j + r + 1)]
                free = disc == 0
                if not free.any():
                    continue      # entirely under an earlier candidate's disc
                # Label only the free part of the disc, so every offered
                # candidate owns at least one pixel of the BEV render and the
                # marker path, and labels stay 1..N in list order.
                disc[free] = len(pts_out) + 1
                pts_out.append([i, j])
                self._frontier_z.append(float(fr[k, 2]))
            return edge, pts_out
        except Exception as exc:
            self.get_logger().warn(f'voxel3d frontier extraction failed: {exc}',
                                   throttle_duration_sec=10.0)
            return edge, []

    def _rank_frontiers(self, fr, gain, cap=None):
        """Indices of the frontiers to offer, best first.

        `cap` overrides max_frontiers: _voxel_frontiers asks for the whole
        order and stops itself, because it drops cell-duplicates on the way.

        Unobserved-neighbour count IS the information gain, so ranking by it
        alone is unambiguous — but it is also degenerate: the least-observed
        height wins every slot, all six candidates come back at one z, and the
        3D map buys nothing over the 2D slab. Take the best frontier at each
        height before taking a second one anywhere, so the offered set spans
        the band. Levels are visited in order of their best frontier, so the
        single best candidate overall is still rank 0.
        """
        cap = self._max_frontiers if cap is None else int(cap)
        order = np.argsort(-gain)
        if not self._fr_z_stratify:
            return order[:cap]

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
                    if len(picked) >= cap:
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
        keep = self._points_in_polygon(self._to_map_arr(xy), self._search_poly)
        keep &= ~self._blacklisted_mask(xy)
        if not keep.any():
            return self._recover_into_area(agent, len(xy))
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

    def _to_map_arr(self, xy):
        """Vectorised `_to_map`, for the search-area test.

        `search_area_xy` is authored in the MAP frame, so grid coordinates must
        be brought into it before the point-in-polygon test — otherwise the
        polygon is effectively offset by `_marker_offset` and the robot is
        judged out of bounds while it is well inside its sector.
        """
        off = self._marker_offset
        if off is None or xy.size == 0:
            return xy
        return xy - np.array([off[0], off[1]], dtype=float)

    def _to_map(self, x, y):
        """Grid-frame metres -> the map frame the markers are stamped with.

        Identical to what `_build_path` applies, so a marker and the waypoint
        it represents can never land in different places.
        """
        off = self._marker_offset
        if off is None:
            return float(x), float(y)
        return float(x - off[0]), float(y - off[1])

    def _map_to_grid_xy(self, x, y):
        """Inverse of `_to_map`: map-frame metres -> grid-frame metres."""
        off = self._marker_offset
        if off is None:
            return float(x), float(y)
        return float(x + off[0]), float(y + off[1])

    def _map_to_grid(self, agent, x, y):
        """A MAP-frame point -> the grid cell `_goal_points` holds.

        `agent.map_xy_to_grid` takes GRID-frame metres. Feeding it a map-frame
        point directly (a lane waypoint, the sector centroid — anything
        authored in config coordinates) produces a cell that `_build_path`
        then shifts by `-offset` on the way out, so the drone is sent to the
        point minus the offset: off by the whole grid-to-map vector, which on
        a scene with a map origin is the width of the sector.
        """
        gx, gy = self._map_to_grid_xy(x, y)
        return list(agent.map_xy_to_grid(gx, gy))

    def _recover_into_area(self, agent, n_out):
        """A goal that pulls the robot back INSIDE the search area.

        Filtering frontiers to the search area is correct — an arm must not be
        credited for ground it was not asked to cover. Returning None when the
        filter empties is not: the planner then holds its last goal, and a
        robot that has just driven itself to the boundary sits on it for the
        rest of the run. That reads as a stuck local planner and is a search
        that gave up.

        The area centroid is always inside a convex sector and is the cheapest
        target that re-enters one; once back in, ordinary frontier selection
        takes over again.
        """
        if self._search_poly is None:
            return None
        cx, cy = sect.polygon_centroid(self._search_poly)
        here = self._to_map(*self._agent_xy(agent))
        what = (f'all {n_out} frontiers outside the search area' if n_out
                else 'no frontier candidates')
        self.get_logger().warn(
            f'{what} — steering to the sector centroid '
            f'({cx:.0f}, {cy:.0f}), {math.dist(here, (cx, cy)):.0f} m away',
            throttle_duration_sec=15.0)
        return self._map_to_grid(agent, float(cx), float(cy))

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
        keep = self._points_in_polygon(self._to_map_arr(xy), self._search_poly)
        keep &= ~self._blacklisted_mask(xy)
        if not keep.any():
            return self._recover_into_area(agent, len(xy))
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
        """The next lane goal, as a grid cell — ONE short goal, never the path.

        Same actuation as every other arm: the cell goes into `_goal_points`,
        `_command` publishes a two-pose path on /global_plan and the activator
        NavigateTask keeps droan_gl steering by it. What differs from a
        frontier pick is only where the point comes from: a `lawnmower.Sweep`
        that walks the drone to the sector in legs of `lawnmower_leg_m`, then
        along the lanes in legs of the same length, advancing as each is
        reached or passed and skipping one the drone makes no progress on.

        FRAMES. The lanes are authored in the MAP frame (from search_area_xy);
        the agent's position and `_goal_points` are in the GRID frame. Both
        conversions go through the same offset `_build_path` applies, so the
        goal the drone is sent lands on the lane it was cut from.
        """
        if self._lm_path is None or len(self._lm_path) == 0:
            return None
        now = time.time()
        here = np.array(self._to_map(*self._agent_xy(agent)))
        if self._lm_needs_anchor or self._lm_sweep is None:
            self._lm_needs_anchor = False
            # DIFFERENT EDGES FOR DIFFERENT DRONES. `boustrophedon(start_xy)`
            # enters at the lane end nearest the hint; a fleet spawned in one
            # cluster would all enter their strips at the same edge and sweep
            # in step, which leaves the far edges of every sector for last.
            # Odd robots are hinted with the sector corner FARTHEST from them
            # instead, so neighbouring drones start at opposite ends.
            idx = self._robot_index_of_self() or 0
            hint = (float(here[0]), float(here[1]))
            if idx % 2 == 1:
                poly = np.asarray(self._search_poly, dtype=float)
                far = poly[int(np.argmax(np.linalg.norm(poly - here, axis=1)))]
                hint = (float(far[0]), float(far[1]))
                self.get_logger().info(
                    f'lawnmower: robot index {idx} enters at the FAR edge '
                    f'({hint[0]:.0f}, {hint[1]:.0f}) so neighbours start apart')
            self._lm_path = lm.boustrophedon(
                self._search_poly, self._lm_spacing_used,
                axis=self._lm_axis, start_xy=hint)
            self._lm_idx = 0
            self._lm_sweep = lm.Sweep(
                self._lm_path, reach_m=self._lm_reach, leg_m=self._lm_leg_m,
                start_xy=here, stall_s=self._lm_stall_s,
                z_fn=self._lm_leg_z_for, stall_lifts=self._lm_lifts)
            n_ob = len(self._obstacles) if self._obstacles is not None else 0
            self.get_logger().info(
                f'lawnmower: clearance surface over {n_ob} known obstacle(s), '
                f'+{self._obst_clear:.1f} m, cruise {self._altitude:.0f} m, '
                f'band [{self._min_alt:.0f}, {self._max_alt:.0f}] m; stall '
                f'{self._lm_stall_s:.0f} s -> lift {self._lm_lift_m:.0f} m '
                f'x{self._lm_lifts} then skip')
            entry = self._lm_sweep.loop[0]
            self.get_logger().info(
                f'lawnmower: anchored at ({here[0]:.0f}, {here[1]:.0f}); '
                f'transit {len(self._lm_sweep.transit) + 1} leg(s) to the '
                f'entry ({entry[0]:.0f}, {entry[1]:.0f}) '
                f'{float(np.hypot(*(entry - here))):.0f} m away, then a '
                f'{len(self._lm_sweep.loop)}-goal loop over '
                f'{len(self._lm_path)} lane points, legs <= '
                f'{self._lm_leg_m:.0f} m, reach {self._lm_reach:.0f} m')
        elif (self._lm_last_pick_t is not None
              and now - self._lm_last_pick_t > max(5.0, 3.0 * self._plan_period_s)):
            # The sweep was paused (a target detour holds _assign off), not
            # stalled: restart the stall clock rather than skipping a leg.
            self._lm_sweep.resume(now)
            self.get_logger().info(
                'lawnmower: resuming the sweep after a detour',
                throttle_duration_sec=5.0)
        self._lm_last_pick_t = now

        g = self._lm_sweep.update(here, now)
        if g is None:
            return None
        if g.index != self._lm_idx and g.z is not None:
            # Say once per goal what height the leg is flown at and why.
            top, k = ((self._obstacles.top_along(
                self._lm_sweep.point(max(0, g.index - 1)), g.xy))
                if self._obstacles is not None and len(self._obstacles)
                else (None, -1))
            if top is not None and g.z > self._altitude + 0.05:
                self.get_logger().info(
                    f'lawnmower: goal #{g.index} leg flown at {g.z:.1f} m — '
                    f'{self._obstacles.classes[k]} top {top:.1f} m under it '
                    f'(+{self._obst_clear:.1f} m); next leg {g.through_z:.1f} m')
        self._lm_idx = g.index
        if g.lifted:
            self.get_logger().warn(
                f'lawnmower: no progress on goal #{g.index} for '
                f'{self._lm_stall_s:.0f} s — lifting it to {g.z:.1f} m '
                f'(lift {g.lift} of {self._lm_lifts})')
        if g.skipped:
            self.get_logger().warn(
                f'lawnmower: no progress on goal #{g.index - 1} for '
                f'{self._lm_stall_s:.0f} s after {self._lm_lifts} lift(s) — '
                f'skipping to ({g.xy[0]:.0f}, {g.xy[1]:.0f})')
        if g.phase != self._lm_phase:
            self._lm_phase = g.phase
            self.get_logger().info(
                f'lawnmower: {g.phase} (lap {g.lap + 1}) from '
                f'({here[0]:.0f}, {here[1]:.0f}) — goal #{g.index} '
                f'({g.xy[0]:.0f}, {g.xy[1]:.0f}) '
                f'{float(np.hypot(*(g.xy - here))):.0f} m away')
        cell = self._map_to_grid(agent, float(g.xy[0]), float(g.xy[1]))
        # `map_xy_to_grid` CLIPS to the grid. A lane point past the map edge
        # (a padded sector wider than map_extent_m) comes back as the edge
        # cell, and the drone is sent there — so judge reach against THAT, or
        # the sweep waits at every clipped lane end for the stall timer.
        sent = np.array(self._to_map(*agent.grid_to_map_xy(*cell)))
        res_m = agent.args.map_resolution / 100.0
        if float(np.hypot(*(sent - g.xy))) > 2.0 * res_m:
            self.get_logger().warn(
                f'lawnmower: goal #{g.index} ({g.xy[0]:.0f}, {g.xy[1]:.0f}) is '
                f'outside the occupancy grid; clipped to ({sent[0]:.0f}, '
                f'{sent[1]:.0f}). The sector (search_area_xy + '
                f'search_area_pad_m) does not fit map_extent_m — shrink the '
                'pad or grow the map', throttle_duration_sec=30.0)
            self._lm_sweep.override(g.index, sent)
            g.xy = sent
        self._lm_leg = (self._map_to_grid_xy(float(g.xy[0]), float(g.xy[1])),
                        self._map_to_grid_xy(float(g.through[0]), float(g.through[1])))
        self._lm_leg_z = ((float(g.z), float(g.through_z))
                          if g.z is not None and g.through_z is not None else None)
        return cell

    def _lm_leg_z_for(self, p, q, lift=0):
        """Altitude for the lane leg p->q (map frame): the clearance surface
        over the known obstacles, plus `lift` stall lifts, in the flight band.
        Cruise when nothing stands under the leg — the detector's height."""
        top = None
        if self._obstacles is not None and len(self._obstacles):
            top, _k = self._obstacles.top_along(p, q)
        return clr.leg_z(top, self._altitude, self._obst_clear,
                         lo=self._min_alt, hi=self._max_alt,
                         lift_m=float(lift) * self._lm_lift_m)

    @staticmethod
    def _annotation_dirs():
        """Where the launcher writes a generated scene's annotations, most
        authoritative first.

        THE SOURCE TREE, not a sibling of wherever this file runs from. Under
        colcon this module executes from build/search_baselines/, whose
        sibling build/raven_nav/annotations is a stale copy from raven_nav's
        last build — the launcher writes into
        src/global/planners/raven_nav/annotations. Walk up to the workspace
        root and go down the source path from there; the installed share is
        whatever colcon last saw and comes last."""
        here = os.path.dirname(os.path.abspath(__file__))
        dirs = []
        anc = here
        for _ in range(8):
            anc = os.path.dirname(anc)
            cand = os.path.join(anc, 'src', 'global', 'planners',
                                'raven_nav', 'annotations')
            if os.path.isdir(cand):
                dirs.append(cand)
                break
        dirs.append(os.path.normpath(os.path.join(
            here, '..', '..', 'raven_nav', 'annotations')))
        try:
            from ament_index_python.packages import get_package_share_directory
            dirs.append(os.path.join(get_package_share_directory('raven_nav'),
                                     'annotations'))
        except Exception:
            pass
        return dirs

    @classmethod
    def _annotation_file(cls, scene, suffixes):
        """First existing `<scene><suffix>.json` over `suffixes` (major) x
        `_annotation_dirs()` (minor), or ''."""
        cands = [os.path.join(d, f'{scene}{suffix}.json')
                 for suffix in suffixes for d in cls._annotation_dirs()]
        return next((c for c in cands if os.path.exists(c)), '')

    def _load_scene_region(self, key):
        """The scene's disaster-affected polygon, WORLD xy, from
        `<scene>_region.json` — `(path, [[x, y], ...])`, or raise.

        `key` names the entry ('affected' | 'burn' | 'region'; see
        scene_gen/disaster/region.py for what each is). Raises rather than
        returns nothing: see search_area_source in __init__."""
        scene = self._obst_scene or os.environ.get('RESULTS_SCENE', '').strip()
        if not scene:
            raise RuntimeError(
                'search_area_source=scene needs the scene name: set '
                'RESULTS_SCENE (the launcher names the annotation files after '
                'it) or known_obstacles_scene')
        path = self._annotation_file(scene, ('_region',))
        if not path:
            raise RuntimeError(
                f'search_area_source=scene: no {scene}_region.json in '
                f'{self._annotation_dirs()} — the Isaac launcher writes it at '
                f'scene build with GT_ANNOTATIONS=on (a wildfire scene; '
                f'scene_gen/disaster/region.py). Use search_area_source: '
                f'config to fly search_area_xy instead')
        with open(path) as fh:
            doc = json.load(fh)
        entry = next((e for e in doc if isinstance(e, dict)
                      and e.get('class') == key), None) if isinstance(doc, list) else None
        poly = [[float(p[0]), float(p[1])] for p in (entry or {}).get('polygon_xy') or []]
        if len(poly) < 3:
            have = sorted({e.get('class') for e in doc if isinstance(e, dict)}) \
                if isinstance(doc, list) else []
            raise RuntimeError(
                f'search_area_source=scene: {path} has no usable '
                f'{key!r} polygon (entries: {have})')
        return path, poly

    def _load_known_obstacles(self):
        """The scene's ground-truth obstacle boxes, as authored (WORLD).

        `known_obstacles_file` wins; else `known_obstacles_scene` (default
        $RESULTS_SCENE, the name the launcher writes the GT under) is looked
        up in raven_nav's annotations dir — the SOURCE tree first, because
        that is where the launcher writes and the installed share copy is
        whatever colcon last saw. No file is not an error: the arm flies at
        cruise everywhere and relies on the stall lift, and says so."""
        path = self._obst_file
        scene = self._obst_scene or os.environ.get('RESULTS_SCENE', '').strip()
        if not path and scene:
            # `<scene>_obstacles.json` is what the launcher writes for the
            # houses / trees / cars (the GT file itself is people only);
            # `<scene>.json` is accepted too, for a hand-authored scene whose
            # one file carries every class — the class filter below sorts it.
            path = self._annotation_file(scene, ('_obstacles', ''))
        if not path or not os.path.exists(path):
            self.get_logger().warn(
                f'known obstacles: no annotation file (file={self._obst_file!r}, '
                f'scene={scene!r}) — lane legs fly at cruise everywhere; only the '
                'stall lift can get the lawnmower over a canopy')
            return
        try:
            boxes = clr.load_boxes(path, self._obst_classes)
        except Exception as exc:
            self.get_logger().warn(f'known obstacles: {path} unreadable: {exc}')
            return
        self._obstacles_world = clr.KnownObstacles(boxes, self._obst_inflate)
        # In a map-frame scene the boxes are used as authored; the world
        # shift, when there is one, is applied by _place_search_area.
        if self._search_area_frame != 'world':
            self._obstacles = self._obstacles_world
        tally = {}
        for c in self._obstacles_world.classes:
            tally[c] = tally.get(c, 0) + 1
        tops = self._obstacles_world.top
        self.get_logger().info(
            f'known obstacles: {len(boxes)} boxes from {path} '
            f'({", ".join(f"{v} {k}" for k, v in sorted(tally.items()))}); '
            f'tops {tops.min():.1f}-{tops.max():.1f} m'
            + (' (world frame, shifted into map with the search area)'
               if self._search_area_frame == 'world' else '')
            if len(boxes) else
            f'known obstacles: {path} holds no {self._obst_classes} boxes — '
            'was the launcher run with GT_ANNOTATIONS on?')

    def _blacklisted_mask(self, xy):
        """True for every row of `xy` (grid-frame metres, Nx2) inside
        `frontier_blacklist_radius_m` of a blacklisted frontier."""
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        mask = np.zeros(xy.shape[0], dtype=bool)
        if not self._frontier_blacklist or xy.shape[0] == 0:
            return mask
        now = time.time()
        if self._blacklist_ttl > 0.0:
            self._frontier_blacklist = [
                b for b in self._frontier_blacklist if now - b[2] <= self._blacklist_ttl]
        for bx, by, _ in self._frontier_blacklist:
            mask |= np.hypot(xy[:, 0] - bx, xy[:, 1] - by) <= self._blacklist_r
        return mask

    def _stall_watchdog(self, here, now):
        """Blacklist and release the held goal when the drone stops making
        progress toward it. Returns True if the lock was just released."""
        here_t = (float(here[0]), float(here[1]))
        if (self._progress_xy is None
                or math.dist(here_t, self._progress_xy) >= self._stall_dist):
            self._progress_xy, self._progress_t = here_t, now
            return False
        if (self._locked_goal is None or self._stall_s <= 0.0
                or now - self._progress_t < self._stall_s
                or now - self._locked_at < self._stall_s):
            return False
        gx, gy = (self._locked_goal_xy if self._locked_goal_xy is not None
                  else (float('nan'), float('nan')))
        self._frontier_blacklist.append((gx, gy, now))
        self.get_logger().warn(
            f'frontier ({gx:.0f}, {gy:.0f}) unreachable: no progress for '
            f'{now - self._progress_t:.0f} s at ({here_t[0]:.0f}, {here_t[1]:.0f}), '
            f'{math.dist(here_t, (gx, gy)):.0f} m short — BLACKLISTED '
            f'(r={self._blacklist_r:.0f} m, {len(self._frontier_blacklist)} total), '
            'releasing the lock')
        self._locked_goal, self._locked_score, self._locked_goal_xy = None, None, None
        self._progress_t = now
        return True

    def _commit(self, cand, cand_score, cand_xy, here):
        """Hold the current target unless the challenger is clearly better.

        Without this the drone re-argmaxes every tick, and because the value map
        shifts as it flies, two frontiers swap places repeatedly and it arrives
        at neither.
        """
        now = time.time()
        self._stall_watchdog(here, now)
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
        prev_goals = list(self._goal_points)
        self._goal_points = []
        map_size = obstacle_map.shape[0]

        if self._nav_mode == 'gpt':
            # THE VLM ROUND IS ASYNCHRONOUS. A round is a generation of
            # seconds (26 s per attempt on the 3B model), and it used to run
            # INSIDE the tick: no /global_plan was published while it ran, so
            # a slow or failing round parked the drone for minutes (the
            # measured stall on the 250 m suburb). Now the request is built and
            # sent from a worker thread, the tick keeps commanding the current
            # goal, and the answer is applied on the tick it arrives — against
            # the candidate list the VLM was actually shown.
            pend = self._pending_round
            if pend is not None and pend.get('done'):
                self._pending_round = None
                self._apply_round(pend, map_size)
                return
            if pend is None and len(target_point_list) > 0 and step >= self._warmup_steps:
                self._start_round(target_edge_map, target_point_list, top_view_map,
                                  vis_pose_pred)
            # Meanwhile: the goal already being flown, or the nearest
            # candidate when there is none (t=0, or a goal just blacklisted).
            if prev_goals and len(prev_goals) >= self._num_robots:
                self._goal_points = prev_goals
            else:
                for i in range(self._num_robots):
                    self._goal_points.append(
                        self._nearest_candidate(target_point_list, self._agents[i])
                        or self._fallback_goal(self._agents[i], map_size))

        elif self._nav_mode == 'vlfm':
            for i in range(self._num_robots):
                pick = self._vlfm_pick(target_point_list, self._agents[i])
                self._goal_points.append(
                    pick if pick is not None
                    else self._fallback_goal(self._agents[i], map_size))

        elif self._nav_mode == 'frontier':
            for i in range(self._num_robots):
                self._goal_points.append(
                    self._frontier_pick(target_point_list, self._agents[i])
                    or self._fallback_goal(self._agents[i], map_size))

        elif self._nav_mode == 'lawnmower':
            # No frontier is consulted at all: the lane path IS the plan. The
            # occupancy map still builds and publishes, so the run is readable
            # against the other arms, but nothing reads it for selection.
            for i in range(self._num_robots):
                self._goal_points.append(
                    self._lawnmower_pick(self._agents[i])
                    or self._fallback_goal(self._agents[i], map_size))

        elif self._nav_mode == 'nearest':
            for i in range(self._num_robots):
                _, _, points = detect_frontier(
                    explored_map, obstacle_map, grid_pose[i],
                    threshold_point=self._frontier_threshold)
                self._goal_points.append(
                    points[0] if points
                    else self._fallback_goal(self._agents[i], map_size))
        else:
            raise ValueError(f'unknown nav_mode {self._nav_mode!r}')

    def _start_round(self, target_edge_map, target_point_list, top_view_map,
                     vis_pose_pred):
        """Kick off one VLM round on a worker thread. Everything the round
        needs is SNAPSHOTTED here, because the map, the frontier list and the
        robot poses will all have moved by the time the answer comes back."""
        round_start = time.time()
        since_last = (None if self._last_round_start is None
                      else round_start - self._last_round_start)
        self._last_round_start = round_start
        self._round += 1
        pend = {'done': False, 'round': self._round, 'since_last': since_last,
                'target_point_list': [list(p) for p in target_point_list],
                'edge': np.array(target_edge_map, copy=True),
                'top_view': np.array(top_view_map, copy=True),
                'vis_pose': [list(v) for v in vis_pose_pred],
                'assignment': None, 'candidate_maps': [], 'payload_bytes': 0,
                'build_s': 0.0, 'call_s': 0.0, 'call': {}, 'error': None}

        def _work():
            try:
                t_build = time.time()
                cms = chat_utils.get_all_candidate_maps(
                    pend['edge'], pend['top_view'], pend['vis_pose'])
                message = chat_utils.message_prepare(
                    system_prompt.system_prompt, cms, self._goal_name)
                pend['build_s'] = time.time() - t_build
                pend['payload_bytes'] = sum(len(b.getvalue()) for b in cms)
                pend['candidate_maps'] = cms
                t_call = time.time()
                pend['assignment'] = chat_utils.chat_with_gpt4v(message)
                pend['call_s'] = time.time() - t_call
                pend['call'] = dict(chat_utils.LAST_CALL)
            except Exception as exc:          # noqa: BLE001
                pend['error'] = f'{exc}\n{traceback.format_exc()}'
            finally:
                pend['done'] = True

        self._pending_round = pend
        threading.Thread(target=_work, name='vlm-round', daemon=True).start()

    def _apply_round(self, pend, map_size):
        """The VLM's answer, resolved against the list it was shown."""
        if pend.get('error'):
            self.get_logger().error(
                f"[search_planner] round {pend['round']} raised: {pend['error']}")
        assignment = pend.get('assignment') or {}
        tpl = pend['target_point_list']
        self._record_round(assignment, len(tpl), pend['candidate_maps'],
                           pend['payload_bytes'], pend['build_s'], pend['call_s'],
                           pend['since_last'], call=pend.get('call'))
        if self._publish_vis and pend['candidate_maps']:
            self._publish_vlm_image(pend['candidate_maps'])
        self._goal_points = []
        for i in range(self._num_robots):
            self._goal_points.append(self._resolve_frontier(assignment, i, tpl, map_size))

    def _nearest_candidate(self, target_point_list, agent):
        """The closest offered frontier to this agent, or None."""
        if not target_point_list:
            return None
        here = np.array(self._agent_xy(agent))
        xy = np.array([agent.grid_to_map_xy(p[0], p[1]) for p in target_point_list])
        return list(target_point_list[int(np.argmin(np.linalg.norm(xy - here, axis=1)))])

    def _goal_voxel_blocked(self, i, agent):
        """True when this robot's goal, at the height it will be flown, sits
        in or beside an OCCUPIED voxel of the live map (26-neighbourhood: one
        voxel of margin, the same reach droan's expansion gives an obstacle).
        False without a voxel map or without a goal."""
        vm = self._voxel_map
        if vm is None or i >= len(self._goal_points):
            return False
        goal = self._goal_points[i]
        gx, gy = agent.grid_to_map_xy(goal[0], goal[1])       # voxel-map frame
        z = None
        if self._nav_mode == 'lawnmower' and getattr(self, '_lm_leg_z', None):
            z = self._lm_leg_z[0]
        if z is None:
            z = self._frontier_altitude(i)
        if z is None:
            z = self._clamp_alt(self._altitude)
        idx = vm.to_idx(np.array([[gx, gy, float(z)]]))[0]
        lo = np.maximum(idx - 1, 0)
        hi = np.minimum(idx + 2, vm.dims)
        if np.any(hi <= lo):
            return False
        return bool((vm.grid[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]] == VOX_OCCUPIED).any())

    def _recheck_goals(self):
        """Drop every goal the map now says is inside an obstacle. Returns
        True if any goal was dropped (the caller re-assigns this tick).

        Per arm: vlfm / frontier / gpt / nearest BLACKLIST the point (so the
        source stops offering it) and release it; the lawnmower LIFTS the leg
        (then skips it), which is its own version of the same ladder."""
        if self._voxel_map is None or self._agents is None:
            return False
        now = time.time()
        if self._track_instances and self._active_target is not None:
            # FLYING TO A PERSON. A detection whose approach point sits in
            # occupied voxels — a canopy, a roofline — is a place the drone
            # can never get to, and holding it parks the search on what is
            # most likely a false positive. Give it up: mark it VISITED (the
            # same book-keeping a reached instance gets, so it is never picked
            # again) and let the frontier search resume.
            agent = self._agents[0]
            tx, ty = self._active_target
            z = self._waypoint_z(agent)
            vm = self._voxel_map
            idx = vm.to_idx(np.array([[float(tx), float(ty), float(z)]]))[0]
            lo, hi = np.maximum(idx - 1, 0), np.minimum(idx + 2, vm.dims)
            if np.all(hi > lo) and bool(
                    (vm.grid[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]] == VOX_OCCUPIED).any()):
                mx, my = self._to_map(tx, ty)
                self._visited_targets.append((float(tx), float(ty)))
                self.get_logger().warn(
                    f'target at ({mx:.0f}, {my:.0f}) sits inside occupied voxels at '
                    f'{z:.0f} m — unreachable, likely a false positive; GIVING UP '
                    f'(marked visited, {len(self._visited_targets)} total)')
                self._active_target = None
                self._goal_points = []
                return True
            return False                      # a reachable person: not a frontier
        if not self._goal_points:
            return False
        dropped = False
        for i, agent in enumerate(self._agents):
            if not self._goal_voxel_blocked(i, agent):
                continue
            goal = self._goal_points[i]
            gx, gy = agent.grid_to_map_xy(goal[0], goal[1])
            mx, my = self._to_map(gx, gy)
            if self._nav_mode == 'lawnmower' and self._lm_sweep is not None:
                what = self._lm_sweep.blocked(now)
                self.get_logger().warn(
                    f'lawnmower: goal #{self._lm_idx} ({mx:.0f}, {my:.0f}) is inside '
                    f'newly observed voxels — {what}')
            else:
                self._frontier_blacklist.append((float(gx), float(gy), now))
                self._locked_goal, self._locked_score, self._locked_goal_xy = None, None, None
                self.get_logger().warn(
                    f'goal ({mx:.0f}, {my:.0f}) is inside newly observed voxels — '
                    f'BLACKLISTED (r={self._blacklist_r:.0f} m, '
                    f'{len(self._frontier_blacklist)} total), re-picking now')
            dropped = True
        if dropped:
            self._goal_points = []
        return dropped

    def _record_round(self, assignment, num_frontiers, candidate_maps,
                      payload_bytes, build_s, call_s, since_last, call=None):
        """Per-round telemetry: JSONL under the results dir, a String topic so it
        lands in the mcap, and one INFO line."""
        call = dict(chat_utils.LAST_CALL) if call is None else dict(call)
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
            # Under voxel3d `Frontier_Det` never runs and LAST_DET stays
            # empty, so report what the 3D source offered instead — count and
            # heights, the one thing the slab could never say.
            'frontier_det': (
                {'source': 'voxel3d', 'n_offered': num_frontiers,
                 'max_frontiers': self._max_frontiers,
                 'z_m': [round(float(z), 1) for z in self._frontier_z]}
                if self._frontier_source == 'voxel3d'
                else dict(explored_map_utils.LAST_DET)),
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

    def _fallback_goal(self, agent, map_size):
        """Where a robot goes when there is NO candidate: the sector centroid
        if a search area is set, else upstream's random cell.

        The polygon is applied at the SOURCE (`_voxel_frontiers`), so "every
        frontier is out of bounds" reaches `_assign` as an EMPTY list, never
        as a list `_recover_into_area` gets to reject. Without this a drone
        that has driven itself to the sector edge — and every drone at t=0,
        before the first carve — was sent to a random cell of the whole grid,
        up to half the extent outside its sector, re-drawn every tick. Shared
        by every arm; the gpt arm in particular has no pick function of its
        own in which to recover.
        """
        if self._search_poly is not None:
            goal = self._recover_into_area(agent, 0)
            if goal is not None:
                return goal
        return self._random_goal(map_size)

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
        # DOWNSAMPLE FIRST. `object_pcd` only ever grows (upstream appends every
        # detection's points and retires none), and DBSCAN's neighbour search
        # at eps 4 m over a cloud that dense is quadratic in practice: at ~23k
        # points this call alone held the plan loop for 10-17 s per tick
        # (py-spy: every sample inside cluster_dbscan), so the drone was
        # re-planned at 0.07 Hz and the viz topics looked dead. A 0.5 m voxel
        # grid caps the cloud at one point per half-metre cell — a person is a
        # handful of cells, a house a few hundred — and the centroids come out
        # the same (measured 0.5 m of the GT people either way).
        try:
            vox = max(0.25, min(1.0, self._inst_eps / 8.0))
            ds = pcd.voxel_down_sample(vox)
            if len(ds.points) >= self._inst_min_pts:
                pcd = ds
        except Exception:
            pass
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
            here = self._to_map(*self._agent_xy(agent))
            # camera_position is open3d (x, y, z); map z is its y, the same
            # convention the voxel integration uses.
            z = float(agent.camera_position[1])
            # FRAME CHECK. Markers are drawn from GRID coordinates
            # (`grid_to_map_xy`) while the search-area outline is drawn from
            # CONFIG coordinates. If those two frames disagree, the outline
            # lands correctly and every marker is offset — which is exactly
            # what a rolling local map does once the robot starts away from
            # the map origin. Logging both against ODOMETRY (true map frame)
            # makes the offset a number instead of an impression.
            od = self._obs[0].get('odom')
            if od is not None:
                try:
                    ox, oy = float(od[0, 3]), float(od[1, 3])
                    dx, dy = here[0] - ox, here[1] - oy
                    self.get_logger().info(
                        f'frame check: odom=({ox:.1f}, {oy:.1f}) '
                        f'grid_to_map=({here[0]:.1f}, {here[1]:.1f}) '
                        f'offset=({dx:+.1f}, {dy:+.1f}) '
                        f'origins_grid={list(map(int, agent.origins_grid))}',
                        throttle_duration_sec=10.0)
                except Exception:
                    pass
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
            cr, cg, cb = robot_rgb(self._robot_index_of_self() or 0)
            r.color = ColorRGBA(r=cr, g=cg, b=cb, a=0.95)
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
                t.color = ColorRGBA(r=cr, g=cg, b=cb, a=0.55)
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
                    mx, my = self._to_map(*o3d_xz_to_map_xy(float(q[0]), float(q[2])))
                    dm.points.append(Point(x=mx, y=my, z=float(q[1])))
                ma.markers.append(dm)

                dl = _mk('search_detection_id', 0, Marker.TEXT_VIEW_FACING)
                c = np.asarray(pcd.points).mean(axis=0)
                cx, cy = self._to_map(*o3d_xz_to_map_xy(float(c[0]), float(c[2])))
                dl.pose.position.x, dl.pose.position.y = cx, cy
                dl.pose.position.z = float(c[1]) + 8.0
                dl.scale.z = 2.5
                dl.color = ColorRGBA(r=1.0, g=0.4, b=0.95, a=0.95)
                dl.text = (f'{self._goal_name} DETECTED: {n_det} pts, '
                           f'{len(self._target_instances)} instance(s)')
                ma.markers.append(dl)

            # THE SEARCH AREA ITSELF, so "is it looking in the right place"
            # is answerable by looking. Drawn every tick rather than latched:
            # a sector is derived from num_robots and the robot index, so it
            # can change between runs and a stale latched outline would be
            # worse than none.
            if self._search_poly is not None and len(self._search_poly) >= 3:
                area = _mk('search_area', 0, Marker.LINE_STRIP)
                area.scale.x = 1.2
                area.color = ColorRGBA(r=cr, g=cg, b=cb, a=0.95)
                ring = list(self._search_poly) + [self._search_poly[0]]
                for q in ring:
                    area.points.append(
                        Point(x=float(q[0]), y=float(q[1]), z=0.5))
                ma.markers.append(area)

                al = _mk('search_area_id', 0, Marker.TEXT_VIEW_FACING)
                cx, cy = sect.polygon_centroid(self._search_poly)
                al.pose.position.x, al.pose.position.y = float(cx), float(cy)
                al.pose.position.z = 3.0
                al.scale.z = 4.0
                al.color = ColorRGBA(r=cr, g=cg, b=cb, a=0.9)
                al.text = f'{self._robots[0]} search area'
                ma.markers.append(al)

                # THE LAWNMOWER'S LANES. This arm plans the WHOLE coverage
                # path at start-up and then flies it one <= lawnmower_leg_m
                # leg at a time, so /global_plan only ever carries the current
                # leg: from outside, an on-plan sweep and a drone wandering
                # look identical. Drawing the plan is what makes "is it
                # covering the sector" answerable by looking, and it is the
                # only view that shows the ground the sweep has NOT reached
                # yet — which is where an undetected survivor would be.
                #
                # MAP frame, drawn RAW with no _to_map, exactly like the
                # sector ring above: `_lm_path` is cut from `search_area_xy`
                # and `_place_search_area` shifts both by the same offset.
                # (Contrast the robot/target markers, which start in GRID
                # coordinates and must be transformed.)
                if (self._nav_mode == 'lawnmower'
                        and self._lm_path is not None and len(self._lm_path) >= 2):
                    lanes = _mk('search_lanes', 0, Marker.LINE_STRIP)
                    lanes.scale.x = 0.8
                    lanes.color = ColorRGBA(r=cr, g=cg, b=cb, a=0.30)
                    lanes.points = [Point(x=float(q[0]), y=float(q[1]), z=1.0)
                                    for q in self._lm_path]
                    ma.markers.append(lanes)

                    sw = self._lm_sweep
                    if sw is not None and len(sw.loop):
                        nt = len(sw.transit)
                        # The one-time walk from the takeoff point to the
                        # entry lane end. Distinct from the lanes because it
                        # covers nothing and is flown once, so time spent
                        # here is overhead, not coverage.
                        if nt:
                            tr = _mk('search_lane_transit', 0, Marker.LINE_STRIP)
                            tr.scale.x = 0.6
                            tr.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.35)
                            tr.points = [Point(x=float(q[0]), y=float(q[1]), z=1.0)
                                         for q in sw.transit]
                            ma.markers.append(tr)

                        # Progress THIS LAP, solid over the dim plan. Reset
                        # every lap rather than left filled in: the sweep
                        # repeats until the sim budget runs out, and a bar
                        # that stays full after lap 1 stops being a progress
                        # indicator at all. The lap count is in the label.
                        li = max(0, int(sw.idx) - nt)
                        done = sw.loop[:li + 1]
                        if len(done) >= 2:
                            dn = _mk('search_lanes_done', 0, Marker.LINE_STRIP)
                            dn.scale.x = 1.6
                            dn.color = ColorRGBA(r=cr, g=cg, b=cb, a=0.95)
                            dn.points = [Point(x=float(q[0]), y=float(q[1]), z=1.2)
                                         for q in done]
                            ma.markers.append(dn)

                        # The leg being flown right now, drawn at the reach
                        # radius so the disc IS the tolerance that retires it.
                        g = sw.point(int(sw.idx))
                        gm = _mk('search_lane_goal', 0, Marker.SPHERE)
                        gm.pose.position.x = float(g[0])
                        gm.pose.position.y = float(g[1])
                        gm.pose.position.z = 1.5
                        gm.scale.x = gm.scale.y = gm.scale.z = 2.0 * self._lm_reach
                        gm.color = ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.35)
                        ma.markers.append(gm)

                        gl = _mk('search_lanes_id', 0, Marker.TEXT_VIEW_FACING)
                        gl.pose.position.x = float(g[0])
                        gl.pose.position.y = float(g[1])
                        gl.pose.position.z = 6.0
                        gl.scale.z = 2.5
                        gl.color = ColorRGBA(r=cr, g=cg, b=cb, a=0.9)
                        gl.text = (
                            f'lawnmower {sw.phase} {li}/{len(sw.loop)} '
                            f'lap {int(sw.lap) + 1} | '
                            f'{self._lm_spacing_used:.0f} m lanes')
                        ma.markers.append(gl)

            active = self._active_target
            for k, xy in enumerate(self._target_instances):
                done = self._is_visited(xy)
                is_active = (active is not None
                             and math.dist(active, xy) < 1e-6)
                d = _mk('search_target', k, Marker.CYLINDER)
                _tx, _ty = self._to_map(float(xy[0]), float(xy[1]))
                d.pose.position.x, d.pose.position.y = _tx, _ty
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
                tl.pose.position.x, tl.pose.position.y = _tx, _ty
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
        inst = self._instance_centroids(agent)
        # ONLY MY SECTOR. A detection across the boundary is another drone's
        # to visit; chasing it leaves my own ground unswept and has two
        # drones converge on one person. Judged in the MAP frame, the frame
        # `search_area_xy` is authored in (same test the frontier picks use).
        if inst and self._search_poly is not None:
            keep = self._points_in_polygon(self._to_map_arr(np.asarray(inst)),
                                           self._search_poly)
            n_out = int((~keep).sum())
            if n_out:
                self.get_logger().info(
                    f'targets: ignoring {n_out} instance(s) outside this '
                    'sector', throttle_duration_sec=15.0)
            inst = [t for t, k in zip(inst, keep) if k]
        self._target_instances = inst
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

        # Stalled on the target chosen last tick -> give it up (see __init__).
        now = time.time()
        if self._active_target is not None and self._stall_s > 0.0:
            here_t = (float(here[0]), float(here[1]))
            if (self._tgt_progress_xy is None
                    or math.dist(here_t, self._tgt_progress_xy) >= self._stall_dist):
                self._tgt_progress_xy, self._tgt_progress_t = here_t, now
            elif (now - self._tgt_progress_t >= self._stall_s
                  and now - self._tgt_active_since >= self._stall_s):
                tx, ty = float(self._active_target[0]), float(self._active_target[1])
                self._visited_targets.append((tx, ty))
                self.get_logger().warn(
                    f'target ({tx:.1f}, {ty:.1f}) unreachable: no progress for '
                    f'{now - self._tgt_progress_t:.0f} s at ({here_t[0]:.0f}, '
                    f'{here_t[1]:.0f}), {math.dist(here_t, (tx, ty)):.0f} m short — '
                    f'marked VISITED (blacklisted), {len(self._visited_targets)} done')
                self._tgt_progress_t = now
                self._active_target = None

        unvisited = [t for t in self._target_instances if not self._is_visited(t)]
        if not unvisited:
            self._active_target = None
            return None
        prev = self._active_target
        self._active_target = min(unvisited, key=lambda t: math.dist(here, t))
        if prev is None or math.dist(prev, self._active_target) > 1e-6:
            self._tgt_active_since = now
            self._tgt_progress_xy, self._tgt_progress_t = (float(here[0]), float(here[1])), now
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

    def _build_path(self, agent, goal_xy, offset, through_xy=None):
        """The Path droan_gl steers by.

        'waypoints' (default) is raven_nav's shape: the goal, then one point
        past it along the approach direction so droan has a heading THROUGH the
        final pose rather than stopping dead on it. Two poses, and the local
        planner owns everything between here and there.

        `through_xy` replaces that second pose with a point the caller already
        knows the drone should continue to — the lawnmower's next lane point —
        so the heading through the goal is the LANE's, not the approach's, and
        droan gets progress credit along it. Still two poses.

        'fmm_path' prepends upstream's own FMM waypoints. See `plan_output`.
        """
        if self._plan_output == 'waypoints':
            cur = self._agent_xy(agent)
            if through_xy is not None and math.dist(through_xy, goal_xy) > 1e-6:
                pts = [tuple(goal_xy), tuple(through_xy)]
                # THE LAWNMOWER'S OWN HEIGHTS: this leg's clearance z on the
                # goal, the NEXT leg's on the through pose, so the drone
                # descends toward the next goal rather than onto this one.
                leg_z = getattr(self, '_lm_leg_z', None)
                if leg_z is not None:
                    return self._path_msg(pts, offset, [leg_z[0], leg_z[1]])
                return self._path_msg(pts, offset, self._waypoint_z(agent))
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

    def _on_target(self, agent):
        """True while `_goal_xy` is overriding the planner's pick with a
        detected target — the same two conditions, restated."""
        if self._track_instances:
            return self._active_target is not None
        return bool(getattr(agent, 'found_goal', False)
                    and getattr(agent, 'nearest_point', None) is not None)

    def _through_xy(self, agent):
        """Second pose for `_build_path`, when the pick supplied one.

        Only the lawnmower sets `_lm_leg`; every other arm leaves it None and
        gets the approach-direction extension. Not conditioned on nav_mode,
        so the actuation stays shared. A target detour overrides the lane
        goal in `_goal_xy`, so the lane's continuation must not be attached
        to it either."""
        leg = getattr(self, '_lm_leg', None)
        if leg is None or self._on_target(agent):
            return None
        return leg[1]

    def _agent_xy(self, agent):
        """Where this agent currently is, in map-frame metres."""
        return agent.grid_to_map_xy(*agent.current_grid_pose)

    def _frontier_altitude(self, i):
        """Height of the frontier this robot was assigned, when the frontier
        source supplies one. A 2D slab cannot, so this returns None there and
        the caller falls back to the configured cruise altitude."""
        if self._frontier_source != 'voxel3d':
            return None
        try:
            goal = self._goal_points[i]
        except (IndexError, TypeError):
            return None
        cell = (int(goal[0]), int(goal[1]))
        # _goal_points holds grid cells; match it back to the candidate it came
        # from so the z belongs to THIS frontier and not to whichever was last.
        for k, p in enumerate(self._frontier_candidates):
            if (int(p[0]), int(p[1])) == cell and k < len(self._frontier_z):
                z = self._clamp_alt(self._frontier_z[k])
                self._goal_z_cache[i] = (cell, z)
                return z
        # The list is re-extracted every tick, and the assigned frontier is
        # carved away as the drone closes on it — that is what arriving at a
        # frontier means. A fresh lookup then finds nothing and the waypoint
        # would jump to cruise mid-approach; for the gpt arm, which holds a
        # goal for a whole round, that is most of the approach. While the goal
        # cell is unchanged the height it was assigned at stands.
        cached = self._goal_z_cache.get(i)
        if cached is not None and cached[0] == cell:
            return cached[1]
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
        # One z for the whole path, or one per pose.
        zs = (list(z) if isinstance(z, (list, tuple, np.ndarray))
              else [z] * len(pts))
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._map_frame
        for (x, y), pz in zip(pts, zs):
            ps = PoseStamped()
            ps.header = path.header
            # The grid lives in the merge frame; droan_gl wants this robot's own
            # local 'map', so drop the boot_enu offset back off on the way out.
            ps.pose.position.x = float(x - offset[0])
            ps.pose.position.y = float(y - offset[1])
            ps.pose.position.z = float(pz)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    def _set_local_speed(self, i, mps):
        """Carry `mps` to droan_gl on the NavigateTask goal.

        `NavigateTask.Goal.max_speed_mps` caps that goal's rollouts (0 =
        droan's own max_velocity). In 'activator' mode the activator is
        RE-SENT with the new cap; droan_gl preempts the running goal with it
        (`execute_navigate`: the old handle is aborted "Preempted ...", the
        new one takes the plan, the mode and the cap), so there is no cancel
        window to race. A 'goal_per_round' goal carries the cap anyway.
        Idempotent per robot: nothing is sent while the cap is unchanged."""
        # The gpt arm is exempt: the paper flies one speed. Decided HERE, so
        # the actuation path (_command) stays identical for every arm.
        if self._nav_mode == 'gpt' or mps <= 0.0 or self._speed_set.get(i) == mps:
            return
        self._speed_set[i] = float(mps)
        self.get_logger().info(
            f'[{self._robots[i]}] NavigateTask max_speed -> {mps:.1f} m/s '
            f'({self._speed_label(mps)})')
        if (self._nav_activation in ('activator', 'follower')
                and not getattr(self, '_stop', False)):
            self._send_activator(i, why=f'speed {mps:.1f} m/s')

    def _speed_label(self, mps):
        """Which gear a cap is, for the log."""
        if self._transit_speed > 0.0 and mps == self._transit_speed \
                and mps != self._explore_speed:
            return 'transit to sector'
        if mps == self._target_speed and mps != self._explore_speed:
            return 'target approach'
        if mps == self._target_speed:
            return 'search'
        return 'explore'

    def _stamp_speed(self, goal, i):
        """Put this robot's cap on a NavigateTask goal, if the message has the
        field. A task_msgs built before `max_speed_mps` existed has not, and
        that must be loud: the cap would silently never reach droan."""
        mps = float(self._speed_set.get(i) or 0.0)
        if hasattr(goal, 'max_speed_mps'):
            goal.max_speed_mps = mps
        elif mps > 0.0:
            self.get_logger().warn(
                'task_msgs NavigateTask has no max_speed_mps — rebuild the '
                'workspace (bws); the speed cap is NOT reaching droan_gl',
                throttle_duration_sec=60.0)
        return mps

    def _log_ground_speed(self, i, agent):
        """`[robot] ground speed V m/s over T s sim | cap C (gear) | in/out of
        sector` every speed_log_period_s of SIM time — the line that says
        whether a requested cap actually moved the drone."""
        try:
            now = self.get_clock().now().nanoseconds / 1e9
        except Exception:
            return
        xy = np.array(self._agent_xy(agent), dtype=float)
        last = self._speed_hist.get(i)
        if last is None:
            self._speed_hist[i] = (now, xy)
            return
        dt = now - last[0]
        if dt < self._speed_log_period:
            return
        v = float(np.linalg.norm(xy - last[1])) / dt if dt > 0 else 0.0
        self._speed_hist[i] = (now, xy)
        cap = self._speed_set.get(i)
        where = ''
        if self._search_poly is not None and self._search_poly_converted:
            here = np.array([self._to_map(*xy)], dtype=float)
            inside = bool(self._points_in_polygon(here, self._search_poly)[0])
            where = ' | inside sector' if inside else ' | OUTSIDE sector'
        self.get_logger().info(
            f'[{self._robots[i]}] ground speed {v:.2f} m/s over {dt:.0f} s sim'
            + (f' | cap {cap:.1f} ({self._speed_label(cap)})' if cap
               else ' | cap: droan default')
            + where)

    def _intent_speed(self, agent):
        """droan's cap for what this robot is doing NOW: the approach to a
        detected person, the transit to a sector it is not yet inside, or
        exploring inside it. Judged in the MAP frame like every other
        search-area test, and only once the polygon is placed there."""
        if self._on_target(agent):
            return self._target_speed
        if (self._transit_speed > 0.0 and self._search_poly is not None
                and self._search_poly_converted):
            here = np.array([self._to_map(*self._agent_xy(agent))], dtype=float)
            if not bool(self._points_in_polygon(here, self._search_poly)[0]):
                return self._transit_speed
        return self._explore_speed

    def _command(self, i, agent, offset):
        self._set_local_speed(i, self._intent_speed(agent))
        goal_xy = self._goal_xy(i, agent)
        path = self._build_path(agent, goal_xy, offset,
                                through_xy=self._through_xy(agent))
        self._plan_pubs[i].publish(path)

        if self._nav_activation == 'activator':
            self._ensure_activator(i)
            return
        if self._nav_activation == 'follower':
            # The local planner follows /global_plan itself (MIGHTY's bridge);
            # the only NavigateTask it gets is the speed-only goal above.
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
        self._stamp_speed(goal, i)
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

        The goal also carries the speed cap (`max_speed_mps`); a change of cap
        re-sends it through `_set_local_speed`, and droan preempts.
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
        self._send_activator(i)

    def _send_activator(self, i, why=''):
        """Send the activator goal now — a fresh one, or a replacement that
        droan_gl preempts the live one with (a new cap)."""
        nav = self._nav[i]
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
        mps = self._stamp_speed(goal, i)
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
            f'[{robot}] NavigateTask activator accepted — local planner steering by '
            f'{self._plan_tpl.format(robot=robot)}, max_speed '
            f'{mps:.1f} m/s' + (f' ({why})' if why else ''))

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
            # Grid-frame corner -> map frame, the same shift every marker and
            # the published path get (identity in frame_mode 'local').
            _gx, _gy = self._to_map(-ox * res, -(size - oy) * res)
            msg.info.origin.position.x = _gx
            msg.info.origin.position.y = _gy
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
            _gx, _gy = self._to_map(-ox * res, -(vm.size - oy) * res)
            msg.info.origin.position.x = _gx
            msg.info.origin.position.y = _gy
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
            # The voxel map is integrated in the GRID frame (the camera poses
            # it carves from had the merge offset added), so shift back like
            # everything else published in `map`. Identity in 'local' mode.
            _off = self._marker_offset
            if _off is not None:
                xyz = xyz - np.array([_off[0], _off[1], 0.0], dtype=xyz.dtype)
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
                cx, cy = self._to_map(*agent.grid_to_map_xy(p[0], p[1]))
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
                cx, cy = self._to_map(*agent.grid_to_map_xy(p[0], p[1]))
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
            gx, gy = self._to_map(*agent.grid_to_map_xy(int(ci), int(cj)))
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

    _DET_BINS = ((0.0, 0.3), (0.3, 0.5), (0.5, 0.65), (0.65, 0.8), (0.8, 1.01))

    def _track_detector(self, i, agent):
        """Fold this tick's detector result into the run's per-robot tally
        and log the gate decision (see detector_log_period_s in __init__).
        `last_detection` is what the `_detect` wrapper recorded for THIS
        tick's frame: `goal_hits` boxes of the goal class, best `goal_max`."""
        ld = getattr(agent, 'last_detection', None) or {}
        st = self._det_stats.get(i)
        if st is None:
            st = {'ticks': 0, 'proposed': 0, 'passed': 0, 'max': 0.0,
                  'bins': [0] * len(self._DET_BINS), 'last_summary': None,
                  'max_at_tick': 0}
            self._det_stats[i] = st
        st['ticks'] += 1
        hits = int(ld.get('goal_hits', 0) or 0)
        gmax = float(ld.get('goal_max', 0.0) or 0.0)
        gate = self._sem_threshold
        robot = self._robots[i]
        if hits:
            st['proposed'] += 1
            for k, (lo, hi) in enumerate(self._DET_BINS):
                if lo <= gmax < hi:
                    st['bins'][k] += 1
                    break
            if gmax > st['max']:
                st['max'] = gmax
                st['max_at_tick'] = st['ticks']
            floor = self._detector_log_conf
            boxes = [(c, b) for c, b in (ld.get('goal_boxes') or []) if c >= floor]
            boxes_txt = ', '.join(
                f'{c:.2f} @ px{b} {b[2] - b[0]}x{b[3] - b[1]}' if len(b) == 4
                else f'{c:.2f}' for c, b in boxes)
            if gmax > gate:
                st['passed'] += 1
                self.get_logger().info(
                    f'[{robot}] detector PASS: {self._goal_name} {gmax:.3f} > gate '
                    f'{gate:.2f} -> mapped into object_pcd | {len(boxes)} box(es) '
                    f'>= {floor:.2f}: {boxes_txt} | other classes '
                    f'{[t for t in ld.get("top", []) if t[0] != self._goal_name][:3]} '
                    f'| run max {st["max"]:.3f} | passes {st["passed"]}'
                    f'/{st["proposed"]} proposals')
            elif gmax >= floor:
                # Unthrottled on purpose: this is the band a threshold
                # decision is made from, and every frame of it is wanted.
                self.get_logger().info(
                    f'[{robot}] detector SEEN: {self._goal_name} {gmax:.3f} '
                    f'(below gate {gate:.2f}, not mapped) | {len(boxes)} box(es) '
                    f'>= {floor:.2f}: {boxes_txt} | other classes '
                    f'{[t for t in ld.get("top", []) if t[0] != self._goal_name][:3]} '
                    f'| run max {st["max"]:.3f}')
            else:
                self.get_logger().info(
                    f'[{robot}] detector below gate: {self._goal_name} best '
                    f'{gmax:.3f} < {floor:.2f} ({hits} box(es)) — not mapped | '
                    f'run max {st["max"]:.3f}', throttle_duration_sec=10.0)
        now = None
        try:
            now = self.get_clock().now().nanoseconds / 1e9
        except Exception:
            pass
        if now is not None:
            if st['last_summary'] is None:
                st['last_summary'] = now
            elif now - st['last_summary'] >= self._detector_log_period:
                st['last_summary'] = now
                self._log_detector_summary(i)

    def _log_detector_summary(self, i, final=False):
        st = self._det_stats.get(i)
        if st is None:
            return
        labels = ('<0.3', '0.3-0.5', '0.5-0.65', '0.65-0.8', '>=0.8')
        bins = ' '.join(f'{lab}:{n}' for lab, n in zip(labels, st['bins']))
        self.get_logger().info(
            f'[{self._robots[i]}] detector summary{" (FINAL)" if final else ""}: '
            f'{st["ticks"]} ticks | "{self._goal_name}" proposed on {st["proposed"]} '
            f'| passed gate {self._sem_threshold:.2f} on {st["passed"]} '
            f'| run max {st["max"]:.3f} (tick {st["max_at_tick"]}) '
            f'| conf bins {bins}')

    def _publish_agent_images_now(self):
        for i, agent in enumerate(self._agents):
            if agent.annotated_image is not None:
                self._publish_image(self._agent_image_pubs[i], agent.annotated_image,
                                    self._robots[i])

    def _publish_detection_images(self):
        """The annotated FPV as JPEG, only on a tick whose best goal-class
        score reached `detector_log_conf` (see detection_image_topic_template).
        `annotated_image` is BGR (what `_publish_image` flips); the JPEG
        encoder takes BGR as is."""
        for i, agent in enumerate(self._agents):
            ld = getattr(agent, 'last_detection', None) or {}
            img = getattr(agent, 'annotated_image', None)
            if img is None or float(ld.get('goal_max', 0.0)) < self._detector_log_conf:
                continue
            try:
                ok, buf = cv2.imencode(
                    '.jpg', np.ascontiguousarray(img.astype(np.uint8)),
                    [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if not ok:
                    continue
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self._robots[i]
                msg.format = 'jpeg'
                msg.data = buf.tobytes()
                self._det_image_pubs[i].publish(msg)
            except Exception as exc:
                self.get_logger().warn(f'detection image publish failed: {exc}',
                                       throttle_duration_sec=10.0)


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

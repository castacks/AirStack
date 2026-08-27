"""AirStack-side subclass of upstream's `ROS_Agent`.

Everything about the method — occupancy/explored map from RGBD, frontier
detection, FMM planning, the VLM frontier assignment — is upstream's. What
differs for AirStack is only:

  * the world frame (ENU `map`, z up) instead of habitat / a Livox `camera_init`,
  * float-metre depth instead of 16UC1 millimetres,
  * a configurable detector vocabulary instead of the hardcoded indoor list.

Those three are overridden here so the vendored tree stays as close to upstream
as possible.
"""

import threading

import numpy as np

from conavgpt2.vendor.agents import ros2_agents
from conavgpt2.vendor.agents.ros2_agents import ROS_Agent

# ENU map frame (x fwd/east, y left/north, z up) -> upstream's open3d convention,
# which indexes the ground plane with axes 0 and 2 and treats axis 1 as height.
# A proper rotation (det=+1): (x, y, z) -> (x, z, -y). Using a reflection here
# instead would mirror the rendered top view and flip every heading the VLM sees.
# The class list upstream's ROS_Agent hardcodes before it resolves the goal.
# Reproduced here only so the subclass can tell whether a goal will survive
# `super().__init__`; the vendored file is not edited.
# CLOSED-SET CHECKPOINTS NEED A no-op `set_classes`.
#
# The VENDORED `Object_Detection_and_Segmentation.__init__` calls
# `yolo_model_w_classes.set_classes(classes)` unconditionally
# (utils/detection_segmentation.py:49) — inside the constructor, before this
# subclass gets control. A COCO checkpoint has no such method and raises there,
# so selecting yolov8x killed the planner at start-up.
#
# `YOLOWorld` is a SEPARATE CLASS, not a subclass of `YOLO` (verified), so
# attaching a no-op to `YOLO` cannot shadow the real open-vocabulary method —
# open-vocab models keep their own. This is preferred over editing the vendored
# file, which stays a clean copy of upstream.
try:
    from ultralytics import YOLO as _UltralyticsYOLO
    if not hasattr(_UltralyticsYOLO, 'set_classes'):
        def _set_classes_noop(self, classes):
            """Closed-set model: the vocabulary is fixed, so this is a no-op.

            `AirStackAgent` resolves `goal_id` from the checkpoint's own
            `names` instead of from the requested list.
            """
            return None
        # TAGGED, because the probe below asks `hasattr(model, 'set_classes')`
        # to decide open-vocab vs closed-set — and installing this shim would
        # otherwise answer YES for a COCO checkpoint. That sent yolov8x down
        # the open-vocab path: `self.classes` became the requested ['person']
        # while the model kept emitting COCO ids 0-79, and the first non-person
        # detection crashed the vendored overlay with
        # `IndexError: list index out of range` (visualization.py:46).
        _set_classes_noop._noop = True
        _UltralyticsYOLO.set_classes = _set_classes_noop
except Exception:      # ultralytics absent -> the planner fails later, clearly
    pass


UPSTREAM_CLASSES = ['chair', 'bed', 'potted plant', 'toilet', 'tv_screen',
                    'couch', 'person', 'sink']

T_MAP_TO_O3D = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


def o3d_xz_to_map_xy(o3d_x, o3d_z):
    """Inverse of T_MAP_TO_O3D restricted to the ground plane."""
    return float(o3d_x), float(-o3d_z)


# The vendored `ROS_Agent.__init__` CONSTRUCTS the detector itself — line 87 of
# ros2_agents.py, before this subclass gets any control — and that constructor
# is what puts YOLO and MobileSAM on the GPU. To run against the shared service
# instead, the name it looks up has to be replaced for the duration of that
# call; there is no injection point and the vendored tree stays a clean copy of
# upstream. Serialised because the name being swapped is module-global.
_DETECTOR_PATCH_LOCK = threading.Lock()


class AirStackAgent(ROS_Agent):
    def __init__(self, args, agent_id, goal_name, classes=None,
                 depth_min_m=0.5, depth_max_m=60.0, depth_border_px=0,
                 detector=None):
        self._depth_min_m = float(depth_min_m)
        self._depth_max_m = float(depth_max_m)
        self._depth_border_px = int(depth_border_px)
        # args.print_images is what makes upstream compute the annotated FPV and
        # the per-agent map render at all, so the ROS wrapper leaves it on and
        # gates only the PNG writes here.
        self.save_debug_images = False

        # send_queue/receive_queue only feed the open3d GUI, which is stripped;
        # upstream never touches them unless args.visualize is set.
        #
        # `ROS_Agent.__init__` sets self.classes to a hardcoded INDOOR list and
        # then does `self.classes.index(goal_name)`, so ANY goal outside that
        # list — 'car', 'smoke', anything aerial — raises before the subclass
        # gets to substitute its own vocabulary. Hand the base a name it is
        # guaranteed to have and correct the three fields immediately after.
        base_goal = goal_name if goal_name in UPSTREAM_CLASSES else 'person'
        if detector is None:
            super().__init__(args, agent_id, base_goal, None, None)
        else:
            # `detector` is already-built (a detector_client.RemoteDetector),
            # so nothing is loaded locally: no YOLO, no MobileSAM, no CUDA
            # context for either. The factory swallows the three arguments the
            # vendored constructor passes (args, classes, device) — the remote
            # model's vocabulary is set below through the same
            # `set_classes` path an in-process model takes.
            with _DETECTOR_PATCH_LOCK:
                original = ros2_agents.Object_Detection_and_Segmentation
                ros2_agents.Object_Detection_and_Segmentation = (
                    lambda _args, _classes, _device: detector)
                try:
                    super().__init__(args, agent_id, base_goal, None, None)
                finally:
                    ros2_agents.Object_Detection_and_Segmentation = original
        self.goal_name = goal_name

        # OPEN-VOCAB (YOLO-World) vs CLOSED-SET (COCO). A COCO checkpoint has no
        # `set_classes` — its vocabulary is fixed — so calling it raises, and the
        # class ids are the model's OWN, not our list's. Measured on the
        # wildfire frame: yolov8x (COCO) matches yolov8l-world on people
        # (top 0.85 vs 0.84) at 237 ms against 1030 ms, because it runs no
        # text-embedding match. Worth taking whenever the goal is a COCO class.
        model = self.obj_det_seg.yolo_model_w_classes
        # A REAL set_classes, not the shim above.
        _sc = getattr(model, 'set_classes', None)
        self._open_vocab = _sc is not None and not getattr(_sc, '_noop', False)
        if self._open_vocab:
            if classes:
                # Re-setting the prompt vocabulary on the already-built model
                # avoids editing upstream's hardcoded indoor list.
                self.classes = list(classes)
            if goal_name not in self.classes:
                self.classes = list(self.classes) + [goal_name]
            model.set_classes(self.classes)
            self.goal_id = self.classes.index(goal_name)
        else:
            # The checkpoint's own names, so `goal_id` indexes what the model
            # actually emits. `person` is COCO id 0.
            names = dict(getattr(model, 'names', {}) or {})
            self.classes = [names[k] for k in sorted(names)]
            match = [i for i, n in names.items()
                     if str(n).lower() == str(goal_name).lower()]
            if not match:
                raise ValueError(
                    f"goal_name {goal_name!r} is not a class of this closed-set "
                    f"detector. Available: {sorted(set(names.values()))[:12]}... "
                    "Use an open-vocabulary checkpoint (yolov8*-world.pt) for a "
                    "goal outside the model's vocabulary.")
            self.goal_id = int(match[0])

        # Record what the detector actually returns. Upstream consumes
        # detections inside mapping() and keeps nothing when the class/
        # confidence gate rejects them, so "no target" and "target seen but
        # filtered out" are indistinguishable from outside — which is the
        # difference between a broken detector and a wrong threshold.
        self.last_detection = {'n': 0, 'top': [], 'goal_hits': 0}
        _inner = self.obj_det_seg.detect

        def _detect(image, *a, **kw):
            det = _inner(image, *a, **kw)
            try:
                conf = getattr(det, 'confidence', None)
                cid = getattr(det, 'class_id', None)
                n = 0 if conf is None else len(conf)
                top = []
                if n:
                    order = sorted(range(n), key=lambda i: -float(conf[i]))[:5]
                    top = [(self.classes[int(cid[i])]
                            if cid is not None and int(cid[i]) < len(self.classes)
                            else '?', round(float(conf[i]), 3)) for i in order]
                hits, gmax = 0, 0.0
                if n and cid is not None:
                    gc = [float(conf[i]) for i in range(n)
                          if int(cid[i]) == self.goal_id]
                    hits = len(gc)
                    gmax = max(gc) if gc else 0.0
                # goal_max is the number that decides everything: it separates
                # "the detector never proposes the goal class" from "it does,
                # below sem_threshold".
                self.last_detection = {'n': n, 'top': top, 'goal_hits': hits,
                                       'goal_max': round(gmax, 3)}
            except Exception:
                pass
            return det

        self.obj_det_seg.detect = _detect

        # get_transform_matrix() below ignores these, but upstream code paths read
        # them, so keep them well-defined rather than None.
        self.init_sim_position = np.zeros(3)
        self.init_sim_rotation = np.eye(3)

    # ── frame conventions ──────────────────────────────────────────────────────

    def get_transform_matrix(self, pose_matrix):
        """`pose_matrix` is the camera-optical (RDF) pose in the ENU `map` frame.

        Upstream anchors the map at each robot's first pose; here the anchor is the
        shared `map` origin instead, which is what makes several robots' point
        clouds mergeable into one grid at all.
        """
        return T_MAP_TO_O3D @ pose_matrix

    def grid_to_map_xy(self, grid_i, grid_j):
        """Occupancy-grid cell -> (x, y) metres in the ENU `map` frame."""
        res_m = self.args.map_resolution / 100.0
        o3d_x = (float(grid_i) - self.origins_grid[0]) * res_m
        o3d_z = (float(grid_j) - self.origins_grid[1]) * res_m
        return o3d_xz_to_map_xy(o3d_x, o3d_z)

    def map_xy_to_grid(self, x, y):
        res_m = self.args.map_resolution / 100.0
        i = int(np.floor(x / res_m) + self.origins_grid[0])
        j = int(np.floor(-y / res_m) + self.origins_grid[1])
        size = self.map_size
        return [int(np.clip(i, 1, size - 2)), int(np.clip(j, 1, size - 2))]

    def plan_path_map_xy(self):
        """Upstream's FMM path (open3d metres, y zeroed) -> list of map-frame (x, y)."""
        if self.plan_path is None or len(self.plan_path) == 0:
            return []
        return [o3d_xz_to_map_xy(p[0], p[2]) for p in np.asarray(self.plan_path)]

    # ── rendering ──────────────────────────────────────────────────────────────

    def _visualize(self, *args, **kwargs):
        saved = self.args.print_images
        self.args.print_images = 1 if self.save_debug_images else 0
        try:
            return super()._visualize(*args, **kwargs)
        finally:
            self.args.print_images = saved

    # ── depth ──────────────────────────────────────────────────────────────────

    def _preprocess_depth(self, depth, min_d=None, max_d=None):
        """AirStack publishes float metres (with non-finite sky), not 16UC1 mm."""
        min_d = self._depth_min_m if min_d is None else min_d
        max_d = self._depth_max_m if max_d is None else max_d

        depth = np.asarray(depth, dtype=np.float32).copy()
        depth[~np.isfinite(depth)] = 0.0
        depth[depth > max_d] = 0.0
        depth[depth < min_d] = 0.0

        b = self._depth_border_px
        if b > 0:
            depth[-b:, :] = 0.0
            depth[:, -b:] = 0.0
            depth[:b, :] = 0.0
            depth[:, :b] = 0.0
        return depth

"""
gt_annotations.py — ground-truth annotation output for the drone cameras.

Two independent delivery paths, both driven by the same semantic labels that
:mod:`scene_props` puts on the props:

``GTAnnotationRecorder`` (files)
    Attaches a Replicator ``BasicWriter`` to a render product per camera and
    writes an annotation dataset to disk — ``rgb_<n>.png``,
    ``bounding_box_2d_tight_<n>.npy`` + ``..._labels_<n>.json``,
    ``bounding_box_3d_<n>.npy``, ``camera_params_<n>.json`` — one folder per
    camera. This is the one to use for training data.

``add_bbox_ros_publisher`` (live topics)
    Builds a standalone OmniGraph that publishes ``vision_msgs/Detection2DArray``
    and ``Detection3DArray`` on ROS 2, mirroring how ``scene_prep``'s overhead
    camera publisher is wired. Use it to *see* the boxes in Foxglove while a
    mission runs.

Both need prims to carry semantic labels — an unlabelled world produces empty
boxes, not an error. :func:`scene_props.add_vehicles` labels what it spawns;
use :func:`scene_props.label_prims_matching` for scenery that was already there.

Capture rate
------------
The recorder deliberately does **not** use capture-on-play: at 60 render FPS a
BasicWriter would try to write every frame for every drone. Instead capture is
driven explicitly from the sim loop via :meth:`GTAnnotationRecorder.maybe_capture`
at ``capture_hz``, using ``rep.orchestrator.step(delta_time=0.0,
pause_timeline=False)`` — the documented way to grab SDG frames without
advancing or pausing physics.
"""

import os
import time


class GTAnnotationRecorder:
    """Writes an RGB + bounding-box annotation dataset from live sim cameras.

    Args:
        cameras:        list of ``(name, camera_prim_path)``. *name* becomes the
                        output sub-folder, so use something like ``"robot_1"``.
        output_dir:     directory the dataset is written to (created if needed).
        resolution:     ``(width, height)`` of the annotation render product.
                        Independent of whatever the ROS camera streams at.
        capture_hz:     frames per second of wall-clock time to capture at.
                        ``0`` disables throttling (capture on every call).
        bbox_2d_tight / bbox_2d_loose / bbox_3d / semantic_segmentation /
        instance_segmentation / rgb:
                        which annotators the writer emits.
        rt_subframes:   extra RTX subframes rendered per capture to let noisy
                        effects converge. 0 keeps captures cheap.
    """

    def __init__(self, cameras, output_dir, resolution=(1280, 720),
                 capture_hz=2.0, rgb=True, bbox_2d_tight=True,
                 bbox_2d_loose=False, bbox_3d=True, semantic_segmentation=False,
                 instance_segmentation=False, camera_params=True,
                 rt_subframes=0):
        import omni.replicator.core as rep

        self._rep = rep
        self._interval = (1.0 / float(capture_hz)) if capture_hz else 0.0
        self._rt_subframes = int(rt_subframes)
        self._last = 0.0
        self._frames = 0
        self._closed = False
        self._render_products = []
        self._writer = None

        self.output_dir = _ensure_writable_dir(output_dir)

        # Explicit capture only — see module docstring.
        rep.orchestrator.set_capture_on_play(False)

        for name, prim_path in cameras:
            try:
                rp = rep.create.render_product(prim_path, tuple(resolution), name=name)
            except Exception as exc:                            # noqa: BLE001
                print(f"[gt_annotations] could not create render product for "
                      f"{name} ({prim_path}): {exc}", flush=True)
                continue
            self._render_products.append(rp)

        if not self._render_products:
            print("[gt_annotations] no render products — recorder disabled", flush=True)
            self._closed = True
            return

        self._writer = rep.writers.get("BasicWriter")
        self._writer.initialize(
            output_dir=self.output_dir,
            rgb=rgb,
            bounding_box_2d_tight=bbox_2d_tight,
            bounding_box_2d_loose=bbox_2d_loose,
            bounding_box_3d=bbox_3d,
            semantic_segmentation=semantic_segmentation,
            instance_segmentation=instance_segmentation,
            camera_params=camera_params,
        )
        self._writer.attach(self._render_products)

        print(f"[gt_annotations] recording {len(self._render_products)} camera(s) "
              f"@ {capture_hz} Hz → {self.output_dir}", flush=True)

    @property
    def frames_captured(self) -> int:
        return self._frames

    def maybe_capture(self, now: float = None) -> bool:
        """Capture one annotated frame if ``capture_hz`` says it is due.

        Safe to call every iteration of the sim loop. Returns True if a frame
        was captured.
        """
        if self._closed or self._writer is None:
            return False

        now = time.time() if now is None else now
        if self._interval and (now - self._last) < self._interval:
            return False
        self._last = now

        try:
            self._rep.orchestrator.step(delta_time=0.0,
                                        rt_subframes=self._rt_subframes,
                                        pause_timeline=False)
        except TypeError:
            # Older signatures don't take rt_subframes / pause_timeline.
            self._rep.orchestrator.step(delta_time=0.0)
        except Exception as exc:                                # noqa: BLE001
            print(f"[gt_annotations] capture failed: {exc}", flush=True)
            return False

        self._frames += 1
        if self._frames % 20 == 1:
            print(f"[gt_annotations] captured frame {self._frames} "
                  f"→ {self.output_dir}", flush=True)
        return True

    def close(self):
        """Flush pending writes and release the render products."""
        if self._closed:
            return
        self._closed = True
        try:
            if self._writer is not None:
                self._writer.detach()
            for rp in self._render_products:
                rp.destroy()
            self._rep.orchestrator.wait_until_complete()
        except Exception as exc:                                # noqa: BLE001
            print(f"[gt_annotations] close: {exc}", flush=True)
        print(f"[gt_annotations] wrote {self._frames} frame(s) to "
              f"{self.output_dir}", flush=True)


def _ensure_writable_dir(path: str) -> str:
    """Create *path*, falling back to /tmp if it is not writable.

    The default output location lives under the repo bind-mount so the dataset
    lands on the host; when the sim runs without that mount, fall back rather
    than crashing a mission at frame zero.
    """
    try:
        os.makedirs(path, exist_ok=True)
        probe = os.path.join(path, ".write_test")
        with open(probe, "w") as fh:
            fh.write("")
        os.remove(probe)
        return path
    except Exception as exc:                                    # noqa: BLE001
        fallback = os.path.join("/tmp", "gt_annotations")
        os.makedirs(fallback, exist_ok=True)
        print(f"[gt_annotations] {path} not writable ({exc}) — using {fallback}",
              flush=True)
        return fallback


# ---------------------------------------------------------------------------
# Live ROS 2 bounding boxes
# ---------------------------------------------------------------------------

# ROS2CameraHelper's bbox modes → the vision_msgs type each publishes.
BBOX_ROS_TYPES = {
    "bbox_2d_tight": "vision_msgs/Detection2DArray",
    "bbox_2d_loose": "vision_msgs/Detection2DArray",
    "bbox_3d": "vision_msgs/Detection3DArray",
}


def add_bbox_ros_publisher(graph_path: str, camera_prim_path: str,
                           robot_name: str, domain_id: int,
                           types=("bbox_2d_tight", "bbox_3d"),
                           topic_namespace: str = "sensors/gt",
                           frame_id: str = "camera_left",
                           width: int = 640, height: int = 360,
                           publish_labels: bool = True):
    """Publish live ground-truth bounding boxes from one camera over ROS 2.

    Builds a standalone OmniGraph at *graph_path* with its own ROS2Context on
    *domain_id* (each robot is on its own DDS domain in this stack), a render
    product for *camera_prim_path*, and one ROS2CameraHelper per requested
    bbox type. Topics land at
    ``/{robot_name}/{topic_namespace}/{type}`` — e.g.
    ``/robot_1/sensors/gt/bbox_2d_tight``.

    With *publish_labels*, each helper also publishes a ``std_msgs/String`` of
    the semantic-id → label map on ``<topic>_labels``; without it the detections
    carry numeric class ids you cannot resolve.

    Note this adds a second render product for the camera, which costs GPU time
    on top of the RGB/depth streams the ZED subgraph already renders.
    """
    import omni.graph.core as og

    unknown = [t for t in types if t not in BBOX_ROS_TYPES]
    if unknown:
        raise ValueError(f"unsupported bbox type(s) {unknown}; "
                         f"choose from {sorted(BBOX_ROS_TYPES)}")

    controller = og.Controller()
    g = graph_path
    if og.get_graph_by_path(g) is None:
        og.Controller.create_graph({"graph_path": g, "evaluator_name": "execution"})

    ctx = f"{g}/GTContext"
    tick = f"{g}/GTOnPlaybackTick"
    rp = f"{g}/GTCreateRenderProduct"
    frame_const = f"{g}/GTFrameId"
    ns_const = f"{g}/GTNamespace"

    create_nodes = [
        (ctx, "isaacsim.ros2.bridge.ROS2Context"),
        (tick, "omni.graph.action.OnPlaybackTick"),
        (rp, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        (frame_const, "omni.graph.nodes.ConstantString"),
        (ns_const, "omni.graph.nodes.ConstantString"),
    ]
    connects = [
        (f"{tick}.outputs:tick", f"{rp}.inputs:execIn"),
    ]
    set_values = [
        (("inputs:domain_id", ctx), int(domain_id)),
        (("inputs:cameraPrim", rp), camera_prim_path),
        (("inputs:width", rp), int(width)),
        (("inputs:height", rp), int(height)),
        (("inputs:value", frame_const), str(frame_id)),
        (("inputs:value", ns_const), f"{robot_name}/{topic_namespace}"),
    ]

    for bbox_type in types:
        helper = f"{g}/GT_{bbox_type}"
        create_nodes.append((helper, "isaacsim.ros2.bridge.ROS2CameraHelper"))
        connects += [
            (f"{rp}.outputs:execOut", f"{helper}.inputs:execIn"),
            (f"{rp}.outputs:renderProductPath", f"{helper}.inputs:renderProductPath"),
            (f"{ctx}.outputs:context", f"{helper}.inputs:context"),
            (f"{frame_const}.inputs:value", f"{helper}.inputs:frameId"),
            (f"{ns_const}.inputs:value", f"{helper}.inputs:nodeNamespace"),
        ]
        set_values += [
            (("inputs:type", helper), bbox_type),
            (("inputs:topicName", helper), bbox_type),
            (("inputs:enableSemanticLabels", helper), bool(publish_labels)),
            (("inputs:semanticLabelsTopicName", helper), f"{bbox_type}_labels"),
        ]

    controller.edit(
        graph_id=g,
        edit_commands={
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.CONNECT: connects,
            og.Controller.Keys.SET_VALUES: set_values,
        },
    )

    topics = [f"/{robot_name}/{topic_namespace}/{t}" for t in types]
    print(f"[gt_annotations] GT bbox publisher for {camera_prim_path} "
          f"(domain {domain_id}) → {', '.join(topics)}", flush=True)
    return topics

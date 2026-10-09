"""Request-triggered assessment capture on the existing Office drone camera.

No ROS publisher, model input, navigation authority, timeline control or new camera.
Native imports are deferred until an explicitly requested capture starts.
"""
import hashlib
import json
from pathlib import Path
import re
import time
import uuid

CAMERA = "/World/base_link/ZEDCamera/base_link/ZED_X/camera_left"


def pose_matrix(position, quaternion_xyzw):
    """Column-vector rigid pose for offline diagnostics, not map authority."""
    import numpy as np
    position, quaternion = np.asarray(position, dtype=float), np.asarray(quaternion_xyzw, dtype=float)
    if (position.shape != (3,) or quaternion.shape != (4,)
            or not np.isfinite(position).all() or not np.isfinite(quaternion).all()
            or not np.isclose(np.linalg.norm(quaternion), 1., atol=1e-3, rtol=0)):
        raise ValueError("invalid rigid pose")
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    matrix = np.eye(4)
    matrix[:3, 3] = position
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    return matrix


def pose_errors(first, second):
    """Translation norm and geodesic rotation residual; no alignment fitting."""
    import numpy as np
    import math
    translation = float(np.linalg.norm(first[:3, 3] - second[:3, 3]))
    rotation = math.degrees(math.acos(float(np.clip(
        (np.trace(first[:3, :3].T @ second[:3, :3]) - 1) / 2, -1, 1))))
    return dict(translation_error_m=translation, rotation_error_degrees=rotation)


def usd_pose_matrix(matrix, meters_per_unit=1., *, optical=False):
    """USD row-vector world pose → metric column pose, optionally optical axes."""
    import numpy as np
    value = np.asarray(matrix, dtype=float)
    if (value.shape != (4, 4) or not np.isfinite(value).all()
            or not np.isfinite(meters_per_unit) or meters_per_unit <= 0):
        raise ValueError("invalid USD pose")
    result = value.T.copy()
    result[:3, 3] *= meters_per_unit
    if optical:
        result[:3, :3] = result[:3, :3] @ np.diag([1., -1., -1.])
    return result


def stage_mount_record(stage):
    """Composed USD diagnostics only; not historical renderer/PhysX authority."""
    from pxr import UsdGeom, UsdPhysics
    cache = UsdGeom.XformCache()
    paths = ["/World/base_link", "/World/base_link/body", "/World/base_link/body/body",
             "/World/base_link/ZEDCamera", "/World/base_link/ZEDCamera/base_link",
             "/World/base_link/ZEDCamera/base_link/ZED_X", CAMERA,
             "/World/base_link/ZEDCamera/base_link/ZED_X/camera_right"]
    chain = []
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid() or not prim.IsActive():
            raise ValueError("missing active camera attachment prim: " + path)
        xform = UsdGeom.Xformable(prim)
        chain.append(dict(path=path, rigid_body_api=prim.HasAPI(UsdPhysics.RigidBodyAPI),
            reset_xform_stack=xform.GetResetXformStack(),
            local_transform=[list(row) for row in xform.GetLocalTransformation()],
            world_transform=[list(row) for row in cache.GetLocalToWorldTransform(prim)]))
    joints = []
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith("/World/base_link/ZEDCamera/") and prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            joints.append(dict(path=str(prim.GetPath()),
                body0=[str(p) for p in joint.GetBody0Rel().GetTargets()],
                body1=[str(p) for p in joint.GetBody1Rel().GetTargets()]))
    return dict(phase="composed USD callback read; not historical render state",
                chain=chain, joints=joints)


class CaptureRequest:
    """Bounded, renewable captures; IDs are never resumed or overwritten."""
    def __init__(self, directory, *, max_frames=3, max_duration_s=15):
        self.directory = Path(directory)
        self.max_frames, self.max_duration_s = max_frames, max_duration_s
        self.seen = set()
        self.active = None
        self.started = self.count = 0
        self.reason = "idle"

    def poll(self, now):
        path = self.directory / "office-camera-request.json"
        if not path.exists():
            self.active = None
            self.reason = "idle"
            return
        if path.stat().st_size > 4096:
            raise ValueError("oversized capture request")
        value = json.loads(path.read_text())
        name = value.get("capture_id")
        if (not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", name)
                or type(value.get("enabled")) is not bool):
            raise ValueError("invalid capture request")
        if not value["enabled"]:
            self.active = None
            self.reason = "disabled"
        elif name != self.active and name not in self.seen:
            self.seen.add(name)
            output = self.directory / ("office-camera-" + name)
            output.mkdir(exist_ok=False)
            self.active, self.output = name, output
            self.started, self.count, self.reason = now, 0, "recording"
        if self.active and (now - self.started >= self.max_duration_s or self.count >= self.max_frames):
            self.reason = "frame_limit" if self.count >= self.max_frames else "duration_limit"
            self.active = None


def payload_record(data):
    """Validate the same-callback payload without rebasing its render clock."""
    import numpy as np
    rgb = np.asarray(data["rgb"])
    segmentation = data["semantic_segmentation"]
    mask = np.asarray(segmentation["data"]).squeeze()
    if (rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] not in (3, 4)
            or mask.dtype != np.uint32 or mask.shape != rgb.shape[:2]):
        raise ValueError("invalid native RGB/segmentation payload")
    reference = data["ReferenceTime"]
    numerator, denominator = reference["referenceTimeNumerator"], reference["referenceTimeDenominator"]
    if (type(numerator) is not int or type(denominator) is not int
            or numerator < 0 or denominator <= 0):
        raise ValueError("invalid native render reference")
    rgb = np.ascontiguousarray(rgb[:, :, :3])
    camera = data["camera_params"]
    # Retain the renderer's own pose, not current USD transforms for a past frame.
    view = np.asarray(camera["cameraViewTransform"], dtype=float).reshape(4, 4)
    if not np.isfinite(view).all():
        raise ValueError("nonfinite rendered camera pose")
    ros_time = data["IsaacReadSimulationTime"]["simulationTime"]
    if (isinstance(ros_time, (bool, np.bool_))
            or not isinstance(ros_time, (int, float, np.integer, np.floating))
            or not np.isfinite(ros_time) or ros_time < 0):
        raise ValueError("invalid same-render ROS simulation clock")
    return rgb, mask, dict(
        source_stamp_ns=numerator * 1_000_000_000 // denominator,
        render_reference=dict(numerator=numerator, denominator=denominator),
        ros_simulation_time_s=float(ros_time),
        ros_source_stamp_ns=int(float(ros_time) * 1_000_000_000),
        ros_clock_source="same-callback IsaacReadSimulationTime render-product annotator",
        rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),
        width=int(rgb.shape[1]), height=int(rgb.shape[0]),
        camera_view_transform=view.tolist(),
        camera_params={key: value.tolist() if hasattr(value, "tolist") else value
                       for key, value in camera.items()},
        semantic_labels={str(key): value for key, value in segmentation["info"]["idToLabels"].items()})


class OfficeCameraTeacher:
    def __init__(self, directory, stage, *, launcher_sha256, warn=print):
        self.requests = CaptureRequest(directory)
        self.stage, self.warn = stage, warn
        self.episode = uuid.uuid4().hex
        self.launcher_sha256 = launcher_sha256
        self.producer_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.writer = None
        self.next_poll = 0
        self.error = None
        self.last_status = None
        self.initial_mount = None

    def poll(self):
        now = time.monotonic()
        if now < self.next_poll:
            return
        self.next_poll = now + .25
        try:
            self.requests.poll(now)
            if self.writer is not None and self.writer.capture_id != self.requests.active:
                self.writer.detach()
                self.writer = None
            if self.requests.active and self.writer is None:
                self.error = None
                self._attach()
        except Exception as error:
            self.error = str(error)
            self.requests.active = None
            self.requests.reason = "error"
            if self.writer is not None:
                try:
                    self.writer.detach()
                except Exception:
                    pass
                self.writer = None
            self.warn("[office-camera-teacher] disabled: " + str(error))
        status = dict(episode_id=self.episode, capture_id=self.requests.active,
            frames=self.requests.count, reason=self.requests.reason, error=self.error,
            writer_attached=self.writer is not None, execution_dispatch=False)
        if status != self.last_status:
            try:
                status["receipt_monotonic_s"] = now
                path = self.requests.directory / "office-camera-status.json"
                temporary = path.with_suffix(".json.tmp")
                temporary.write_text(json.dumps(status, sort_keys=True) + "\n")
                temporary.replace(path)
                status.pop("receipt_monotonic_s")
                self.last_status = status
            except Exception as error:
                self.warn("[office-camera-teacher] status write failed: " + str(error))

    def _attach(self):
        import numpy as np
        import omni.replicator.core as rep
        from pxr import UsdGeom, UsdRender
        from isaacsim.core.utils.semantics import get_semantics

        products = [str(prim.GetPath()) for prim in self.stage.Traverse()
                    if prim.IsA(UsdRender.Product)
                    and [str(path) for path in UsdRender.Product(prim).GetCameraRel().GetTargets()] == [CAMERA]]
        if len(products) != 1:
            raise ValueError("expected one existing drone-left render product, found " + str(len(products)))
        owner = self
        self.initial_mount = stage_mount_record(self.stage)

        class PairedWriter(rep.Writer):
            def __init__(self):
                self.capture_id = owner.requests.active
                self.annotators = [rep.AnnotatorRegistry.get_annotator("rgb"),
                    rep.AnnotatorRegistry.get_annotator("semantic_segmentation",
                        init_params={"colorize": False, "semanticTypes": ["class"]}),
                    rep.AnnotatorRegistry.get_annotator("ReferenceTime"),
                    rep.AnnotatorRegistry.get_annotator("IsaacReadSimulationTime"),
                    rep.AnnotatorRegistry.get_annotator("camera_params")]

            def write(self, data):
                if (owner.requests.active != self.capture_id
                        or owner.requests.count >= owner.requests.max_frames
                        or time.monotonic() - owner.requests.started >= owner.requests.max_duration_s):
                    return
                try:
                    rgb, mask, record = payload_record(data)
                    entities = []
                    cache = UsdGeom.XformCache()
                    for entity in ("blue_marker", "orange_marker"):
                        prim = owner.stage.GetPrimAtPath("/World/RRMMarkers/" + entity)
                        exists = bool(prim.IsValid() and prim.IsActive())
                        identity = prim.GetAttribute("rrm:entity_id").Get() if exists else None
                        kind = prim.GetAttribute("rrm:kind").Get() if exists else None
                        if exists and (identity != entity or ("class", entity) not in get_semantics(prim).values()):
                            raise ValueError("actual prim semantic identity mismatch")
                        entities.append(dict(entity_id=entity, scene_prim=str(prim.GetPath()),
                            exists=exists, kind=kind, observed_entity_id=identity,
                            world_position_stage_units=list(cache.GetLocalToWorldTransform(prim).ExtractTranslation()) if exists else None))
                    record.update(schema_version="rrm-office-camera-capture/v1", episode_id=owner.episode,
                        capture_id=self.capture_id, camera_prim=CAMERA, camera_frame_id="camera_left",
                        camera_topic="/robot_1/sensors/front_stereo/left/image_rect", render_product=products[0],
                        receipt_monotonic_s=time.monotonic(), launcher_sha256=owner.launcher_sha256,
                        producer_sha256=owner.producer_sha256, stage_entities=entities,
                        stage_meters_per_unit=float(UsdGeom.GetStageMetersPerUnit(owner.stage)),
                        stage_observation_phase="writer callback; not historical rendered marker transforms",
                        map_alignment_verified=False, ros_frame_matched=False, execution_dispatch=False)
                    record["stage_mount"] = stage_mount_record(owner.stage)
                    record["attachment_before_capture"] = owner.initial_mount
                    # Separate actual vehicle input to simulated sensors from PX4
                    # estimate. This read is callback-phase, not historical render pose.
                    try:
                        from physical_truth import body_record
                        from pegasus.simulator.logic.vehicle_manager import VehicleManager
                        vehicle = VehicleManager.get_vehicle_manager().vehicles["/World/base_link"]
                        record["vehicle_physical_input"] = body_record(vehicle, "/World/base_link")
                        record["vehicle_physical_input"]["phase"] = "writer callback; not historical render state"
                        from omni.isaac.core.world import World
                        record["vehicle_physical_input"]["read_simulation_time_s"] = float(World.instance().current_time)
                        record["vehicle_physical_input"]["body_local_rotation_inv_xyzw"] = vehicle._body_local_rotation_inv.as_quat().tolist()
                    except Exception as error:
                        record["vehicle_physical_input"] = {"error": str(error)}
                    folder = owner.requests.output / str(owner.requests.count)
                    folder.mkdir(exist_ok=False)
                    np.save(folder / "rgb.npy", rgb, allow_pickle=False)
                    np.save(folder / "semantic-pixels.npy", mask, allow_pickle=False)
                    (folder / "capture.json").write_text(json.dumps(record, sort_keys=True, indent=2) + "\n")
                    owner.requests.count += 1
                except Exception as error:
                    owner.error = str(error)
                    owner.requests.active = None
                    owner.requests.reason = "error"
                    owner.warn("[office-camera-teacher] payload rejected: " + str(error))

        self.writer = PairedWriter()
        self.writer.attach(products[0])

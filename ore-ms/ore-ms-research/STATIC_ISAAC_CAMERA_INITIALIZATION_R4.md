# Static Isaac camera initialization qualification R4

27 November 2026 UTC. **PASS offline.** The corrected exporter reads the effective USD camera and pinhole intrinsics before attaching the R3 writers. It stops on a mismatched pose. The earlier R4 run remains **FAIL**. No Isaac Sim execution or retry was performed.

## Failure trace

The pinned stage defines `/World/ReceiptCamera` with translation `(0, 0, 1)` and a double-precision `xformOp:rotateX` of 90°. The failed exporter opened that stage, saved the original transform-operation objects, constructed the legacy Isaac `Camera` wrapper, reset the world, and called `camera.initialize()`. The first observed error occurred afterward, when `op.Set(value)` attempted to reuse one of the saved objects. The traceback does not name the individual operation or report a replacement stage or prim. [R4 execution report and retained evidence](STATIC_ISAAC_GEOMETRY_CAPTURE_R4.md) · [Pinned authored stage](static-isaac-capture-r1/input/stage.usda)

The tagged 5.1 source wraps an existing Camera prim when its path is valid. Its constructor delegates through `BaseSensor` and `SingleXFormPrim` into `XFormPrim` with transform reset enabled by default. The reset implementation removes existing Euler operations, including the fixture's `xformOp:rotateX`, and installs translation, orientation and scale operations. That constructor path can invalidate the saved operation without recreating the prim. The retained run did not log individual operation identities, so the exact invalidation event cannot be confirmed retrospectively. The source supports constructor-time property removal as the leading explanation. It supplies no evidence that `Camera.initialize()` itself replaced the stage. [Camera constructor](https://github.com/isaac-sim/IsaacSim/blob/v5.1.0/source/extensions/isaacsim.sensors.camera/isaacsim/sensors/camera/camera.py#L229) · [BaseSensor wrapper](https://github.com/isaac-sim/IsaacSim/blob/v5.1.0/source/extensions/isaacsim.core.api/python/impl/sensors/base_sensor.py#L62) · [Transform-reset default](https://github.com/isaac-sim/IsaacSim/blob/v5.1.0/source/extensions/isaacsim.core.prims/python/impl/single_xform_prim.py#L86) · [Operation reset implementation](https://github.com/isaac-sim/IsaacSim/blob/v5.1.0/source/extensions/isaacsim.core.prims/python/impl/xform_prim.py#L1194)

## Corrected setup

The camera stays authored by the pinned USD. The exporter copies the authored matrix values, initializes a Replicator render product against the camera path, and reacquires the camera from the active pinned stage. This uses the render-product creation API also used by the Isaac Camera wrapper, without that wrapper's transform canonicalization. The exporter does not rewrite any camera pose. It checks the fresh prim's validity and stationarity and compares the effective matrix to both the authored matrix and the frozen optical-frame convention using the verifier's existing `64 × ulp(10.0)` roundoff allowance. A replacement prim at the approved path must pass the same checks. [Corrected exporter](static-isaac-geometry-r4/capture_r4.py) · [Replicator render-product API](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/1.12.27/source/extensions/omni.replicator.core/docs/API.html#omni.replicator.core.create.render_product)

The exporter reads the render product's camera binding and resolution, and the fresh camera's projection, focal length, filmback apertures and offsets. It derives the centered pinhole matrix with the same focal-length and aperture ratios as the pinned Camera API and requires the frozen intrinsics exactly. This checks the effective camera configuration. Live renderer agreement remains a separate check. The `camera-setup/` directory provides a synced subphase journal and authored and effective readbacks. A mismatch or exception propagates through the original top-level lifecycle gate before acquisition. [Render-product readback APIs](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.utils/docs/index.html#render-product) · [Pinned intrinsics derivation](https://github.com/isaac-sim/IsaacSim/blob/v5.1.0/source/extensions/isaacsim.sensors.camera/isaacsim/sensors/camera/camera.py#L1703)

## Offline qualification and live dependency

Ten CPU-only camera tests cover retained and replaced prims, stale operation avoidance, missing or invalid cameras, a different root stage, changed translation and orientation, invalid filmback and intrinsics, and durable exception and success records. All 46 R2/R3 recorder tests and seven frozen receipt tests passed. The persisted R1–R4 manifests and frozen M05–M08 checksums verified. The original failed exporter remains byte-identical in its capture bundle and at commit `4d386b41c`. Code after camera setup is byte-identical, including the 5 m and 15 m controls, the 10 m issue and target observations, exact two-second timing, native reference association, ray checks and shutdown. [Offline gate results](static-isaac-geometry-r4/camera-qualification.json) · [Deterministic camera tests](static-isaac-geometry-r4/test_camera_setup.py)

```bash
research=ore-ms/ore-ms-research
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$research/static-isaac-annotator-r3:$research/static-isaac-recorder-r2" \
  python3 -m unittest discover -v -s "$research/static-isaac-geometry-r4"
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$research/static-isaac-recorder-r2" \
  python3 -m unittest discover -v -s "$research/static-isaac-annotator-r3"
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -v -s "$research/static-isaac-recorder-r2"
PYTHONDONTWRITEBYTECODE=1 python3 "$research/static-isaac-receipt/test_verify.py" -v
```

A separately authorized native run must confirm that the direct render product binds to the pinned camera, leaves the surveyed pose and intrinsics within the frozen checks, and delivers native R3 writer packets in the existing configuration. No geometry, timing or receipt gate has been requalified on live data.

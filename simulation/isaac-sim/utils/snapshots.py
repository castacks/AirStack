"""snapshots — aim a review camera and write viewport PNGs a host can read.

The launch-script benches can tell you what they BUILT (counts, `[settle]`
lines, a readiness banner). Only a picture tells you they built the right
thing, and this is the missing sense: a camera prim the bench aims, the active
viewport retargeted onto it, and a PNG on disk.

    place_camera(stage, eye, target)       aim /World/ReviewCamera
    snapshot(path)                         write what the viewport draws
    views_around(stage, {name: (x, y)}, …) a top and an oblique per point
    overview(stage, centre, span_m, …)     one plumb shot of a whole area

WHERE THE FILES HAVE TO GO
--------------------------
Only one directory inside the container is visible from outside it:

    container   /isaac-sim/.nvidia-omniverse/logs/<name>
    host        ~/docker/isaac-sim/logs/<name>

A capture written anywhere else is written, announced in the log, and
unreachable by whoever asked for it. Benches take the directory as an env var
(`SNAP_DIR=`) precisely so the caller can point it at the mount.

IMPORT IT BY FILE PATH, AT THE TAIL, INSIDE A TRY
-------------------------------------------------
This module imports `carb`, `omni.kit.app` and `pxr` at module level, and none
of those exist until `SimulationApp(...)` has built the Kit environment — the
same reason every launcher's own `pxr` import sits below its `SimulationApp`
call. And Isaac's script runner does not reliably set `__file__`, so a plain
`import snapshots` depends on a `sys.path` entry set hundreds of lines earlier
still being intact. Hence:

    import importlib.util as _ilu
    _spec = _ilu.spec_from_file_location(
        "snapshots", os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py"))
    _snaps = _ilu.module_from_spec(_spec); _spec.loader.exec_module(_snaps)

and wrap the calls in `try` — a failed capture must never take down a scene
someone is looking at.

UNITS
-----
`place_camera`, `snapshot` and `overview`'s `centre` are in STAGE units.
`views_around` is the one entry point that takes METRES and applies `ssf`
itself. Get the factor from `scene_prep.get_stage_meters_per_unit(stage) ->
(mpu, ssf)`; on a centimetre stage `mpu=0.01`, `ssf=100`.
"""

import math
import os

import carb
import omni.kit.app
from pxr import Gf, Sdf, UsdGeom

CAMERA_PATH = "/World/ReviewCamera"

#: 18 mm on this aperture frames 1.16x the eye height across and 0.65x down.
#: So `h ~ span / 1.16` to fit `span` metres in frame.
FOCAL_MM = 18.0
APERTURE_MM = 20.955

#: Viewport decorations that `capture_viewport_to_file` would otherwise bake
#: into the PNG. The world reference grid is the dangerous one: it is Kit's,
#: not the scene's, and it reads in a capture as a defect in the ground —
#: which is exactly how it has been misdiagnosed before, once on a bench whose
#: scene contained no overlay at all. Everything here is best-effort: a key
#: that does not exist in this Kit build is set and ignored, which is cheaper
#: than probing and keeps the list readable.
_DECOR_SETTINGS = (
    ("/app/viewport/grid/enabled", False),
    ("/app/viewport/outline/enabled", False),
    ("/app/viewport/boundingBoxes/enabled", False),
    ("/app/viewport/show/camera", False),
    ("/app/viewport/show/lights", False),
    ("/app/viewport/show/audio", False),
    ("/app/viewport/show/skeletons", False),
    ("/persistent/app/viewport/displayOptions", 0),
)


def hide_decorations() -> None:
    """Turn off grid, gizmos and outlines so the PNG shows only the scene."""
    s = carb.settings.get_settings()
    for key, value in _DECOR_SETTINGS:
        try:
            if isinstance(value, bool):
                s.set_bool(key, value)
            else:
                s.set_int(key, int(value))
        except Exception:                                        # noqa: BLE001
            pass


def _pump(frames: int) -> None:
    app = omni.kit.app.get_app()
    for _ in range(max(0, int(frames))):
        app.update()


def _look_at(eye, target):
    """World transform for a camera at *eye* looking at *target*.

    Built as a matrix rather than Euler angles on purpose. A USD camera looks
    down its own -Z with +Y up, which is exactly the convention
    `Gf.Matrix4d.SetLookAt` encodes, so inverting the view matrix gives the
    prim transform with no rotation-order question to get wrong.

    THE PLUMB CASE IS PINNED. Looking straight down, the view direction is
    parallel to +Z and any up vector in the XY plane is legal — so the yaw is
    unconstrained and whatever falls out is arbitrary. `up = +Y` fixes it so a
    top-down capture reads like a map: +X to the right, +Y up the image. A
    quarter-turned plumb view has been read off a capture as a real yaw error
    in the scene, and an asset's yaw-offset "fixed" against it.
    """
    e = Gf.Vec3d(*[float(v) for v in eye])
    t = Gf.Vec3d(*[float(v) for v in target])
    fwd = t - e
    if fwd.GetLength() < 1e-9:
        fwd = Gf.Vec3d(0.0, 0.0, -1.0)
    fwd = fwd.GetNormalized()
    up = Gf.Vec3d(0.0, 1.0, 0.0) if abs(fwd[2]) > 0.999 else Gf.Vec3d(0.0, 0.0, 1.0)
    return Gf.Matrix4d().SetLookAt(e, t, up).GetInverse()


def place_camera(stage, eye, target, focal_mm: float = FOCAL_MM):
    """Define (or reuse) `/World/ReviewCamera`, aim it, and make it active.

    *eye* and *target* are 3-tuples in STAGE units on a Z-up stage.
    """
    cam = UsdGeom.Camera.Define(stage, Sdf.Path(CAMERA_PATH))
    prim = cam.GetPrim()
    cam.CreateFocalLengthAttr().Set(float(focal_mm))
    cam.CreateHorizontalApertureAttr().Set(APERTURE_MM)
    cam.CreateVerticalApertureAttr().Set(APERTURE_MM * 9.0 / 16.0)
    # Wide enough for a whole city block from 300 m up; near enough for a
    # close-up on one body.
    cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.5, 20000.0))

    xf = UsdGeom.Xformable(prim)
    ops = [op for op in xf.GetOrderedXformOps()
           if op.GetOpType() == UsdGeom.XformOp.TypeTransform]
    op = ops[0] if ops else None
    if op is None:
        xf.ClearXformOpOrder()
        op = xf.AddTransformOp()
    op.Set(_look_at(eye, target))

    try:
        from omni.kit.viewport.utility import get_active_viewport
        vp = get_active_viewport()
        if vp is not None:
            vp.camera_path = CAMERA_PATH
    except Exception as exc:                                     # noqa: BLE001
        print(f"[snapshots] could not retarget the viewport: {exc}")
    return prim


def snapshot(path: str, frames: int = 40) -> str:
    """Write the active viewport to *path*. Returns the path.

    *frames* app updates before the capture, because the image is ray-traced
    and converges over frames — a capture taken immediately after a camera
    move is a picture of the noise. Ten more after it, because
    `capture_viewport_to_file` returns a future this deliberately does not
    await: pumping is enough inside a loop, but for the LAST capture before an
    exit, trust `ls` over this function's return value.
    """
    hide_decorations()
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    _pump(frames)
    try:
        from omni.kit.viewport.utility import (capture_viewport_to_file,
                                               get_active_viewport)
        capture_viewport_to_file(get_active_viewport(), path)
    except Exception as exc:                                     # noqa: BLE001
        print(f"[snapshots] capture failed: {exc}")
        return ""
    _pump(10)
    print(f"[snapshots] -> {path}")
    return path


def overview(stage, centre, span_m: float, out_path: str, ssf: float = 1.0,
             frames: int = 40) -> str:
    """One plumb shot covering roughly *span_m* metres across.

    The eye goes to `span_m * 0.95`, which at the default focal length frames
    about 1.11x `span_m` horizontally by 0.62x vertically — a little margin
    across, and a wide-screen crop down.
    """
    cx, cy = float(centre[0]), float(centre[1])
    h = float(span_m) * 0.95
    place_camera(stage, (cx * ssf, cy * ssf, h * ssf), (cx * ssf, cy * ssf, 0.0))
    return snapshot(out_path, frames)


def views_around(stage, points: dict, out_dir: str, ssf: float = 1.0,
                 top_h: float = 60.0, obl_dist: float = 45.0,
                 obl_h: float = 22.0, frames: int = 40,
                 target_z: float = 1.0) -> list:
    """`<name>_top.png` + `<name>_obl.png` for each ``{name: (x, y)}``, in METRES.

    The oblique looks from the south-west so the default sky's sun is on the
    subject rather than behind it, and aims at *target_z* (subject height, not
    the ground) — aiming at 0 puts the subject at the top of the frame.
    """
    written = []
    d = float(obl_dist) / math.sqrt(2.0)
    for name, (x, y) in sorted(points.items()):
        x, y = float(x), float(y)
        place_camera(stage, (x * ssf, y * ssf, float(top_h) * ssf),
                     (x * ssf, y * ssf, 0.0))
        written.append(snapshot(os.path.join(out_dir, f"{name}_top.png"), frames))
        place_camera(stage, ((x - d) * ssf, (y - d) * ssf, float(obl_h) * ssf),
                     (x * ssf, y * ssf, float(target_z) * ssf))
        written.append(snapshot(os.path.join(out_dir, f"{name}_obl.png"), frames))
    return [p for p in written if p]

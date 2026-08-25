"""snapshots.py — viewport captures from authored cameras, for reviewing a
scene build without a person at the GUI.

A launcher that builds a big scene can only be judged by looking at it, and
the agent running it cannot see the Kit window. This is the missing piece: a
camera prim placed by (eye, target), the active viewport retargeted to it,
and `capture_viewport_to_file` after enough frames for the renderer to
converge. `views_around` turns a list of named world points into a top-down
plus an oblique for each — the two angles a review needs — so a launcher can
do

    import importlib.util, os
    _p = os.path.join(os.path.dirname(__file__), "..", "utils", "snapshots.py")
    _s = importlib.util.spec_from_file_location("snapshots", _p)
    snapshots = importlib.util.module_from_spec(_s); _s.loader.exec_module(snapshots)
    snapshots.views_around(stage, {"lot": (x, y), "queue": (x2, y2)}, SNAP_DIR)

after its own banner and leave the PNGs where the host can read them.

Loaded by path (importlib) like `scene_prep.py`, because Isaac's script
runner does not reliably set `__file__` for sys.path tricks.
"""

import math
import os

import carb
import omni.kit.app
from pxr import Gf, UsdGeom

CAM = "/World/ReviewCamera"

# VIEWPORT DECORATIONS OFF BEFORE CAPTURING, and this is not cosmetic.
# `capture_viewport_to_file` grabs what the viewport draws, which INCLUDES
# Kit's world reference grid — a regular lattice over the whole frame. It was
# read as a defect in the scene twice: first as a burn-scar overlay printing a
# 3 m grid across the plat, then again on a bench whose ground is plain grass
# and which has no overlay in it at all. That second capture is what proved it
# was the viewport, because there was nothing in the scene left to blame.
# Anything that draws on top of the render and is not geometry belongs in this
# list; the names differ between Kit versions, so all the known spellings are
# set and the misses are harmless.
_DECOR_SETTINGS = (
    "/app/viewport/grid/enabled",
    "/persistent/app/viewport/grid/enabled",
    "/app/viewport/show/grid",
    "/app/viewport/outline/enabled",
    "/persistent/app/viewport/outline/enabled",
    "/app/viewport/show/audio",
    "/app/viewport/show/camera",
    "/app/viewport/show/light",
)


def hide_decorations():
    """Turn off grid / selection outline / gizmos in the active viewport."""
    try:
        import carb.settings
        st = carb.settings.get_settings()
        for key in _DECOR_SETTINGS:
            try:
                st.set_bool(key, False)
            except Exception:
                pass
    except Exception as exc:
        carb.log_warn(f"[snapshots] could not clear viewport settings: {exc}")
    # The per-viewport API is the authoritative one where it exists; the carb
    # settings above are what older builds read.
    try:
        import omni.kit.viewport.utility as vp
        api = vp.get_active_viewport_window()
        if api is not None and hasattr(api, "viewport_api"):
            for attr in ("draw_grid", "draw_selection", "draw_gizmos"):
                if hasattr(api.viewport_api, attr):
                    setattr(api.viewport_api, attr, False)
    except Exception:
        pass


def _look_at(eye, target):
    """XYZ Euler (degrees) for a -Z-forward, +Y-up camera at *eye* looking at
    *target*, on a Z-up stage.

    STRAIGHT DOWN IS A SPECIAL CASE, and getting it wrong quietly rotates
    every top-down capture. When the eye is directly over the target,
    `dx` and `dy` are both zero, `atan2(0, 0)` returns 0 and the yaw comes out
    -90 — so the image is turned a quarter turn and world +X runs UP the
    frame instead of to the right. Nothing about the picture looks broken,
    which is the problem: a car correctly pointing along +Y was read off such
    a capture as pointing along +X, and an asset's `yaw-offset` was "fixed"
    against it. Pin the yaw to 0 for a plumb view so +X is right and +Y is up,
    the way a map reads.
    """
    dx, dy, dz = (target[0] - eye[0], target[1] - eye[1], target[2] - eye[2])
    horiz = math.hypot(dx, dy)
    yaw = 0.0 if horiz < 1e-6 else math.degrees(math.atan2(dy, dx)) - 90.0
    pitch = math.degrees(math.atan2(dz, horiz)) + 90.0
    return Gf.Vec3f(pitch, 0.0, yaw)


def place_camera(stage, eye, target, focal_mm=18.0):
    cam = UsdGeom.Camera.Get(stage, CAM)
    if not cam:
        cam = UsdGeom.Camera.Define(stage, CAM)
        cam.GetHorizontalApertureAttr().Set(20.955)
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.5, 20000.0))
    cam.GetFocalLengthAttr().Set(float(focal_mm))
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*eye))
    xf.AddRotateXYZOp().Set(_look_at(eye, target))
    try:
        import omni.kit.viewport.utility as vp
        vp.get_active_viewport().camera_path = CAM
    except Exception as exc:
        carb.log_warn(f"[snapshots] could not retarget the viewport: {exc}")


def snapshot(path, frames=40):
    """Capture the active viewport to *path* after *frames* updates."""
    import omni.kit.viewport.utility as vp
    hide_decorations()
    app = omni.kit.app.get_app()
    for _ in range(frames):
        app.update()
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    vp.capture_viewport_to_file(vp.get_active_viewport(), path)
    for _ in range(10):
        app.update()
    print(f"[snapshots] -> {path}")


def views_around(stage, points, out_dir, ssf=1.0, top_h=60.0, obl_dist=45.0,
                 obl_h=22.0, frames=40):
    """Top-down + oblique capture for each named world point (metres).

    *ssf* is the stage's scene-scale factor (units per metre) so callers pass
    metres. The oblique looks from the south-west, which keeps the sun on the
    subject under the default sky.
    """
    for name, (x, y) in points.items():
        X, Y = float(x) * ssf, float(y) * ssf
        place_camera(stage, (X, Y, top_h * ssf), (X, Y, 0.0))
        snapshot(os.path.join(out_dir, f"{name}_top.png"), frames)
        d = obl_dist * ssf / math.sqrt(2.0)
        place_camera(stage, (X - d, Y - d, obl_h * ssf), (X, Y, 1.0 * ssf))
        snapshot(os.path.join(out_dir, f"{name}_obl.png"), frames)


def overview(stage, centre, span_m, out_path, ssf=1.0, frames=40):
    """One top-down of the whole plate: height chosen so *span_m* fits an
    18 mm lens."""
    X, Y = float(centre[0]) * ssf, float(centre[1]) * ssf
    h = span_m * ssf * 0.95
    place_camera(stage, (X, Y, h), (X, Y, 0.0))
    snapshot(out_path, frames)

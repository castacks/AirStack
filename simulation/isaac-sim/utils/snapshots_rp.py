"""Headless viewport-free capture, via a Replicator render product.

WHY THIS EXISTS BESIDE `snapshots.py`
-------------------------------------
`snapshots.py` captures the ACTIVE VIEWPORT with
`omni.kit.viewport.utility.capture_viewport_to_file`. That is the right tool
on a workstation container, where Kit has a real window behind the viewport,
and every launcher in this repo uses it.

IT SEGFAULTS ON THE OSMO WORKSPACE POD. Measured 2026-08-30 on
`airstack-dev-175`, assembling `suburb_hurricane_500_l3`: the scene built,
the ground truth was written, and the process died with SIGSEGV inside

    save_aov_to_file  (omni.kit.widget.viewport/capture.py:175)
    capture_aov       (omni.kit.viewport.utility/__init__.py:395)
    snapshot          (utils/snapshots.py:129)

The reason is in the compose file's own comment: the container the pod runs
is the `isaac-sim-livestream` variant — *"Headless: no X server, no display,
no GUI window"*. There is no window behind the viewport, so the AOV the
capture asks for has nothing backing it. Nothing in the launcher is wrong;
the capture path simply does not apply to that container.

WHAT THIS DOES INSTEAD is the standard Isaac SDG path: define an explicit
camera prim, attach a Replicator RENDER PRODUCT to it, step the orchestrator,
and read the `rgb` annotator straight out of memory. No viewport, no window,
no AOV file writer — so it works on any container that can render at all,
headless or not. It is also strictly more capable: the resolution is a
parameter rather than whatever the window happens to be.

The API deliberately mirrors `snapshots.py` — `place_camera`, `overview`,
`views_around`, same arguments in the same order — so a launcher can swap one
import for the other and change nothing else.

    from snapshots_rp import overview, views_around      # headless pod
    from snapshots   import overview, views_around       # desktop container
"""

import math
import os

import carb
from pxr import Gf, Sdf, UsdGeom

CAM = "/World/reviewCamRP"

# 1600 x 1600 rather than a 16:9 frame. Every subject here is a PLAN — a
# plate, a lot, a stretch of flooded street — and a square frame wastes none
# of it on sky. The number is a compromise: 2048 costs about 2.2x the render
# time for detail that a review pass does not use, and 1024 is not enough to
# tell a stripped roof deck from a bare rafter set at plate scale.
RES = (1600, 1600)

# RTX needs several subframes to converge a path-traced frame; one step gives
# a visibly noisy image and the denoiser has not settled. 24 is where the
# noise stops being visible on these scenes.
SUBFRAMES = 24

_STATE = {"rp": None, "annot": None, "res": None}


def _look_at(eye, target):
    """XYZ Euler (degrees) for a -Z-forward, +Y-up camera at *eye* looking at
    *target*, on a Z-up stage. Lifted verbatim from `snapshots._look_at`,
    INCLUDING its special case, which is load-bearing:

    STRAIGHT DOWN IS A SPECIAL CASE. When the eye is directly over the
    target, `dx` and `dy` are both zero, `atan2(0, 0)` returns 0 and the yaw
    comes out -90 — so the image is turned a quarter turn and world +X runs
    UP the frame instead of to the right. Nothing about the picture looks
    broken, which is exactly the problem: a car correctly pointing along +Y
    was once read off such a capture as pointing along +X. Pin the yaw to 0
    for a plumb view so +X is right and +Y is up, the way a map reads.
    """
    dx, dy, dz = (target[0] - eye[0], target[1] - eye[1], target[2] - eye[2])
    horiz = math.hypot(dx, dy)
    yaw = 0.0 if horiz < 1e-6 else math.degrees(math.atan2(dy, dx)) - 90.0
    pitch = math.degrees(math.atan2(dz, horiz)) + 90.0
    return Gf.Vec3f(pitch, 0.0, yaw)


def place_camera(stage, eye, target, focal_mm=18.0):
    """Define (once) and pose the review camera. Same contract as
    `snapshots.place_camera`, minus the viewport retarget — there is no
    viewport to retarget."""
    cam = UsdGeom.Camera.Get(stage, CAM)
    if not cam:
        cam = UsdGeom.Camera.Define(stage, CAM)
        cam.GetHorizontalApertureAttr().Set(20.955)
        # 20 km far plane: a 1 km plate seen from 950 m with anything tall on
        # it needs far more than the 1000 m Kit defaults to, and a clipped
        # horizon reads as missing geometry rather than as a clip.
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.5, 20000.0))
    cam.GetFocalLengthAttr().Set(float(focal_mm))
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*eye))
    xf.AddRotateXYZOp().Set(_look_at(eye, target))
    return cam


def _ensure_product(res):
    """One render product for the whole run, rebuilt only if `res` changes.

    A NEW RENDER PRODUCT PER CAPTURE LEAKS AND THEN STALLS. Each one holds
    its own GPU buffers, and Replicator does not release them promptly when
    the Python handle goes out of scope; a dozen subjects at 1600 x 1600 is
    enough to matter on a shared card. Reuse the product and only re-pose the
    camera it is attached to.
    """
    import omni.replicator.core as rep
    if _STATE["rp"] is not None and _STATE["res"] == tuple(res):
        return _STATE["annot"]
    _STATE["rp"] = rep.create.render_product(CAM, resolution=tuple(res))
    _STATE["annot"] = rep.AnnotatorRegistry.get_annotator("rgb")
    _STATE["annot"].attach([_STATE["rp"]])
    _STATE["res"] = tuple(res)
    # Capture on demand, never on play — the timeline is stopped in every
    # assembly launcher and an orchestrator that waits for play never fires.
    rep.orchestrator.set_capture_on_play(False)
    return _STATE["annot"]


def _write_png(rgb, path):
    """`rgb` is an (H, W, 4) uint8 array from the annotator."""
    import numpy as np
    from PIL import Image
    arr = np.asarray(rgb)
    if arr.ndim == 3 and arr.shape[2] == 4:
        arr = arr[:, :, :3]
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    Image.fromarray(arr.astype("uint8")).save(path)


def snapshot(stage, path, res=RES, subframes=SUBFRAMES):
    """Render the review camera to `path`. Returns True on success.

    Never raises: a review capture that fails must not take the scene down
    with it — the scene is the expensive part and the whole point of writing
    the ground truth before the pictures.
    """
    try:
        import numpy as np
        import omni.kit.app
        import omni.replicator.core as rep
        annot = _ensure_product(res)
        app = omni.kit.app.get_app()

        # PUMP, STEP, PUMP — and then CHECK THE PIXELS.
        #
        # `orchestrator.step` returning is NOT a guarantee the frame is ready.
        # Measured 2026-08-30: with a second Isaac process sharing the card,
        # eleven of twelve captures came back as uniform black 7.5 kB PNGs
        # while every one of them printed success. The annotator had data —
        # it was just an unrendered buffer. A capture pass that reports twelve
        # frames and writes eleven blanks is worse than one that fails loudly.
        for _ in range(8):
            app.update()
        for attempt in range(3):
            rep.orchestrator.step(rt_subframes=int(subframes))
            for _ in range(4):
                app.update()
            data = annot.get_data()
            if data is None or not len(data):
                continue
            arr = np.asarray(data)
            # A real frame of an outdoor scene has both sky and ground in it.
            # Uniform (std ~ 0) means nothing was drawn. The threshold is
            # loose on purpose — a legitimately flat frame would still carry
            # sensor-level variation well above 1.0/255.
            if float(arr[..., :3].std()) > 1.0:
                _write_png(arr, path)
                print(f"[snapshots_rp] -> {path}")
                return True
            carb.log_warn(f"[snapshots_rp] blank frame for {path} "
                          f"(attempt {attempt + 1}/3); re-rendering")
        print(f"[snapshots_rp] GAVE UP on {path}: frame stayed blank")
        return False
    except Exception as exc:
        carb.log_warn(f"[snapshots_rp] capture failed for {path}: {exc}")
        print(f"[snapshots_rp] capture FAILED for {path}: {exc}")
        return False


def overview(stage, centre, span_m, out_path, ssf=1.0, res=RES,
             subframes=SUBFRAMES):
    """One top-down of the whole plate, height chosen so `span_m` fits an
    18 mm lens. Same signature as `snapshots.overview` with `frames` replaced
    by `res`/`subframes`."""
    X, Y = float(centre[0]) * ssf, float(centre[1]) * ssf
    place_camera(stage, (X, Y, span_m * ssf * 0.95), (X, Y, 0.0))
    return snapshot(stage, out_path, res, subframes)


def views_around(stage, points, out_dir, ssf=1.0, top_h=60.0, obl_dist=45.0,
                 obl_h=22.0, res=RES, subframes=SUBFRAMES, azimuth_deg=225.0,
                 aim_h=1.0):
    """Top-down + oblique capture for each named world point (metres).

    The oblique looks from the south-west by default (`azimuth_deg` 225 —
    the compass bearing of the CAMERA from the subject: 0 = east of it,
    90 = north, 180 = west, 270 = south), which keeps the sun on the subject
    under the default sky — same choice `snapshots.views_around` makes, and
    for the same reason. A fire review wants the camera on the BURNING
    elevation and aimed at the band, not at the doorstep: pass `azimuth_deg`
    and `aim_h` (metres above ground the oblique looks at).
    """
    ok = 0
    for name, (x, y) in points.items():
        X, Y = float(x) * ssf, float(y) * ssf
        place_camera(stage, (X, Y, top_h * ssf), (X, Y, 0.0))
        ok += bool(snapshot(stage, os.path.join(out_dir, f"{name}_top.png"),
                            res, subframes))
        a = math.radians(float(azimuth_deg))
        cx_, cy_ = X + obl_dist * ssf * math.cos(a), Y + obl_dist * ssf * math.sin(a)
        place_camera(stage, (cx_, cy_, obl_h * ssf), (X, Y, float(aim_h) * ssf))
        ok += bool(snapshot(stage, os.path.join(out_dir, f"{name}_obl.png"),
                            res, subframes))
    return ok

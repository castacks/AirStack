"""preview — photograph each archetype as it is baked.

A library is a few hundred USDs of rubble with names like
`SM_Building_26_pancaked.usd`. Nothing about that file says whether the wreck
inside it reads as a collapsed building or as a heap of grey shards, and
opening them one at a time in Kit costs a minute each — so in practice nobody
looks, and a pipeline regression rides into every scene that references the
library. The bake is the ONE moment the geometry is already on a lit stage
with a settled pile on it, so a capture there is nearly free.

Two frames per rung, and each answers a different question:

    _obl.png   does this read as a damaged building? Silhouette, facade,
               which way it fell.
    _top.png   what did it leave on the ground? A plumb view is the only one
               that shows the debris ring, and debris is what a drone
               searching the rubble actually flies over.

FRAMED OFF THE MEASURED BBOX, never a constant. The library runs from a 12 m
shed to a 96 m tower, and one camera distance cannot hold both — the same
mistake `scene_launch_script` records making with its wreck close-ups, where a
fixed 48 m put the eye inside an 81 m building and photographed the inside of
its own facade.

WHY NOT `simulation/isaac-sim/utils/snapshots.py`
------------------------------------------------
It does this job for the launchers and this module deliberately repeats a
little of it. `scene_gen/` sits at the repo root because it is sim-agnostic:
importing from `simulation/isaac-sim/` would make the generator depend on the
simulator's tree. Everything Kit-specific here is imported lazily inside the
functions, so the module still imports under a plain `python3`.

NOTHING HERE MAY RAISE INTO THE BAKE. A bake is an hours-long batch job and a
missing PNG is not a reason to lose it, so every entry point returns "" on
failure and says why.
"""

from __future__ import annotations

import math
import os

#: 18 mm on a 20.955 mm aperture at 16:9 — the same optics as the launchers'
#: review captures, so a bake preview and a scene close-up of the same wreck
#: are comparable pictures.
FOCAL_MM = 18.0
APERTURE_MM = 20.955
_TAN_H = (APERTURE_MM / 2.0) / FOCAL_MM          # 0.582
_TAN_V = _TAN_H * 9.0 / 16.0                     # 0.327

CAMERA_PATH = "/World/PreviewCamera"

#: Where previews go, relative to the disaster's library directory. Beside the
#: USDs rather than in a sibling tree, so copying a library copies its
#: pictures and a manifest's `preview` field stays valid.
PREVIEW_DIR = "previews"

#: App updates before a capture. The image is ray-traced and converges over
#: frames; a capture taken straight after a camera move is a picture of the
#: noise. 24 is the floor that came out clean on the bake's lighting rig
#: (dome 900 + one key), against the launchers' 40-48 for a whole city.
FRAMES = 24

#: ...and for the FIRST capture of a process, which also pays for the
#: ray-tracing pipelines Kit compiles on demand. Everything after it renders a
#: warm renderer; only the first frame of the first cell renders a cold one,
#: and one black picture at the head of a library is exactly the kind of thing
#: that gets read as a broken pipeline.
FIRST_FRAMES = 90
_warmed = False

#: Viewport furniture that would otherwise be baked into the PNG. The world
#: grid is the dangerous one — it is Kit's, not the scene's, and reads in a
#: capture as a defect in the ground.
_DECOR = (
    ("/app/viewport/grid/enabled", False),
    ("/app/viewport/outline/enabled", False),
    ("/app/viewport/boundingBoxes/enabled", False),
    ("/app/viewport/show/camera", False),
    ("/app/viewport/show/lights", False),
    ("/persistent/app/viewport/displayOptions", 0),
)


def preview_dir(library_dir: str) -> str:
    return os.path.join(library_dir, PREVIEW_DIR)


def _hide_decorations():
    import carb
    s = carb.settings.get_settings()
    for key, value in _DECOR:
        try:
            if isinstance(value, bool):
                s.set_bool(key, value)
            else:
                s.set_int(key, int(value))
        except Exception:                                        # noqa: BLE001
            pass


def _pump(n):
    import omni.kit.app
    app = omni.kit.app.get_app()
    for _ in range(max(0, int(n))):
        app.update()


def _look_at(eye, target):
    """World transform for a camera at *eye* aimed at *target*.

    A matrix, not Euler angles: a USD camera looks down its own -Z with +Y up,
    which is what `SetLookAt` encodes, so there is no rotation order to get
    wrong. The plumb case pins up to +Y so a top-down frame reads like a map
    (+X right, +Y up) instead of taking an arbitrary yaw from the degenerate
    cross product.
    """
    from pxr import Gf

    e = Gf.Vec3d(*[float(v) for v in eye])
    t = Gf.Vec3d(*[float(v) for v in target])
    fwd = t - e
    if fwd.GetLength() < 1e-9:
        fwd = Gf.Vec3d(0.0, 0.0, -1.0)
    fwd = fwd.GetNormalized()
    up = (Gf.Vec3d(0.0, 1.0, 0.0) if abs(fwd[2]) > 0.999
          else Gf.Vec3d(0.0, 0.0, 1.0))
    return Gf.Matrix4d().SetLookAt(e, t, up).GetInverse()


def _place(stage, eye, target):
    from pxr import Gf, Sdf, UsdGeom

    cam = UsdGeom.Camera.Define(stage, Sdf.Path(CAMERA_PATH))
    cam.CreateFocalLengthAttr().Set(FOCAL_MM)
    cam.CreateHorizontalApertureAttr().Set(APERTURE_MM)
    cam.CreateVerticalApertureAttr().Set(APERTURE_MM * 9.0 / 16.0)
    cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.25, 20000.0))
    xf = UsdGeom.Xformable(cam.GetPrim())
    ops = [op for op in xf.GetOrderedXformOps()
           if op.GetOpType() == UsdGeom.XformOp.TypeTransform]
    if ops:
        op = ops[0]
    else:
        xf.ClearXformOpOrder()
        op = xf.AddTransformOp()
    op.Set(_look_at(eye, target))
    try:
        from omni.kit.viewport.utility import get_active_viewport
        vp = get_active_viewport()
        if vp is not None:
            vp.camera_path = CAMERA_PATH
    except Exception as exc:                                     # noqa: BLE001
        print(f"[preview] could not retarget the viewport: {exc}")
    return cam.GetPrim()


def _capture(path: str, frames: int = FRAMES) -> str:
    global _warmed

    _hide_decorations()
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    if not _warmed:
        frames = max(frames, FIRST_FRAMES)
        _warmed = True
    _pump(frames)
    try:
        from omni.kit.viewport.utility import (capture_viewport_to_file,
                                               get_active_viewport)
        vp = get_active_viewport()
        if vp is None:
            print("[preview] no active viewport — is this a --no-window run?")
            return ""
        capture_viewport_to_file(vp, path)
    except Exception as exc:                                     # noqa: BLE001
        print(f"[preview] capture failed: {exc}")
        return ""
    # `capture_viewport_to_file` returns a future this deliberately does not
    # await; pumping is enough inside a loop, but the file only exists a few
    # frames later — so a caller that must KNOW should stat it, not trust the
    # return value of the last capture before an exit.
    _pump(8)
    return path


def trimmed_bounds(stage, paths, keep: float = 0.90):
    """`world_bounds`, ignoring the outermost pieces. For the OBLIQUE only.

    A cell is a wreck plus the debris it shed, and the debris ring throws
    considerably wider than the building — so framing the full bbox frames the
    lot, and the building it is a picture OF ends up a third of the frame.
    Measured on `BG_Building_D_pancaked`: the ring reaches about 1.5x the
    footprint, and the building read small enough to be hard to judge.

    The plumb view deliberately keeps the full bounds, because the ring is
    exactly what that view exists to show. Here the subject is the building,
    so the outer *keep* fraction of pieces (by distance of each piece's centre
    from the whole cell's centre) sets the frame and the stragglers are
    allowed out of it. Nothing is deleted — this only moves a camera.

    Falls back to the full bounds whenever the trim cannot be computed, so a
    cell of one merged mesh behaves exactly as before.
    """
    from pxr import Sdf, UsdGeom

    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_,
                                  UsdGeom.Tokens.render,
                                  UsdGeom.Tokens.proxy])
    boxes = []
    for p in paths or ():
        prim = stage.GetPrimAtPath(Sdf.Path(str(p)))
        if not prim or not prim.IsValid():
            continue
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        mn, mx = r.GetMin(), r.GetMax()
        boxes.append(((float(mn[0]), float(mn[1]), float(mn[2])),
                      (float(mx[0]), float(mx[1]), float(mx[2]))))
    full = world_bounds(stage, paths)
    if len(boxes) < 5 or full is None:
        return full
    (cx, cy, _cz), _size = full
    def _key(b):
        mx_, my_ = 0.5 * (b[0][0] + b[1][0]), 0.5 * (b[0][1] + b[1][1])
        return math.hypot(mx_ - cx, my_ - cy)
    boxes.sort(key=_key)
    boxes = boxes[:max(1, int(round(len(boxes) * float(keep))))]
    lo = [min(b[0][i] for b in boxes) for i in range(3)]
    hi = [max(b[1][i] for b in boxes) for i in range(3)]
    if any(hi[i] - lo[i] <= 0.0 for i in range(3)):
        return full
    return ((0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]),
             0.5 * (lo[2] + hi[2])),
            (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]))


def world_bounds(stage, paths):
    """``(centre, size)`` in world metres over *paths*, or None if empty.

    Every purpose the bake authors is included: the fragments are default
    purpose, but a referenced asset can put its geometry under `render`, and
    a bbox that missed it framed the camera on a fifth of the building.
    """
    from pxr import Gf, Sdf, UsdGeom

    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_,
                                  UsdGeom.Tokens.render,
                                  UsdGeom.Tokens.proxy])
    box = Gf.BBox3d()
    got = False
    for p in paths or ():
        prim = stage.GetPrimAtPath(Sdf.Path(str(p)))
        if not prim or not prim.IsValid():
            continue
        b = cache.ComputeWorldBound(prim)
        r = b.ComputeAlignedRange()
        if r.IsEmpty():
            continue
        box = Gf.BBox3d.Combine(box, b) if got else b
        got = True
    if not got:
        return None
    rng = box.ComputeAlignedRange()
    if rng.IsEmpty():
        return None
    mn, mx = rng.GetMin(), rng.GetMax()
    return ((float(mn[0] + mx[0]) * 0.5, float(mn[1] + mx[1]) * 0.5,
             float(mn[2] + mx[2]) * 0.5),
            (float(mx[0] - mn[0]), float(mx[1] - mn[1]),
             float(mx[2] - mn[2])))


def _basis(bearing_deg: float, elev_deg: float):
    """``(e, u, w)`` — the unit vector from the aim point toward the eye, and
    the camera's right and up axes, matching `_look_at`'s convention."""
    th, az = math.radians(float(elev_deg)), math.radians(float(bearing_deg))
    e = (math.cos(th) * math.cos(az), math.cos(th) * math.sin(az),
         math.sin(th))
    # f = -e is the view direction; world up is +Z away from plumb.
    f = (-e[0], -e[1], -e[2])
    wu = (0.0, 1.0, 0.0) if abs(f[2]) > 0.999 else (0.0, 0.0, 1.0)
    u = (f[1] * wu[2] - f[2] * wu[1], f[2] * wu[0] - f[0] * wu[2],
         f[0] * wu[1] - f[1] * wu[0])
    n = math.sqrt(sum(c * c for c in u)) or 1.0
    u = tuple(c / n for c in u)
    w = (u[1] * f[2] - u[2] * f[1], u[2] * f[0] - u[0] * f[2],
         u[0] * f[1] - u[1] * f[0])
    return e, u, w


def _fit_distance(size, aim_drop: float = 0.0, bearing_deg: float = 225.0,
                  elev_deg: float = 22.0, margin: float = 1.06) -> float:
    """How far back the eye must be for all eight bbox corners to be in frame.

    EXACT, not a bounding sphere. The sphere is the easy answer and it wastes
    the frame: it is the box's DIAGONAL in every direction, so a long low
    workshop seen along its length is photographed as if it were as tall as it
    is long, and the subject came out filling about 40% of the picture. Over a
    library of a few hundred rungs that is the difference between a sheet you
    can read damage off and one you cannot.

    The camera axes are perpendicular to the eye direction, so a corner's
    offset across the frame does not depend on the distance and the constraint
    solves in closed form. With `a = c . e` (the corner's depth toward the
    eye) and `c . u`, `c . w` its offsets across frame:

        depth = d - a,  and the corner is in frame when
        |c.u| <= tan_h * (d - a)  and  |c.w| <= tan_v * (d - a)

    so d >= a + |c.u| / tan_h and d >= a + |c.w| / tan_v, maximised over the
    eight corners.

    *aim_drop* is how far BELOW the bbox centre the camera aims, and it shifts
    the corners rather than being ignored — aiming low without paying for it
    crops the subject by exactly that much at the top, which on a tower is the
    part that says whether it is still standing.
    """
    e, u, w = _basis(bearing_deg, elev_deg)
    hx, hy, hz = (0.5 * float(v) for v in size)
    best = 0.0
    for sx in (-hx, hx):
        for sy in (-hy, hy):
            for sz in (-hz, hz):
                # Relative to the AIM point, which sits `aim_drop` below the
                # bbox centre.
                c = (sx, sy, sz + float(aim_drop))
                a = c[0] * e[0] + c[1] * e[1] + c[2] * e[2]
                cu = abs(c[0] * u[0] + c[1] * u[1] + c[2] * u[2])
                cw = abs(c[0] * w[0] + c[1] * w[1] + c[2] * w[2])
                best = max(best, a + cu / _TAN_H, a + cw / _TAN_V)
    return max(best, 1e-3) * float(margin)


def capture_cell(stage, paths, out_stem: str, ssf: float = 1.0,
                 bearing_deg: float = 225.0, elev_deg: float = 22.0,
                 frames: int = FRAMES) -> dict:
    """Photograph what is at *paths*. Returns ``{"obl": path, "top": path}``.

    *out_stem* is the path without a suffix; `_obl.png` and `_top.png` are
    added. *ssf* converts metres to stage units, matching the rest of the
    generator — the bake's own stage is metres, so it is 1.

    The oblique looks from the SOUTH-WEST by default, which is where
    `prepare_stage` aims its key light: from the other side every facade in
    the library is in its own shadow.
    """
    got = {}
    try:
        full = world_bounds(stage, paths)
        if full is None:
            print("[preview] nothing to photograph")
            return got
        # The OBLIQUE frames the building; the plumb view below frames
        # everything, debris ring included. See `trimmed_bounds`.
        bounds = trimmed_bounds(stage, paths) or full
        (cx, cy, cz), size = bounds
        # In METRES — the bake's stage is metres, but do not assume it.
        cx, cy, cz = cx / ssf, cy / ssf, cz / ssf
        sx, sy, sz = (v / ssf for v in size)

        # Aim BELOW the centroid: a collapsed building's mass is at its base,
        # and a camera level with the centroid of a bbox that still includes
        # the thrown fragments looks over the top of the pile at the horizon.
        drop = 0.15 * sz
        d = _fit_distance((sx, sy, sz), drop, bearing_deg, elev_deg)
        e, _u, _w = _basis(bearing_deg, elev_deg)
        tz = max(0.0, cz - drop)
        eye = (cx + d * e[0], cy + d * e[1], max(1.5, tz + d * e[2]))
        _place(stage, tuple(v * ssf for v in eye),
               tuple(v * ssf for v in (cx, cy, tz)))
        p = _capture(out_stem + "_obl.png", frames)
        if p:
            got["obl"] = p

        # Plumb, on the FULL bounds — the debris ring is what this view is
        # for. Framed on the FOOTPRINT, not the bounding sphere: the ring is
        # flat, so height would only push the camera needlessly far back.
        (fx, fy, fz), (fsx, fsy, fsz) = full
        fx, fy, fz = fx / ssf, fy / ssf, fz / ssf
        fsx, fsy, fsz = (v / ssf for v in (fsx, fsy, fsz))
        h = (fz + 0.5 * fsz) + _fit_distance((fsx, fsy, 0.0), 0.0,
                                             0.0, 90.0, margin=1.08)
        _place(stage, tuple(v * ssf for v in (fx, fy, h)),
               tuple(v * ssf for v in (fx, fy, fz - 0.5 * fsz)))
        p = _capture(out_stem + "_top.png", frames)
        if p:
            got["top"] = p
    except Exception as exc:                                     # noqa: BLE001
        # A bake is an hours-long batch job. A missing picture is not a
        # reason to lose it.
        print(f"[preview] FAILED: {type(exc).__name__}: {exc}")
    return got

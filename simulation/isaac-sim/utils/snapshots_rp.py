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


# ---------------------------------------------------------------------------
# THE BACKGROUND-DOMINATED GATE (STREAM C, 2026-08-31)
# ---------------------------------------------------------------------------
#
# `std > 1` in `snapshot()` catches a BLANK frame. It does not catch a frame
# that rendered FINE but shows the wrong thing: the review lead's finding on
# `deep_water_obl.png`/`dry_inland_obl.png`/`shoreline_obl.png` (V2_L3) was a
# real, non-blank image of the finite terrain slab's own cut EDGE hanging
# over the background HDRI ground — three stacked bands (water, ground,
# soil) with acres of empty background beyond them. That frame has plenty of
# std; it is simply a picture of the wrong thing.
#
# Two independent, LOUD-ONLY signals (never retried — see the docstring on
# `_flag_offplate`):
#
#   1. GEOMETRIC — is the point the camera is actually AIMED AT inside
#      `region` at all? Cheap, exact, and catches a broader class of bug
#      (an un-scaled coordinate, a swapped x/y) than just the framing
#      complaint.
#   2. COLOUR — what fraction of the frame falls inside the HDRI-ground
#      colour band, measured 2026-08-31 from `deep_water_obl.png`'s
#      lower-left quadrant (the known-bad background):
#      RGB mean (103, 114, 75), std (24, 17, 19), P5-P95 per channel
#      R [64, 143] G [84, 139] B [45, 109]. `_HDRI_GROUND_RGB_LO/HI` widen
#      that range by ~10 for margin.
#
#      THIS SIGNAL IS WEAK ON ITS OWN. Measured on the same six V2_L3
#      subjects: `worst_house_top`'s lower-left quadrant (in-scene lawn, not
#      background) reads (90, 85, 59) — inside the same band, because the
#      suburb's own grass renders a similar green under the same sky. A
#      colour hit alone is a "look at this," not a verdict, which is why it
#      is one of two checks and neither one aborts anything.
_HDRI_GROUND_RGB_LO = (55, 75, 35)
_HDRI_GROUND_RGB_HI = (155, 150, 120)
_HDRI_GROUND_FRAC = 0.35


def _offplate_fraction(arr):
    """Fraction of `arr`'s pixels (`(H, W, >=3)` uint8-ish) inside the
    calibrated HDRI-ground colour band. Pure numpy, no `pxr`."""
    import numpy as np
    rgb = np.asarray(arr)[..., :3].astype(np.int16)
    lo = np.array(_HDRI_GROUND_RGB_LO, dtype=np.int16)
    hi = np.array(_HDRI_GROUND_RGB_HI, dtype=np.int16)
    band = np.all((rgb >= lo) & (rgb <= hi), axis=-1)
    return float(band.mean())


def _flag_offplate(arr, path, region=None, target=None):
    """After a successful (non-blank) capture: flag, never retry, a frame
    that looks like it is dominated by the off-plate background. See the
    banner above this function for the two signals and why the colour one is
    deliberately weak. `region`/`target` are `(x0,y0,x1,y1)` / `(x,y)` in the
    SAME world-metre units — both optional, and the geometric check is
    skipped (not flagged) when either is missing, so a caller with no plate
    bounds to give (every non-hurricane `views_around`/`overview` user) gets
    silence rather than a spurious warning.
    """
    name = os.path.splitext(os.path.basename(path))[0]
    if region is not None and target is not None:
        x0, y0, x1, y1 = region
        tx, ty = float(target[0]), float(target[1])
        if not (min(x0, x1) <= tx <= max(x0, x1)
                and min(y0, y1) <= ty <= max(y0, y1)):
            msg = ("[snapshots_rp] SUBJECT OFF-PLATE: {0} aims at "
                  "({1:.1f}, {2:.1f}), outside region {3} -- probably a "
                  "coordinate/scale bug, not a framing choice"
                  .format(name, tx, ty, tuple(round(v, 1) for v in region)))
            carb.log_warn(msg)
            print(msg)
    try:
        frac = _offplate_fraction(arr)
    except Exception:
        return
    if frac > _HDRI_GROUND_FRAC:
        msg = ("[snapshots_rp] BACKGROUND-DOMINATED: {0} is {1:.0f}% inside "
              "the HDRI-ground colour band (threshold {2:.0f}%) -- likely "
              "framing the plate edge or empty background, not review "
              "content. Not retried; a human should look."
              .format(name, frac * 100.0, _HDRI_GROUND_FRAC * 100.0))
        carb.log_warn(msg)
        print(msg)


def snapshot(stage, path, res=RES, subframes=SUBFRAMES, region=None,
            target=None):
    """Render the review camera to `path`. Returns True on success.

    Never raises: a review capture that fails must not take the scene down
    with it — the scene is the expensive part and the whole point of writing
    the ground truth before the pictures.

    `region`/`target`, if given, feed the background-dominated sanity gate
    (`_flag_offplate`) run after a successful capture — see its docstring.
    Both default to `None`, so an existing caller that never passes them
    gets exactly the old behaviour.
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
                _flag_offplate(arr, path, region, target)
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
             subframes=SUBFRAMES, region=None):
    """One top-down of the whole plate, height chosen so `span_m` fits an
    18 mm lens. Same signature as `snapshots.overview` with `frames` replaced
    by `res`/`subframes`, plus an optional `region` (world metres) threaded
    to the background-dominated sanity gate (`_flag_offplate`) — harmless
    when omitted."""
    X, Y = float(centre[0]) * ssf, float(centre[1]) * ssf
    place_camera(stage, (X, Y, span_m * ssf * 0.95), (X, Y, 0.0))
    return snapshot(stage, out_path, res, subframes, region=region,
                    target=(float(centre[0]), float(centre[1])))


# A mature tree's canopy is 6-8 m across, and the oblique camera sits 22 m up
# at 45 m out — close enough that a trunk a few metres off the lens fills the
# whole lower frame. That is exactly what happened to `shoreline_obl` on the
# MUD_L3/FLAT_L2 plates: the flood the camera exists to show was hidden behind
# one tree, and the render was reviewed as "worse" partly on that basis.
#
# 12 m clears the canopy with margin without moving the camera far enough to
# change what is being reviewed.
_CLEAR_M = 12.0

# "Look toward the plate centre" cone half-width. Wide on purpose: this is a
# tie-break against the plate edge, not a demand that every oblique look
# exactly at the centre — a subject already >= `review_points`'s
# `edge_margin_m` inside the region only needs to avoid pointing the
# far/background half of the frame OFF the plate, which a +/-60 deg cone
# around the true inward bearing already does at every range this module
# uses.
_INWARD_CONE_DEG = 60.0


def _inward_bearing_deg(x, y, region):
    """Bearing (deg, this module's convention: 0 = +X/east, CCW) from the
    REGION'S CENTRE to `(x, y)` — the OUTWARD radial direction.

    An oblique camera positioned at this bearing FROM the subject (i.e.
    `azimuth_deg` equal to this) sits further from the centre than the
    subject and looks back across it toward the interior, so whatever is
    visible in-frame BEHIND the subject is more of the plate, not its edge.
    That is what "the azimuth looks inward" means operationally — see
    `_clear_azimuth`.

    `None` if `region` is not given, or the subject IS the centre (every
    bearing is equally inward from there).
    """
    if region is None:
        return None
    x0, y0, x1, y1 = region
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    dx, dy = float(x) - cx, float(y) - cy
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return None
    return math.degrees(math.atan2(dy, dx))


def _ang_dist(a, b):
    """Smallest separation between two bearings (deg), in [0, 180]."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


def _clear_azimuth(X, Y, avoid, obl_dist, ssf, preferred,
                   region=None, span=90.0, steps=13):
    """Pick the oblique camera's azimuth against TWO constraints AT ONCE:
    clear of an obstacle that would fill the frame (trees), and looking
    toward the plate's interior rather than its edge.

    `avoid` is world-metre `(x, y)` centres (trees, in practice). `region`,
    if given, is the world-metre `(x0, y0, x1, y1)` plate bounds — see
    `_inward_bearing_deg`.

    SOLVED JOINTLY, not tree-avoidance-then-inward or vice versa: every
    candidate bearing is scored on (a) whether it clears the nearest
    obstacle by `_CLEAR_M`, (b) whether it falls within `_INWARD_CONE_DEG`
    of the true inward bearing, and ties within the same (a, b) tier are
    broken by closeness to `preferred` — which is what keeps the sun where
    `views_around`'s docstring promises it is (the LIGHTING-preserving
    choice) whenever more than one candidate satisfies both hard
    constraints. Candidates come from two sweeps: a fine one symmetric
    around `preferred` out to +/-`span`/2 (identical to the old single-
    constraint sweep, so a caller with no `avoid`/`region` sees no behaviour
    change), plus a coarse 10-degree full-circle sweep so an inward-
    compatible bearing has somewhere to be found even when nothing near
    `preferred` clears both bars.

    Only the camera moves. Nothing about the scene changes, so a review
    render stays a faithful picture of the plate that was built.
    """
    x_m, y_m = X / max(ssf, 1e-9), Y / max(ssf, 1e-9)
    inward = _inward_bearing_deg(x_m, y_m, region)
    near = []
    if avoid:
        near = [(ox, oy) for ox, oy in avoid
                if abs(ox - x_m) < 2.5 * obl_dist
                and abs(oy - y_m) < 2.5 * obl_dist]
    if not near and inward is None:
        return preferred          # nothing to solve for -- old fast path

    half = max(1, (steps - 1) // 2)
    candidates = []
    for k in range(steps):
        off = ((-1) ** k) * (0.5 * span * ((k + 1) // 2) / half)
        candidates.append(preferred + off)
    candidates.extend(preferred + d for d in range(-170, 180, 10))

    def clearance(az):
        if not near:
            return float("inf")
        a = math.radians(az)
        cx_ = X + obl_dist * ssf * math.cos(a)
        cy_ = Y + obl_dist * ssf * math.sin(a)
        return min(math.hypot(cx_ - ox * ssf, cy_ - oy * ssf)
                  for ox, oy in near)

    def inward_ok(az):
        return inward is None or _ang_dist(az, inward) <= _INWARD_CONE_DEG

    best, best_key = preferred, None
    for az in candidates:
        clear_ok = clearance(az) >= _CLEAR_M * ssf
        tier = (0 if (clear_ok and inward_ok(az)) else
               1 if clear_ok else
               2 if inward_ok(az) else 3)
        key = (tier, _ang_dist(az, preferred))
        if best_key is None or key < best_key:
            best_key, best = key, az
    return best


def _half_vfov_deg(focal_mm, aperture_mm=20.955):
    """Half the vertical field of view (deg) for a `focal_mm` lens.

    `RES` is always square (1600x1600 — see its own comment), so the
    vertical FOV a square RENDER actually shows equals the horizontal one
    regardless of whatever vertical-aperture default the `UsdGeom.Camera`
    schema carries: a square resolution has aspect 1, so "fit horizontal"
    and "fit vertical" land on the same number. Using the one aperture this
    module actually authors (`place_camera`'s `20.955`) is therefore exact
    for every camera this file poses, not an approximation.
    """
    return math.degrees(math.atan(float(aperture_mm)
                                  / (2.0 * max(1e-6, float(focal_mm)))))


def _top_ray_ground_point(x, y, az_deg, obl_dist, obl_h, aim_h, half_fov_deg):
    """World `(x, y)` where the oblique's TOP-OF-FRAME ray crosses `z = 0`,
    or `None` if that ray points at or above the horizon (the top of frame
    is sky, not ground — nothing to cap).

    Pure 2-D: the top ray shares the centre ray's azimuth (rotating the
    frame's vertical extent is a pitch-only change about the camera's local
    horizontal axis) and is pitched `half_fov_deg` shallower.
    """
    theta_c = math.degrees(math.atan2(obl_h - aim_h, max(1e-6, obl_dist)))
    theta_top = theta_c - half_fov_deg
    if theta_top <= 0.5:
        return None
    horiz = obl_h / math.tan(math.radians(theta_top))
    a = math.radians(az_deg)
    cam_x, cam_y = x + obl_dist * math.cos(a), y + obl_dist * math.sin(a)
    ux, uy = -math.cos(a), -math.sin(a)      # camera -> subject -> beyond
    return (cam_x + horiz * ux, cam_y + horiz * uy)


def _inside_region(px, py, region):
    x0, y0, x1, y1 = region
    return (min(x0, x1) <= px <= max(x0, x1)
           and min(y0, y1) <= py <= max(y0, y1))


def _cap_oblique_range(x, y, az_deg, obl_dist, obl_h, aim_h, region,
                       focal_mm=18.0, min_scale=0.3, steps=10):
    """Shrink `obl_dist` (never `obl_h` — the elevation is a deliberate per-
    subject choice) until the frame's TOP-OF-FRAME ray lands inside `region`,
    or give up at `min_scale` of the original ("where possible" — a subject
    close enough to an edge that even a near-vertical look still overshoots
    is a real plate-shape limit, not a bug in this function).

    Returns `(obl_dist_to_use, was_capped, still_outside)`. `region=None`
    (no plate bounds known) is a no-op: `(obl_dist, False, False)`.
    """
    if region is None:
        return obl_dist, False, False
    half_fov = _half_vfov_deg(focal_mm)

    def outside(d):
        pt = _top_ray_ground_point(x, y, az_deg, d, obl_h, aim_h, half_fov)
        return pt is not None and not _inside_region(pt[0], pt[1], region)

    if not outside(obl_dist):
        return obl_dist, False, False
    d = obl_dist
    floor = min_scale * obl_dist
    for _ in range(steps):
        d = max(floor, d * 0.85)
        if not outside(d):
            return d, True, False
        if d <= floor:
            break
    return d, True, outside(d)


def views_around(stage, points, out_dir, ssf=1.0, top_h=60.0, obl_dist=45.0,
                 obl_h=22.0, res=RES, subframes=SUBFRAMES, azimuth_deg=225.0,
                 aim_h=1.0, avoid=None, region=None, focal_mm=18.0):
    """Top-down + oblique capture for each named world point (metres).

    The oblique looks from the south-west by default (`azimuth_deg` 225 —
    the compass bearing of the CAMERA from the subject: 0 = east of it,
    90 = north, 180 = west, 270 = south), which keeps the sun on the subject
    under the default sky — same choice `snapshots.views_around` makes, and
    for the same reason. A fire review wants the camera on the BURNING
    elevation and aimed at the band, not at the doorstep: pass `azimuth_deg`
    and `aim_h` (metres above ground the oblique looks at).

    `avoid`: world-metre `(x, y)` centres the OBLIQUE camera should not be
    planted on top of (tree trunks). The bearing is nudged off them by
    `_clear_azimuth` — see there for why, and why only the camera moves.

    `region`: world-metre `(x0, y0, x1, y1)` plate bounds, optional. When
    given, THREE things change, none of which move the subject itself:
    `_clear_azimuth` also requires the chosen bearing to look toward the
    plate's interior (see its docstring and `_inward_bearing_deg`);
    `_cap_oblique_range` shrinks `obl_dist` if the frame's top edge would
    otherwise ray-trace past the plate; and both captures feed `region` to
    `snapshot`'s background-dominated sanity gate. Every one of the three is
    a no-op when `region` is `None`, so an existing caller sees no change.
    """
    ok = 0
    for name, (x, y) in points.items():
        x, y = float(x), float(y)
        X, Y = x * ssf, y * ssf
        place_camera(stage, (X, Y, top_h * ssf), (X, Y, 0.0), focal_mm=focal_mm)
        ok += bool(snapshot(stage, os.path.join(out_dir, f"{name}_top.png"),
                            res, subframes, region=region, target=(x, y)))
        az = _clear_azimuth(X, Y, avoid, obl_dist, ssf, float(azimuth_deg),
                            region=region)
        d_use, capped, still_bad = _cap_oblique_range(
            x, y, az, float(obl_dist), float(obl_h), float(aim_h), region,
            focal_mm=focal_mm)
        if capped:
            print("[snapshots_rp] {0}: oblique range {1:.0f}m -> {2:.0f}m "
                  "to keep the top-of-frame ray on the plate{3}".format(
                      name, obl_dist, d_use,
                      " (still misses -- subject is near an edge)"
                      if still_bad else ""))
        a = math.radians(az)
        cx_, cy_ = (X + d_use * ssf * math.cos(a),
                   Y + d_use * ssf * math.sin(a))
        place_camera(stage, (cx_, cy_, obl_h * ssf), (X, Y, float(aim_h) * ssf),
                    focal_mm=focal_mm)
        ok += bool(snapshot(stage, os.path.join(out_dir, f"{name}_obl.png"),
                            res, subframes, region=region, target=(x, y)))
    return ok

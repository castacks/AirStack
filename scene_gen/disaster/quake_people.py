"""Damage-tied casualty population for urban earthquake scenes.

This deliberately replaces, rather than supplements, the ordinary city
pedestrian pass.  Every visible figure is prone and belongs to one of two
claims that the composed damage geometry can support:

* ``interior_trapped`` -- inside a DG3/DG4 partial collapse, set back from
  the recorded failed face and seated on an upward-facing triangle from the
  building that is actually on stage;
* ``rubble_trapped`` -- outside a DG4/DG5 fall face, seated on that
  building's composed rubble (or the real ground directly below it) and
  covered by two to four small contact-solved rubble pieces.

The RenderPeople pose/scale/axis corrections and the body-cover geometry are
not reimplemented here.  They are the existing ``fire_people`` and
``tornado_people`` contracts used by the urban-fire population pass.
"""

import math
import os
import random


CASUALTY_STATES = ("interior_casualty", "rubble_casualty")
INTERIOR_POSES = (("buried_reach", 0.38), ("lying_supine", 0.24),
                  ("lying_prone", 0.22), ("lying_side_l", 0.08),
                  ("lying_side_r", 0.08))
RUBBLE_POSES = (("buried_reach", 0.34), ("lying_supine_open", 0.16),
                ("lying_prone_reach", 0.18), ("lying_side_l", 0.12),
                ("lying_side_r", 0.12), ("lying_curled_l", 0.08))
OCCLUSION = (("feet_shins", 0.27), ("legs", 0.31),
             ("midriff", 0.24), ("torso", 0.18))
MAX_COVERED_FRAC = 0.55
MIN_PERSON_SPACING_M = 4.0

# One composed-triangle cache per building for the duration of a population
# pass.  Support probing and opening visibility ask questions of the same
# final damaged mesh; rebuilding those arrays for every candidate was both
# slow and made it tempting to skip the visibility gate entirely.
_GEOMETRY_CACHE = {}


def disable_generic_population(config):
    """Disable the ordinary city pedestrian pass in this in-memory config.

    Earthquake casualties are authored after damage assembly, because their
    support and cover must be measured against the final composed geometry.
    Leaving ``usds.humans`` populated would first compose every ordinary
    pedestrian rig only to deactivate it later.  Besides being semantically
    wrong for a casualty-only review, that costs minutes on a dense plate.

    Returns the number of generic human assets removed from the pool.
    """
    usds = config.setdefault("usds", {})
    removed = len(usds.get("humans") or ())
    usds["humans"] = []
    disaster = config.setdefault("disaster", {})
    disaster["humans_strewn"] = [0, 0]
    disaster["humans_prone_fraction"] = 0.0
    return removed


def base_grade(label):
    """Canonical ``DGn``/foundation grade from decorated record labels."""
    label = str(label or "DG0")
    if label.startswith("AEC_"):
        label = label[4:]
    return label.split("+", 1)[0]


def _weighted(rng, table):
    total = sum(float(w) for _v, w in table)
    pick = rng.random() * total
    for value, weight in table:
        pick -= float(weight)
        if pick <= 0.0:
            return value
    return table[-1][0]


def _rotate(x, y, yaw_deg):
    a = math.radians(float(yaw_deg))
    c, s = math.cos(a), math.sin(a)
    return c * x - s * y, s * x + c * y


def side_normal_world(rec, side):
    local = {"S": (0.0, -1.0), "E": (1.0, 0.0),
             "N": (0.0, 1.0), "W": (-1.0, 0.0)}[str(side)]
    return _rotate(local[0], local[1], rec.get("yaw_deg", 0.0))


def face_center(rec, side):
    """World face centre, outward normal, tangent and half run length."""
    w, d = float(rec.get("W", 20.0)), float(rec.get("D", 20.0))
    side = str(side)
    local = {"S": (0.0, -0.5 * d), "N": (0.0, 0.5 * d),
             "E": (0.5 * w, 0.0), "W": (-0.5 * w, 0.0)}[side]
    dx, dy = _rotate(local[0], local[1], rec.get("yaw_deg", 0.0))
    nx, ny = side_normal_world(rec, side)
    return (float(rec["x"]) + dx, float(rec["y"]) + dy,
            nx, ny, -ny, nx, 0.5 * (w if side in ("S", "N") else d))


def point_in_footprint(x, y, rec, margin=0.0):
    """True when a world point is inside a record's oriented footprint."""
    dx, dy = float(x) - float(rec["x"]), float(y) - float(rec["y"])
    lx, ly = _rotate(dx, dy, -float(rec.get("yaw_deg", 0.0)))
    return (abs(lx) <= 0.5 * float(rec.get("W", 20.0)) + float(margin)
            and abs(ly) <= 0.5 * float(rec.get("D", 20.0)) + float(margin))


def damage_sides(rec):
    """Ordered failed faces, strongest/measured reach first."""
    sides = list(rec.get("failure_sides") or rec.get("fall_sides") or ())
    sides = [str(s) for s in sides if str(s) in "SENW"]
    reach = rec.get("reach_m") or rec.get("extent_m") or {}
    return sorted(dict.fromkeys(sides),
                  key=lambda s: (-float(reach.get(s, 0.0)), "SENW".index(s)))


def population_budget(buildings, requested=None):
    """A bounded casualty count derived only from genuinely damaged shells."""
    eligible = {str(r.get("prim")) for r in buildings or ()
                if not r.get("mono") and base_grade(r.get("grade")) in
                ("DG3", "DG4", "DG5") and damage_sides(r)}
    if requested is not None and int(requested) >= 0:
        return min(int(requested), max(0, len(eligible) * 2))
    if not eligible:
        return 0
    # Dense enough for search-and-rescue without recreating the 134-person
    # ordinary pedestrian population on a 500 m plate.
    return min(len(eligible) * 2,
               min(64, max(12, int(round(0.75 * len(eligible))))))


def _inside_bounds(x, y, bounds, margin=1.0):
    if not bounds:
        return True
    x0, y0, x1, y1 = (float(v) for v in bounds)
    return (x0 + margin <= x <= x1 - margin
            and y0 + margin <= y <= y1 - margin)


def _spaced(x, y, made, distance=MIN_PERSON_SPACING_M):
    d2 = float(distance) ** 2
    return all((float(r["x"]) - x) ** 2 + (float(r["y"]) - y) ** 2 >= d2
               for r in made)


def _candidate_rows(buildings, grades):
    out = []
    for rec in buildings or ():
        grade = base_grade(rec.get("grade"))
        label = str(rec.get("grade") or "")
        if (rec.get("mono") or grade not in grades or not damage_sides(rec)
                or "tilt" in label.lower() or "lean" in label.lower()):
            continue
        out.append(rec)
    return out


def _support_z(stage, rec, x, y, ceiling_m, ssf, body_footprint=True):
    """Measured support from the composed building, in metres."""
    from . import quake_collapse

    # A floor must support a meaningful patch under the body.  A rubble
    # heightfield is tessellated much more finely, so its centre sample is
    # intentionally small; the added cover still contacts the body itself.
    if body_footprint:
        half_w, half_d = 0.30, 0.58
    else:
        half_w = half_d = 0.045
    try:
        z = quake_collapse._deck_support_z(
            stage, str(rec.get("prim") or ""), float(x) * ssf,
            float(y) * ssf, half_w * ssf, half_d * ssf,
            float(ceiling_m) * ssf, margin=0.08 * ssf,
            candidates=_building_geometry(stage, rec))
    except Exception:
        return None
    return None if z is None else float(z) / float(ssf)


def _prone_support_points(x, y, pose, yaw_deg, height_m=1.78):
    """World XY samples spanning the actual soles-to-head prone body.

    RenderPeople's placement origin is at the soles, not at the body's
    centre.  A symmetric rectangle around ``(x, y)`` therefore validates the
    wrong footprint and can leave most of a person hanging past a slab edge.
    Use the same pose/roll body axis as the existing burial code and sample
    the length plus both sides of the torso.
    """
    from . import fire_people
    from . import tornado_people

    roll = fire_people.LYING_ROLL[str(pose)]
    ux, uy = tornado_people._body_axis(pose, yaw_deg, roll)
    vx, vy = -uy, ux
    reach = float(height_m)
    points = [(float(x) + ux * reach * f,
               float(y) + uy * reach * f)
              for f in (0.06, 0.28, 0.50, 0.72, 0.94)]
    for f in (0.32, 0.68):
        cx = float(x) + ux * reach * f
        cy = float(y) + uy * reach * f
        for across in (-0.18, 0.18):
            points.append((cx + vx * across, cy + vy * across))
    return points


def _prone_support_z(stage, rec, x, y, ceiling_m, ssf, pose, yaw_deg):
    """Flat surviving-slab support under every sampled part of a body."""
    heights = []
    for px, py in _prone_support_points(x, y, pose, yaw_deg):
        if not point_in_footprint(px, py, rec, margin=-0.15):
            return None
        qz = _support_z(stage, rec, px, py, ceiling_m, ssf,
                        body_footprint=False)
        if qz is None:
            return None
        heights.append(float(qz))
    # A person can bridge centimetre-scale fracture roughness, not a missing
    # floor or the edge between two different storeys.
    if max(heights) - min(heights) > 0.18:
        return None
    return max(heights)


def _prone_body_center_xy(x, y, pose, yaw_deg, height_m=1.78):
    """Approximate torso centre in the same root-at-soles frame."""
    from . import fire_people
    from . import tornado_people

    roll = fire_people.LYING_ROLL[str(pose)]
    ux, uy = tornado_people._body_axis(pose, yaw_deg, roll)
    return (float(x) + 0.50 * float(height_m) * ux,
            float(y) + 0.50 * float(height_m) * uy)


def _building_geometry(stage, rec):
    """Cached world-space triangles below one final damaged building."""
    from . import quake_collapse

    root = str(rec.get("prim") or "")
    key = (id(stage), root)
    if key not in _GEOMETRY_CACHE:
        _GEOMETRY_CACHE[key] = quake_collapse._deck_support_candidates(
            stage, root)
    return _GEOMETRY_CACHE[key]


def _segment_triangle_hits(p0, p1, A, B, C, eps=1e-7):
    """Vectorised Moller-Trumbore test for a finite segment.

    Returns one bool per triangle.  Faces are treated as double-sided: glass
    or a surviving wall blocks a review ray whichever winding the source USD
    used.  This pure helper is covered without Kit by the unit tests.
    """
    import numpy as np

    A, B, C = np.asarray(A), np.asarray(B), np.asarray(C)
    if not len(A):
        return np.zeros(0, dtype=bool)
    origin = np.asarray(p0, dtype=float)
    direction = np.asarray(p1, dtype=float) - origin
    e1, e2 = B - A, C - A
    h = np.cross(np.broadcast_to(direction, e2.shape), e2)
    det = np.einsum("ij,ij->i", e1, h)
    valid = np.abs(det) > eps
    inv = np.zeros_like(det, dtype=float)
    inv[valid] = 1.0 / det[valid]
    s = origin - A
    u = inv * np.einsum("ij,ij->i", s, h)
    q = np.cross(s, e1)
    v = inv * np.einsum("j,ij->i", direction, q)
    t = inv * np.einsum("ij,ij->i", e2, q)
    return (valid & (u >= -eps) & (v >= -eps) & (u + v <= 1.0 + eps)
            & (t > eps) & (t < 1.0 - eps))


def _clear_segment(stage, rec, p0, p1, ssf):
    """True when the final damaged building leaves ``p0 -> p1`` open."""
    import numpy as np

    q0 = np.asarray(p0, dtype=float) * float(ssf)
    q1 = np.asarray(p1, dtype=float) * float(ssf)
    seg_lo, seg_hi = np.minimum(q0, q1), np.maximum(q0, q1)
    for _path, lo, hi, A, B, C, _nz, _tz in _building_geometry(stage, rec):
        lo, hi = np.asarray(lo), np.asarray(hi)
        if np.any(hi < seg_lo - 1e-4) or np.any(lo > seg_hi + 1e-4):
            continue
        if np.any(_segment_triangle_hits(q0, q1, A, B, C)):
            return False
    return True


def _interior_review_geometry(rec, x, y, z, side, setback, tangent_sign=1.0):
    """The two drone camera rays used both for placement and review.

    The first eye is a low drone oblique roughly 11--15 m from the body. The
    second is farther and higher, roughly 17--23 m away, and offset along the
    facade. Both extend a near-facade ray rather than rotating around to an
    intact elevation, so they show the exterior and the same opening a
    searching drone would have to see through.
    """
    nx, ny = side_normal_world(rec, side)
    tx, ty = -ny, nx
    target = (float(x), float(y), float(z) + 0.35)
    d1, d2 = float(setback) + 3.0, float(setback) + 3.8
    near1 = (float(x) + nx * d1, float(y) + ny * d1,
             float(z) + 1.75)
    near2 = (float(x) + nx * d2 + tx * 1.25 * tangent_sign,
             float(y) + ny * d2 + ty * 1.25 * tangent_sign,
             float(z) + 2.45)

    def extend(eye, factor):
        return tuple(target[i] + factor * (eye[i] - target[i])
                     for i in range(3))

    eye1, eye2 = extend(near1, 3.0), extend(near2, 4.0)
    return target, (eye1, eye2)


def _rubble_review_geometry(rec, x, y, z, side, tangent_sign=1.0):
    """Two outside-drone rays to a casualty on the debris apron.

    A point can be outside the nominal building footprint and still sit in a
    dark pocket beneath the collapsed shell.  These rays are deliberately
    farther out and higher than the authored cover pieces.  Placement uses
    them as geometry gates; the review pass later reuses the exact same eyes.
    """
    nx, ny = side_normal_world(rec, side)
    tx, ty = -ny, nx
    target = (float(x), float(y), float(z) + 0.42)
    eye1 = (float(x) + nx * 10.0,
            float(y) + ny * 10.0,
            float(z) + 7.5)
    eye2 = (float(x) + nx * 14.0 + tx * 4.0 * tangent_sign,
            float(y) + ny * 14.0 + ty * 4.0 * tangent_sign,
            float(z) + 11.0)
    return target, (eye1, eye2)


def _clear_of_other_buildings(rec, buildings, target, eye):
    """Cheap city-level occlusion gate for a drone-to-casualty segment."""
    subject = str(rec.get("prim") or "")
    others = [r for r in buildings or ()
              if str(r.get("prim") or "") != subject]
    for i in range(3, 25):
        t = i / 24.0
        x = target[0] + t * (eye[0] - target[0])
        y = target[1] + t * (eye[1] - target[1])
        z = target[2] + t * (eye[2] - target[2])
        for other in others:
            if z <= float(other.get("H", 0.0)) + 0.5 \
                    and point_in_footprint(x, y, other, margin=0.25):
                return False
    return True


def _ground_support_z(stage, rec, x, y, ssf):
    """The actual city ground below a rubble casualty; zero is last resort."""
    from . import quake_collapse

    prim = str(rec.get("prim") or "")
    parent = prim.split("/generated/", 1)[0] if "/generated/" in prim else ""
    root = parent + "/ground" if parent else "/"
    try:
        z = quake_collapse._deck_support_z(
            stage, root, float(x) * ssf, float(y) * ssf,
            0.045 * ssf, 0.045 * ssf, 1.0 * ssf, margin=0.08 * ssf)
    except Exception:
        z = None
    return 0.0 if z is None else float(z) / float(ssf)


def _person_record(rng, rec, state, x, y, z, side, pose_table,
                   yaw_deg=None):
    from . import fire_people

    pose = _weighted(rng, pose_table)
    usd, pose, _rigged = fire_people._pick_human(
        rng, pose, allow_posed=False)
    alive = rng.random() < 0.72
    return {
        "id": None,
        "state": state,
        "class": ("interior_trapped" if state == "interior_casualty"
                  else "rubble_trapped"),
        "cls": ("interior_trapped" if state == "interior_casualty"
                else "rubble_trapped"),
        "status": "injured" if alive else "fatality",
        "alive": alive,
        "usd": usd,
        "rigged": True,
        "x": round(float(x), 3), "y": round(float(y), 3),
        "z": round(float(z), 3),
        "yaw_deg": round(rng.uniform(0.0, 360.0)
                         if yaw_deg is None else float(yaw_deg), 1),
        "pose": pose, "prone": True,
        "side": str(side),
        "nearest_building": rec.get("prim"),
        "nearest_style": rec.get("style"),
        "nearest_grade": rec.get("grade"),
        "building_prim": rec.get("prim"),
        "building_style": rec.get("style"),
        "building_grade": rec.get("grade"),
        "construction_type": rec.get("type") or rec.get("btype") or "rc",
        "active": True,
    }


def _interior_person(stage, rec, buildings, rng, made, bounds, ssf):
    # Benchmark subjects belong just behind the exposed edge, not deep in a
    # dark room. A prone body is aligned mostly ALONG the facade below so its
    # head and feet remain inside even at this shallow setback.
    min_setback = 1.02
    max_setback = min(1.58, max(1.25, 0.14 * min(
        float(rec.get("W", 20.0)), float(rec.get("D", 20.0)))))
    h = float(rec.get("H", 12.0))
    for side in damage_sides(rec):
        cx, cy, nx, ny, tx, ty, half = face_center(rec, side)
        for _attempt in range(10):
            setback = rng.uniform(min_setback, max_setback)
            along = rng.uniform(-0.42, 0.42) * half
            x = cx - nx * setback + tx * along
            y = cy - ny * setback + ty * along
            if (not _inside_bounds(x, y, bounds, margin=0.8)
                    or not point_in_footprint(x, y, rec, margin=-0.45)
                    or not _spaced(x, y, made)):
                continue
            body_yaw = (math.degrees(math.atan2(ty, tx))
                        + rng.choice((0.0, 180.0))
                        + rng.uniform(-9.0, 9.0))
            draft = _person_record(
                rng, rec, "interior_casualty", x, y, 0.0, side,
                INTERIOR_POSES, yaw_deg=body_yaw)
            # Probe several real storey bands; never invent H/storey
            # arithmetic as a floor elevation. The accepted z always comes
            # from triangles beneath the WHOLE soles-to-head footprint.
            ceilings = [0.74 * h, 0.60 * h, 0.46 * h, 0.34 * h]
            rng.shuffle(ceilings)
            for ceiling in ceilings:
                z = _prone_support_z(
                    stage, rec, x, y, ceiling, ssf, draft["pose"],
                    draft["yaw_deg"])
                if z is None or not (1.2 <= z <= h - 1.0):
                    continue
                tangent_sign = 1.0 if along <= 0.0 else -1.0
                body_x, body_y = _prone_body_center_xy(
                    x, y, draft["pose"], draft["yaw_deg"])
                target, eyes = _interior_review_geometry(
                    rec, body_x, body_y, z, side, setback, tangent_sign)
                # A failure-side label alone is not evidence that THIS bay
                # at THIS storey is open.  Old GAC manifests often call every
                # debris-runout side a failure side.  Require both final
                # camera rays to pass through the composed mesh itself.
                if not all(_clear_segment(stage, rec, target, eye, ssf)
                           for eye in eyes):
                    continue
                if not all(_clear_of_other_buildings(
                        rec, buildings, target, eye) for eye in eyes):
                    continue
                out = draft
                out.update(
                    z=round(float(z), 3),
                    z_mode="measured_surviving_slab",
                    setback_m=round(setback, 2), covered_frac=0.0,
                    cover_piece_count=0, occlusion="broken_shell",
                    support_samples=len(_prone_support_points(
                        x, y, out["pose"], out["yaw_deg"])),
                    full_body_support_verified=True,
                    review_geometry_verified=True,
                    review_target=[round(v, 3) for v in target],
                    review_eyes=[[round(v, 3) for v in eye]
                                 for eye in eyes])
                return out
    return None


def _runout(rec, side):
    reach = rec.get("reach_m") or {}
    extent = rec.get("extent_m") or {}
    value = float(reach.get(side, 0.0) or extent.get(side, 0.0) or 0.0)
    if value <= 0.0:
        value = min(6.0, 0.22 * float(rec.get("H", 12.0)))
    return max(1.2, value)


def _rubble_person(stage, rec, buildings, rng, made, bounds, ssf):
    sides = damage_sides(rec)
    for side in sides:
        cx, cy, nx, ny, tx, ty, half = face_center(rec, side)
        run = _runout(rec, side)
        # Use the outer half of the debris apron.  The old 20--62% band put
        # many subjects underneath the collapsed shell even though their XY
        # point was technically outside the intact footprint.
        near = min(7.5, max(1.0, 0.52 * run))
        far = min(8.5, max(near + 0.35, 0.84 * run))
        for _attempt in range(16):
            distance = rng.uniform(near, far)
            along = rng.uniform(-0.44, 0.44) * half
            x = cx + nx * distance + tx * along
            y = cy + ny * distance + ty * along
            if (not _inside_bounds(x, y, bounds, margin=1.3)
                    or any(point_in_footprint(x, y, other, margin=0.25)
                           for other in buildings or ())
                    or not _spaced(x, y, made)):
                continue
            z = _support_z(stage, rec, x, y, min(7.0, 0.38 * float(
                rec.get("H", 12.0))), ssf, body_footprint=False)
            if z is None:
                z = _ground_support_z(stage, rec, x, y, ssf)
            if z < -0.25 or z > 6.0:
                continue
            tangent_sign = 1.0 if along <= 0.0 else -1.0
            target, eyes = _rubble_review_geometry(
                rec, x, y, z, side, tangent_sign)
            # "Outside the footprint" is not enough: reject rubble caves
            # and street-canyon occlusion.  Both a lower and a higher drone
            # must have a direct ray to the un-covered body before its small
            # intentional cover pieces are added.
            if not all(_clear_segment(stage, rec, target, eye, ssf)
                       for eye in eyes):
                continue
            if not all(_clear_of_other_buildings(
                    rec, buildings, target, eye) for eye in eyes):
                continue
            out = _person_record(rng, rec, "rubble_casualty",
                                 x, y, z, side, RUBBLE_POSES)
            out.update(z_mode=("measured_rubble_surface" if z > 0.12 else
                               "measured_ground"),
                       wall_distance_m=round(distance, 2),
                       runout_m=round(run, 2),
                       exposure_zone_frac=round(distance / run, 3),
                       review_geometry_verified=True,
                       review_target=[round(v, 3) for v in target],
                       review_eyes=[[round(v, 3) for v in eye]
                                    for eye in eyes])
            return out
    return None


def _cover_specs(rec, rng):
    """2--4 contact-solved pieces over one rubble casualty."""
    from . import fire_people
    from . import tornado_people

    pattern = _weighted(rng, OCCLUSION)
    span = fire_people._FIRE_OCCLUSION_SPANS[pattern]
    spans = tornado_people._trim_spans((span,), MAX_COVERED_FRAC)
    reach = fire_people.NOMINAL_HEIGHT_M
    roll = fire_people.LYING_ROLL[rec["pose"]]
    ux, uy = tornado_people._body_axis(rec["pose"], rec["yaw_deg"], roll)
    specs = []
    for lo, hi in spans:
        metres = (hi - lo) * reach
        count = max(2, min(4, int(math.ceil(metres / 0.30))))
        step = (hi - lo) / float(count)
        for k in range(count):
            s0, s1 = lo + k * step, lo + (k + 1) * step
            t = 0.5 * (s0 + s1)
            _klass, _along, across, thick = \
                fire_people._draw_fire_cover_stock(rng)
            along = max(0.10, (s1 - s0) * reach * rng.uniform(0.84, 0.96))
            px = float(rec["x"]) + ux * t * reach
            py = float(rec["y"]) + uy * t * reach
            lift = fire_people.lying_lift(rec["pose"])
            crest = min(tornado_people._crest(
                rec["pose"], s0, s1, reach), lift * 2.2)
            spec = tornado_people._cover_piece(
                px, py, ux, uy, along, across, thick, "rubble_slab",
                float(rec["z"]) + crest, float(rec["z"]), rng,
                propped=(rng.random() < 0.25))
            btype = str(rec.get("construction_type") or "rc")
            if btype == "urm":
                klass = "brick_chunk" if rng.random() < 0.68 else "concrete_chunk"
            else:
                klass = "concrete_chunk" if rng.random() < 0.84 else "beam_chunk"
            spec["class"] = klass
            spec["l"], spec["w"] = spec.pop("len"), spec.pop("wide")
            spec["over_record_id"] = rec["id"]
            specs.append(spec)
    covered = min(MAX_COVERED_FRAC,
                  sum(max(0.0, hi - lo) for lo, hi in spans))
    rec["occlusion"] = pattern
    rec["covered_frac"] = round(covered, 3)
    rec["cover_piece_count"] = len(specs)
    return specs


def _hide_generic_people(stage, placements):
    hidden, failures = 0, []
    for p in placements or ():
        if p.get("category") != "human":
            continue
        path = str(p.get("prim_path") or "")
        prim = stage.GetPrimAtPath(path) if path else None
        if prim and prim.IsValid() and (not prim.IsActive() or prim.SetActive(False)):
            hidden += 1
        else:
            failures.append(path or "<missing prim_path>")
    if failures:
        raise RuntimeError("could not deactivate generic city humans: "
                           + ", ".join(failures[:8]))
    return hidden


def _author_cover(stage, specs, parent, ssf):
    if not specs:
        return []
    from . import planks
    from . import quake_flow

    base = quake_flow.materials(stage, parent)
    concrete = quake_flow._c_look_at(stage, parent, base, "concrete",
                                     tag="quake_people")
    brick = quake_flow._c_look_at(stage, parent, base, "brick",
                                  tag="quake_people")
    mats = {"concrete_chunk": concrete, "brick_chunk": brick,
            "beam_chunk": concrete}
    return planks.build(stage, parent + "/quake_people_debris", specs,
                        mats, ssf, verbose=True)


def replace_population(stage, placements, buildings, parent="/World/stage",
                       ssf=1.0, seed=11, total=None, bounds_by_city=None,
                       verbose=True):
    """Remove ordinary pedestrians and author only damage-tied casualties.

    Returns ``(records, report)``.  A failure to remove a generic human, to
    convert a casualty rig, or to satisfy the casualty-only invariant raises:
    those are dataset correctness failures, not optional decoration.
    """
    from . import fire_people
    import scene_generator as sg

    ssf = float(ssf)
    _GEOMETRY_CACHE.clear()
    hidden = _hide_generic_people(stage, placements)
    wanted = population_budget(buildings, requested=total)
    rng = random.Random(int(seed) + 0xE917)
    partial = _candidate_rows(buildings, ("DG3", "DG4"))
    rubble = _candidate_rows(buildings, ("DG4", "DG5"))
    rng.shuffle(partial)
    rng.shuffle(rubble)
    interior_target = int(round(0.46 * wanted))
    rubble_target = wanted - interior_target
    made = []

    def bounds_for(rec):
        return (bounds_by_city or {}).get(rec.get("city"))

    for rec in partial:
        if sum(r["state"] == "interior_casualty" for r in made) >= interior_target:
            break
        got = _interior_person(stage, rec, buildings, rng, made,
                               bounds_for(rec), ssf)
        if got:
            made.append(got)

    used_rubble = set()
    for rec in rubble:
        if sum(r["state"] == "rubble_casualty" for r in made) >= rubble_target:
            break
        got = _rubble_person(stage, rec, buildings, rng, made,
                             bounds_for(rec), ssf)
        if got:
            made.append(got)
            used_rubble.add(str(rec.get("prim")))

    # If a real slab could not be found at enough partial collapses, fill the
    # requested search population at measured rubble/ground supports instead
    # of inventing a floor elevation.  DG3 is allowed only in this fallback.
    fill = _candidate_rows(buildings, ("DG3", "DG4", "DG5"))
    rng.shuffle(fill)
    for rec in fill:
        if len(made) >= wanted:
            break
        if str(rec.get("prim")) in used_rubble:
            continue
        got = _rubble_person(stage, rec, buildings, rng, made,
                             bounds_for(rec), ssf)
        if got:
            made.append(got)
            used_rubble.add(str(rec.get("prim")))

    # Stable IDs are assigned after every support gate, so there are no gaps.
    cover = []
    for i, rec in enumerate(made):
        rec["id"] = "eqp_{0:04d}".format(i)
        if rec["state"] == "rubble_casualty":
            cover.extend(_cover_specs(rec, rng))

    human_placements, skipped = fire_people.to_placements(
        made, tag_ids=True)
    if skipped:
        raise RuntimeError("earthquake casualty conversion skipped records: "
                           + repr(skipped))
    sg.apply_placements(stage, human_placements,
                        parent_path=parent + "/quake_people",
                        scene_scale_factor=ssf, instance_categories=set())
    authored = {p.get("fire_people_id"): p for p in human_placements}
    for rec in made:
        p = authored.get(rec["id"])
        rec["prim"] = str((p or {}).get("prim_path") or "")
        rec["asset"] = os.path.basename(str(rec.get("usd") or ""))
        rec["roll_deg"] = round(float((p or {}).get("roll_deg", 0.0)), 2)
        rec["pitch_deg"] = round(float((p or {}).get("pitch_deg", 0.0)), 2)
        rec["render_z"] = round(float((p or {}).get("z_m", rec["z"])), 3)
        prim = stage.GetPrimAtPath(rec["prim"]) if rec["prim"] else None
        rec["active"] = bool(prim and prim.IsValid() and prim.IsActive())
    cover_paths = _author_cover(stage, cover, parent, ssf)

    healthy = [r["id"] for r in made
               if r.get("state") not in CASUALTY_STATES or not r.get("prone")]
    inactive = [r["id"] for r in made if not r.get("active")]
    if healthy or inactive:
        raise RuntimeError("casualty-only population gate failed: healthy={0}, "
                           "inactive={1}".format(healthy, inactive))
    report = {
        "generic_humans_deactivated": hidden,
        "requested": wanted,
        "authored": len(made),
        "interior_casualties": sum(r["state"] == "interior_casualty"
                                   for r in made),
        "rubble_casualties": sum(r["state"] == "rubble_casualty"
                                 for r in made),
        "standing_or_walking": 0,
        "cover_pieces": len(cover),
        "cover_meshes": len(cover_paths),
        "underfilled": max(0, wanted - len(made)),
    }
    if verbose:
        print("[quake_people] generic hidden {0}; casualties {1}/{2}: "
              "interior {3}, rubble {4}; cover {5} pieces / {6} meshes"
              .format(hidden, len(made), wanted,
                      report["interior_casualties"],
                      report["rubble_casualties"], len(cover),
                      len(cover_paths)), flush=True)
        if report["underfilled"]:
            print("[quake_people] NOTE: {0} requested slot(s) refused because "
                  "no real support/clear footprint was found".format(
                      report["underfilled"]), flush=True)
    return made, report

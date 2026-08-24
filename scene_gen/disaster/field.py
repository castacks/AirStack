"""field — where the disaster actually hit.

WHAT A FIELD IS, AND WHY IT IS THE ONLY THING SEVERITY TOUCHES
--------------------------------------------------------------
Every knob under ``disaster.*`` is a *maximum*, reached only where the field
reads 1.0. ``disaster.field`` shapes that intensity over the region, and it is
what separates one disaster type from another: an earthquake attenuates
radially from its epicentre, a tornado carves a narrow corridor and leaves the
rest of the city alone, a fire runs downwind as an ellipse.

The division of labour, which the whole staged pipeline rests on:

    disaster TYPE     the SHAPE of the field        (radial / path / ellipse)
    severity          the EXTENT and INTENSITY      (radius, falloff, duration)
    the field         evaluated at one position     -> local damage
    the LADDER        quantises that to a level     (see `levels.py`)

**Severity shapes the field. It never shapes the ladder.** That is what lets
Stage A bake one archetype library and reuse it at every severity: the library
is indexed by (type, level), and severity only decides which level each asset
lands on. `tests/test_severity_shapes_only_the_field.py` holds the line.

FIELD KINDS
-----------
``uniform``  ``inside`` everywhere. Hurricanes — no edge to speak of.
``radial``   ``inside`` within ``radius_m`` of ``center``, easing to
             ``outside`` over ``falloff_m``. Earthquakes, conflagrations.
``path``     ``inside`` within ``width_m`` of a polyline, easing over
             ``falloff_m``. Tornado tracks.
``ellipse``  A wind-driven front spreading from ``origin_m`` at ``head_mps``
             downwind, ``flank_mps`` across and ``back_mps`` upwind. This is
             the wildfire front, which used to live only inside
             `disaster/fire.py` as arrival-time arithmetic rather than as a
             field — so fire was the one disaster whose "where did it hit"
             could not be asked the same way as the others'.

Moved here out of `scene_generator.py`, which is the USD writer and had no
business owning the damage model.
"""

import math


def smoothstep(t: float) -> float:
    """Hermite ease on [0, 1] — softer edges than a linear ramp."""
    t = min(1.0, max(0.0, t))
    return t * t * (3.0 - 2.0 * t)


def point_segment_dist(px, py, ax, ay, bx, by) -> float:
    vx, vy = bx - ax, by - ay
    seg2 = vx * vx + vy * vy
    if seg2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / seg2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def path_segments(cfg: dict, region: tuple) -> list:
    """Segments for a ``kind: path`` track, building the default straight
    sweep across *region* on ``heading_deg`` when no ``points`` are given.
    Shared by :func:`make_damage_field` and :func:`make_scour_density` so both
    agree on where the track actually is.
    """
    pts = cfg.get("points")
    if not pts:
        rx0, ry0, rx1, ry1 = region
        mx, my = (rx0 + rx1) / 2.0, (ry0 + ry1) / 2.0
        ang = math.radians(float(cfg.get("heading_deg", 45.0)))
        reach = math.hypot(rx1 - rx0, ry1 - ry0)
        pts = [[mx - math.cos(ang) * reach, my - math.sin(ang) * reach],
               [mx + math.cos(ang) * reach, my + math.sin(ang) * reach]]
    return [(float(pts[i][0]), float(pts[i][1]),
             float(pts[i + 1][0]), float(pts[i + 1][1]))
            for i in range(len(pts) - 1)] or [
        (float(pts[0][0]), float(pts[0][1]),
         float(pts[0][0]), float(pts[0][1]))]


def arrival_time(u, v, head, flank, back) -> float:
    """When an elliptical front reaches wind-frame point (u, v). Seconds.

    At time t the burnt region is an ellipse with semi-axis ``a = A*t`` along
    the wind, ``b = B*t`` across it, centred ``c = C*t`` downwind of the
    ignition point, where ``A = (head+back)/2``, ``C = (head-back)/2``,
    ``B = flank``.

    Every axis is linear in t, so substituting ``s = 1/t`` turns "smallest t
    whose ellipse contains (u, v)" into one quadratic in s::

        ((u*s - C)/A)**2 + ((v*s)/B)**2 = 1

    The largest positive root is the earliest arrival. Returns ``inf`` for
    points the front never reaches.

    This is `disaster.fire`'s spread solver, lifted so the wildfire front can
    be expressed as a field like every other disaster. `fire.py` keeps driving
    the Flow emitters off the same arithmetic.
    """
    A = 0.5 * (head + back)
    C = 0.5 * (head - back)
    B = flank
    if A <= 0.0 or B <= 0.0:
        return float("inf")
    if abs(u) < 1e-9 and abs(v) < 1e-9:
        return 0.0

    qa = (u * u) / (A * A) + (v * v) / (B * B)
    qb = -2.0 * u * C / (A * A)
    qc = (C * C) / (A * A) - 1.0

    disc = qb * qb - 4.0 * qa * qc
    if qa <= 0.0 or disc < 0.0:
        return float("inf")
    s = (-qb + math.sqrt(disc)) / (2.0 * qa)
    return 1.0 / s if s > 0.0 else float("inf")


def make_damage_field(field_cfg: dict, region: tuple):
    """Build ``f(x, y) -> intensity`` (0..1) from a ``disaster.field`` spec.

    *region* is ``(x0, y0, x1, y1)`` in metres.

    The returned callable carries ``.lo`` / ``.hi`` — the intensity bounds over
    the whole region. Callers use them to decide what is *possible* anywhere
    (e.g. whether any building can stay intact) before sampling.
    """
    cfg = field_cfg or {}
    kind = str(cfg.get("kind", "uniform")).lower()
    inside = float(cfg.get("inside", 1.0))
    outside = float(cfg.get("outside", 0.0))

    def _tag(fn, lo, hi):
        fn.lo, fn.hi = lo, hi
        return fn

    if kind == "uniform":
        return _tag(lambda x, y: inside, inside, inside)

    def _ease(dist, full, fall):
        """inside within *full*, easing to outside across *fall* beyond it."""
        if dist <= full:
            return inside
        return inside + (outside - inside) * smoothstep(
            (dist - full) / max(fall, 1e-6))

    if kind == "radial":
        cx, cy = cfg.get("center", [0.0, 0.0])
        cx, cy = float(cx), float(cy)
        radius = float(cfg.get("radius_m", 80.0))
        falloff = float(cfg.get("falloff_m", 120.0))
        return _tag(lambda x, y: _ease(math.hypot(x - cx, y - cy),
                                       radius, falloff),
                    min(inside, outside), max(inside, outside))

    if kind == "path":
        half_w = float(cfg.get("width_m", 60.0)) / 2.0
        falloff = float(cfg.get("falloff_m", 40.0))
        segs = path_segments(cfg, region)

        def f(x, y):
            d = min(point_segment_dist(x, y, *s) for s in segs)
            return _ease(d, half_w, falloff)

        return _tag(f, min(inside, outside), max(inside, outside))

    if kind == "ellipse":
        ox, oy = cfg.get("origin_m", [0.0, 0.0])
        ox, oy = float(ox), float(oy)
        th = math.radians(float(cfg.get("heading_deg", 45.0)))
        cos_t, sin_t = math.cos(th), math.sin(th)
        head = float(cfg.get("head_mps", 1.0))
        flank = float(cfg.get("flank_mps", head / 4.0))
        back = float(cfg.get("back_mps", head / 12.0))
        duration = float(cfg.get("duration_s", 600.0))

        # Intensity is how LONG the front has been on this spot, normalised by
        # the whole burn — 1.0 where it arrived first, 0 where it never got to.
        # That is the same ordering `damage.level_for_age` reads, expressed as
        # a field so the quantiser does not have to know about fire.
        def f(x, y):
            dx, dy = x - ox, y - oy
            u = dx * cos_t + dy * sin_t
            v = -dx * sin_t + dy * cos_t
            t = arrival_time(u, v, head, flank, back)
            if not math.isfinite(t) or t > duration:
                return outside
            burnt = (duration - t) / max(duration, 1e-6)
            return outside + (inside - outside) * burnt

        return _tag(f, min(inside, outside), max(inside, outside))

    raise ValueError(f"Unknown disaster.field.kind {kind!r} "
                     "(expected uniform, radial, path or ellipse)")


def make_scour_density(field_cfg: dict, region: tuple, shape: float = 1.6):
    """Build ``g(x, y) -> [0, 1]``: a density *gradient* peaked at the
    disaster's core (track centreline / epicentre), falling smoothly to 0 at
    the same distance where :func:`make_damage_field`'s own ease finishes.

    Distinct from ``make_damage_field``'s intensity, which is a *plateau* —
    flat ``inside`` through the whole width or radius, easing only at the very
    edge. That shape is right for deciding whether a building in the corridor
    survives (a tornado hits everything in its path about equally hard), but
    wrong for ground scour: in real tornado aftermath the debris is visibly
    densest right under the vortex track and thins out well before the edge of
    the damage zone, not uniform across the whole width.

    Reuses the field's own geometry so the *extent* debris can appear in is
    exactly what ``disaster.field`` already configures — only the density
    gradient inside that extent changes. ``shape`` > 1 concentrates density
    nearer the core than a plain smoothstep falloff would; 1.0 is the
    smoothstep.
    """
    cfg = field_cfg or {}
    kind = str(cfg.get("kind", "uniform")).lower()

    def _peak(d, reach):
        t = min(1.0, max(0.0, d / max(reach, 1e-6)))
        return (1.0 - smoothstep(t)) ** shape

    if kind == "path":
        half_w = float(cfg.get("width_m", 60.0)) / 2.0
        falloff = float(cfg.get("falloff_m", 40.0))
        reach = half_w + falloff
        segs = path_segments(cfg, region)
        return lambda x, y: _peak(min(point_segment_dist(x, y, *s)
                                      for s in segs), reach)

    if kind == "radial":
        cx, cy = cfg.get("center", [0.0, 0.0])
        cx, cy = float(cx), float(cy)
        radius = float(cfg.get("radius_m", 80.0))
        falloff = float(cfg.get("falloff_m", 120.0))
        reach = radius + falloff
        return lambda x, y: _peak(math.hypot(x - cx, y - cy), reach)

    if kind == "ellipse":
        # Debris concentrates where the front burned longest, which is the
        # same quantity the damage field returns.
        f = make_damage_field(cfg, region)
        return lambda x, y: max(0.0, min(1.0, f(x, y))) ** shape

    return lambda x, y: 1.0   # uniform field: no core to concentrate around

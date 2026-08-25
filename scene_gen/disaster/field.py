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

ONE CLASS PER SHAPE
-------------------
:class:`DamageField` is the general form and every kind is a subclass of it,
registered in :data:`FIELDS` and built by :func:`make_damage_field`. An
instance is CALLABLE — ``f(x, y) -> intensity`` — and carries ``.lo`` / ``.hi``,
so it is a drop-in for the closure this used to return.

    ``uniform``  :class:`UniformField`  ``inside`` everywhere. Hurricanes — no
                 edge to speak of.
    ``radial``   :class:`RadialField`   ``inside`` within ``radius_m`` of
                 ``center``, easing to ``outside`` over ``falloff_m``.
                 Earthquakes, conflagrations.
    ``path``     :class:`PathField`     ``inside`` within ``width_m`` of a
                 polyline, easing over ``falloff_m``. Tornado tracks.
    ``ellipse``  :class:`EllipseField`  A wind-driven front spreading from
                 ``origin_m`` at ``head_mps`` downwind, ``flank_mps`` across
                 and ``back_mps`` upwind. This is the wildfire front, which
                 used to live only inside `disaster/fire.py` as arrival-time
                 arithmetic rather than as a field — so fire was the one
                 disaster whose "where did it hit" could not be asked the same
                 way as the others'.

Radial and path share :class:`_EasedField`, because they are the same plateau
with a soft rim and differ only in what "distance from the core" means. That
sharing is the point: the geometry used to be written twice, once for
intensity and once for scour density, and the two could drift.

WHICH DISASTER GETS WHICH
-------------------------
The config names the kind (`compile_disaster.py` writes it), so a preset can
ask for any shape. `disaster.kinds` records the default each disaster type
would pick if a hand-written config omits it.

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

    Shared by :class:`EllipseField` and by `disaster.fire`, which drives the
    Flow emitters off it — so "when did the front reach here" and "how hard
    was this spot hit" are answered by the same arithmetic.
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


# ---------------------------------------------------------------------------
# The general form
# ---------------------------------------------------------------------------

class DamageField:
    """``f(x, y) -> intensity`` over a region, plus the debris density on it.

    Subclass responsibilities, and only these:

      ``kind``        the ``disaster.field.kind`` string that selects it.
      ``_configure``  read the kind's own keys out of ``self.cfg``.
      ``intensity``   the damage at a position.
      ``density``     how thickly ground scour lies there — see
                      :meth:`density` for why it is not the same curve.

    ``lo`` / ``hi`` are the intensity bounds over the whole region. Callers use
    them to decide what is *possible* anywhere (e.g. whether any building can
    stay intact) before sampling.
    """

    kind = "uniform"

    def __init__(self, cfg: dict = None, region: tuple = (0.0, 0.0, 0.0, 0.0)):
        self.cfg = dict(cfg or {})
        self.region = tuple(float(v) for v in region)
        self.inside = float(self.cfg.get("inside", 1.0))
        self.outside = float(self.cfg.get("outside", 0.0))
        self._configure()

    def _configure(self) -> None:
        """Read this kind's keys. Nothing to read for a uniform field."""

    # -- intensity ---------------------------------------------------------
    def intensity(self, x: float, y: float) -> float:
        return self.inside

    def __call__(self, x: float, y: float) -> float:
        return self.intensity(x, y)

    @property
    def lo(self) -> float:
        return min(self.inside, self.outside)

    @property
    def hi(self) -> float:
        return max(self.inside, self.outside)

    # -- scour density -----------------------------------------------------
    def density(self, x: float, y: float, shape: float = 1.6) -> float:
        """Debris density at a position, in [0, 1].

        Distinct from :meth:`intensity`, which is a *plateau* — flat ``inside``
        through the whole width or radius, easing only at the very edge. That
        shape is right for deciding whether a building in the corridor survives
        (a tornado hits everything in its path about equally hard), but wrong
        for ground scour: in real tornado aftermath the debris is visibly
        densest right under the vortex track and thins out well before the edge
        of the damage zone, not uniform across the whole width.

        The *extent* is the field's own geometry, so debris can only appear
        where ``disaster.field`` already reaches; only the gradient inside that
        extent differs. ``shape`` > 1 concentrates density nearer the core than
        a plain smoothstep falloff would; 1.0 is the smoothstep.

        A uniform field has no core to concentrate around, hence 1.0.
        """
        return 1.0

    def scour_density(self, shape: float = 1.6):
        """:meth:`density` as a plain ``g(x, y)`` callable."""
        return lambda x, y: self.density(x, y, shape)

    @staticmethod
    def _peak(d: float, reach: float, shape: float) -> float:
        t = min(1.0, max(0.0, d / max(reach, 1e-6)))
        return (1.0 - smoothstep(t)) ** shape


class UniformField(DamageField):
    """``inside`` everywhere. A hurricane has no edge to speak of."""

    kind = "uniform"

    @property
    def lo(self) -> float:
        return self.inside

    @property
    def hi(self) -> float:
        return self.inside


class _EasedField(DamageField):
    """A plateau with a soft rim: ``inside`` out to ``full``, easing to
    ``outside`` across ``falloff``.

    Radial and path are this same curve over two different distances, which is
    the only thing a subclass has to supply.
    """

    full = 0.0
    falloff = 0.0

    def distance(self, x: float, y: float) -> float:
        """Metres from the core — the epicentre, or the track centreline."""
        raise NotImplementedError

    def intensity(self, x, y):
        d = self.distance(x, y)
        if d <= self.full:
            return self.inside
        return self.inside + (self.outside - self.inside) * smoothstep(
            (d - self.full) / max(self.falloff, 1e-6))

    def density(self, x, y, shape=1.6):
        # Reach is where the intensity ease finishes, so scour stops exactly
        # where the damage does.
        return self._peak(self.distance(x, y), self.full + self.falloff, shape)


class RadialField(_EasedField):
    """Attenuating from a point. Earthquakes, explosions, conflagrations."""

    kind = "radial"

    def _configure(self):
        cx, cy = self.cfg.get("center", [0.0, 0.0])
        self.center = (float(cx), float(cy))
        self.full = float(self.cfg.get("radius_m", 80.0))
        self.falloff = float(self.cfg.get("falloff_m", 120.0))

    def distance(self, x, y):
        return math.hypot(x - self.center[0], y - self.center[1])


class PathField(_EasedField):
    """A corridor along a polyline. Tornado tracks."""

    kind = "path"

    def _configure(self):
        self.full = float(self.cfg.get("width_m", 60.0)) / 2.0
        self.falloff = float(self.cfg.get("falloff_m", 40.0))
        self.segments = path_segments(self.cfg, self.region)

    def distance(self, x, y):
        return min(point_segment_dist(x, y, *s) for s in self.segments)


class EllipseField(DamageField):
    """A wind-driven front spreading from an ignition point. Wildfire.

    Intensity is how LONG the front has been on this spot, normalised by the
    whole burn — 1.0 where it arrived first, 0 where it never got to. That is
    the same ordering `damage.level_for_age` reads, expressed as a field so the
    quantiser does not have to know about fire.
    """

    kind = "ellipse"

    def _configure(self):
        ox, oy = self.cfg.get("origin_m", [0.0, 0.0])
        self.origin = (float(ox), float(oy))
        self.heading_deg = float(self.cfg.get("heading_deg", 45.0))
        th = math.radians(self.heading_deg)
        self._cos, self._sin = math.cos(th), math.sin(th)
        self.head = float(self.cfg.get("head_mps", 1.0))
        self.flank = float(self.cfg.get("flank_mps", self.head / 4.0))
        self.back = float(self.cfg.get("back_mps", self.head / 12.0))
        self.duration = float(self.cfg.get("duration_s", 600.0))

    def wind_frame(self, x, y):
        """(u, v) — downwind and across-wind metres from the ignition point."""
        dx, dy = x - self.origin[0], y - self.origin[1]
        return (dx * self._cos + dy * self._sin,
                -dx * self._sin + dy * self._cos)

    def arrival(self, x, y) -> float:
        """Seconds until the front reaches ``(x, y)``; ``inf`` if never.

        `disaster.fire` plans its ignition schedule off this, so the emitters
        and the damage levels cannot disagree about where the fire went.
        """
        u, v = self.wind_frame(x, y)
        return arrival_time(u, v, self.head, self.flank, self.back)

    def intensity(self, x, y):
        t = self.arrival(x, y)
        if not math.isfinite(t) or t > self.duration:
            return self.outside
        burnt = (self.duration - t) / max(self.duration, 1e-6)
        return self.outside + (self.inside - self.outside) * burnt

    def density(self, x, y, shape=1.6):
        # Debris concentrates where the front burned longest, which is the
        # same quantity the intensity returns.
        return max(0.0, min(1.0, self.intensity(x, y))) ** shape


#: kind string -> class. `disaster.kinds` maps disaster types onto these.
FIELDS = {c.kind: c for c in (UniformField, RadialField, PathField,
                              EllipseField)}


def make_damage_field(field_cfg: dict, region: tuple) -> DamageField:
    """Build the :class:`DamageField` a ``disaster.field`` spec asks for.

    *region* is ``(x0, y0, x1, y1)`` in metres. The result is callable, so
    every caller that treated this as a factory for ``f(x, y)`` is unaffected.
    """
    cfg = field_cfg or {}
    kind = str(cfg.get("kind", "uniform")).lower()
    cls = FIELDS.get(kind)
    if cls is None:
        raise ValueError(f"Unknown disaster.field.kind {kind!r} "
                         f"(expected {', '.join(sorted(FIELDS))})")
    return cls(cfg, region)


def make_scour_density(field_cfg: dict, region: tuple, shape: float = 1.6):
    """``g(x, y) -> [0, 1]``, the debris density on this field.

    Thin wrapper on :meth:`DamageField.density`, kept because it reads better
    at the call site than reaching through a field object for one closure.
    """
    return make_damage_field(field_cfg, region).scour_density(shape)

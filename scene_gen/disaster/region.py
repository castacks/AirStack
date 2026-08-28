"""region stage — the disaster-affected area, as a polygon the planner can read.

WHAT THIS IS
------------
The burnt region of `disaster/fire.py` written out as an ORDINARY POLYGON in
the placements frame, so a search planner can size its search area to the part
of the plat the fire actually touched instead of sweeping the whole 1 km plat.
Everything here is plain arithmetic — no `pxr`, no numpy, no stage — because it
runs host-side in the launcher, in the tests, and anywhere else the answer key
has to be checked without Isaac.

The shape is not an approximation of the fire model, it IS the fire model.
`fire._ignition_time` says the burnt region at time t is an ellipse in the WIND
FRAME (u along `heading_deg`, v across it): semi-axis a = A*t along the wind,
b = B*t across, centred c = C*t downwind of the ignition point, with
A = (head+back)/2, C = (head-back)/2, B = flank. `front_polygon` walks that
ellipse's parametric form and rotates it back into world xy. The only error is
the n-gon's chord sagitta, which at n=72 is under 0.1% of the semi-axis.

TWO TIMES, NOT ONE
------------------
`scene_api.build_scene` defines the damage field as

    age(x, y) = elapsed - arrival(x, y)

(scene_api.py ~line 470-500), so the ground the FIRE REACHED is exactly the
ellipse at T = elapsed. That is `burn`, and it is NOT the polygon a searcher
should be given.

The answer key is the survivors, and `disaster/people.py` does not put them in
the black. `at_home` (~line 2408) stages figures in houses with

    -0.20 * span < age < 0.12 * span

— i.e. up to 0.20*span SECONDS AHEAD of the front, in front yards where the
stay-or-go decision is still live. So ground truth lies outside the burn
ellipse by construction, and a search area clipped to `burn` misses people.

A FIXED LEAD IS NOT ENOUGH, AND THAT WAS MEASURED, NOT ASSUMED. `at_home` is
only one scenario. The gridlock queue runs OUTBOUND along a street — away from
the fire, which is the whole idea — so it sits well past the front, and on the
shipped 1 km scene the 54 survivors' arrival times run from ~110 s to ~1220 s
against an `elapsed` of 450 s. That is a lead of nearly 1.8*span at the far
end, where 0.20*span would have covered one of them. Worse, most of the misses
are CROSS-WIND cases: the ellipse's flank half-axis is only `flank * T`, so
buying flank width with time is the most expensive way there is to buy it.

`affected` is therefore the ellipse at

    T = elapsed + max(lead_frac * span, people_lead_s + margin_s)

where `people_lead_s` is how far ahead of the front the FURTHEST-AHEAD survivor
actually is (`max(-burn_age_s)` over the people records) and `margin_s` keeps
that survivor off the boundary. `lead_frac` is the floor: it is what the
polygon falls back to when there are no records to read, and it keeps a scene
whose people all sit inside the black from reporting a search area that hugs
the front. The result carries `lead_bound` saying which of the two won, because
"people" means the polygon is sized by the answer key and "lead_frac" means it
is sized by the model — and those are two different things to trust.

Both polygons are returned: the burn is what the imagery shows, the affected
area is what the planner must cover.

WHAT THIS IS NOT
----------------
It is not the visible scar. `ground.feathered_coverage` fingers the overlay up
to `finger_m` (= 0.8 * 0.10 * span_m, ~80 m on a 1 km plat) PAST the front, so
the burnt ground on screen reaches further than either polygon here. That
overshoot is what the planner's `search_area_pad_m` exists for; this file
reports the model's own geometry and leaves the padding to the planner.

USAGE
-----
    from disaster import region
    reg = region.affected_polygons(
        fcfg, elapsed, span, (x0, y0, x1, y1),
        people_lead_s=region.people_lead_s(records))
    reg["affected"]        # [[x, y], ...] ccw, clipped to the plat
    region.point_in_polygon(px, py, reg["affected"])
"""

import math

# The floor on how far ahead of the front the affected area reaches, as a
# fraction of `span`. It matches the `-0.20 * span < age` window
# `people.at_home` stages front-yard survivors in — that is the ONE scenario
# whose lead is known in advance, and everything else is measured off the
# records instead (see the module docstring).
DEFAULT_LEAD_FRAC = 0.20

# How far past the furthest-ahead survivor the front is run, seconds. A
# survivor exactly on the boundary is a coin flip for anything that tests
# containment downstream, and 20 s of front is a few metres of ground.
DEFAULT_MARGIN_S = 20.0


# ---------------------------------------------------------------------------
# the front
# ---------------------------------------------------------------------------

def front_polygon(origin_m, heading_deg, head_mps, flank_mps, back_mps, t_s,
                  n=72):
    """The burnt region at `t_s` seconds, as `[[x, y], ...]` metres, ccw.

    The ellipse of `fire._ignition_time` evaluated rather than inverted: at
    time t the front is

        u = C*t + A*t*cos(phi),   v = B*t*sin(phi)

    in the wind frame, with A = (head+back)/2, C = (head-back)/2, B = flank.
    The wind frame's u axis is (cos th, sin th) and its v axis is
    (-sin th, cos th) for th = `heading_deg`, which is the SAME rotation
    `fire.plan_ignition` uses to go the other way — keep the two in step or the
    polygon and the emitters describe different fires.

    Counter-clockwise, with NO repeated closing point: the first vertex is not
    duplicated at the end, because every consumer here treats the ring as
    implicitly closed.

    Returns `[]` for a non-positive time or a degenerate ellipse (a fire with
    no head and no flank never reaches anything, and neither does one at t=0).
    """
    t = float(t_s)
    a_rate = 0.5 * (float(head_mps) + float(back_mps))
    c_rate = 0.5 * (float(head_mps) - float(back_mps))
    b_rate = float(flank_mps)
    if t <= 0.0 or a_rate <= 0.0 or b_rate <= 0.0:
        return []

    a, b, c = a_rate * t, b_rate * t, c_rate * t
    ox, oy = float(origin_m[0]), float(origin_m[1])
    th = math.radians(float(heading_deg))
    ct, st = math.cos(th), math.sin(th)

    n = max(3, int(n))
    out = []
    for k in range(n):
        phi = 2.0 * math.pi * k / float(n)
        u = c + a * math.cos(phi)
        v = b * math.sin(phi)
        out.append([ox + u * ct - v * st, oy + u * st + v * ct])
    return out


# ---------------------------------------------------------------------------
# polygon arithmetic
# ---------------------------------------------------------------------------

def clip_rect(poly_xy, rect):
    """Sutherland-Hodgman clip of `poly_xy` to the axis-aligned `rect`.

    `rect` is `(x0, y0, x1, y1)` in either order per axis — the bounds are
    sorted here, because `binfo["region"]` is a plat extent and nobody should
    have to remember which corner it names first.

    The front is a convex ellipse and the rect is convex, so the clip is exact
    and stays a single ring; the general algorithm is used anyway so a caller
    can pass any convex ring. Returns `[]` when nothing survives — a fire whose
    whole ellipse is off the plat.
    """
    if not poly_xy:
        return []
    x0, x1 = sorted((float(rect[0]), float(rect[2])))
    y0, y1 = sorted((float(rect[1]), float(rect[3])))

    # (inside test, intersection parameter) per edge of the rect, in ccw order.
    edges = (
        (lambda p: p[0] >= x0, 0, x0),
        (lambda p: p[0] <= x1, 0, x1),
        (lambda p: p[1] >= y0, 1, y0),
        (lambda p: p[1] <= y1, 1, y1),
    )

    ring = [[float(p[0]), float(p[1])] for p in poly_xy]
    for inside, axis, bound in edges:
        if not ring:
            return []
        out = []
        prev = ring[-1]
        prev_in = inside(prev)
        for cur in ring:
            cur_in = inside(cur)
            if cur_in != prev_in:
                d = cur[axis] - prev[axis]
                # Parallel to the clip line AND straddling it is impossible;
                # the guard is only against a zero-length edge.
                s = 0.0 if abs(d) < 1e-15 else (bound - prev[axis]) / d
                out.append([prev[0] + (cur[0] - prev[0]) * s,
                            prev[1] + (cur[1] - prev[1]) * s])
            if cur_in:
                out.append(list(cur))
            prev, prev_in = cur, cur_in
        ring = out
    return ring


def polygon_area(poly_xy):
    """Absolute shoelace area, m2. 0 for anything under three vertices."""
    if not poly_xy or len(poly_xy) < 3:
        return 0.0
    s = 0.0
    n = len(poly_xy)
    for i in range(n):
        ax, ay = poly_xy[i][0], poly_xy[i][1]
        bx, by = poly_xy[(i + 1) % n][0], poly_xy[(i + 1) % n][1]
        s += ax * by - bx * ay
    return abs(s) * 0.5


def polygon_bbox(poly_xy):
    """`(x0, y0, x1, y1)` of a ring, or None when it is empty."""
    if not poly_xy:
        return None
    xs = [float(p[0]) for p in poly_xy]
    ys = [float(p[1]) for p in poly_xy]
    return (min(xs), min(ys), max(xs), max(ys))


def point_in_polygon(x, y, poly_xy):
    """Ray-parity point-in-ring. The ring is implicitly closed.

    Crossings are counted on the half-open span `ay <= y < by` (either
    direction), which is the standard way of counting a vertex exactly on the
    ray once rather than twice or not at all. A point exactly ON the boundary
    is undefined either way — everything here is measured in metres against a
    72-gon, so a metre of tolerance matters and a float does not.
    """
    if not poly_xy or len(poly_xy) < 3:
        return False
    x, y = float(x), float(y)
    inside = False
    n = len(poly_xy)
    for i in range(n):
        ax, ay = float(poly_xy[i][0]), float(poly_xy[i][1])
        bx, by = float(poly_xy[(i + 1) % n][0]), float(poly_xy[(i + 1) % n][1])
        if (ay > y) != (by > y):
            xc = ax + (y - ay) * (bx - ax) / (by - ay)
            if x < xc:
                inside = not inside
    return inside


# ---------------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------------

def people_lead_s(records):
    """How far AHEAD of the front the furthest-ahead survivor is, seconds.

    `people.py` stamps every record with `burn_age_s` = `age(x, y)`, so a
    NEGATIVE age is a person the front has not reached yet and `-burn_age_s` is
    their lead. The maximum over the records — floored at 0, because a scene
    whose survivors are all inside the black needs no lead at all — is what
    `affected_polygons` sizes the affected area from.

    One sentinel to know about: `age` returns -1.0 for ground the front NEVER
    reaches, which is indistinguishable here from a person one second ahead of
    it. Nothing in the shipped configs produces it — `compile_wildfire` always
    gives a positive `back_mps`, so the ellipse eventually covers the plane and
    every arrival is finite — but a hand-written `back_mps: 0` would make this
    silently under-report. Records with no `burn_age_s` are skipped.
    """
    lead = 0.0
    for r in records or ():
        try:
            lead = max(lead, -float(r["burn_age_s"]))
        except (KeyError, TypeError, ValueError):
            continue
    return lead


def affected_polygons(fire_cfg, elapsed_s, span_s, region,
                      lead_frac=DEFAULT_LEAD_FRAC, n=72, people_lead_s=0.0,
                      margin_s=DEFAULT_MARGIN_S):
    """The burn and the affected area, clipped to the plat.

    `fire_cfg` is the merged fire block — `fire.DEFAULTS` updated with what
    `compile_disaster.compile_wildfire` emits — so it carries `origin_m`,
    `heading_deg`, `head_mps`, `flank_mps` and `back_mps`. `elapsed_s` and
    `span_s` are `build_scene`'s own, and `region` is the plat extent
    `(x0, y0, x1, y1)`.

    `people_lead_s` is `max(-burn_age_s)` over the survivor records — use the
    `people_lead_s()` helper above — i.e. how far ahead of the front the
    furthest-ahead person actually is. The affected front is run to

        T = elapsed + max(lead_frac * span, people_lead_s + margin_s)

    so the answer key sets the size whenever the answer key is the binding
    constraint, and `lead_frac` is the floor for when there is nobody to read.

    Returns

        {"burn": [[x, y], ...],        # the ellipse at T = elapsed
         "affected": [[x, y], ...],    # ...at T = elapsed + lead_s
         "t_burn_s": T, "t_affected_s": T,
         "lead_s": s, "lead_bound": "people" | "lead_frac",
         "people_lead_s": s, "lead_frac": f, "margin_s": s}

    `lead_bound` is worth carrying downstream: "people" says the polygon was
    sized by the survivors it has to contain, "lead_frac" says it was sized by
    the model alone.
    """
    origin = fire_cfg.get("origin_m", [0.0, 0.0])
    heading = float(fire_cfg.get("heading_deg", 0.0))
    head = float(fire_cfg.get("head_mps", 0.0))
    flank = float(fire_cfg.get("flank_mps", 0.0))
    back = float(fire_cfg.get("back_mps", 0.0))

    frac_lead = float(lead_frac) * float(span_s)
    ppl_lead = float(people_lead_s) + float(margin_s)
    lead = max(frac_lead, ppl_lead)

    t_burn = float(elapsed_s)
    t_aff = t_burn + lead

    def ring(t):
        return clip_rect(
            front_polygon(origin, heading, head, flank, back, t, n=n), region)

    return {
        "burn": ring(t_burn),
        "affected": ring(t_aff),
        "t_burn_s": t_burn,
        "t_affected_s": t_aff,
        "lead_s": lead,
        "lead_bound": "people" if ppl_lead > frac_lead else "lead_frac",
        "people_lead_s": float(people_lead_s),
        "lead_frac": float(lead_frac),
        "margin_s": float(margin_s),
    }
